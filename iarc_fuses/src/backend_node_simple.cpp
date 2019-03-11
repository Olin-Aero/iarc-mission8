//#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 
//#define EIGEN_DONT_VECTORIZE

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

#include "loop_closure.hpp"

void eig2msg(
        const Eigen::Isometry3d& e,
        geometry_msgs::Pose& p){
    geometry_msgs::Transform xfm_msg = 
        tf2::eigenToTransform(e).transform;
    tf2::Transform xfm_tf;
    tf2::fromMsg(xfm_msg, xfm_tf);
    tf2::toMsg(xfm_tf, p);
}

class BackEndNodeSimple{
  private:
      ros::NodeHandle& nh_;

      ros::Publisher p0_pub_;
      ros::Publisher p1_pub_;

      image_transport::CameraSubscriber sub_;
      image_transport::Publisher pub0_, pub1_;
      image_transport::ImageTransport it_;
      tf::TransformListener tf_;
      tf::TransformBroadcaster tfb_; // TODO : use tf2

      ros::ServiceServer srv_;
      ros::Time prv_;
      bool lc_req_;

      std::vector<Frame> kfs_;

      cv::Ptr<cv::ORB> orb;
      cv::Ptr<cv::DescriptorMatcher> matcher_;

      std::string map_frame_;
      std::string odom_frame_;
      bool new_kf_;

      std::vector<std::string> srcs_; // observation sources
      Eigen::Isometry3d T_o2m_;
      image_geometry::PinholeCameraModel cam_;

  public:
      BackEndNodeSimple(ros::NodeHandle& nh)
          :nh_(nh),it_(nh), lc_req_(true){
              // dealing with fixed map frame for now
              // TODO : consider multi-robot configuration
              map_frame_ = "map";
              odom_frame_ = "odom";

              // T_a2b = source frame a, target frame b
              T_o2m_ = Eigen::Isometry3d::Identity(); // map coordinates

              new_kf_ = false;
              // TODO: nh_.getParam<>( srcs_ ... ) to get camera sources

              sub_ = it_.subscribeCamera("hmm", 10, &BackEndNodeSimple::data_cb, this);
              pub0_ = it_.advertise("lc_img0", 2);
              pub1_ = it_.advertise("lc_img1", 2);

              srv_ = nh_.advertiseService("run_lc",
                      &BackEndNodeSimple::loop_closure_cb, this);

              // TODO : Backend state management 
              // srv_ = nh_advertiseService("reset_lc", ...) << reset keyframe data or cache

              // TODO : publish before & after trajectories
              p0_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_0", 10);
              p1_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_1", 10);

              orb = cv::ORB::create();
              matcher_=cv::DescriptorMatcher::create( "BruteForce-Hamming" );

          }

      bool loop_closure_cb(
              std_srvs::EmptyRequest&,
              std_srvs::EmptyResponse& 
              ){
          // debug: enforce loop closure
          // (not really happening right now)
          lc_req_ = true;
          return true;
      }

      int nmatch(
              const cv::Mat& dsc0, const cv::Mat& dsc1,
              std::vector<cv::DMatch>& matches,
              float lowe_ratio=0.8
              ){
          // returns number of matches across dsc0<->dsc1
          // TODO: support cross-matching verification
          // and epipolar geometry validation.

          std::vector< std::vector<cv::DMatch> > matches_knn;
          matcher_->knnMatch(dsc0, dsc1, matches_knn, 2);

          int n = 0;
          for(auto& m2 : matches_knn){
              if(m2[0].distance < lowe_ratio * m2[1].distance){
                  matches.push_back( m2[0] );
                  ++n;
              }
          }
          return n;
          // TODO : recycle match results from here
      }

      bool is_keyframe(const Frame& kf1){
          if (kfs_.size() <= 0) return true; // no pervious frame to compare to

          const Frame& kf0 = kfs_.back();

          std::vector<cv::DMatch> matches;
          int ixn = 2 * nmatch(kf0.dsc, kf1.dsc, matches);
          int uxn = ( kf0.kpt.size() + kf1.kpt.size() - ixn);
          float iou = float(ixn) / uxn;

          // "insignificant" overlap with previous frame == keyframe
          // threshold = < 10%
          // TODO : tune
          return iou < 0.33;
      }

      void data_cb(
              const sensor_msgs::ImageConstPtr& img_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg){

          if( (info_msg->header.stamp - prv_).toSec() < 0.1 ){
              return;
          }
          prv_ = info_msg->header.stamp;

          auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
          cam_.fromCameraInfo(info_msg);

          // populate frame, visual data
          Frame kf1;
          std::vector<cv::KeyPoint> kpt1;
          kf1.img = cv_ptr->image;
          orb->detectAndCompute(cv_ptr->image, cv::Mat(), kpt1, kf1.dsc);
          for(auto& p : kpt1){kf1.kpt.push_back(p.pt);}

          if(!is_keyframe(kf1)) return;

          try{
              // take care of transforms
              tf::StampedTransform xform;
              Eigen::Isometry3d pose;

              // populate frame, geometric data
              tf_.waitForTransform(odom_frame_, info_msg->header.frame_id, info_msg->header.stamp,
                      ros::Duration(0.1));
              tf_.lookupTransform(odom_frame_, info_msg->header.frame_id, info_msg->header.stamp, xform); 
              tf::transformTFToEigen(xform, pose);

              //std::cout << "good" << std::endl;
              kf1.pose = pose;
              kfs_.push_back(std::move(kf1)); // TODO : verify kf1 copy/assignment persistence
              new_kf_ = true; // yay, new keyframe!!
              std::cout << "KF " << kfs_.size() << std::endl;
          }catch(tf::LookupException& e){
              std::cout << e.what() << std::endl;
          }catch(tf::TransformException& e){
              std::cout << e.what() << std::endl;
          }catch(tf::ExtrapolationException e){
              std::cout << e.what() << std::endl;
          }catch(tf::ConnectivityException& e){
              std::cout << e.what() << std::endl;
          }

      }

      void step(){
          if(kfs_.size() <= 4 || !new_kf_){
              tf::Transform xform;
              tf::transformEigenToTF(T_o2m_, xform);
              tf::StampedTransform xform_stamped(
                      xform, ros::Time::now(), // TODO: not now?
                      map_frame_, odom_frame_);
              tfb_.sendTransform(xform_stamped);
              return;
          }
          new_kf_ = false;
          //if(!lc_req_) return;

          // prepare loop closure
          //int lc_idx = 0;
          auto& kf1 = kfs_.back();

          // search loop closure

          bool run_lc = false;
          int lc_idx = -1; //(kfs_.size() - 1 -2);
          std::vector<cv::DMatch> matches;

          //for(auto& kfs : kfs_){
          //for(auto it=kfs_.begin(); it!=kfs_.end(); ++it){
          //for(auto it=kfs_.rbegin()+2; it!=kfs_.rend(); ++it){
          for(int i = kfs_.size()-3;  i>0; --i){
              //auto& kf0 = *it;
              auto& kf0 = kfs_[i];

              //for(auto& kf0 : kfs_){
              // object-level pointer-match check
              // don't match against self since it's stupid
              if(&kf0 == &kf1) continue;

              // evaluate jaccard score
              // TODO: avoid code repetition
              // TODO: employ weak-strong sequential matches to avoid overprocessing
              // Look into BoW
              int ixn = 2*nmatch(kf0.dsc,  kf1.dsc, matches);
              int uxn = (kf0.kpt.size() + kf1.kpt.size() - ixn);
              float iou = float(ixn) / uxn;

              if(iou > 0.2){ 
                  // TODO: magic; verified by plotting, but not exactly intuitive.
                  //lc_idx = std::distance(it, kfs_.rend())-1;
                  //lc_idx = std::distance(it, kfs_.end());
                  lc_idx = i;

                  if (lc_idx < kfs_.size() - 3){
                      run_lc = true;
                      break;
                  }
              }
          }

          if(!run_lc || lc_idx < 0 || lc_idx >= kfs_.size() -1){
              // no loop closure candidate detected
              return;
          }

          // reject neighboring frames
          if( lc_idx >= kfs_.size() - 3) return;
          //std::cout << "index : " << lc_idx << ' ' << dbg << std::endl;

          // optimization data
          Frame& kf0 = kfs_[lc_idx];

          std::vector<Eigen::Isometry3d> lc_poses; // subvector of loop chain poses
          for(auto it=kfs_.begin()+lc_idx; it != kfs_.end(); ++it){
              lc_poses.push_back( it->pose );
          }
          std::cout << "LC_POSES LEN : " << lc_poses.size() << "=" << lc_idx << "<->" << (kfs_.size() - 1) << std::endl;

          Eigen::Isometry3d prv_pose1 = lc_poses.back();

          std::vector<Eigen::Isometry3d> opt_poses;

          // run loop closure resolution (BA)
          // TODO: make non-stupid function signature
          // bool loop_closure(std::vector<Frame>& kfs_, ...)
          cv::Mat K(cam_.intrinsicMatrix());

          std::cout << lc_poses.back().matrix() << std::endl;
          bool lc_suc = loop_closure(
                  lc_poses, opt_poses,
                  kf0.img, kf1.img, K,
                  kf0, kf1, false);

          std::cout << "Loop Closure Success: " << lc_suc << std::endl;
          cv::RNG rng(12345);

          if(lc_suc){
              // viz loop closure
              std_msgs::Header hdr;
              hdr.frame_id = cam_.tfFrame();
              hdr.stamp = ros::Time::now();

              cv::Mat viz_img;
              cv::hconcat(kf0.img, kf1.img, viz_img);
              for(auto& m : matches){
                  auto pt1 = kf0.kpt[ m.queryIdx ];
                  auto pt2_raw = kf1.kpt[ m.trainIdx ];
                  auto pt2 = cv::Point(pt2_raw.x+kf0.img.cols, pt2_raw.y);
                  cv::line(viz_img, pt1, pt2, cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), 5);
              }

              for(auto& m : matches){
                  auto pt1 = kf0.kpt[ m.queryIdx ];
                  auto pt2_raw = kf1.kpt[ m.trainIdx ];
                  auto pt2 = cv::Point(pt2_raw.x+kf0.img.cols, pt2_raw.y);
                  cv::circle(viz_img, pt1, 3, cv::Scalar(255,0,0), 3);
                  cv::circle(viz_img, pt2, 3, cv::Scalar(255,0,0), 3);
              }
              //cv::drawMatches(
              //        kfs_[lc_idx].img, kfs_[lc_idx].kpt,
              //        kfs_.back().img, kfs_.back().kpt,
              //        matches, viz_img);

              pub0_.publish(cv_bridge::CvImage(
                          hdr, "bgr8",
                          viz_img).toImageMsg());
              pub1_.publish(cv_bridge::CvImage(
                          hdr, "bgr8",
                          kfs_.back().img).toImageMsg());

              Eigen::Isometry3d opt_pose1 = opt_poses.back();

              // rectify keyframe poses
              // hopefully math is correct
              Eigen::Isometry3d T_cor = prv_pose1 * opt_pose1.inverse();

              nav_msgs::Path t0, t1;
              t0.header.frame_id = "map";
              t1.header.frame_id = "map";
              t0.header.stamp = ros::Time::now();
              t1.header.stamp = ros::Time::now();

              //std::cout << "why are they the same? " << std::endl;
              //std::cout << kfs_.back().pose.matrix().inverse() << std::endl;
              //std::cout << opt_poses.back().matrix().inverse() << std::endl;

              for(auto it = kfs_.begin() + lc_idx; it != kfs_.end(); ++it){
                  geometry_msgs::PoseStamped p;
                  eig2msg(T_o2m_*(it->pose), p.pose);
                  p.header.frame_id="map";
                  p.header.stamp=ros::Time::now();
                  t0.poses.push_back(p);// std::move(p) );
              }

              for(auto it = opt_poses.begin(); it != opt_poses.end(); ++it){
                  geometry_msgs::PoseStamped p;
                  eig2msg(T_o2m_*(*it), p.pose);
                  p.header.frame_id="map";
                  p.header.stamp=ros::Time::now();
                  t1.poses.push_back(p);// std::move(p) );
              }

              std::cout << t0.poses.back().pose << std::endl;
              std::cout << t1.poses.back().pose << std::endl;

              p0_pub_.publish( t0 );
              p1_pub_.publish( t1 );

              for(auto it = std::make_pair(opt_poses.begin(), kfs_.begin() + lc_idx);
                      it.first != opt_poses.end() && it.second != kfs_.end();
                      ++it.first, ++it.second){

                  /*
                     std::cout << "__________" << std::endl;
                  // previous map -> pose
                  std::cout << (T_o2m_ * (*it.first)).matrix() << std::endl;
                  // updated map -> pose
                  std::cout << (T_o2m_ * opt_pose1 * prv_pose1.inverse() * T_cor * (*it.first)).matrix() << std::endl;
                  */

                  (*it.second).pose = T_cor * (*it.first);
              }

              // NOTE:
              // prv_pose1 = T_b2o
              // opt_pose1 = T_b2o'
              // T_o2m_ * T_b2o' = T_b2m_
              // T_o2m_' * T_b2o = T_b2m_
              // T_o2m_ * T_b2o' = T_o2m_' * T_b2o
              // T_o2m_' = T_o2m_ * T_b2o' * T_b2o^{-1}
              // T_o2m_' = T_o2m_ * T_b2o * T_b2o'^{-1}
              //T_o2m_ = T_o2m_ * prv_pose1 * opt_pose1.inverse();
              T_o2m_ = T_o2m_ * opt_pose1 * prv_pose1.inverse();

              ros::Time now = ros::Time::now(); // use stamp?

              tf::Transform xform;
              tf::transformEigenToTF(T_o2m_, xform);
              tf::StampedTransform xform_stamped(
                      xform, now, // TODO: not now?
                      map_frame_, odom_frame_);
              tfb_.sendTransform(xform_stamped);

              //tf::Transform xform;
              //tf::transformEigenToTF(opt_pose1, xform);
              //tf::StampedTransform xform_stamped(
              //        xform, now,
              //        odom_frame_, "camera_optical_lc_post");
              //tfb_.sendTransform(xform_stamped);

              //tf::Transform xform_pre;
              //tf::transformEigenToTF(prv_pose1, xform_pre);
              //tf::StampedTransform xform_stamped_pre(
              //        xform_pre, now,
              //        odom_frame_, "camera_optical_lc_pre");
              //tfb_.sendTransform(xform_stamped_pre);

              //cv::imshow("lc0", imgs.front());
              //cv::imshow("lc1", imgs.back());
              //cv::waitKey(1);

              lc_req_ = false;
          }
          }

          void run(){
              //cv::namedWindow("lc0", cv::WINDOW_NORMAL);
              //cv::namedWindow("lc1", cv::WINDOW_NORMAL);
              ros::Rate rate(50);
              while(ros::ok()){
                  ros::spinOnce();
                  step();
                  rate.sleep();
              }
              ros::shutdown();
          }
      };

      int main(int argc, char* argv[]){
          ros::init(argc, argv, "loop_closure_node");
          ros::NodeHandle nh;
          BackEndNodeSimple node(nh);
          node.run();
      }
