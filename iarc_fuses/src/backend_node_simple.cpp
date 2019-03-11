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
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

#include "loop_closure.hpp"

class BackEndNodeSimple{
  private:
      ros::NodeHandle& nh_;

      ros::Publisher p0_pub_;
      ros::Publisher p1_pub_;

      image_transport::CameraSubscriber sub_;
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

  public:
      BackEndNodeSimple(ros::NodeHandle& nh)
          :nh_(nh),it_(nh), lc_req_(false){
              // dealing with fixed map frame for now
              // TODO : consider multi-robot configuration
              map_frame_ = "map";
              odom_frame_ = "odom";

              // T_a2b = source frame a, target frame b
              T_o2m_ = Eigen::Isometry3d::Identity(); // map coordinates

              new_kf_ = false;
              // TODO: nh_.getParam<>( srcs_ ... ) to get camera sources

              sub_ = it_.subscribeCamera("hmm", 10, &BackEndNodeSimple::data_cb, this);
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

      int nmatch(const cv::Mat& dsc0, const cv::Mat& dsc1,
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
                  ++n;
              }
          }
          return n;
          // TODO : recycle match results from here
      }

      bool is_keyframe(const Frame& kf1){
          if (kfs_.size() <= 0) return true; // no pervious frame to compare to

          const Frame& kf0 = kfs_.back();

          int ixn = nmatch(kf0.dsc, kf1.dsc);
          int uxn = ( kf0.kpt.size() + kf1.kpt.size() - ixn);
          float iou = float(ixn) / uxn;

          // "insignificant" overlap with previous frame == keyframe
          // threshold = < 10%
          // TODO : tune
          return iou < 0.2;
      }

      void data_cb(
              const sensor_msgs::ImageConstPtr& img_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg){

          if( (info_msg->header.stamp - prv_).toSec() < 0.25 ){
              return;
          }
          prv_ = info_msg->header.stamp;

          auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");

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
              tf_.waitForTransform(odom_frame_, info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(0.1));
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
          //if(!lc_req_) return;
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

          // prepare loop closure
          int lc_idx = kfs_.size()-3;
          auto& kf1 = kfs_.back();

          // search loop closure
          //for(auto it=kfs_.rbegin()+2; it != kfs_.rend(); ++it){
          //    //+2 to skip two neighboring frames
          //    auto& kf0 = *it;
          bool run_lc = false;
          for(int i=kfs_.size()-3; i>=0; --i){
              auto& kf0 = kfs_[i];

              // object-level pointer-match check
              // don't match against self since it's stupid
              if(&kf0 == &kf1) continue;

              // evaluate jaccard score
              // TODO: avoid code repetition
              // TODO: employ weak-strong sequential matches to avoid overprocessing
              // Look into BoW
              int ixn = nmatch(kf0.dsc,  kf1.dsc);
              int uxn = (kf0.kpt.size() + kf1.kpt.size() - ixn);
              float iou = float(ixn) / uxn;

              if(iou > 0.1){ 
                  // TODO: magic; verified by plotting, but not exactly intuitive.
                  lc_idx = i;
                  run_lc = true;
                  break;
              }
              //--lc_idx;
          }

          if(!run_lc || lc_idx < 0 || lc_idx >= kfs_.size() - 1){
              // no loop closure candidate detected
              return;
          }

          // optimization data
          Frame& kf0 = kfs_[lc_idx];

          std::vector<Eigen::Isometry3d> lc_poses; // subvector of loop chain poses
          for(auto it=kfs_.begin()+lc_idx; it != kfs_.end(); ++it){
              lc_poses.push_back( it->pose );
          }
          std::cout << "LC_POSES LEN : " << lc_poses.size() << std::endl;

          Eigen::Isometry3d prv_pose1 = lc_poses.back();
          Eigen::Isometry3d opt_pose1;

          // run loop closure resolution (BA)
          // TODO: make non-stupid function signature
          // bool loop_closure(std::vector<Frame>& kfs_, ...)
          bool lc_suc = loop_closure(
                  lc_poses, opt_pose1,
                  kf0.img, kf1.img,
                  kf0, kf1, true);

          std::cout << "Loop Closure Success: " << lc_suc << std::endl;

          if(lc_suc){
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
          ros::Rate rate(4);
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
