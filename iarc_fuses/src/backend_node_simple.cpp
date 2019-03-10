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
      tf::TransformBroadcaster tfb_;

      std::vector<Eigen::Isometry3d> poses;
      std::vector<cv::Mat> imgs;
      ros::ServiceServer srv_;
      ros::Time prv_;
      bool lc_req_;

      std::vector<Frame> kfs_;

      cv::Ptr<cv::ORB> orb;
      cv::Ptr<cv::DescriptorMatcher> matcher_;

  public:
      BackEndNodeSimple(ros::NodeHandle& nh)
          :nh_(nh),it_(nh), lc_req_(false){
              sub_ = it_.subscribeCamera("hmm", 10, &BackEndNodeSimple::data_cb, this);
              srv_ = nh_.advertiseService("run_lc",
                      &BackEndNodeSimple::loop_closure_cb, this);

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
          lc_req_ = true;
          return true;
      }

      int nmatch(const cv::Mat& dsc0, const cv::Mat& dsc1,
              float lowe_ratio=0.8
              ){
          std::vector< std::vector<cv::DMatch> > matches_knn;
          matcher_->knnMatch(dsc0, dsc1, matches_knn, 2);

          int n = 0;
          for(auto& m2 : matches_knn){
              if(m2[0].distance < lowe_ratio * m2[1].distance){
                  ++n;
              }
          }
          return n;

          // TODO : recycle match results from here ^^
      }

      bool is_keyframe(const Frame& d1){
          if (kfs_.size() <= 0) return true;

          const Frame& d0 = kfs_.back();

          int ixn = nmatch(d0.dsc, d1.dsc);
          int uxn = ( d0.kpt.size() + d1.kpt.size() - ixn);
          float iou = float(ixn) / uxn;

          // "insignificant" overlap with previous frame == keyframe
          return iou < 0.1;
      }

      void data_cb(
              const sensor_msgs::ImageConstPtr& img_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg){

          if( (info_msg->header.stamp - prv_).toSec() < 0.25 ){
              return;
          }
          prv_ = info_msg->header.stamp;

          auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");

          // populate frame
          Frame d1;
          std::vector<cv::KeyPoint> kpt0, kpt1;
          orb->detectAndCompute(cv_ptr->image, cv::Mat(), kpt1, d1.dsc);
          for(auto& p : kpt0){d1.kpt.push_back(p.pt);}

          if(!is_keyframe(d1)) return;

          // take care of transforms
          tf::StampedTransform xform;
          Eigen::Isometry3d pose;

          try{
              tf_.waitForTransform("odom", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(0.1));
              tf_.lookupTransform("odom", info_msg->header.frame_id, info_msg->header.stamp, xform); 
              tf::transformTFToEigen(xform, pose);

              //std::cout << "good" << std::endl;
              poses.push_back(pose); 
              imgs.push_back( cv_ptr->image );
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
          if(poses.size() <= 2) return;
          //if(!new_kf_) return;

          // i0 = search_lc();
          // d0 = kfs_[i0];
          // d1 = kfs_.back();
          // bool lc_suc = loop_closure(poses, d0, d1, opt_pose, false);

          Eigen::Isometry3d p0=poses.front(), p1=poses.back();

          Eigen::Isometry3d opt_pose;
          //if(imgs.size() > 200){
          Frame d0, d1; // should technically be managed by self

          // d0 = ??
          // d1 = new_frame_
          // 1 ) detect_loop_closure
          // for (auto& f : kfs_){ if(loop_detected(f, 

          bool lc_suc = loop_closure(
                  poses, opt_pose, 
                  //p2, opt_pose,
                  imgs.front(), imgs.back(),
                  //imgs[ std::max(0, int(imgs.size()) - 20)],
                  //imgs.back(),
                  d0, d1, true);
          // WxH = 856x480
          //std::cout << "OptPose = " << std::endl << opt_pose.matrix() << std::endl;

          std::cout << "LC Suc : " << lc_suc << std::endl;
          if(lc_suc){

              ros::Time now = ros::Time::now();

              tf::Transform xform;
              tf::transformEigenToTF(opt_pose, xform);
              tf::StampedTransform xform_stamped(
                      xform, now,
                      "odom", "camera_optical_lc_post");
              tfb_.sendTransform(xform_stamped);

              tf::Transform xform_pre;
              tf::transformEigenToTF(p1, xform_pre);
              tf::StampedTransform xform_stamped_pre(
                      xform_pre, now,
                      "odom", "camera_optical_lc_pre");
              tfb_.sendTransform(xform_stamped_pre);

              //cv::imshow("lc0", imgs.front());
              //cv::imshow("lc1", imgs.back());
              //cv::waitKey(1);

              poses.clear();
              imgs.clear();
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
