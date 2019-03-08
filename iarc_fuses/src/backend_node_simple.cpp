#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 
#define EIGEN_DONT_VECTORIZE

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
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "loop_closure.hpp"

class BackEndNodeSimple{
  private:
      ros::NodeHandle& nh_;
      image_transport::CameraSubscriber sub_;
      image_transport::ImageTransport it_;
      tf::TransformListener tf_;
      tf::TransformBroadcaster tfb_;

      std::vector<Eigen::Isometry3d> poses, poses_gt;
      std::vector<cv::Mat> imgs;

  public:
      BackEndNodeSimple(ros::NodeHandle& nh)
          :nh_(nh),it_(nh){
              sub_ = it_.subscribeCamera("hmm", 10, &BackEndNodeSimple::data_cb, this);
          }

      void data_cb(
              const sensor_msgs::ImageConstPtr& img_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg){
          auto cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");

          // take care of transforms
          tf::StampedTransform xform, xform_gt;
          Eigen::Isometry3d pose, pose_gt;

          try{
              tf_.waitForTransform("odom", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(0.1));
              tf_.lookupTransform("odom", info_msg->header.frame_id, info_msg->header.stamp, xform_gt); 

              tf_.waitForTransform("noise", info_msg->header.frame_id, info_msg->header.stamp, ros::Duration(0.1));
              tf_.lookupTransform("noise", info_msg->header.frame_id, info_msg->header.stamp, xform); 

              tf::transformTFToEigen(xform.inverse(), pose);
              tf::transformTFToEigen(xform_gt.inverse(), pose_gt);

              if(imgs.size() == 0){
                  // start with truth info
                  tf::transformTFToEigen(xform_gt.inverse(), pose);
              }else{
                  tf::transformTFToEigen(xform.inverse(), pose);
              }

          }catch(tf::LookupException& e){
              std::cout << e.what() << std::endl;
              return;
          }catch(tf::TransformException& e){
              std::cout << e.what() << std::endl;
              return;
          }catch(tf::ExtrapolationException e){
              std::cout << e.what() << std::endl;
              return;
          }catch(tf::ConnectivityException& e){
              std::cout << e.what() << std::endl;
              return;
          }

          poses.push_back(pose); 
          poses_gt.push_back(pose_gt);
          imgs.push_back( cv_ptr->image );
      }

      void step(){
          Eigen::Isometry3d opt_pose;
          if(imgs.size() > 200){
              Frame d0, d1; // should technically be managed by self
              bool lc_suc = loop_closure(
                      poses, opt_pose, 
                      imgs.front(), imgs.back(),
                      //imgs[ std::max(0, int(imgs.size()) - 20)],
                      //imgs.back(),
                      d0, d1, true);
              std::cout << "LC Suc : " << lc_suc << std::endl;
              if(lc_suc){
                  tf::Transform xform;
                  tf::transformEigenToTF(opt_pose.inverse(), xform);
                  tf::StampedTransform xform_stamped(xform, ros::Time::now(), "odom", "camera_optical_lc_post");
                  tfb_.sendTransform(xform_stamped);

                  tf::Transform xform_pre;
                  tf::transformEigenToTF(poses.back().inverse(), xform_pre);
                  tf::StampedTransform xform_stamped_pre(
                          xform_pre, ros::Time::now(), "odom", "camera_optical_lc_pre");
                  tfb_.sendTransform(xform_stamped_pre);

                  tf::Transform xform_gt;
                  tf::transformEigenToTF(poses_gt.back().inverse(), xform_gt);
                  tf::StampedTransform xform_stamped_gt(
                          xform_gt, ros::Time::now(), "odom", "camera_optical_lc_gt");
                  tfb_.sendTransform(xform_stamped_gt);

                  cv::imshow("lc0", imgs.front());
                  cv::imshow("lc1", imgs.back());
                  cv::waitKey(1);

                  poses.clear();
                  imgs.clear();


              }
          }
      }

      void run(){
          cv::namedWindow("lc0", cv::WINDOW_NORMAL);
          cv::namedWindow("lc1", cv::WINDOW_NORMAL);
          ros::Rate rate(10);
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
