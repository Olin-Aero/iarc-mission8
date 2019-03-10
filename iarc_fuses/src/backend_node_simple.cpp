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
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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

  public:
	  BackEndNodeSimple(ros::NodeHandle& nh)
		  :nh_(nh),it_(nh), lc_req_(false){
			  sub_ = it_.subscribeCamera("hmm", 10, &BackEndNodeSimple::data_cb, this);
			  srv_ = nh_.advertiseService("run_lc",
					  &BackEndNodeSimple::loop_closure_cb, this);

              // TODO : publish before & after trajectories
              p0_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_0", 10);
              p1_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_1", 10);
		  }

	  bool loop_closure_cb(
			  std_srvs::EmptyRequest&,
			  std_srvs::EmptyResponse& 
			  ){
		  lc_req_ = true;
		  return true;
	  }

	  void data_cb(
			  const sensor_msgs::ImageConstPtr& img_msg,
			  const sensor_msgs::CameraInfoConstPtr& info_msg){

          if( (info_msg->header.stamp - prv_).toSec() < 0.25 ){
              return;
          }
          prv_ = info_msg->header.stamp;

		  auto cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");


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
		  if(!lc_req_) return;
		  if(poses.size() <= 2) return;

          Eigen::Isometry3d p0=poses.front(), p1=poses.back();

		  Eigen::Isometry3d opt_pose;
		  //if(imgs.size() > 200){
		  Frame d0, d1; // should technically be managed by self

		  //auto&& p2 = {p0,p1};

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
