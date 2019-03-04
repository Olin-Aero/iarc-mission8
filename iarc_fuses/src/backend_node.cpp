#include <cstddef> //size_t
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include "ba.hpp"

/* G2O */
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

/* ROS */
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace iarc{
	struct Landmark{
		size_t base_id;
		size_t pose_id;
		Eigen::Vector3d position;
		std::string name;
	};

	struct Pose{
		double stamp;
		size_t base_id;
		Eigen::Isometry3d pose;
	};

	struct BoW{
		BoW(cv::Mat& img, cv::Mat& kpt, cv::Mat& dsc);
		bool match(BoW&);
	};

	struct Frame{
		Pose pose;
		cv::Mat img;
		cv::Mat kpt;
		cv::Mat dsc;
		BoW bow;
		//g2o::VertexSE3Expmap v;
	};

	struct Edge{
		g2o::EdgeSE3Expmap edge;
	};

	// float p2p_err(const Node& a, const Node& b)
	// pose-to-pose correspondent keypoint reprojection error
	// return 0.0 if there exist insufficient number of inlier matches
	// indicating that the visual relative pose cannot be computed
	// (i.e. the two poses are NOT related apart from graph)
	float p2p_err(const Frame& a, const Frame& b);
	
	struct BASolver{
		g2o::SparseOptimizer optimizer;

		BASolver(){
			g2o::BlockSolver_6_3::LinearSolverType* linearSolver =  new
				g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();

			g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );

			g2o::OptimizationAlgorithmLevenberg* algorithm = new
				g2o::OptimizationAlgorithmLevenberg( block_solver );

			optimizer.setAlgorithm( algorithm );
			optimizer.setVerbose( false );
		}
	};

	struct PoseGraph{
		std::vector<Frame> frame;
		BASolver solver;

		bool loop_close(){ // attempt to run BA + update drone & landmark positions
			// TODO : impl
			// 1. translate Graph to G2O data structure

			// Add Camera Intrinsic Params
			// solver.optimizer.addParameter(camera)
			// reference:
			// g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
			// camera->setId(0);
			// optimizer.addParameter( camera );

			// Add Vertices
			// TODO : I think for multi-robot scenarios a dummy vertex is necessary
			// that serves as global origin that gets fixed?
			// solver.optimizer.addVertex(g2o::VertexSE3Expmap( ... frame.pose ... ))
			// reference:
			//g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
			//v->setId(i);
			//if( i== 0 )
			//	v->setFixed( true );  // fix the first node
			//// the predetermined value is identity pose

			//v->setEstimate( g2o::SE3Quat() );
			//optimizer.addVertex( v );


			// Add feature points
			// solver.optimizer.addVertex( _matched_feature_points_ )
			// reference:
			//g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
			//v->setId( 2 + i );
			//// set depth to 1
			//double z = 1;
			//double x = ( pts1[i].x - cx ) * z / fx;
			//double y = ( pts1[i].y - cy ) * z / fy;
			//v->setMarginalized( true );
			//v->setEstimate( Eigen::Vector3d(x, y, z) );
			//optimizer.addVertex( v );

			// Add Frame->Point Edges (old/new frame -> matched 2D points)
			// g2o::EdgeProjectXYZ2UV* edges
			// reference:
			//g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
			//edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>
			//		(optimizer.vertex(i + 2)) );
			//edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>
			//		(optimizer.vertex(0)) );
			//edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
			//edge->setInformation( Eigen::Matrix2d::Identity() );
			//edge->setParameterId(0, 0);
			//// kernel function
			//edge->setRobustKernel( new g2o::RobustKernelHuber() );
			//optimizer.addEdge( edge );
			//edges.push_back( edge );


			// 2. Run G2o Optimization
		    // std::cout << "starting optimize" << std::endl;
			// optimizer.setVerbose( true );
			// optimizer.initializeOptimization();
			// optimizer.optimize(10);
			// std::cout << "finish optimize" << std::endl;
			return false;
		}

		void insert(Frame& nxt){
			// if(! is_keyframe(nxt) ) return;

			// search loop closure candidate
			for(auto& prv : frame){
				if(!prv.bow.match(nxt.bow)) continue;
				float err = p2p_err(prv, nxt); // assume mean inlier reprojection error
				if(0.1f > err || err > 1.0f) continue; // error too big (erroneous or unstable match) or too small (not worth BA)
				// match + "adequate" pose error
				if(loop_close()) break; // successful loop closure
			}

			// automatically insert relative edge from previous recorded base position
			// with continuous input front-end odometry assumption
			for(auto it=frame.rbegin(); it!=frame.rend(); ++it){
				auto& prv = *it;
				if(prv.pose.base_id == nxt.pose.base_id){
					// TODO : impl
					// don't forget to populate default pose-to-pose information matrix!!
					//edges.push_back( rel_edge(prv, nxt)
				}
			}

			// finalize & cache frame
			frame.push_back(nxt);
		}

	};

	class BackendManager{
		PoseGraph graph;
		std::vector<Landmark> landmark;
	};

	/* ROS Start */
	struct FrameSub{
		using img_t=sensor_msgs::Image;
		using info_t=sensor_msgs::CameraInfo;
		using pose_t=geometry_msgs::PoseStamped;

		message_filters::Subscriber<pose_t> pose_sub;
		message_filters::Subscriber<img_t> img_sub;
		message_filters::Subscriber<info_t> info_sub;
		message_filters::TimeSynchronizer<pose_t,img_t,info_t> sync;
		// NOTE: or use policy-based synchronizer?

		void data_cb(
				const pose_t::ConstPtr& pose_msg,
				const img_t::ConstPtr& img_msg,
				const info_t::ConstPtr& info_msg
				);

		FrameSub(ros::NodeHandle& nh,
				std::string& pose_topic,
				std::string& img_topic,
				std::string& info_topic
				//int pose_queue_size,
				//int img_queue_size,
				//int info_queue_size
				):
			pose_sub(nh, pose_topic, 8),
			img_sub(nh, img_topic, 4),
			info_sub(nh, info_topic, 8),
			sync(pose_sub, img_sub, info_sub, 30) // TODO :smaller queue_size?
		{
			sync.registerCallback(boost::bind(&FrameSub::data_cb, this, _1));
		}
	};

	class BackEndManagerROS{
		BackendManager mgr;
		ros::NodeHandle nh;
		std::vector<FrameSub> subs; // subscribers

		BackEndManagerROS();
	};

}

int main(){
	// test single-robot case
}
