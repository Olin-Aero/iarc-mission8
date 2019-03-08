// for std
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 
#define EIGEN_DONT_VECTORIZE

#include <iostream>

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "loop_closure.hpp"

// intrinsic camera parameters
double cx = 322.45;
double cy = 174.24;
double fx = 396.2;
double fy = 399.8;

// fast weak match with high recall / low precision
bool weak_match(
	const cv::Mat& img0,
	const cv::Mat& img1,
	Frame& d0,
	Frame& d1);

// slow strong match with high recall / high precision
bool strong_match(
		const cv::Mat& img0,
		const cv::Mat& img1,
		Frame& d0,
		Frame& d1,
		// return matching indices into d0->kpt, d1->kpt
		std::vector<std::pair<int, int>>& m
		);

// TODO: BoW impl.
bool weak_match(const cv::Mat&, const cv::Mat&, Frame&, Frame&){
	return true;
}

// KPT-DSC-EM Match
bool strong_match(
		const cv::Mat&,
		const cv::Mat&,
		Frame& d0,
		Frame& d1,
		// return matching indices into d0->kpt, d1->kpt
		std::vector<std::pair<int, int>>& m
		){
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create( "BruteForce-Hamming" );

    double knn_match_ratio = 0.8;
    std::vector< std::vector<cv::DMatch> > matches_knn;
    matcher->knnMatch(d0.dsc, d1.dsc, matches_knn, 2);
    for(size_t i = 0; i < matches_knn.size(); ++i)
    {
        if( matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance ){
			m.push_back(
					std::make_pair(
						matches_knn[i][0].queryIdx,
					   	matches_knn[i][0].trainIdx
						));

		}
    }

	return (m.size() >= 20);
}

bool loop_closure(
		const std::vector<Eigen::Isometry3d>& poses,
		Eigen::Isometry3d& optimized_pose,
		const cv::Mat& img0,
		const cv::Mat& img1,

		// params to maybe use in the future
		Frame& d0,
		Frame& d1,
		bool proc
		){

	if(proc){
		d0.kpt.clear();
		d1.kpt.clear();
		cv::Ptr<cv::ORB> orb = cv::ORB::create();
		std::vector<cv::KeyPoint> kpt0, kpt1;
		orb->detectAndCompute(img0, cv::Mat(), kpt0, d0.dsc);
		orb->detectAndCompute(img1, cv::Mat(), kpt1, d1.dsc);
		for(auto& p : kpt0){d0.kpt.push_back(p.pt);}
		for(auto& p : kpt1){d1.kpt.push_back(p.pt);}
	}

	if(! weak_match(img0, img1, d0, d1)){
		std::cout << "WEAK MATCH FAILED" << std::endl;
		return false;
	}
	std::vector< std::pair<int,int> > m;
	if(! strong_match(img0, img1, d0, d1, m)){
		std::cout << "STRONG MATCH FAILED" << std::endl;
		return false;
	}

	// g2o_setup()
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver =  new
        g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* algorithm = new
        g2o::OptimizationAlgorithmLevenberg( block_solver );
    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );

	// add_node()
	int idx = 0;
	Eigen::Isometry3d tx0;

	for(auto& p : poses){
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(idx);
		//Eigen::Isometry3d p_e;
		//tf::transformTFToEigen(p, p_e);

        if( idx == 0 ){
            v->setFixed( true );  // fix the first node
			tx0 = p; // remember transformation to apply to landmark
		}
		v->setEstimate( g2o::SE3Quat( p.linear(), p.translation() ) );
        optimizer.addVertex( v );

		// increment vertex count
		++idx;
    }

	// setup simple alias
	const int pose_idx0 = 0;
	const int pose_idx1 = (idx-1);
	const int lmk_idx0 = idx;

    // set euclidean landmark feature point, based on img1
    for(size_t i = 0; i < m.size(); ++i)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( lmk_idx0 + i );

        // set depth to 1 & and add euclidean point
		// TODO : apply first pose to this
		
        double z = 1;
        double x = ( d0.kpt[m[i].first].x - cx ) * z / fx;
        double y = ( d0.kpt[m[i].first].y - cy ) * z / fy;

        v->setMarginalized( true );
        v->setEstimate( tx0 * Eigen::Vector3d{x,y,z} );
        //v->setEstimate( Eigen::Vector3d{x,y,z} );
		// <<<< NOTE : check valid transform
        optimizer.addVertex( v );
    }

    g2o::VertexSE3Expmap* v_init = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d v_init_pose = v_init->estimate();
    std::cout << "Initial Pose = " << std::endl << v_init_pose.matrix() << std::endl;

    // set intrinsic camera parameter
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );

	// set edges

	// first frame
    std::vector<g2o::EdgeProjectXYZ2UV*> edges;
    for(size_t i = 0; i < m.size(); ++i)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>
                (optimizer.vertex(lmk_idx0 + i)) ); // target point
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>
                (optimizer.vertex(pose_idx0)) ); // observation pose
        edge->setMeasurement( Eigen::Vector2d(d0.kpt[m[i].first].x, d0.kpt[m[i].first].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        // keneral function
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back( edge );
    }


    // last frame
    for(size_t i = 0; i < m.size(); ++i)
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>
            (optimizer.vertex(lmk_idx0+i)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>
                (optimizer.vertex(pose_idx1)) );
        edge->setMeasurement( Eigen::Vector2d(d1.kpt[m[i].second].x, d1.kpt[m[i].second].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        // keneral function
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back( edge );
    }

	// TODO : do I need to add pose-based edges?
	//Eigen::Matrix<double,6,1> odom_Hv;
	//float spi2 = 1.0 / pow(1., 2);
	//float sri2 = 1.0 / pow(1., 2);

	//odom_Hv << sri2, sri2, sri2, spi2, spi2, spi2;
	//std::cout << odom_Hv << std::endl;
	//Eigen::Matrix<double,6,6> odom_H = odom_Hv.asDiagonal();
	////Eigen::Matrix<double,6,6> odom_H = Eigen::Matrix<double,6,6>::Identity();

    ////std::vector<g2o::EdgeSE3Expmap*> p_edges;

	//for(size_t i=1; i < poses.size(); ++i){
	//	//g2o::EdgeSE3* edge = new g2o::EdgeSE3();
	//	g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();

	//	g2o::VertexSE3Expmap* v0 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i-1));
	//	g2o::VertexSE3Expmap* v1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));

	//	//edge->setVertex(i-1, v0);
	//	//edge->setVertex(i,   v1);
	//	edge->setVertex(0, v0);
	//	edge->setVertex(1,   v1);

	//	edge->setMeasurement( g2o::SE3Quat(v1->estimate()).inverse() * g2o::SE3Quat(v0->estimate()) );

	//	// xyz - rpy
	//	edge->setInformation(odom_H);

    //    optimizer.addEdge( edge );
    //    //p_edges.push_back( edge );
	//}

	// run optimization
	std::cout << "starting optimize" << std::endl;
    optimizer.setVerbose( false );
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    std::cout << "finish optimize" << std::endl;

    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose =v->estimate();
    std::cout << "Pose = " << std::endl << pose.matrix() << std::endl;

	optimized_pose = pose;

    //for( size_t i = 0; i < m.size(); ++i )
    //{
    //    g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>
    //        (optimizer.vertex(lmk_idx0 + i));
    //    std::cout << "vertex id " << lmk_idx0 + i << ", pos = ";
    //    Eigen::Vector3d pos = v->estimate();
    //    std::cout << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;
    //}

    //int inliers = 0;
    //for( auto e : edges )
    //{
    //    e->computeError();
    //    if( e->chi2() > 1 )
    //    {
    //        std::cout << "error = " << e->chi2() << std::endl;
    //    }
    //    else
    //    {
    //        inliers++;
    //    }
    //}

	//std::cout << " -- start pose edges -- " << std::endl;
    //for( auto e : p_edges )
    //{
    //    e->computeError();
    //    if( e->chi2() > 1 )
    //    {
    //        std::cout << "error = " << e->chi2() << std::endl;
    //    }
    //    else
    //    {
    //        std::cout << "inlier! error = " << e->chi2() << std::endl;
    //        inliers++;
    //    }
    //}

    //std::cout << "inliers in total points: " <<
    //    inliers << "/" << m.size() << std::endl; // TODO : this number is actually different now!
    //optimizer.save("/tmp/ba.g2o");

	// TODO : determine success criterion
    return 1;
}

//__attribute__((force_align_arg_pointer)) int main( int argc, char** argv )
//{
//    if(argc != 3)
//    {
//        std::cout << "Usage: g2o_example img1, img2" << std::endl;
//        exit(1);
//    }
//
//	Eigen::Matrix4d tmp;
//	tmp << 
//    0.999561, -0.000456634,    0.0296313,    0.0144603,
//-2.80105e-05,     0.999866,    0.0163533,    0.0133354,
//  -0.0296349,    -0.016347,     0.999427,    0.0139213,
//           0,            0,            0,            1;
//	std::cout << tmp << std::endl;
//
//
//	std::vector<Eigen::Isometry3d> poses{
//		Eigen::Isometry3d::Identity(),
//		Eigen::Isometry3d(tmp)//::Identity()
//	};
//	
//	Frame d1, d2;
//    cv::Mat img1 = cv::imread( argv[1] );
//    cv::Mat img2 = cv::imread( argv[2] );
//
//	bool suc = loop_closure(
//			poses,
//			img1, img2,
//			d1, d2);
//
//	std::cout << suc << std::endl;
//    return 0;
//}
