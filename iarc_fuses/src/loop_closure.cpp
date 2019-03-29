// for std
#include <iostream>
#include <cmath>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/concept_check.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "loop_closure.hpp"
#include "utils.hpp"

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
        //std::vector<std::pair<int, int>>& m
        std::vector<cv::DMatch>& m
        );

// TODO: BoW impl.
bool weak_match(const cv::Mat&, const cv::Mat&, Frame&, Frame&){
    return true;
}

Eigen::Vector2d cam_map(g2o::CameraParameters& cp, const Eigen::Vector3d& trans_xyz){
    double f = cp.focal_length;
    double cx = cp.principle_point[0];
    double cy = cp.principle_point[1];

    double x = trans_xyz[0];
    double y = trans_xyz[1];
    double z = trans_xyz[2];

    return {x / z * f + cx, y / z *f + cy};
}

// KPT-DSC-EM Match
bool strong_match(
        const cv::Mat&,
        const cv::Mat&,
        const cv::Mat& K,
        Frame& d0,
        Frame& d1,
        // return matching indices into d0->kpt, d1->kpt
        std::vector<cv::DMatch>& m
        ){
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create( "BruteForce-Hamming" );

    double lowe_ratio = 0.8;
    std::vector< std::vector<cv::DMatch> > matches_knn;
    matcher->knnMatch(d0.dsc, d1.dsc, matches_knn, 2);

    std::vector<cv::Point2d> p0, p1;

    std::vector<cv::DMatch> m_raw;
    for(auto& m2 : matches_knn){
        if(m2[0].distance < lowe_ratio * m2[1].distance){
            m_raw.push_back(m2[0]);
            p0.push_back( d0.kpt[m2[0].queryIdx] );
            p1.push_back( d1.kpt[m2[0].trainIdx] );
        }
    }

    // epipolar filter
    cv::Mat msk;
    //cv::Mat Fmat = cv::findFundamentalMat(p0, p1, cv::FM_RANSAC, 2.0, 0.999, msk);
    cv::Mat E = cv::findEssentialMat(p0, p1, K,
            cv::RANSAC, 0.999, 3.0, msk);

    //std::cout << msk.size << std::endl;
    // apply filter
    m.clear();
    for(int i=0; i<msk.rows; ++i){
        if(!msk.at<char>(i)) continue;
        m.push_back(m_raw[i]);
    }

    float iou = jaccard(d0, d1, m.size());
    bool suc = (iou > 0.2) && (m.size() > 50);
    if (suc){
        std::cout << "IOU" << iou << std::endl;
        cv::Mat Rxn, txn;
        cv::recoverPose(E, p0, p1, K, Rxn, txn);
        std::cout << "R" << Rxn << std::endl;
        std::cout << "t" << txn << std::endl;
    }
    return suc;
}

float jaccard(
        Frame& d0,
        Frame& d1,
        int n
        ){
    float ixn = 2 * n;
    float uxn = d0.kpt.size()+d1.kpt.size()-ixn;
    float iou = (ixn / uxn);

    return iou;
}

bool loop_closure(
        const std::vector<Eigen::Isometry3d>& poses,
        const std::vector<cv::Point_<double>>& p0,
        const std::vector<cv::Point_<double>>& p1,
        const cv::Mat& K,
        std::vector<Eigen::Isometry3d>& opt_poses
        ){
    size_t n_l = p0.size();
    size_t n_p = poses.size();

    cv::Mat Kd;
    K.convertTo(Kd, CV_64F);

    const double fx = Kd.at<double>(0,0);
    const double fy = Kd.at<double>(1,1);
    const double cx = Kd.at<double>(0,2);
    const double cy = Kd.at<double>(1,2);

    // assert p0.size() == p1.size()

    // g2o_setup()
    g2o::SparseOptimizer optimizer;
    auto linear_solver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>> ();
    g2o::OptimizationAlgorithmLevenberg* algorithm = new
        g2o::OptimizationAlgorithmLevenberg( g2o::make_unique<g2o::BlockSolver_6_3>(
                    std::move(linear_solver)));
    optimizer.setAlgorithm( algorithm );

    // set intrinsic camera parameter
    Eigen::Vector2d pp(cx,cy);
    g2o::CameraParameters* camera = new g2o::CameraParameters(
            fx, pp, 0 );
    camera->setId(0);
    optimizer.addParameter( camera );

    // add_node()
    int idx = 0;
    Eigen::Isometry3d tx0;

    for(auto& p : poses){
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(idx);
        if( idx == 0 ){
            v->setFixed( true );  // fix the first node
            tx0 = p; // remember transformation to apply to landmark
        }
        // NOTE: vertices are set to inverse
        // because g2o requires that somehow.
        // probably makes computation more efficient.

        //v->setEstimate( g2o::SE3Quat( p.linear(), p.translation() ) );
        v->setEstimate(g2o::SE3Quat( p.linear(), p.translation() ).inverse());
        v->setMarginalized(false);
        optimizer.addVertex( v );

        // increment vertex count
        ++idx;
    }

    // setup simple alias
    const int pose_idx0 = 0;
    const int pose_idx1 = (idx-1);
    const int lmk_idx0 = idx;

    // get_depth() --> may not be necessary
    std::vector<float> zs;
    get_depth(poses.front(), poses.back(), p0, p1, K, zs);
    for(auto& z : zs){
        std::cout << z << ", ";
    }
    std::cout << std::endl;

    // set_landmark_estimates()
    // euclidean 3d landmark for feature points, based on img1
    for(size_t i = 0; i < n_l; ++i)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( lmk_idx0 + i );

        // set depth to 1 & and add euclidean point
        // TODO : apply first pose to this
        double pxl_x = p0[i].x;
        double pxl_y = p0[i].y;

        //double z = std::isfinite(d0.z[i]) ? d0.z[i] : 1.0; //5.0;
        //double z = 1.0;
        double z = zs[i];
        //z = std::isfinite(z)? z: 1.0; // rectify z
        double x = ( pxl_x - cx ) * z / fx;
        double y = ( pxl_y - cy ) * z / fy;

        v->setEstimate( tx0 * Eigen::Vector3d(x,y,z) );
        v->setMarginalized( true );
        optimizer.addVertex( v );
    }

    g2o::VertexSE3Expmap* v_init = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(pose_idx1) );
    Eigen::Isometry3d v_init_pose = v_init->estimate();
    std::cout << "Initial Pose1 = " << std::endl << v_init_pose.inverse().matrix() << std::endl;

    // set edges

    // first frame
    std::vector<g2o::EdgeProjectXYZ2UV*> edges;
    std::vector<g2o::EdgeSE3Expmap*> odom_edges;

    for(size_t i = 0; i < n_l; ++i)
    {
        double pxl_x = p0[i].x;
        double pxl_y = p0[i].y;

        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>
                (optimizer.vertex(lmk_idx0 + i)) ); // target point
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>
                (optimizer.vertex(pose_idx0)) ); // observation pose
        edge->setMeasurement( Eigen::Vector2d(pxl_x, pxl_y) );
        edge->setInformation(0.25*Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        auto ker = new g2o::RobustKernelHuber();
        ker->setDelta( 0.5 );
        edge->setRobustKernel(ker);

        optimizer.addEdge( edge );
        edges.push_back( edge );
    }

    // last frame
    for(size_t i = 0; i < n_l; ++i)
    {
        double pxl_x = p1[i].x;
        double pxl_y = p1[i].y;

        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>
                (optimizer.vertex(lmk_idx0+i)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>
                (optimizer.vertex(pose_idx1)) );
        edge->setMeasurement( Eigen::Vector2d(pxl_x, pxl_y) );
        edge->setInformation(0.25*Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);

        auto ker = new g2o::RobustKernelHuber();
        ker->setDelta( 0.5 );
        edge->setRobustKernel(ker);

        optimizer.addEdge( edge );
        edges.push_back( edge );
    }

    // TODO : do I need to add pose-based edges?

    Eigen::Matrix<double,6,1> odom_Hv;
    float spi2 = 1.0 / pow(0.1, 2);
    float sri2 = 1.0 / pow(1.0 * M_PI/180.0, 2); // <<<<<<<<<<<<<<<<<<<< THIS

    // vertices 
    
    //float spi2 = 1.0;
    //float sri2 = 1.0;

    odom_Hv << sri2, sri2, sri2, spi2, spi2, spi2;

#if 1
    for(size_t i=1; i < n_p; ++i){
        g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();

        Eigen::Matrix<double,6,6> odom_H = odom_Hv.asDiagonal();

        g2o::VertexSE3Expmap* v0 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i-1));
        g2o::VertexSE3Expmap* v1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));

        edge->setVertex(0, v0);
        edge->setVertex(1, v1);

        edge->setMeasurement( g2o::SE3Quat(
                    v1->estimate() * v0->estimate().inverse() ));
        edge->setInformation(odom_H);

        auto ker = new g2o::RobustKernelHuber();
        ker->setDelta(10.0);
        edge->setRobustKernel(ker);

        edge->computeError();
        double chi2 = edge->chi2();
        if ( std::isfinite(chi2) ){
            optimizer.addEdge( edge );
            odom_edges.push_back( edge );
        }else{
            // should not reach here
            std::cout << "degenerate odom chi" << std::endl;
            delete edge;
        }
    }
#endif

#if 0
    Eigen::Matrix<double,6,1> pose_Hv;
    float spi2_p = 1.0 / pow(10.0, 2);
    float sri2_p = 1.0 / pow(5.0 * M_PI/180.0, 2);
    pose_Hv << sri2_p, sri2_p, sri2_p, spi2_p, spi2_p, spi2_p;

    for(size_t i=1; i < n_p; ++i){
        g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
        Eigen::Matrix<double,6,6> pose_H = pose_Hv.asDiagonal();

        g2o::VertexSE3Expmap* v0 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::VertexSE3Expmap* v1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));

        edge->setVertex(0, v0);
        edge->setVertex(1, v1);

        edge->setMeasurement( g2o::SE3Quat(
                    v1->estimate() * v0->estimate().inverse() ));
        edge->setInformation(pose_H);

        auto ker = new g2o::RobustKernelHuber();
        ker->setDelta(10.0);
        edge->setRobustKernel(ker);

        edge->computeError();
        double chi2 = edge->chi2();
        if ( std::isfinite(chi2) ){
            optimizer.addEdge( edge );
            odom_edges.push_back( edge );
        }else{
            // should not reach here
            std::cout << "degenerate odom chi" << std::endl;
            delete edge;
        }
    }
#endif

    for(auto& e : edges){
        e->computeError();
        //std::cout << "ve0 : " << e->chi2() << std::endl;
    }
    for(auto& e : odom_edges){
        e->computeError();
        //std::cout << "oe0 : " << e->chi2() << std::endl;
    }

    // run optimization
    std::cout << "starting optimize" << std::endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    int res = optimizer.optimize(50);

    optimizer.computeActiveErrors();
    std::cout << "active-chi2" << optimizer.activeChi2() << std::endl;
    std::cout << "active-robust-chi2" << optimizer.activeRobustChi2() << std::endl;
    std::cout << "chi2" << optimizer.chi2() << std::endl;

    if(res <= 0) return false;
    //if(optimizer.activeRobustChi2() > 1000) return false; //TODO : tune

    std::cout << "finish optimize" << std::endl;

    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(pose_idx1) );
    Eigen::Isometry3d pose = v->estimate();
    std::cout << "Final Pose1 = " << std::endl << pose.inverse().matrix() << std::endl;

    opt_poses.clear();
    for(int i=pose_idx0; i<=pose_idx1; ++i){
        auto* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(i) );
        Eigen::Isometry3d tmp = v->estimate();
        Eigen::Isometry3d tmpi = v->estimate().inverse();

        if( i == pose_idx1 ){
            std::cout << "DBG " << std::endl;
            std::cout << tmpi.matrix() << std::endl;
        }

        //opt_poses.push_back(tmpi);
        opt_poses.emplace_back( tmpi );
    }

    for(auto& e : edges){
        e->computeError();
        std::cout << "ve1 : " << e->chi2() << std::endl;
    }
    for(auto& e : odom_edges){
        e->computeError();
        std::cout << "oe1 : " << e->chi2() << std::endl;
    }

    //std::cout << "inliers in total points: " <<
    //    inliers << "/" << m.size() << std::endl; // TODO : this number is actually different now!
    optimizer.save("/tmp/ba.g2o");

    // TODO : determine success criterion
    return 1;
}

bool loop_closure(
        const std::vector<Eigen::Isometry3d>& poses,
        //Eigen::Isometry3d& optimized_pose,
        std::vector<Eigen::Isometry3d>& opt_poses,
        const cv::Mat& img0,
        const cv::Mat& img1,
        const cv::Mat& K,

        // params to maybe use in the future
        Frame& d0,
        Frame& d1,
        bool proc
        ){

    if(proc){
        d0.kpt.clear();
        d1.kpt.clear();

        d0.img = img0;
        d1.img = img1;
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
    std::vector<cv::DMatch> m;
    if(! strong_match(img0, img1, K, d0, d1, m)){
        std::cout << "STRONG MATCH FAILED" << std::endl;
        return false;
    }

    std::cout << "Number of Poses : " << poses.size() << std::endl;
    std::cout << "Number of Matches : " << m.size() << std::endl;

    cv::Mat Kd;
    K.convertTo(Kd, CV_64F);
    std::vector<cv::Point_<double>> p0, p1;
    extract_points(m, d0.kpt, d1.kpt, p0, p1);

    return loop_closure(poses, p0, p1, Kd,
            opt_poses);
}

bool full_loop_closure(
        const std::vector<Frame>& kfs,
        const std::vector<FramePair>& obs,
        const cv::Mat& K,
        std::vector<Eigen::Isometry3d>& opt_poses
        ){

}
