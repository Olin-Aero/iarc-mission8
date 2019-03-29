#ifndef __IARC_LOOP_CLOSURE_HPP__
#define __IARC_LOOP_CLOSURE_HPP__

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

#include "frame.hpp"

class LoopClosureSolver{
    // camera intrinsic parameters
    cv::Mat K_;
    double fx_, fy_, cx_, cy_;

    // information matrix
    Eigen::Matrix<double,6,6> odom_H_;

    // handles
    g2o::SparseOptimizer optimizer_;

    LoopClosureSolver(const cv::Mat& K):K_(K){
        // parameter setup
        cv::Mat Kd;
        K_.convertTo(Kd, CV_64F);
        fx_ = Kd.at<double>(0,0);
        fy_ = Kd.at<double>(1,1);
        cx_ = Kd.at<double>(0,2);
        cy_ = Kd.at<double>(1,2);

        Eigen::Matrix<double,6,1> odom_Hv;
        float spi2 = 1.0 / pow(0.1, 2);
        float sri2 = 1.0 / pow(1.0 * M_PI/180.0, 2); // <<<<<<<<<<<<<<<<<<<< THIS
        odom_Hv << sri2, sri2, sri2, spi2, spi2, spi2;
        odom_H_ = odom_Hv.asDiagonal();


        // g2o setup
        auto linear_solver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
        auto algorithm = new g2o::OptimizationAlgorithmLevenberg( g2o::make_unique<g2o::BlockSolver_6_3>(
                    std::move(linear_solver)));
        optimizer_.setAlgorithm(algorithm);
    }

    void add_camera(){
        // TODO(jamie) : avoid doing this every single time
        Eigen::Vector2d pp(cx_, cy_);
        g2o::CameraParameters* camera = new g2o::CameraParameters(
                fx_, pp, 0);
        camera->setId(0);
        optimizer_.addParameter( camera );
    }

    void add_poses(
            const std::vector<Eigen::Isometry3d>& poses,
            Eigen::Isometry3d& tx0
            ){

        int idx = 0;
        for(auto& p : poses){
            g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
            v->setId(idx);
            if( idx == 0 ){
                v->setFixed( true );  // fix the first node
                tx0 = p; // remember transformation to apply to landmark
            }

            // NOTE: vertices are set to inverse
            // because g2o requires pose as [global->local] transform.
            v->setEstimate(g2o::SE3Quat( p.linear(), p.translation() ).inverse());
            v->setMarginalized(false);
            optimizer_.addVertex( v );

            // increment vertex count
            ++idx;
        }
    }

    void add_points(
            const std::vector<cv::Point_<double>>& pt,
            const std::vector<float> zs,
            const Eigen::Isometry3d& tx0,
            const int idx0
            ){
        // set_landmark_estimates()
        // euclidean 3d landmark for feature points, based on img1
        int n_l = pt.size();
        for(int i=0; i<n_l; ++i)
        {
            g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
            v->setId(idx0+i);

            // set depth to 1 & and add euclidean point
            // TODO : apply first pose to this
            double pxl_x = pt[i].x;
            double pxl_y = pt[i].y;

            double z = (zs.size()>i)? zs[i] : 1.0;
            z = (std::isfinite(z))? z : 1.0; // rectify z

            // unproject
            double x = ( pxl_x - cx_ ) * z / fx_;
            double y = ( pxl_y - cy_ ) * z / fy_;

            v->setEstimate( tx0 * Eigen::Vector3d(x,y,z) );
            v->setMarginalized( true );
            optimizer_.addVertex( v );
        }
    }

    void add_observations(
            const std::vector<cv::Point_<double>>& pt,
            const int lmk_idx0,
            const int pose_idx
            ){

        int n_l = pt.size();
        for(int i = 0; i < n_l; ++i)
        {
            double pxl_x = pt[i].x;
            double pxl_y = pt[i].y;

            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>
                    (optimizer_.vertex(lmk_idx0 + i)) ); // target point
            edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>
                    (optimizer_.vertex(pose_idx)) ); // observation pose
            edge->setMeasurement( Eigen::Vector2d(pxl_x, pxl_y) );
            edge->setInformation(0.25*Eigen::Matrix2d::Identity() );
            edge->setParameterId(0, 0);

            auto ker = new g2o::RobustKernelHuber();
            ker->setDelta( 0.5 );
            edge->setRobustKernel(ker);

            optimizer_.addEdge( edge );
        }
    }

    void add_odometry(
            const std::vector<Eigen::Isometry3d>& poses
            ){

        int n_p = poses.size();

        for(int i=1; i < n_p; ++i){
            g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();

            g2o::VertexSE3Expmap* v0 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(i-1));
            g2o::VertexSE3Expmap* v1 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer_.vertex(i));

            edge->setVertex(0, v0);
            edge->setVertex(1, v1);

            edge->setMeasurement( g2o::SE3Quat(
                        v1->estimate() * v0->estimate().inverse() ));
            edge->setInformation(odom_H_);

            auto ker = new g2o::RobustKernelHuber();
            ker->setDelta(10.0);
            edge->setRobustKernel(ker);

            edge->computeError();
            double chi2 = edge->chi2();
            if ( std::isfinite(chi2) ){
                optimizer_.addEdge( edge );
            }else{
                // should not reach here
                std::cout << "degenerate odom chi" << std::endl;
                delete edge;
            }
        }

    }

    bool optimize(
            const int pose_idx0,
            const int pose_idx1,
            std::vector<Eigen::Isometry3d>& opt_poses,
            bool verbose
            ){
        // run optimization
        optimizer_.setVerbose(verbose);
        optimizer_.initializeOptimization();
        int res = optimizer_.optimize(50);

        if(res <= 0){
            std::cerr << "Optimization Failed : " << res << std::endl;
            return false;
        }

        //TODO : consider loop closure failure conditions
        // if(optimizer_.activeRobustChi2() > 1000) return false;
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer_.vertex(pose_idx1) );

        opt_poses.clear();
        for(int i=pose_idx0; i<=pose_idx1; ++i){
            auto* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer_.vertex(i) );
            Eigen::Isometry3d pose = v->estimate().inverse();
            opt_poses.emplace_back( pose );
        }

        return true;

        /* // if(verbose)
        for(auto& e : edges){
            e->computeError();
            std::cout << "ve1 : " << e->chi2() << std::endl;
        }
        for(auto& e : odom_edges){
            e->computeError();
            std::cout << "oe1 : " << e->chi2() << std::endl;
        }
        */

        //optimizer_.save("/tmp/ba.g2o");
    }

    bool compute(
            const std::vector<Eigen::Isometry3d>& poses,
            const std::vector<cv::Point_<double>>& p0,
            const std::vector<cv::Point_<double>>& p1,

            std::vector<Eigen::Isometry3d>& opt_poses
            ){
        const int pose_idx0 = 0;
        const int pose_idx1 = poses.size() - 1;
        const int lmk_idx0  = poses.size();

        this->add_camera();
        Eigen::Isometry3d tx0;
        this->add_poses(poses, tx0);
        std::vector<float> zs;
        this->add_points(p0, zs, tx0, lmk_idx0);
        this->add_odometry(poses);
        this->add_observations(p0, poses.size(), pose_idx0);
        this->add_observations(p1, poses.size(), pose_idx1);
        return this->optimize(pose_idx0, pose_idx1, opt_poses, false);
    }
};

bool loop_closure(
        const std::vector<Eigen::Isometry3d>& poses,
        const std::vector<cv::Point_<double>>& p0,
        const std::vector<cv::Point_<double>>& p1,
        const cv::Mat& K,
        std::vector<Eigen::Isometry3d>& opt_poses
        );

bool loop_closure(
        const std::vector<Eigen::Isometry3d>& poses,
        std::vector<Eigen::Isometry3d>& opt_poses,
        const cv::Mat& img0,
        const cv::Mat& img1,
        const cv::Mat& K,

        // params to maybe use in the future
        Frame& d0,
        Frame& d1,
        bool proc=true
        );

float jaccard(
        Frame& d0,
        Frame& d1,
        int n
        );

#endif
