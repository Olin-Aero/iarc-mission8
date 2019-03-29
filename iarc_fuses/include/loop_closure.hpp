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
        size_t n_l = pt.size();
        for(size_t i=0; i<n_l; ++i)
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
            optimizer.addVertex( v );
        }
    }

    void add_edges(){

    }

    bool solve(
            const std::vector<Eigen::Isometry3d>& poses,
            const std::vector<cv::Point_<double>>& p0,
            const std::vector<cv::Point_<double>>& p1,

            std::vector<Eigen::Isometry3d>& opt_poses
            ){
        this->add_camera();
        Eigen::Isometry3d tx0;
        this->add_poses(poses, tx0);
        std::Vector<float> zs;
        this->add_points(p0, zs, tx0, poses.size());
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
