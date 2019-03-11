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

struct Frame{
    cv::Mat img;
    std::vector<cv::Point2f> kpt;
    cv::Mat dsc;
    Eigen::Isometry3d pose;
    // no_align_bullshit here needed?
};

struct FramePair{
    // References
    Frame& kf0_;
    Frame& kf1_;
    std::vector<cv::DMatch>& m_;

    // Data Cache
    std::vector<cv::Point2f> kpt0, kpt1;

    FramePair(Frame& kf0, Frame& kf1, std::vector<cv::DMatch>& m):
        kf0_(kf0), kf1_(kf0), m_(m){

            for(auto& match : m_){
                kpt0.push_back( kf0.kpt[match.queryIdx]);
                kpt1.push_back( kf1.kpt[match.trainIdx]);
            }

        }
};

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
        std::vector<std::pair<int, int>>& m
        );

#endif
