#ifndef __FRAME_HPP__
#define __FRAME_HPP__

#include <opencv2/core.hpp>
#include <vector>
#include <Eigen/Geometry>

struct Frame{
    cv::Mat img;
    std::vector<cv::Point2f> kpt;
    cv::Mat dsc;
    Eigen::Isometry3d pose;
    std::vector<float> z;
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


#endif
