#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/sfm.hpp>

template<typename Ti, typename To=Ti>
void get_depth(
        const Eigen::Isometry3d& pose0,
        const Eigen::Isometry3d& pose1,
        const std::vector<cv::Point_<Ti>>& pt0,
        const std::vector<cv::Point_<Ti>>& pt1,
        const cv::Mat& K,
        std::vector<To>& z){
    cv::Mat P0 = cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F);

    cv::Mat Kd;
    K.convertTo(Kd, CV_64F);

    cv::eigen2cv(
            Eigen::Matrix<double,3,4>( (pose1.inverse()*pose0).matrix().block<3,4>(0,0) ),
            P1); // T2 = T_b0>o^{-1} * T_b1>o = T_o>b0 * T_b1>o = T_b1>b0

    //P1.convertTo(P1, CV_64F);

    //std::cout << P1 << std::endl;
    cv::Mat lmk(4, pt0.size(), CV_64F);
    //cv::sfm::triangulatePoints(
    //        std::vector<std::vector<cv::Point_<Ti>>>{{pt0, pt1}},
    //        std::vector<std::vector<cv::Point_<Ti>>>{{P0,P1}},
    //        lmk);
    cv::triangulatePoints(Kd*P0, Kd*P1, pt0, pt1, lmk);

    //cv::Mat lmk3;
    //cv::convertPointsFromHomogeneous(cv::Mat(lmk.t()).reshape(4,1), lmk3);
    //std::cout << "ls : " << lmk3.size << ", lc : " << lmk3.channels();
    //cv::Mat D(5, 1, cv::DataType<double>::type);

    //cv::Mat rvec1, tvec1, dbg_pt1;
    //cv::Rodrigues(P1.colRange(0,3), rvec1);
    //tvec1 = P1.colRange(3,4);

    //cv::Mat rvec2, tvec2, dbg_pt2;
    //cv::Rodrigues(P1.colRange(0,3).t(), rvec2);
    //tvec2 = -P1.colRange(0,3).t() * P1.colRange(3,4);

    //cv::projectPoints(lmk3, rvec1, tvec1, K, D, dbg_pt1
    //        );
    //cv::projectPoints(lmk3, rvec2, tvec2, K, D, dbg_pt2
    //        );

    //for(size_t i=0; i<pt0.size(); ++i){
    //    std::cout << pt1[i] << "," << dbg_pt1.row(i) << std::endl;
    //}

    z.clear();
    z.reserve( pt0.size() );

    for(size_t i=0; i<pt0.size(); ++i){
        z.push_back(
                lmk.at<double>(2, i) / lmk.at<double>(3, i)
                );
    }
}

void get_depth(
        const Eigen::Isometry3d& pose0,
        const Eigen::Isometry3d& pose1,
        const std::vector<cv::KeyPoint>& kpt0,
        const std::vector<cv::KeyPoint>& kpt1,
        const cv::Mat& K,
        std::vector<float>& z);

template<typename T>
void extract_points(
        const std::vector<cv::DMatch> ms,
        const std::vector<cv::KeyPoint>& kpt0,
        const std::vector<cv::KeyPoint>& kpt1,
        std::vector<cv::Point_<T>>& pt0,
        std::vector<cv::Point_<T>>& pt1
        ){
    size_t n = ms.size();
    pt0.clear(); pt1.clear();
    pt0.reserve( n );
    pt1.reserve( n );
    for(auto& m : ms){
        pt0.push_back(kpt0[m.trainIdx].pt );
        pt1.push_back(kpt1[m.queryIdx].pt );
    }
}

template<typename Ti, typename To=Ti>
void extract_points(
        const std::vector<cv::DMatch> ms,
        const std::vector<cv::Point_<Ti>>& kpt0,
        const std::vector<cv::Point_<Ti>>& kpt1,
        std::vector<cv::Point_<To>>& pt0,
        std::vector<cv::Point_<To>>& pt1
        ){
    size_t n = ms.size();
    pt0.clear(); pt1.clear();
    pt0.reserve( n );
    pt1.reserve( n );
    for(auto& m : ms){
        pt0.push_back(kpt0[m.trainIdx]);
        pt1.push_back(kpt1[m.queryIdx]);
    }
}
#endif
