#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>
#include <vector>
#include <iostream>

struct Tracker{
    Tracker(){
    }

    void track(
            // input
            const cv::Mat& img0,
            const cv::Mat& img1,
            const std::vector<cv::Point2f>& pt0,
            // std::vector<size_t>& idx0,

            // params
            float max_err,

            // output
            std::vector<cv::Point2f>& pt1,
            std::vector<size_t>& idx1
            ){
        // TODO : optimize

        cv::Mat gray0, gray1;
        cv::cvtColor(img0, gray0, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);

        std::vector<uint8_t> st01, st10;
        std::vector<float> err01, err10;
        std::vector<cv::Point2f> pt0r;

        // 0 -> 1
        cv::calcOpticalFlowPyrLK(img0, img1, pt0, pt1, st01, err01);
        // 1 -> 0
        cv::calcOpticalFlowPyrLK(img1, img0, pt1, pt0r, st10, err10);

        // bidirectional filter
        std::vector<cv::Point2f> tmp;
        idx1.clear();
        for(size_t i=0; i < pt0.size(); ++i){
            if(st01[i]==0 || st10[i] ==0) continue;
            float err = cv::norm(pt0r[i] - pt0[i]);
            if(err > max_err) continue;
            idx1.push_back( i );
            tmp.push_back( pt1[i] );
        }
        pt1.swap(tmp);
    };

};

#endif
