#include "utils.hpp"

void get_depth(
        const Eigen::Isometry3d& pose0,
        const Eigen::Isometry3d& pose1,
        const std::vector<cv::KeyPoint>& kpt0,
        const std::vector<cv::KeyPoint>& kpt1,
        const cv::Mat& K,
        std::vector<float>& z){
    std::vector<cv::Point_<float>> pt0, pt1;
    pt0.reserve(kpt0.size());
    pt1.reserve(kpt1.size());
    std::transform(kpt0.begin(), kpt0.end(), std::back_inserter(pt0),
            [](cv::KeyPoint const& x){return x.pt;});
    std::transform(kpt1.begin(), kpt1.end(), std::back_inserter(pt1),
            [](cv::KeyPoint const& x){return x.pt;});

    get_depth<float>(pose0, pose1, pt0, pt1, K, z);
}

