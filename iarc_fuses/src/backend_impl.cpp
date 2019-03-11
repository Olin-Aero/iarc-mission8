#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

namespace iarc{
    struct Landmark{

    };

    struct Frame{
        cv::Mat img;
        std::vector<cv::KeyPoint> kpt;
        cv::Mat dsc;
        Eigen::Isometry3d pose;
    };

    class BackendImpl{
        void optimize(){

        }
    };

    /*
    struct Tracker{
    };
    */

    struct Matcher{
        cv::Mat K_;
        float lowe_ = 0.8;
        float maxd_ = 64.0;
        cv::FlannBasedMatcher matcher_;

        Matcher(cv::Mat K, float lowe, float maxd):
            K_(K), lowe_(lowe), maxd_(maxd),
            matcher_(cv::makePtr<cv::flann::LshIndexParams>(12,20,2)){
                //TODO : tune matcher params?
            }

        void filter(
                const std::vector<std::vector<cv::DMatch>>& m_knn,
                std::vector<cv::DMatch>& m
                ){
            m.clear();
            for(auto& m2 : m_knn){
                if(m2[0].distance >= maxd_) continue;
                if(m2[0].distance >= lowe_ * m2[1].distance) continue;
                m.push_back( m2[0] );
            }
        }

        void epifilter(
                const Frame& kf0, const Frame& kf1,
                const std::vector<cv::DMatch>& m_in,
                std::vector<cv::DMatch>& m_out
                ){

            std::vector<cv::Point2d> p0, p1;
            for(auto& m : m_in){
                p0.push_back( kf0.kpt[m.queryIdx].pt );
                p1.push_back( kf1.kpt[m.trainIdx].pt );
            }
            cv::Mat msk;
            cv::findEssentialMat(p0, p1, 
                    K_, cv::RANSAC, 0.999, 1.0, msk);

            m_out.clear();
            for(int i=0; i<msk.rows; ++i){
                if(!msk.at<char>(i)) continue;
                m_out.push_back(m_in[i]);
            }
        }

        void operator()(
                const Frame& kf0,
                const Frame& kf1,
                std::vector<cv::DMatch>& match,
                bool cross=false,
                bool epicheck=false
                ){

            std::vector<cv::DMatch> mbuf;

            std::vector<cv::DMatch>& match0 = (epicheck? mbuf : match);

            if(cross){
                // bidirectional search
                std::vector<cv::DMatch> m01, m10;
                this->operator()(kf0, kf1, m01, false, false);
                this->operator()(kf1, kf0, m10, false, false);

                // fill match
                match0.clear();
                for(auto& m_fw : m01){
                    bool found = false;
                    for(auto& m_bw : m10){
                        if(m_fw.queryIdx == m_bw.trainIdx &&
                                m_fw.trainIdx == m_bw.queryIdx){
                            found=true;
                            break;
                        }
                    }
                    if(found) match0.push_back(m_fw);
                }
            }else{
                // initial match
                std::vector<std::vector<cv::DMatch>> m_knn;
                matcher_.knnMatch(kf0.dsc, kf1.dsc, m_knn, 2);

                // lowe + maxd filter
                filter(m_knn, match0);
            }

            // filter by epipolar constraint
            if(epicheck){ 
                epifilter(kf0, kf1, mbuf, match);
            }
        }
    };

    class FrontendImpl{
        std::vector<Frame> kfs_;
        std::vector<Landmark> lmk_;
        std::vector< std::tuple<int,int,int,int> > obs_;

        Matcher match_;

        bool is_keyframe(Frame& kf1, std::vector<cv::DMatch>& m,
                float max_iou=0.33
                ){
            if( kfs_.size() <= 0) return true;
            const Frame& kf0 = kfs_.back();
            match_(kf0, kf1, m, true);
            int ixn = 2 * m.size();
            int uxn = kf0.kpt.size() + kf1.kpt.size() - ixn;
            float iou = float(ixn) / uxn;
            return iou < max_iou;
        }

        bool is_loopclosure();
        cv::Ptr<cv::ORB> orb;

        Frame extract_frame(
                const Eigen::Isometry3d& pose,
                const cv::Mat& img){
            Frame res;
            orb->detectAndCompute(img, cv::Mat(), res.kpt, res.dsc);
            res.img = img;
            res.pose = pose;

            return res;
        }

        void step(const Eigen::Isometry3d& pose, const cv::Mat& img){
            // extract frame + check if keyframe
            Frame kf1 = extract_frame(pose, img);
            std::vector<cv::DMatch> m0, m1;
            if(!is_keyframe(kf1, m0)) return;

            // insert keyframe
            kfs_.push_back(kf1); // std::move( kf1 ) ??
            size_t idx = kfs_.size()-1;

            // insert neighboring match constraint
            match_.epifilter(kfs_[idx-1], kfs_[idx], m0, m1); 
            for(auto& m: m1){
                obs_.emplace_back(idx-1, idx, m.queryIdx, m.trainIdx);
            }

            if( is_loopclosure() ){
                optimize();
            }

        }

        void optimize(){
            // convert_datatypes()

        }
    };
}
