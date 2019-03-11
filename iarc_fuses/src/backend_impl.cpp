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
