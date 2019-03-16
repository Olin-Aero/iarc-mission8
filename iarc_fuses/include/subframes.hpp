#ifndef __SUBFRAMES_HPP__
#define __SUBFRAMES_HPP__

struct Subframes{
    // keyframe data
    const Frame& kf0_; // reference keyframe
    const cv::Mat K_;
    const std::shared_ptr<Tracker> tracker_; // tracker handle

    std::vector<float> z_; // lmk depth

    // subframes data
    std::vector<size_t> idx_;
    std::vector<Frame> sfs_;
    std::vector<cv::Point2f> pt_;

    Subframes(const Frame& kf0, const cv::Mat& K, const std::shared_ptr<Tracker>& tracker)
        :kf0_(kf0),K_(K),tracker_(tracker){
            sfs_.push_back(kf0);

            // initialize tracking points
            for(auto& p : kf0_.kpt){pt_.push_back(p);}

            // initialize tracking indices
            for(size_t i=0; i<pt_.size(); ++i){
                idx_.push_back(i);
            }

            // initialize default depth to 1
            z_ = std::vector<float>(pt_.size(), 1.0);
        }

    void push_back(const Frame& sf){
        std::vector<cv::Point2f> sub_pt;
        std::vector<size_t> sub_idx;
        tracker_->track(sfs_.back().img, sf.img, pt_, 2.0,
                sub_pt, sub_idx);

        std::vector<size_t> new_idx;
        cv::Mat P1 = cv::Mat::eye(3, 4, CV_32F);
        cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);

        cv::eigen2cv(
                (kf0_.pose.inverse()*sfs_.back().pose).matrix(),
                T2); // T2 = T_b0>o^{-1} * T_b1>o = T_o>b0 * T_b1>o = T_b1>b0
        cv::Mat P2 = T2.rowRange(0,3).colRange(0,4);
        P2.convertTo(P2, CV_32F);

        for(int i=0; i<idx_.size(); ++i){
            int kf_idx = idx_[i]; // corresponding index referencing keyframe points

            bool trk_suc = (std::find(sub_idx.begin(), sub_idx.end(), i) != sub_idx.end());
            if (trk_suc){
                // the point continues to be tracked!
                new_idx.push_back(kf_idx);
            }else if (sfs_.size() >= 2){
                // tracking lost! triangulate with last known connected position.
                cv::Mat lmk(4, 1, CV_32F);
                //std::cout << "[P1]" << P1.size << "-" << P1.type() << std::endl;
                //std::cout << "[P2]" << P2.size << "-" << P2.type() << std::endl;
                //std::cout << P1.rows << ',' << P1.cols << ',' << P2.rows << ',' << P2.cols << std::endl;

                cv::triangulatePoints(P1, P2,
                        std::vector<cv::Point2f>({kf0_.kpt[kf_idx]}),
                        std::vector<cv::Point2f>({pt_[i]}), lmk);

                //std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
                //std::cout << P1 << std::endl;
                //std::cout << P2 << std::endl;
                //std::cout << kf0_.kpt[kf_idx] << std::endl;
                //std::cout << pt_[i] << std::endl;
                //std::cout << lmk << std::endl;
                //std::cout << P2 * lmk << std::endl;


                // acquire point depth
                z_[kf_idx] = lmk.at<float>(2) / lmk.at<float>(3);
                //std::cout << "[z]" << z_[kf_idx] << std::endl;
                if(z_[kf_idx] <= 0 || !std::isfinite(z_[kf_idx])){
                    z_[kf_idx] = 1.0;
                }
            }
        }

        // update data
        sfs_.push_back(sf);
        idx_.swap(new_idx);
        pt_.swap(sub_pt);
    }

    void finalize(){
        // TODO : consider structure-only BA
        if(sfs_.size() < 2) return;

        cv::Mat P1 = cv::Mat::eye(3, 4, CV_32F);
        cv::Mat T2 = cv::Mat::eye(4, 4, CV_32F);
        cv::eigen2cv(
                (kf0_.pose.inverse()*sfs_.back().pose).matrix(),
                T2);
        cv::Mat P2 = T2.rowRange(0,3).colRange(0,4);
        P2.convertTo(P2, CV_32F);

        for(auto& i : idx_){
            cv::Mat lmk(4, 1, CV_32F);
            cv::triangulatePoints(P1, P2,
                    std::vector<cv::Point2f>({kf0_.kpt[i]}),
                    std::vector<cv::Point2f>({pt_[i]}), lmk);

            z_[i] = lmk.at<float>(2) / lmk.at<float>(3);

            //std::cout << "tri " << z_[i] << std::endl;
            if( z_[i] <= 0 || !std::isfinite(z_[i])){
                //std::cout << kf0_.kpt[i] << ';' << pt_[i] << '|';
                //std::cout << lmk.at<float>(2) << ',' << lmk.at<float>(3) << std::endl;
                z_[i] = 1.0;
            }
        }

    }
};

#endif
