#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "loop_closure.hpp"

Eigen::Quaterniond euler2Quaternion(
        const double roll,
        const double pitch,
        const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}


void gen_lmk(
        const Eigen::Isometry3d& src0,
        const Eigen::Isometry3d& src1,
        const Eigen::Matrix3d& K,
        const int w,
        const int h,
        std::vector< cv::Point2d >& p0s,
        std::vector< cv::Point2d >& p1s,
        std::vector< Eigen::Vector3d >& lmks,
        size_t n,
        size_t max_it=1000
        ){

    Eigen::Isometry3d T0i = src0.inverse(); // map -> cam
    Eigen::Isometry3d T1i = src1.inverse(); // map -> cam
    Eigen::Matrix3d Ki = K.inverse();

    lmks.clear();
    for(size_t i=0; i<max_it; ++i){
        Eigen::Vector3d pt0_3{0,0,1};
        pt0_3.block<2,1>(0,0) = Eigen::Vector2d{w/2,h/2} + Eigen::Vector2d{w/2,h/2}.cwiseProduct( Eigen::Vector2d::Random(2));
        //std::cout << "pt : " << pt0_3.transpose() << std::endl;
        Eigen::Vector3d lmk = src0 * ((10.0 * std::rand() / float(RAND_MAX)) * (Ki * pt0_3));
        //std::cout << "lmk : " << lmk.transpose() << std::endl;
        //auto lmk = 5.0 * Eigen::Vector3d::Random(3,1);

        Eigen::Vector3d p0_3 = K * (T0i * lmk);
        Eigen::Vector3d p1_3 = K * (T1i * lmk);
        //std::cout << p0_3.transpose() << std::endl;
        //std::cout << '\t' << p0_3(2) << std::endl;

        if(p0_3(2)<0 || p1_3(2)<0) continue;

        int x0 = p0_3(0) / p0_3(2);
        int y0 = p0_3(1) / p0_3(2);
        int x1 = p1_3(0) / p1_3(2);
        int y1 = p1_3(1) / p1_3(2);

        if(
                x0<0 || x0>=w ||
                y0<0 || y0>=h ||
                x1<0 || x1>=w ||
                y1<0 || y1>=h) continue;

        p0s.emplace_back(x0,y0);
        p1s.emplace_back(x1,y1);
        lmks.push_back(lmk);

        if(lmks.size() >= n){
            break;
        }
    }
}

int main(){
    std::vector<Eigen::Isometry3d> ground_truth;
    std::vector<Eigen::Isometry3d> odometry;
    const float deg = (M_PI / 180.0);

    bool init = false;
    for(float h=0; h<2*M_PI - 30*deg; h += 2*deg){
        //std::cout << h << std::endl;

        float x = cos(h);
        float y = sin(h); // position
        float z = 0.0;
        float rz = h + (M_PI/2); // heading

        Eigen::Isometry3d xfm = Eigen::Isometry3d::Identity();
        xfm.translation() = Eigen::Vector3d({x,y,z});
        xfm.linear() = 
                Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        ground_truth.push_back( xfm );
        if(!init){
            odometry.push_back( xfm );
            init=true;
        }else{
            size_t n = ground_truth.size();
            auto dxfm = ground_truth[n-1] * ground_truth[n-2].inverse();
            //std::cout << dxfm.matrix() << std::endl;
            
            auto dt = 0.01 * Eigen::Vector3d::Random();
            auto dq = 0.01 * Eigen::Vector3d::Random(); //??

            dxfm.rotate(
                    Eigen::AngleAxisd(dq(0), Eigen::Vector3d::UnitZ()));
            dxfm.translate(dt);
            //std::cout << dxfm.matrix() << std::endl;

            odometry.push_back(
                    dxfm * odometry.back()
                    );
        }
    }

    // convert to base_link camera coordinates
    // Pose = T_b2o; Pose_c = T_c2o = T_c2b * T_b2o

    // camera coord --> base_link coord
    auto q = euler2Quaternion(-M_PI/2.0, 0.0, -M_PI/2.0).toRotationMatrix();
    //std::cout << q * Eigen::Vector3d{1,0,0} << std::endl;
    //std::cout << q * Eigen::Vector3d{0,1,0} << std::endl;
    //std::cout << q * Eigen::Vector3d{0,0,1} << std::endl;
    
    //Eigen::Quaterniond q(xfm.linear());
    //std::cout << xfm.linear() << std::endl;
    //std::cout << q.coeffs() << std::endl;

    for(auto& x : ground_truth){
        x.rotate(q);
        //x.linear() = q * x.linear();
    }
    for(auto& x : odometry){
        //x.linear() = q * x.linear();
        x.rotate(q);
    }

    Eigen::Matrix3d K;
    K << 500, 0, 320,
      0, 500, 240,
      0, 0, 1;
    cv::Mat K_cv;
    cv::eigen2cv(K, K_cv);

    std::vector<cv::Point2d> p0s, p1s;
    std::vector<Eigen::Vector3d> lmks;

    gen_lmk(
            ground_truth.front(), ground_truth.back(),
            K,
            640, 480,
            p0s, p1s, lmks,
            128,
            16384); 

    std::vector<Eigen::Isometry3d> lc_result;

    std::cout << "GROUND TRUTH" << std::endl;
    std::cout << ground_truth.back().matrix() << std::endl;

    std::cout << "ODOMETRY" << std::endl;
    std::cout << odometry.back().matrix() << std::endl;

    loop_closure(
            odometry,
            p0s, p1s,
            K_cv,
            lc_result);

    std::cout << "?? " << std::endl;
    std::cout << ground_truth.back().linear().eulerAngles(2,1,0).transpose() << std::endl;
    std::cout << lc_result.back().linear().eulerAngles(2,1,0).transpose() << std::endl;

    std::ofstream gtf("/tmp/gt.txt");
    //std::cout << "GROUND TRUTH" << std::endl;
    for(auto& x : ground_truth){
        //std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
        gtf << x.translation().transpose() << ' ' << x.linear().eulerAngles(2,1,0).transpose() << std::endl;
    }

    std::ofstream odf("/tmp/od.txt");
    //std::cout << "ODOMETRY" << std::endl;
    for(auto& x : odometry){
        //std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
        odf << x.translation().transpose() << ' ' << x.linear().eulerAngles(2,1,0).transpose() << std::endl;
    }

    std::ofstream lcf("/tmp/lc.txt");
    //std::cout << "ODOMETRY" << std::endl;
    for(auto& x : lc_result){
        //std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
        lcf << x.translation().transpose() << ' ' << x.linear().eulerAngles(2,1,0).transpose() << std::endl;
    }


#if 0
    cv::Mat rmat0, rmat1;
    cv::Mat tvec0, tvec1; 
    cv::Mat D = cv::Mat::zeros(5, 1, CV_32F);

    std::vector<cv::Point2f> p0s_r, p1s_r;
    cv::eigen2cv( Eigen::Matrix3d(ground_truth.front().inverse().linear().matrix()), rmat0);
    cv::eigen2cv( Eigen::Vector3d(ground_truth.front().inverse().translation().matrix()), tvec0);

    cv::eigen2cv( Eigen::Matrix3d(ground_truth.back().inverse().linear().matrix()), rmat1);
    cv::eigen2cv( Eigen::Vector3d(ground_truth.back().inverse().translation().matrix()), tvec1);

    cv::Mat lmks_cv(lmks.size(), 3, CV_32F);
    for(int i=0; i<lmks.size(); ++i){
        lmks_cv.at<float>(i, 0) = lmks[i](0);
        lmks_cv.at<float>(i, 1) = lmks[i](1);
        lmks_cv.at<float>(i, 2) = lmks[i](2);
    }
    cv::projectPoints(lmks_cv, rmat0, tvec0, K_cv, D, p0s_r); 
    cv::projectPoints(lmks_cv, rmat1, tvec1, K_cv, D, p1s_r); 


    cv::Mat P0, P1;
    cv::eigen2cv(Eigen::Matrix<double, 3, 4>(ground_truth.front().inverse().matrix().block<3,4>(0,0)), P0);

    cv::eigen2cv(Eigen::Matrix<double, 3, 4>(ground_truth.back().inverse().matrix().block<3,4>(0,0)), P1);

    cv::Mat lmks_r(4, lmks.size(), CV_32F);
    cv::triangulatePoints(K_cv*P0, K_cv*P1, p0s, p1s, lmks_r);

    for(int i=0; i<lmks.size(); ++i){
        std::cout << p0s[i] << '\t';
        std::cout << p0s_r[i] << ";";
        std::cout << p1s[i] << '\t';
        std::cout << p1s_r[i] << ";";

        std::cout << lmks[i] << '\t';
        std::cout << lmks_r.col(i).rowRange(0,3) / lmks_r.at<float>(3,i) << '\t';
        std::cout << "++++++++" << std::endl;
    }

        // cross validation

    std::ofstream gtf("/tmp/gt.txt");
    //std::cout << "GROUND TRUTH" << std::endl;
    for(auto& x : ground_truth){
        //std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
        gtf << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
    }

    std::ofstream odf("/tmp/od.txt");
    //std::cout << "ODOMETRY" << std::endl;
    for(auto& x : odometry){
        //std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
        odf << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2).transpose() << std::endl;
    }
#endif

    return 0;
}
