#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
//#include "loop_closure.hpp"

int main(){
    std::vector<Eigen::Isometry3d> ground_truth;
    std::vector<Eigen::Isometry3d> odometry;

    bool init = false;
    for(float h=0; h<2*M_PI; h += 2 * M_PI / 180.0){
        //std::cout << h << std::endl;

        float x = cos(h);
        float y = sin(h); // position
        float z = 0.0;
        float rz = h + (M_PI/2); // heading

        Eigen::Isometry3d xfm = Eigen::Isometry3d::Identity();
        xfm.translation() = Eigen::Vector3d({x,y,z});
        xfm.linear() = 
                Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        //std::cout << "xfm" << std::endl << xfm.matrix() << std::endl;

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

    std::ofstream gtf("/tmp/gt.txt");
    std::cout << "GROUND TRUTH" << std::endl;
    for(auto& x : ground_truth){
        std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2)[2] << std::endl;
        gtf << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2)[2] << std::endl;
    }

    std::ofstream odf("/tmp/od.txt");
    std::cout << "ODOMETRY" << std::endl;
    for(auto& x : odometry){
        std::cout << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2)[2] << std::endl;
        odf << x.translation().transpose() << ' ' << x.linear().eulerAngles(0,1,2)[2] << std::endl;
    }

    return 0;
}
