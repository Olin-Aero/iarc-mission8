/*

   Interpret Piped data stream from tlm-data-logger to ROS.

Usage : tlm-data-logger -r 20 inet:127.0.0.1:9060 | grep omniscient_bebop2.worldTemperature -B 23 | rosrun iarc_fuses sphinx_logger 


*/
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

void split(const std::string& s, char c,
        std::vector<std::string>& v) {
    std::string::size_type i = 0;
    std::string::size_type j = s.find(c);

    while (j != std::string::npos) {
        v.push_back(s.substr(i, j-i));
        i = ++j;
        j = s.find(c, j);

        if (j == std::string::npos)
            v.push_back(s.substr(i, s.length()));
    }
}

bool s_in(const std::string& s, const std::string& ss){
    return s.find(ss) != std::string::npos;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "sphinx_logger");
    ros::NodeHandle nh;

    tf2::BufferCore buf;
    tf2_ros::TransformListener tfl(buf);
    tf2_ros::TransformBroadcaster tfb;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("ground_truth", 10);

    bool init = false;
    ros::Rate r(10);
    tf2::Transform xfm0;
    while(ros::ok() && !init){
        try{
            geometry_msgs::TransformStamped xfm = buf.lookupTransform("odom", "base_link", ros::Time(0));
            //tf2::convert(xfm, xfm0);
            tf2::fromMsg(xfm.transform,  xfm0);
            init = true;
        }catch(const tf2::LookupException& e){
            std::cout << e.what() << std::endl;
        }catch(const tf2::ConnectivityException& e){
            std::cout << e.what() << std::endl;
        }catch(const tf2::ExtrapolationException& e){
            std::cout << e.what() << std::endl;
        }
        r.sleep();
    }

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now(); //??

    std::string str;
    int cnt=0;

    tf2::Quaternion q;
    float rx = 0.0;
    float ry = 0.0;
    float rz = 0.0;

    tf2::Transform ext_xfm, int_xfm;
    tf2::Quaternion ext_q, int_q;

    ext_q.setRPY(0, 0, -M_PI/2);
    ext_xfm.setOrigin(tf2::Vector3{0,0,0});
    ext_xfm.setRotation(ext_q);

    int_q.setRPY(0,0,0);
    int_xfm.setOrigin( tf2::Vector3{-0.05, -0.015, 0.05} );
    int_xfm.setRotation(int_q);

    geometry_msgs::TransformStamped xfm_tf;

    bool init1 = false;

    while(ros::ok()){
        //std::cin.get('-', 1024);//, str)
        std::getline(std::cin, str);
        //std::cout << '[' << cnt << ']' << ">>" << str<< std::endl;
        if (s_in(str, "--")){
            ++cnt;
        }else{
            std::vector<std::string> v;
            split(str, ':', v);
            if (v.size() != 2) continue;
            std::string& field = v[0];
            std::string& value = v[1];

            if(s_in(field, "omniscient_bebop2.worldPosition.x")) {
                msg.pose.position.x = std::stod(value);
            }else if(s_in(field, "omniscient_bebop2.worldPosition.y")){
                msg.pose.position.y = std::stod(value);
            }else if(s_in(field, "omniscient_bebop2.worldPosition.z")){
                msg.pose.position.z = std::stod(value);
            }else if(s_in(field, "omniscient_bebop2.worldAttitude.x")){
                rx = std::stod(value);
            }else if(s_in(field, "omniscient_bebop2.worldAttitude.y")){
                ry = std::stod(value);
            }else if(s_in(field, "omniscient_bebop2.worldAttitude.z")){
                rz = std::stod(value);
                q.setRPY(rx,ry,rz);
                //q.setEuler(rz,ry,rx);


                msg.pose.orientation = tf2::toMsg(q);

                tf2::Transform xfm;

                tf2::convert(msg.pose, xfm);

                //if(!init1){
                //    std::cout << tf2::transformToEigen(tf2::toMsg(xfm0)).matrix() << std::endl;
                //}
                if(!init1){
                    //std::cout << tf2::transformToEigen(tf2::toMsg(xfmr)).matrix() << std::endl;
                    //std::cout << tf2::transformToEigen(tf2::toMsg(xfm0)).matrix() << std::endl;
                    xfm0 = xfm0 * xfm.inverse();
                    init1= true;
                }
                xfm = xfm0 * xfm;// * int_xfm;
                //if(!init1){
                //    std::cout << tf2::transformToEigen(tf2::toMsg(xfm)).matrix() << std::endl;
                //}
                xfm.setRotation( xfm.getRotation().normalized() );
                //tf2::convert(xfm, msg.pose);
                tf2::toMsg(xfm, msg.pose);

                xfm_tf.transform = tf2::toMsg(xfm.inverse());
                xfm_tf.header.stamp = ros::Time::now();
                xfm_tf.header.frame_id = "base_link";
                xfm_tf.child_frame_id = "ground_truth";

                //tfb.sendTransform( msg.pose );
                if(cnt>0){
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);

                    tfb.sendTransform(xfm_tf);
                }
            }
        }
    }

    ros::shutdown();
}
