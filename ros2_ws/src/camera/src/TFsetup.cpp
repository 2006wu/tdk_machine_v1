#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Setup:public rclcpp::Node{
    public:
        Setup():Node("setup"){
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
            
            place.x = 0.0;
            place.y = 0.0;
            place.z = 0.0;
            // C_x = this->declare_parameter<double>("C_x",1.0);
            this->publish_static();

            car_place_sub = this->create_subscription<geometry_msgs::msg::Point>("place",10,std::bind(&Setup::car_place_callback,this,_1));
            A1_height_sub = this->create_subscription<std_msgs::msg::Float64>("height",10,std::bind(&Setup::A1_height_callback,this,_1));
            A1_angle_sub = this->create_subscription<std_msgs::msg::Float64>("angle1",10,std::bind(&Setup::A1_angle_callback,this,_1));
            A2_angle_sub = this->create_subscription<std_msgs::msg::Float64>("angle2",10,std::bind(&Setup::A2_angle_callback,this,_1));
            timer = this->create_wall_timer(100ms,std::bind(&Setup::publish_dynamic,this));
        }
    private:
        void publish_static(){
            geometry_msgs::msg::TransformStamped t;
            tf2::Quaternion q;
            
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "car";
            t.child_frame_id = "lift";
            t.transform.translation.x = 0;
            t.transform.translation.y = 0.5;
            t.transform.translation.z = 0;
            t.transform.rotation.w = 1;
            static_tf_broadcaster->sendTransform(t);

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "A1";
            t.child_frame_id = "camera";
            t.transform.translation.x = 0;
            t.transform.translation.y = 0.2;
            t.transform.translation.z = 0;
            q.setRPY(0,0,M_PI/2.0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            static_tf_broadcaster->sendTransform(t);

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "camera";
            t.child_frame_id  = "OpenCV";
            t.transform.translation.x = t.transform.translation.y = t.transform.translation.z = 0;
            q.setRPY(-M_PI/2.0,0,-M_PI/2.0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            static_tf_broadcaster->sendTransform(t);
        }
        void publish_dynamic(){
            geometry_msgs::msg::TransformStamped t;
            tf2::Quaternion q;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "car";
            t.transform.translation.x = place.x;
            t.transform.translation.y = place.y;
            t.transform.translation.z = place.z;
            t.transform.rotation.w = 1;
            tf_broadcaster->sendTransform(t);

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "lift";
            t.child_frame_id = "A1";
            t.transform.translation.x = 0;
            t.transform.translation.y = 0;
            t.transform.translation.z = height;
            q.setRPY(angle1,0,0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(t);

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "A1";
            t.child_frame_id = "A2";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.5;
            t.transform.translation.z = 0.0;
            q.setRPY(angle2,0,0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(t);
        }
        void car_place_callback(const geometry_msgs::msg::Point msg){
            place.x = msg.x;
            place.y = msg.y;
            place.z = msg.z;
        }
        void A1_height_callback(const std_msgs::msg::Float64 msg){
            height = msg.data;
        }
        void A1_angle_callback(const std_msgs::msg::Float64 msg){
            angle1 = msg.data;
        }
        void A2_angle_callback(const std_msgs::msg::Float64 msg){
            angle2 = msg.data;
        }

        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr car_place_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr A1_height_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr A1_angle_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr A2_angle_sub;
        rclcpp::TimerBase::SharedPtr timer;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

        double angle1 = 0.0;
        double angle2 = 0.0;
        double height = 0.5;
        geometry_msgs::msg::Point place;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Setup>());
    rclcpp::shutdown();
    return 0;
}
//map - car - lift - A1 - A2 - camera
//    d     s      d    d    s 