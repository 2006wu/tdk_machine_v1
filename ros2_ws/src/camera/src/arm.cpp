#include <cmath>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "camera/algorithm.hpp"

class Arm:public rclcpp::Node{
    public:
        Arm():Node("arm"){
            x_ref = declare_parameter<double>("x_ref", 0.2);
            y_ref = declare_parameter<double>("y_ref", 0.0);

            pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("point_out",10,std::bind(&Arm::plan,this,_1));
            plan_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("arm_result",10);
        }
    private:
        void plan(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            if (msg->header.frame_id != "lift"){
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Expect frame_id=lift, got '%s'", msg->header.frame_id.c_str());
            }
            cmd.x = -(msg->pose.position.x - x_ref);
            cmd.y = -(msg->pose.position.y - y_ref);
            cmd.theta = 0.0;
            std::cout << "ex: " << cmd.x << " ey: " << cmd.y << std::endl;
            plan_pub->publish(cmd);
        }
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr plan_pub;
        geometry_msgs::msg::Pose2D cmd;
        double x_ref,y_ref;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Arm>());
    rclcpp::shutdown();
    return 0;
}