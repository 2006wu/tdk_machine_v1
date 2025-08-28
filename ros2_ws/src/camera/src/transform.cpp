#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "camera/algorithm.hpp"
#include <visualization_msgs/msg/marker.hpp>

class Transform : public rclcpp::Node{
    public:
    Transform() : Node("transform"){
        head_frame = declare_parameter<std::string>("head_frame","lift");
        buf = std::make_unique<tf2_ros::Buffer>(get_clock());
        lis = std::make_shared<tf2_ros::TransformListener>(*buf);
        sub = create_subscription<geometry_msgs::msg::PointStamped>("point_in",10,std::bind(&Transform::call_back,this,_1));
        pub = create_publisher<geometry_msgs::msg::PointStamped>("point_out",10);

        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker",10);
        timer = this->create_wall_timer(100ms,std::bind(&Transform::timer_callback,this));
    }
    private:
        void call_back(const geometry_msgs::msg::PointStamped::SharedPtr msg){
            auto tf = buf->lookupTransform(head_frame,msg->header.frame_id,msg->header.stamp,rclcpp::Duration::from_seconds(0.2));
            tf2::doTransform(*msg,output,tf);
            output.header.frame_id = head_frame;
            pub->publish(output);
            std::cout << "output x: " << output.point.x
            << " y: " << output.point.y
            << " z: " << output.point.z
            << std::endl;
            tri = true;
        }
        void timer_callback(){
            if(tri){
                m.header.frame_id = head_frame;           // 例如 "arm_base"
                m.header.stamp = this->now();
                m.ns = "target_point";
                m.id = 0;                            // (ns,id) 決定唯一性；同一對會被覆蓋更新
                m.type = visualization_msgs::msg::Marker::SPHERE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = output.point.x;
                m.pose.position.y = output.point.y;
                m.pose.position.z = output.point.z;
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = m.scale.z = 0.05;
                m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 255.0;
                m.lifetime = rclcpp::Duration::from_seconds(0.0); // ★ 永久
                marker_pub->publish(m);
            }
        }
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub;
        std::unique_ptr<tf2_ros::Buffer> buf;
        std::shared_ptr<tf2_ros::TransformListener> lis;
        std::string head_frame;
        geometry_msgs::msg::PointStamped output;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        rclcpp::TimerBase::SharedPtr timer;
        visualization_msgs::msg::Marker m;
        bool tri = false;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transform>());
  rclcpp::shutdown();
  return 0;
}