#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "camera/algorithm.hpp"

class Imread:public rclcpp::Node{
    public:
        Imread():Node("read"){
            img_sub = this->create_subscription<sensor_msgs::msg::Image>("/chiang/chiang/color/image_raw",10,std::bind(&Imread::image_callback,this,_1));
            tri_sub = this->create_subscription<std_msgs::msg::Int32>("mode",10,std::bind(&Imread::trigger,this,_1));
            cof1_pub = this->create_publisher<sensor_msgs::msg::Image>("coffee1_image",10);
            cof2_pub = this->create_publisher<sensor_msgs::msg::Image>("coffee2_image",10);
            ora_pub = this->create_publisher<sensor_msgs::msg::Image>("orange_image",10);
        }
    private:
        void trigger(std_msgs::msg::Int32 order){
            if(order.data == 1){
                trig = 1;
            }else if(order.data == 2){
                trig = 2;
            }else if(order.data == 3){
                trig = 3;
            }
        }
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg){
            input = cv_bridge::toCvCopy(msg,"bgr8")->image;
            // cv::imshow("Live",input);
            // cv::waitKey(1);

            if(trig == 1){
                auto msg = cv_bridge::CvImage(header,"bgr8",input).toImageMsg();
                cof1_pub->publish(*msg);
                trig = 0;
            }else if(trig == 2){
                auto msg = cv_bridge::CvImage(header,"bgr8",input).toImageMsg();
                cof2_pub->publish(*msg);
                trig = 0;
            }else if(trig == 3){
                auto msg = cv_bridge::CvImage(header,"bgr8",input).toImageMsg();
                ora_pub->publish(*msg);
                trig = 0;
            }
        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tri_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cof1_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cof2_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ora_pub;
        std_msgs::msg::Header header;
        int trig = 0;
        Mat input;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Imread>());
    rclcpp::shutdown();
    return 0;
}