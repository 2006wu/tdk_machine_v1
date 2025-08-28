#include "cmath"
#include "string"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "camera/algorithm.hpp"

Point sticker_center = Point(-100,-100);
const double eps = 1e-9;

class Process:public rclcpp::Node{
    public:
        Process():Node("coffee2"){
            img_sub = this->create_subscription<sensor_msgs::msg::Image>("coffee2_image",10,std::bind(&Process::image_callback,this,_1));
            info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/chiang/chiang/color/camera_info",10,std::bind(&Process::info_callback,this,_1));
            de_sub = image_transport::create_subscription(this,"/chiang/chiang/depth/image_rect_raw",std::bind(&Process::de_callback,this,_1),"raw");
            tar_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("point_in",10);
            retry_pub = this->create_publisher<std_msgs::msg::Int32>("mode",10);
            timer = this->create_wall_timer(30ms,std::bind(&Process::timer_callback,this));
        }
    private:
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg){
            input = cv_bridge::toCvCopy(msg,"bgr8")->image;
            result = algo.stickerfind(input);
            if(sticker_center.x != -100 && sticker_center.y != -100){
                gotpoint = true;
            }else{
                std::cout << "no target" << std::endl;
                answer.data = 2;
                retry_pub->publish(answer);
            }
        }
        void info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg){
            fx = msg->k[0];
            fy = msg->k[4];
            cx = msg->k[2];
            cy = msg->k[5];
            if(finite(fx) && finite(fy) && std::abs(fx) > eps && std::abs(fy) > eps){
                gotinfo = true;
            }
        }
        void de_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
            if(!gotinfo || !gotpoint){
                return;
            }
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,msg->encoding);
            depth = cv_ptr->image;

            float dep = 0;
            int valid_count = 0;
            int u = sticker_center.x;
            int v = sticker_center.y;
            dep = depth.at<uint16_t>(v,u) * 0.001f;

            if(dep != 0){
                detect = true;
            }else{
                for(int dx = -1; dx <= 1; ++dx){
                    for(int dy = -1; dy <= 1; ++dy){
                        int x = u + dx;
                        int y = v + dy;
                        //這段不會
                        if (x < 0 || x >= depth.cols || y < 0 || y >= depth.rows) continue;
                        float d = depth.at<uint16_t>(y,x) * 0.001f; // 會回 NaN 代表無效
                        if (!std::isfinite(d)) continue;       // 跳過 NaN/Inf
                        if (d > 0.2f && d < 3.0f) {   // 依你的有效範圍
                            dep += d;
                            ++valid_count;
                        }
                    }
                }
                if(valid_count > 0){
                    dep /= valid_count;
                    detect = true;
                }
            }
            if(detect){
                X = (u-cx)*dep/fx;
                Y = (v-cy)*dep/fy;
                Z = dep;
                // std::cout << "x,y,z: " << X << ", " << Y << ", " << Z << std::endl;
                p.header.frame_id = "OpenCV";
                p.header.stamp = this->get_clock()->now(); //img->header.stamp;
                p.point.x = X;
                p.point.y = Y;
                p.point.z = Z;
                tar_pub->publish(p);
                detect = false;
            }else{
                std::cout << "no target" << std::endl;
                answer.data = 2;
                retry_pub->publish(answer);
            }
            sticker_center = Point(-100,-100);
            gotinfo = false;
            gotpoint = false;
        }
        void timer_callback(){
            if(!result.empty() && std::getenv("DISPLAY")){
                cv::imshow("result",result);
                cv::waitKey(1);
            }
        }
        Algorithm algo;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
        image_transport::Subscriber de_sub;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tar_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr retry_pub;
        rclcpp::TimerBase::SharedPtr timer;
        std_msgs::msg::Int32 answer;
        geometry_msgs::msg::PointStamped p;
        bool detect = false,gotinfo = false,gotpoint = false;
        float X,Y,Z,fx,fy,cx,cy;
        Mat input,result,depth;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Process>());
    rclcpp::shutdown();
    return 0;
}

Mat Algorithm::change(Mat &input){
    Mat gray,can,dil,mor,ero,thre,blur;
    cvtColor(input,gray,cv::COLOR_BGR2GRAY);
    Canny(gray,can,80,150);
    dilate(can,dil,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
    morphologyEx(dil,mor,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
    erode(mor,ero,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
    adaptiveThreshold(ero,thre,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY_INV,15,10);
    medianBlur(thre,blur,5);
    return blur;
}

Mat Algorithm::stickerfind(Mat &input){
    int area,dist;
    float ratio,fillratio;
    cv::Rect rect;
    Mat output,result;
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    vector<Point> poi;

    output = input.clone();
    result = change(input);
    findContours(result,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    for(size_t i = 0;i<contours.size();i++){
        area = contourArea(contours[i]);
        rect = boundingRect(contours[i]);
        ratio = (float)rect.width/rect.height;
        fillratio = (float)area / (rect.width * rect.height);
        if(ratio >= 1-0.4 && ratio <= 1+0.4 && fillratio >= 0.6){
            if(area > 7000 && area < 30000){
                Point p1(rect.x + rect.width/2 , rect.y + rect.height/2);
                poi.emplace_back(p1);
            }
        }
    }
    if(poi.size() > 1){
        dist = norm(poi[0]-poi[1]);
        if(dist < 100){
            sticker_center = poi[0];
            circle(output,poi[0],5,red,2);
        }
    }else if(poi.size() == 1){
        sticker_center = poi[0];
        circle(output,poi[0],5,red,2);
    }
    return output;
}