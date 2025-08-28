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

Point orange_center = Point(-100,-100);
const double eps = 1e-9;

void createtrack(){
    cv::namedWindow("控制器");
    cv::resizeWindow("控制器",600,300);
    createTrackbar("Low H","控制器",0,255);
    createTrackbar("Low S","控制器",0,255);
    createTrackbar("Low V","控制器",0,255);
    createTrackbar("High H","控制器",0,255);
    createTrackbar("High S","控制器",0,255);
    createTrackbar("High V","控制器",0,255);

    setTrackbarPos("Low H","控制器",0);//0
    setTrackbarPos("Low S","控制器",50);//50
    setTrackbarPos("Low V","控制器",100);//100
    setTrackbarPos("High H","控制器",40);//40
    setTrackbarPos("High S","控制器",255);//255
    setTrackbarPos("High V","控制器",255);//255
    return;
}

class Process:public rclcpp::Node{
    public:
        Process():Node("orange"){
            //createtrack();
            img_sub = this->create_subscription<sensor_msgs::msg::Image>("orange_image",10,std::bind(&Process::image_callback,this,_1));
            info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("/chiang/chiang/color/camera_info",10,std::bind(&Process::info_callback,this,_1));
            de_sub = image_transport::create_subscription(this,"/chiang/chiang/depth/image_rect_raw",std::bind(&Process::de_callback,this,_1),"raw");
            tar_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("point_in",10);
            retry_pub = this->create_publisher<std_msgs::msg::Int32>("mode",10);
            timer = this->create_wall_timer(30ms,std::bind(&Process::timer_callback,this));
        }
    private:
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg){
            input = cv_bridge::toCvCopy(msg,"bgr8")->image;
            result = algo.orangefind(input);
            if(orange_center.x != -100 && orange_center.y != -100){
                gotpoint = true;
            }else{
                std::cout << "no target" << std::endl;
                answer.data = 3;
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
            int u = orange_center.x;
            int v = orange_center.y;
            dep = depth.at<uint16_t>(v,u) * 0.001f;

            if(finite(dep) && dep != 0){
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
                answer.data = 3;
                retry_pub->publish(answer);
            }
            orange_center = Point(-100,-100);
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
    Mat range,bit,gray,thre,ero,dil,blur;
    Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(9,9));
    /*int LH = getTrackbarPos("Low H", "控制器");
    int LS = getTrackbarPos("Low S", "控制器");
    int LV = getTrackbarPos("Low V", "控制器");
    int HH = getTrackbarPos("High H", "控制器");
    int HS = getTrackbarPos("High S", "控制器");    
    int HV = getTrackbarPos("High V", "控制器");
    Scalar low = Scalar(LH,LS,LV);
    Scalar high = Scalar(HH,HS,HV);*/
    Scalar low = Scalar(0,50,100);
    Scalar high = Scalar(40,255,255);

    cv::inRange(input,low,high,range);
    bitwise_and(input,input,bit,range);
    cv::cvtColor(bit,gray,cv::COLOR_BGR2GRAY);
    threshold(gray,thre,0,255,cv::THRESH_BINARY + cv::THRESH_OTSU);
    erode(thre,ero,Mat());
    dilate(ero,dil,Mat());
    GaussianBlur(dil,blur,cv::Size(7,7),2,2);
    return blur;
}

Mat Algorithm::orangefind(Mat &input){
    int area;
    double dist;
    Mat output,result;
    cv::Rect rect;
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    vector<cv::Rect> poi;
    Point center(input.size().width/2 , input.size().height/2);
    
    output = input.clone();
    result = change(input);
    findContours(result,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    //vector<vector<Point>> polyContours(contours.size());
    for(size_t i=0; i < contours.size(); i++){
        //approxPolyDP(Mat(contours[i]),polyContours[i],5,true);
        area = contourArea(contours[i]);
        if(area > 500){
            rect = boundingRect(contours[i]);
            Point p1(rect.x + rect.width/2 , rect.y + rect.height/2);
            poi.emplace_back(rect);

            // Point p2(input.cols/2,input.rows/2);
            // cv::String text = "(" + std::to_string(p1.x) + "," + std::to_string(p1.y) + ")";
            // dist = norm(center - orange_center);

            // rectangle(output,rect,green,2);
            // circle(output,p2,3,white,1);
            // if(dist < 50){
            //     circle(output,orange_center,5,blue,2);
            //     putText(output,text,p1,cv::FONT_HERSHEY_SIMPLEX,1,blue,2);
            // }else{
            //     circle(output,orange_center,5,red,2);
            //     putText(output,text,p1,cv::FONT_HERSHEY_SIMPLEX,1,red,2);
            // }
        }
    }
    if(poi.size() > 1){
        Point p1(poi[0].x + poi[0].width/2 , poi[0].y + poi[0].height/2);
        Point p2(poi[1].x + poi[1].width/2 , poi[1].y + poi[1].height/2);
        dist = norm(p1-p2);
        if(dist < 100){
            orange_center = p1;
            circle(output,p1,5,red,2);
        }
    }else if(poi.size() == 1){
        Point p1(poi[0].x + poi[0].width/2 , poi[0].y + poi[0].height/2);
        orange_center = p1;
        circle(output,p1,5,red,2);
    }
    return output;
}