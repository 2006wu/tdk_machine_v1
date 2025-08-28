#include "cmath"
#include "string"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "camera/algorithm.hpp"

class Process:public rclcpp::Node{
    public:
        Process():Node("coffee1"){
            img_sub = this->create_subscription<sensor_msgs::msg::Image>("coffee1_image",10,std::bind(&Process::image_callback,this,_1));
            tar_pub = this->create_publisher<std_msgs::msg::Int32>("target",10);
            retry_pub = this->create_publisher<std_msgs::msg::Int32>("mode",10);
            vision_pub = this->create_publisher<std_msgs::msg::Int32>("vision_result",10);
        }
    private:
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg){
            input = cv_bridge::toCvCopy(msg,"bgr8")->image;
            result = algo.main(input);
            if(result == -1){
                std::cout << "no target" << std::endl;
                answer.data = 1;
                retry_pub->publish(answer);
            }else{
                std::cout << "target: " << result << std::endl;
                answer.data = result;
                tar_pub->publish(answer);
                vision_pub->publish(answer);
            }
        }
        Algorithm algo;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tar_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr retry_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vision_pub;
        std_msgs::msg::Int32 answer;
        Mat input;
        int result;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Process>());
    rclcpp::shutdown();
    return 0;
}

int Algorithm::main(Mat &input){
    s1.clear();
    Mat mask = find(input);
    rectfind(mask);
    int target = sort(mask);
    return target;
}

Mat Algorithm::change(Mat &input){
    Mat gray,thre,ero,dil;
    cvtColor(input,gray,cv::COLOR_BGR2GRAY);
    threshold(gray,thre,0,255,cv::THRESH_BINARY + cv::THRESH_OTSU);
    erode(thre,ero,Mat());
    dilate(ero,dil,Mat());
    return dil;
}

Mat Algorithm::warp(Mat &input,const vector<Point> &poly){
    int temp1,temp2,LT,LB,RT,RB;
    int max1=0,min1=1000,max2=0,min2=1000;
    Mat M,warped;
    size_t i;
    vector<Point2f> srcPoints(4),sortPoint(4);

    for(i=0;i<4;i++){
        srcPoints[i] = poly[i];
    }
    for(i=0;i<4;i++){
        temp1 = srcPoints[i].x + srcPoints[i].y;
        if(temp1 > max1){
            max1 = temp1;
            RB = i;
        }
        if(temp1 < min1){
            min1 = temp1;
            LT = i;
        }
        temp2 = srcPoints[i].x - srcPoints[i].y;
        if(temp2 > max2){
            max2 = temp2;
            RT = i;
        }
        if(temp2 < min2){
            min2 = temp2;
            LB = i;
        }
    }
    sortPoint[0] = srcPoints[LT];
    sortPoint[1] = srcPoints[RT];
    sortPoint[2] = srcPoints[RB];
    sortPoint[3] = srcPoints[LB];

    float width = input.size().width;
    float height = input.size().height;
    vector<Point2f> dstPoints = {
        Point2f(0 , 0),
        Point2f(width-1 , 0),
        Point2f(width-1 , height-1),
        Point2f(0 , height-1)
    };

    M = cv::getPerspectiveTransform(sortPoint,dstPoints);
    warpPerspective(input,warped,M,input.size());
    return warped;
};

Mat Algorithm::find(Mat &input){
    int area,apsize;
    int max=-1,second=-1,maxarea=0,secondarea=0;
    Mat result,warped;
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;

    result = change(input);
    findContours(result,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> polyContours(contours.size());
    if(!contours.empty()){
        for(size_t i=0; i < contours.size(); i++){
            approxPolyDP(Mat(contours[i]),polyContours[i],5,true);
            apsize = polyContours[i].size();
            area = contourArea(contours[i]);
            if(apsize >= 4 && isContourConvex(polyContours[i]) && area > 10000){
                if(area > maxarea){
                    secondarea = maxarea;
                    second = max;
                    maxarea = area;
                    max = i;
                }else if(area > secondarea){
                    secondarea =area;
                    second = i;
                }
            }
        }
    }
    if(second >= 0 && second < (int)polyContours.size() && polyContours[second].size() >= 4){
        warped = warp(input,polyContours[second]);
        return warped;
    }else{
        return input;
    }
}

void Algorithm::rectfind(Mat &input){
    int area,apsize,cx,cy;
    float fillratio;
    double dist;
    Rect rect;
    Mat result;
    vector<vector<Point>> contours;
    vector<cv::Vec4i> hierarchy;
    
    result = change(input);
    findContours(result,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> polyContours(contours.size());
    vector<std::pair<Point,int>> squareCenters;
    for(size_t i=0;i<contours.size();i++){
        approxPolyDP(Mat(contours[i]),polyContours[i],5,true);
        area = contourArea(contours[i]);
        rect = boundingRect(contours[i]);
        apsize = polyContours[i].size();
        fillratio = (float)area / (rect.width*rect.height);
        if(apsize == 4 && isContourConvex(polyContours[i])){
            if(fillratio >= 0.8 && area >= 3000 && area < 30000){
                cx = rect.x + rect.width/2;
                cy = rect.y + rect.height/2;
                Point center(cx,cy);
                bool duplicate = false;
                for(const auto& [prevCenter,idx] : squareCenters){
                    dist = norm(center-prevCenter);
                    if(dist < 10){
                        duplicate = true;
                        break;
                    }
                }
                if(!duplicate){
                    squareCenters.emplace_back(center,i);
                    if(i<contours.size() && !contours[i].empty()){
                        s1.emplace_back(rect.x,rect.y,rect.width,rect.height,i,contours[i]);
                    }
                }
            }
        }
    }
    return;
}

int Algorithm::sort(Mat &input){
    size_t i,j;
    int radius,target,tri=0;
    double dif,max=0;
    squares temp;
    vector<double> simdiff;
    Mat gray,mask;
    Scalar squa,core;

    if(s1.size() != 4){
        target = -1;
        return target;
    }
    for(i=0;i<s1.size();i++){
        for(j=0;j<s1.size()-1-i;j++){
            if(s1[j].x > s1[j+1].x){
                temp=s1[j];
                s1[j]=s1[j+1];
                s1[j+1]=temp;
            }
        }
    }
    for(i=0;i<3;i+=2){
        if(s1[i].y > s1[i+1].y){
            temp=s1[i];
            s1[i]=s1[i+1];
            s1[i+1]=temp;
        }
    }

    cvtColor(input,gray,cv::COLOR_BGR2GRAY);
    for(i=0;i<s1.size();i++){
        mask = Mat::zeros(input.size(),CV_8UC1);
        drawContours(mask,vector<vector<Point>>{s1[i].contours},-1,Scalar(255),cv::FILLED);
        squa = mean(gray,mask);

        mask = Mat::zeros(input.size(),CV_8UC1);
        Point center(s1[i].x + s1[i].width/2 , s1[i].y + s1[i].height/2);
        radius = s1[i].height/6;
        circle(mask,center,radius,Scalar(255),-1);
        core = mean(gray,mask);
        // printf("mean:%f\n",core[0]);

        dif = std::abs(squa[0]-core[0]);
        simdiff.emplace_back(dif);
        if(core[0] < 80){
            tri = 1;
        }
    }
    for(i=0;i<s1.size();i++){
        if(simdiff[i] > max){
            max = simdiff[i];
            target = i+1;
        }
    }
    if(tri == 1){
        target += 4;
    }
    return target;
}