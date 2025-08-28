#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using std::vector;
using cv::Mat;
using cv::Rect;
using cv::Point;
using cv::Point2f;
using cv::Scalar;
using cv::createTrackbar;
using cv::setTrackbarPos;
using cv::getTrackbarPos;

using std::placeholders::_1;
using namespace std::chrono_literals;

class Algorithm{
    public:
        int main(Mat &input);
        Mat change(Mat &input);
        Mat warp(Mat &input,const vector<Point> &poly);
        Mat find(Mat &input);
        void rectfind(Mat &input);
        int sort(Mat &input);
        Mat stickerfind(Mat &input);   //修完coffee2後可將Mat換成Point
        Mat orangefind(Mat &input);
    private:
        struct squares{
            int x,y,width,height,n;
            vector<Point> contours;

            squares(){}
            squares(int x_, int y_, int w_, int h_, int n_, const vector<Point>& c)
                : x(x_), y(y_), width(w_), height(h_), n(n_), contours(c) {}
        };
        vector<squares> s1;
        Scalar white = Scalar(255,255,255); 
        Scalar black= Scalar(0,0,0); 
        Scalar green = Scalar(0,255,0); 
        Scalar blue = Scalar(255,0,0); 
        Scalar red = Scalar(0,0,255); 
};