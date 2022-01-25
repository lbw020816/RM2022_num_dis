#include<ros/ros.h>
#include <opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include<ros/ros.h>
using namespace std;
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int lastTime;
int num;
int main(int argc,char **argv)
{
    num=0;
    ros::init(argc, argv, "num_dis");
    ros::start();
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber imageSub =it.subscribe("/armor_detector/armor_num",1,imageCallback);
    cv::namedWindow("num_dis");
    lastTime = ros::Time::now().toSec();
    ros::spin();
    return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv::Mat special;
    cv::Mat hsv;
    cv::Mat img =cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::imshow("num_dis",img);
    cv::waitKey(1);

}

