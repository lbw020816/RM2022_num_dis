#include<ros/ros.h>
#include <opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<ros/ros.h>
using namespace std;
void imageCallback(const sensor_msgs::ImagePtr& msg);

int main(int argc,char **argv)
{
    ros::init(argc, argv, "num_dis");
    ros::start();
    ros::NodeHandle n;
    ros::Subscriber imageSub = n.subscribe("/armor_detector/armor_roi",1,&imageCallback);
    cv::namedWindow("num_dis");
    ros::spin();

    return 0;
}

void imageCallback(const sensor_msgs::ImagePtr& msg)
{
    ROS_INFO("aaa");
    cv::Mat img =cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::imshow("num_dis",img);
    cv::waitKey(1);
}

