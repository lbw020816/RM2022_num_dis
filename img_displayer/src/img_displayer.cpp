/*************************************************************************
  > File Name: video_pub.cpp
  > Author: CYZ
  > Mail:
  > Function: subscribe output img from windmill node and show them
 ************************************************************************/

#include<iostream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

Mat img_show, img_binary, img_roi, coordinate;
string dbg_img_path;
int false_idx=0;
string num2str(double i)

{
  stringstream ss;
  ss << i;
  return ss.str();
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_binary_sub_;
  image_transport::Subscriber image_roi_sub;
  image_transport::Subscriber coordinate_sub;
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/armor_detector/output_img", 1,
                               &ImageConverter::Image_cb, this);
    image_binary_sub_=it_.subscribe("/armor_detector/binary_img", 1,
                                    &ImageConverter::binary_imageCb, this);
    image_roi_sub=it_.subscribe("/armor_detector/armor_roi", 1,
                                    &ImageConverter::roi_imageCb, this);
    coordinate_sub=it_.subscribe("/armor_detector/coordinate", 1,
                                    &ImageConverter::coordinate_imageCb, this);
    nh_.getParam("/dbg_img_path",dbg_img_path);

  }
  void roi_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      img_roi = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow("roi gray", img_roi);
  }

  void coordinate_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      coordinate = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow("coordinate", coordinate);
  }

  void Image_cb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      img_show = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow("another detection result", img_show);
    //    if(!markSensor.img_out.empty())
    //      cv::imshow("feed to number", markSensor.img_out);
    char key=cv::waitKey(1);
    if (key == 's') {
      false_idx++;
      string saveName_src =
          dbg_img_path + num2str(false_idx) + "falsesrc.jpg";

      std::cout<<saveName_src<<endl;
      imwrite(saveName_src, img_show);
    }
    if(key=='q' ||key=='Q')
    {
      //send SIGINT
      system("pkill roslaunch");
    }



  }
  void binary_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      img_binary = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow("binary img", img_binary);
    //    if(!markSensor.img_out.empty())
    //      cv::imshow("feed to number", markSensor.img_out);
    char key=cv::waitKey(1);
    if(key=='q' ||key=='Q')
    {
      //send SIGINT
      system("pkill roslaunch");
    }



  }
};
int main(int argc, char **argv)
{
  ros::init(argc,argv,"img_displayer");
  ros::NodeHandle nh;
  ImageConverter ic;
  ros::spin();
  return 0;
}
