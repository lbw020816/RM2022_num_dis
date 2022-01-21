/// ros header
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/TransformStamped.h>
#include"serial_common/serialWrite.h"
/// opencv header
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/// algorithm header
#include "MarkerSensor.h"
#include "dafu_detect.h"


using namespace std;
using namespace cv;
Mat img_src,img_to_show,roi_to_show,binary_to_show, coordinate_to_show;
/// output
int pix_x, pix_y;
int armorNum=6;
float Z = 0;

// initialize objects as null
MarkSensor *markSensor=NULL;
Dafu_Detector *dafu_detector=NULL;
serial_common::serialWrite tgt_pos;

//flags
bool is_redetect=true;
bool is_top_mode=false;
bool got_img=false;
int is_find_enemy=0;
bool isNotSuccess = 0;

/// input img, output gimbal target
int AutoAim(Mat &bgrImg)
{
  cout<<"=============process 1 frame:aimbot==================="<<endl;
  if(is_redetect)
  {
    markSensor->status=MarkSensor::STATUS_DETECTING;
    is_redetect=false;
    armorNum=6;
    //            ROS_ERROR("REDETECT!!");
  }

  isNotSuccess = markSensor->ProcessFrameLEDXYZ(bgrImg,  Z, pix_x, pix_y);

  if (!isNotSuccess)
  {
    is_find_enemy = 1;
    /// generate msg
    tgt_pos.xlocation=pix_x;
    tgt_pos.ylocation=pix_y;
    tgt_pos.depth=Z*1000;
    tgt_pos.no=armorNum;

    std::cout<<"target pix::  "<<pix_x<<","<<pix_y<<std::endl;
  }
  else
  {
    is_find_enemy=0;
    tgt_pos.xlocation=30000;
    tgt_pos.ylocation=30000;
    tgt_pos.depth=30000;    
    tgt_pos.no=6;

  }


  return is_find_enemy; 
}

int LockTop(Mat &bgrImg)
{
  if(is_redetect)
  {
        markSensor->status=MarkSensor::STATUS_TOP_REDETECT;
        is_redetect=false;
        //ROS_ERROR("REDETECT!!");
  }
   if(markSensor->GetTopPos(bgrImg, pix_x, pix_y,Z))
   {
       is_find_enemy = 1;
        /// generate msg
        tgt_pos.xlocation=pix_x;
        tgt_pos.ylocation=pix_y;
        tgt_pos.depth=Z*1000;
        tgt_pos.no=markSensor->center_in_rect;
   }
   else
   {
        is_find_enemy=0;
        /// generate msg
        tgt_pos.xlocation=30000;
        tgt_pos.ylocation=30000;
        tgt_pos.depth=Z*1000;
        tgt_pos.no=0;
   }
   
}
///  subscribe img and do call-back functions
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher roi_image_pub_;
  image_transport::Publisher coordinate_pub_;
  image_transport::Publisher binary_image_pub_;
  image_transport::Publisher show_image_pub_;

  ros::Publisher serial_pub;
  ros::Subscriber numpred_sub;
  ros::Subscriber WM_activator_sub;

  ros::Publisher fps_pub;
  ros::Publisher depth_pub;
  ros::Publisher is_large_pub;

  std_msgs::Bool is_large_msg;
  std_msgs::Float32 fps_msg;
  std_msgs::Float32 depth_msg;
  
  bool is_red,ifshow=1;
  int cam_idx=1;
  int begin_counter=cv::getTickCount();

public:
  //most initialize is done here
  ImageConverter()
    : it_(nh_)
  {


    nh_.getParam("/ifshow",ifshow);
    nh_.getParam("/cam_idx",cam_idx);

    /// new object (ap,cp,mp) will automatically load params from params.yaml
    AlgoriParam ap;
    CamParams cp(cam_idx,false);
    MarkerParams mp(ifshow);
    /// auto shoot
    markSensor=new MarkSensor(ap,cp,mp);
    /// dafu
    dafu_detector=new Dafu_Detector(ap,cp);
    
    // Subscrive to input video feed and publish output video feed
    // ones recieved, run callback imageCb, in imageCb there are two situation:autoshoot and windmill
    image_sub_ = it_.subscribe("/MVCamera/image_raw", 1,  &ImageConverter::Image_cb, this);

    // Subscribe to the serail msg sent from stm32
    // the msg is whether windmill mode is activated  ros::Subscriber cfg_sub;
    WM_activator_sub=nh_.subscribe<std_msgs::String>("/serial/read",1,&ImageConverter::Mode_cb,this);

    // publishers
    roi_image_pub_ = it_.advertise("/armor_detector/armor_roi", 1);
    binary_image_pub_ = it_.advertise("/armor_detector/binary_img", 1);
    show_image_pub_ = it_.advertise("/armor_detector/output_img", 1);
    coordinate_pub_ = it_.advertise("/armor_detector/coordinate", 1);
    serial_pub=nh_.advertise<serial_common::serialWrite>("/write_pixel",20);
    fps_pub=nh_.advertise<std_msgs::Float32>("/fps",1);
    depth_pub=nh_.advertise<std_msgs::Float32>("/estidepth",1);
    is_large_pub=nh_.advertise<std_msgs::Bool>("/mv_param/is_large",3);



  }

  ~ImageConverter()
  {
  }

  /// subscribe  serial msgs and swith to windmill(dafu) mode
  void Mode_cb(const std_msgs::StringConstPtr &msg)
  {
    unsigned char mode_normal=0x02;
    unsigned char mode_windMill_cw=0x01,mode_windMill_ccw=0x03;  // rotate direction
    unsigned char mode_top=0x04;
    unsigned char mode_b_windMill=0x05;
    ROS_INFO_STREAM("Read: " << msg->data);
    cout<<"msg->data[0]:"<<msg->data[0]<<endl;
    ROS_WARN("debug: mode changed!!");
    if(msg->data[0]==mode_normal)
   {
      ROS_WARN("aimbot");
      dafu_detector->status=dafu_detector->PR_NA;
      is_top_mode=false;
      is_redetect=true;
    }
    else if(msg->data[0]==mode_windMill_cw)
    {
      ROS_WARN("PR cw");
      dafu_detector->status=dafu_detector->PR_cw;
      is_top_mode=false;
    }
    else if(msg->data[0]==mode_windMill_ccw)
    {
      ROS_WARN("PR ccw");
      dafu_detector->status=dafu_detector->PR_ccw;
      is_top_mode=false;
    }
    else if(msg->data[0]==mode_top)
    {
      ROS_WARN("top");
      dafu_detector->status=dafu_detector->PR_NA;
      is_redetect=true;
      is_top_mode=true;
    }
    else if(msg->data[0]==mode_b_windMill)
    {
      ROS_WARN("PR big");
      dafu_detector->status=dafu_detector->PR_big;//
      is_top_mode=false;
    }

    // if two modes use different resolution, you can modify camera params here
    // markSensor->cp=CamParams(cam_idx,false);
  }

///
/// \brief void Image_cb(const sensor_msgs::ImageConstPtr& msg)
/// subscribe image from mv_cam node, and perfrom algorithm
///
  void Image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    //convert ROS image msg to opencv Mat
    try
    {
        img_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
        markSensor->imgTime = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->header.stamp;
        got_img=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ///main work
    process_frame();

    //check fps
    // open a new terminal and type: rostopic hz fps
    fps_msg.data=1;
    fps_pub.publish(fps_msg);
    depth_msg.data=Z;
    depth_pub.publish(depth_msg);
  }

///
/// \brief void process_frame()
/// process one frame here, most of the work is in this function
///
  void process_frame()
  {
    //windMill mode
    if(dafu_detector->status)
    {
        Point tgtarmor;

        //find target position
        tgtarmor=dafu_detector->UnlockPR(img_src,markSensor->ap.is_red,dafu_detector->status);

        /// for visualization
        img_to_show=img_src;

        // publish target msgs
        if (tgtarmor!=Point(0,0)) 
        {
            is_find_enemy = 1;
            tgt_pos.xlocation=tgtarmor.x;
            tgt_pos.ylocation=tgtarmor.y;
          // waitKey(1);
            tgt_pos.depth=0;
            tgt_pos.no=0;
            std::cout<<"target pix::  "<<tgtarmor.x<<","<<tgtarmor.y<<std::endl;
        }
        else  // no target
       {
            is_find_enemy=0;
            tgt_pos.xlocation=30000;
            tgt_pos.ylocation=30000;
            tgt_pos.depth=30000;
            tgt_pos.no=30000;
        }
          // waitKey(1);
    }
    if(is_top_mode)
    {
        LockTop(img_src);
        img_to_show=markSensor->img_show;
        //unfinshed,top
    }
    if(!is_top_mode && !dafu_detector->status ) //auto shoot
    {
        AutoAim(img_src); //llj:autoshoot main code here
        /// for visualization
        img_to_show=markSensor->img_show;
        roi_to_show=markSensor->ROI_bgr;
        coordinate_to_show=markSensor->coordinate;

    }
    
    ///Originally we plan to use small resolution in detect mode and large resolution in windmill
    ///mode. However, now we use small in both mode.
    //        is_large_msg.data=!is_windMill_mode;  //if wm mode, then small resolution
    //        is_large_pub.publish(is_large_msg);  //resolution of next frame will be ok...
    tgt_pos.header.stamp=markSensor->imgTime;
    serial_pub.publish(tgt_pos);

    

    //show image for debug
    if(ifshow)
    {
        sensor_msgs::ImagePtr show_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_to_show).toImageMsg();
        show_img_msg->header.stamp=ros::Time::now();
        show_image_pub_.publish(show_img_msg);

        if(!roi_to_show.empty())
        {
          // imshow("roi_to_show",roi_to_show);
          //waitKey(1);
        sensor_msgs::ImagePtr roi_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi_to_show).toImageMsg();
        roi_img_msg->header.stamp = ros::Time::now();
        roi_image_pub_.publish(roi_img_msg);
        }

        if(!coordinate_to_show.empty())
        {
          // imshow("roi_to_show",roi_to_show);
          //waitKey(1);
        sensor_msgs::ImagePtr coordinate_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", coordinate_to_show).toImageMsg();
        coordinate_msg->header.stamp = ros::Time::now();
        coordinate_pub_.publish(coordinate_msg);
        }

        if(binary_to_show.empty())
        return ;
        sensor_msgs::ImagePtr binary_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_to_show).toImageMsg();
        binary_img_msg->header.stamp=ros::Time::now();
        binary_image_pub_.publish(binary_img_msg);

    }
  }

};


int main(int argc, char** argv)
{
    // initialize this node
    ros::init(argc, argv, "image_converter");

    // create handler
    ImageConverter ic;

    // wait for msgs
    ros::spin();
    return 0;
}
