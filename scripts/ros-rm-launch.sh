#! /bin/bash 
cd /home/nuaa/RM
usb_ttl_device=/dev/ttyUSB0
echo "usb device is ${usb_ttl_device}"
#echo "nvidia"|sudo chmod 777 ${usb_ttl_device}
#echo "nvidia"|sudo chmod 777 src/RM_NUAA_ROS_YZ/ros_dynamic_test/cfg/dyn_param.cfg
export usb_ttl=${usb_ttl_device}

# launch this repo
source devel/setup.bash 
#if in real battle
roslaunch windMill MV_cam.launch
#if debug using videos
#roslaunch windMill video_test.launch
