#! /bin/bash
cd /home/catmulti7/2020Vision
usb_ttl_device=$(ls /dev/ttyTHS2*)
echo "usb device is ${usb_ttl_device}"
export usb_ttl=${usb_ttl_device}

source devel/setup.bash
roslaunch windMill MV_cam.launch
