#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "iomanip"
#include <stdlib.h>
#include<serial_common/serialWrite.h>
#include<serial_common/gimbal.h>
#include<string>
#include<iostream>
#include<stdio.h>
#define DATA_LEN 10
#define REC_DATA_LEN 30uint8_t

serial::Serial ser;//声明串口对象
ros::Time sendTime;

typedef struct 
{
    uint16_t header;
    uint8_t length;
    uint8_t cmd_id;
    float yaw;
    float pitch;
    float vx;
    float vy;
    float wz;
    uint16_t crc;
    uint16_t end;
}__attribute__((packed)) robot_status;

typedef struct
{
    uint8_t header1 = 0xAA;
    uint8_t length = DATA_LEN;
    uint8_t cmd_id = 0x81;
    short yaw;
    short pitch;
    short depth;
    uint8_t crc;
}__attribute__((packed)) selfaim_info;


//serial_common::gimbal receive_msg;

unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) {
    unsigned char byte_crc = 0;
    for (unsigned char i = 0; i < data_lenth; i++) {
        byte_crc += InputBytes[i];
    }
    return byte_crc;
}

void Data_disintegrate(short Data, unsigned char *LData,
                       unsigned char *HData) {
    *LData = Data & 0XFF;          // 0xFF = 1111 1111
    *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}

void write_callback(const serial_common::serialWrite::ConstPtr& msg)
{
    selfaim_info info;
    info.yaw = msg->xlocation;
    info.pitch =  msg->depth;
    info.depth =msg->ylocation;
    std::cout<<"serial:   "<<info.yaw<<"  "<<info.pitch<<"  "<<info.depth<<std::endl;
    info.crc = Add_CRC((uint8_t*)&info, DATA_LEN - 1);
    ser.write((uint8_t*)&info,DATA_LEN);   //发送串口数据
    sendTime=ros::Time::now();
    auto delay=sendTime-msg->header.stamp;
    std::cout<<"dalay for get img to send is "<<delay<<std::endl;
    //printf("got location:: (%d,  %d )\n",msg->xlocation,msg->ylocation);
}

void receive_process(std::string &read_buffer)
{
  if(read_buffer[0]!=0xAA &&  read_buffer[1]!=0x02 &&  read_buffer[2]!=0x04 )
  {
    return;
  }
  int patch_num=(int)read_buffer[3];

}
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_common_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("/write", 33, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial/read", 1);
    ros::Publisher status_pub = nh.advertise<serial_common::gimbal>("/robot_status", 1);

    //设置串口属性，并打开串口

    const char *usb_ttl=getenv("usb_ttl");
    if(usb_ttl==NULL)
    {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
      ROS_WARN_STREAM("SYSTEM USB NOT DETECTED");
      if(!ser.isOpen())
      {
          ser.setPort("/dev/ttyUSB1");
          ser.open();
      }
    }else
    {
      ROS_WARN_STREAM("usb name is"<<usb_ttl);
      ser.setPort(usb_ttl);
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();

    }
    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    ros::Rate loop_rate(260);
    serial_common::gimbal status;
    std_msgs::String mode;
    std::string rec_buffer;

    while(ros::ok())
    {
        if(ser.available()){
            
           ser.readline(rec_buffer,ser.available(),"\x11\xbb");
           //std::cout<<rec_buffer<<std::endl;
           const unsigned char* buffer = (unsigned char*)rec_buffer.c_str();
            if(buffer[0]==0x00 && buffer[1]==0xAA )
            {
               // std::cout<<"frame head"<<std::endl;
                if(buffer[3]==0x03)
                {
                    robot_status* data = (robot_status*)buffer;
                    //std::cout<<"size"<<sizeof(robot_status)<<std::endl;
                    if (data->end == 0x11bb && data->length == sizeof(robot_status)) 
                    
		    {
			            //ROS_WARN("receiving");
                        status.yaw=data->yaw;
                        status.pitch=data->pitch;
                        status.vx=data->vx;
                        status.vy=data->vy;
                        status.wz=data->wz;
                        status_pub.publish(status);
                        printf("%f", data->wz);
                        rec_buffer.clear();
                    }
                }
                if(buffer[3]==0x02)
                {
		            // std::cout<<"receive ser"<<std::endl;
		            // ROS_WARN("mode change");
                    mode.data=buffer[4];
		            std::cout<<mode.data<<std::endl;
                    read_pub.publish(mode);
                    rec_buffer.clear();
                }
            }
            if (rec_buffer.size() > 512) {
                rec_buffer.clear();
            }
            // result.data = ser.read(ser.available());
            // std::string read_buffer=result.data.c_str();
            // if(read_buffer[0]==0xAA)
            // {
            //     if(read_buffer[2]==0x03)
            //     {
            //         yaw.data=read_buffer[3];
            //         pitch.data=read_buffer[4];
            //         speed_x.data=read_buffer[5];
            //         speed_y.data=read_buffer[6];
            //         w.data=read_buffer[7];
            //         pitch_pub.publish(pitch);
            //     }
            //     if(read_buffer[2]==0x02)
            //     {
            //         read_pub.publish(result);
            //     }
            // }
            // To get data from stm32, you need further process on read_buffer. Not finished.
            
        }

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();

    }


}
