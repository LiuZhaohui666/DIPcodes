#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp> 
#include<opencv2/highgui/highgui.hpp> 
#include<opencv2/imgproc/imgproc.hpp> 
#include<stdio.h> 
using namespace cv;
using namespace std;
int main(int argc, char** argv) 
{ 
// ROS节点初始化
 ros::init(argc, argv, "image_publisher");
 ros::NodeHandle n;
 ros::Time time = ros::Time::now(); 
 ros::Rate loop_rate(5); // 定义节点句柄
 image_transport::ImageTransport it(n);
 image_transport::Publisher pub = it.advertise("/camera/image_color", 1);
 sensor_msgs::ImagePtr msg;
 // opencv准备读取视频 
 cv::VideoCapture video;
 video.open("/home/zhaohui/test.mp4");
 if( !video.isOpened() ) 
{ 
ROS_INFO("Read Video failed!\n");
 return 0;
 }
 Mat frame;
 int count = 0;
 while(1)
 { video >> frame;
   if( frame.empty() ) break;
    count++;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
   pub.publish(msg);
    ROS_INFO( "read the %dth frame successfully!", count );
    loop_rate.sleep();
    ros::spinOnce();
 } 
    video.release();
  return 0;
 }
