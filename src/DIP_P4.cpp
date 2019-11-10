#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include<complex.h>
#include<math.h>
#include <iostream>
#include<time.h>
#include<vector>
#include<string>

using namespace cv;
using namespace std;


int PickupColor(Mat& inputframe,Mat& outputframe);

int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(0);//打开相机

    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",5);//定义速度发布器
    geometry_msgs::Twist cmd_go;

    Mat img;
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);

    while (ros::ok())
    {

        capture.read(img);
        if(img.empty())
        {
            break;
        }

        //img= imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/1.jpg");

        //Mat frame;
        //cvtColor(img,frame, CV_BGR2GRAY );

        Mat show_pic;
        int direction=PickupColor(img,show_pic);
        cout<<direction<<endl;//kkk 颜色判断 红1 绿2 蓝3 黄4

        switch(direction) {
            case 1:
                cmd_go.linear.y = 1;
                break;
            case 2:
                cmd_go.linear.y = 2;
                break;
            case 3:
                cmd_go.linear.z = 3;
                break;
            case 4:
                cmd_go.angular.x = 4;
                break;
            case 0:
                cmd_go.angular.x = 4;
            default:
                cmd_go.angular.z = 0;
        }
        pub.publish(cmd_go);

        imshow("img",img);
        imshow("frame",show_pic);

        ros::spinOnce();
        waitKey(5);
    }

    return 0;
}

int PickupColor(Mat&inputframe, Mat& outputframe)

{

    Mat hsvframe;
    int red_Num=0;// 1
    int green_Num=0;// 2
    int blue_Num=0;// 3
    int yellow_Num=0;// 4

    cvtColor(inputframe, hsvframe, COLOR_BGR2HSV);

    outputframe = Mat(hsvframe.rows, hsvframe.cols,CV_8UC3, cv::Scalar(255, 255, 255));

    int rowNumber = hsvframe.rows;

    int colNumber = hsvframe.cols;

    double H = 0.0, S = 0.0, V = 0.0;

    for (int i = 0; i < rowNumber; i++)
    {
        for (int j = 0; j < colNumber; j++)
        {

            H = hsvframe.at<Vec3b>(i, j)[0];

            S = hsvframe.at<Vec3b>(i, j)[1];

            V = hsvframe.at<Vec3b>(i, j)[2];

            //red
            if(((H >= 0 && H <= 10) || (H >= 156 && H <= 180)) && S >= 43 && V >= 46)

            {

                outputframe.at<Vec3b>(i, j)[0]=inputframe.at<Vec3b>(i, j)[0];

                outputframe.at<Vec3b>(i, j)[1]=inputframe.at<Vec3b>(i, j)[1];

                outputframe.at<Vec3b>(i, j)[2]=inputframe.at<Vec3b>(i, j)[2];

                red_Num++;
            }

            //green
            if((H >= 35 && H <= 77) && S>=43 && V >= 46)

            {

                outputframe.at<Vec3b>(i, j)[0]=inputframe.at<Vec3b>(i, j)[0];

                outputframe.at<Vec3b>(i, j)[1]=inputframe.at<Vec3b>(i, j)[1];

                outputframe.at<Vec3b>(i, j)[2]=inputframe.at<Vec3b>(i, j)[2];

                green_Num++;

            }

            //blue
            if((H >= 100 && H <= 124) && S >= 43 && V >= 46)

            {

                outputframe.at<Vec3b>(i, j)[0]=inputframe.at<Vec3b>(i, j)[0];

                outputframe.at<Vec3b>(i, j)[1]=inputframe.at<Vec3b>(i, j)[1];

                outputframe.at<Vec3b>(i, j)[2]=inputframe.at<Vec3b>(i, j)[2];

                blue_Num++;
            }

            //yellow
            if( (H >= 26 && H <= 34) && S >= 43 && V >= 46)

            {

                outputframe.at<Vec3b>(i, j)[0]=inputframe.at<Vec3b>(i, j)[0];

                outputframe.at<Vec3b>(i, j)[1]=inputframe.at<Vec3b>(i, j)[1];

                outputframe.at<Vec3b>(i, j)[2]=inputframe.at<Vec3b>(i, j)[2];

                yellow_Num++;
            }
        }

    }
    int flag=0;
    if(red_Num>=blue_Num&&red_Num>=green_Num&&red_Num>=yellow_Num)flag=1;
    if(green_Num>=red_Num&&green_Num>=blue_Num&&green_Num>=yellow_Num)flag=2;
    if(blue_Num>=red_Num&&blue_Num>=green_Num&&blue_Num>=yellow_Num)flag=3;
    if(yellow_Num>=blue_Num&&yellow_Num>=green_Num&&yellow_Num>=red_Num)flag=4;

    if(red_Num<=50 && blue_Num<=50 && green_Num<=50 &&yellow_Num<=50) //不在此四种颜色范围内(还要调节阈值)
        flag=0;

    return flag;
}
