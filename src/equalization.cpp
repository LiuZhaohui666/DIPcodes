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
#include "sensor_msgs/Image.h"

#define LINEAR_X 0

using namespace cv;


class Histogram1D
{
private:
    int histSize[1]; // 项的数量
    float hranges[2]; // 统计像素的最大值和最小值
    const float* ranges[1];
    int channels[1]; // 仅计算一个通道

public:
    Histogram1D()
    {
        // 准备1D直方图的参数
        histSize[0] = 256;
        hranges[0] = 0.0f;
        hranges[1] = 255.0f;
        ranges[0] = hranges;
        channels[0] = 0;
    }

    Mat getHistogram(const Mat &image)
    {
        Mat hist;
        // 计算直方图
        calcHist(&image ,// 要计算图像的
                 1,                // 只计算一幅图像的直方图
                 channels,        // 通道数量
                 Mat(),            // 不使用掩码
                 hist,            // 存放直方图
                 1,                // 1D直方图
                 histSize,        // 统计的灰度的个数
                 ranges);        // 灰度值的范围
        return hist;
    }

    Mat getHistogramImage(const Mat &image)
    {
        Mat hist = getHistogram(image);

        // 最大值，最小值
        double maxVal = 0.0f;
        double minVal = 0.0f;

        minMaxLoc(hist, &minVal, &maxVal);

        //显示直方图的图像
        Mat histImg(histSize[0], histSize[0], CV_8U, Scalar(255));

        // 设置最高点为nbins的90%
        int hpt = static_cast<int>(0.9 * histSize[0]);
        //每个条目绘制一条垂直线
        for (int h = 0; h < histSize[0]; h++)
        {
            float binVal = hist.at<float>(h);
            int intensity = static_cast<int>(binVal * hpt / maxVal);
            // 两点之间绘制一条直线
            line(histImg, Point(h, histSize[0]), Point(h, histSize[0] - intensity), Scalar::all(0));
        }
        return histImg;
    }
};


int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(1);//打开zed相机


    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
    ros::NodeHandle n;

    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器


    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat img;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高


    while (ros::ok())
    {
        capture.read(img);
        if(img.empty())
        {
            break;
        }
        Mat frame;
        cvtColor(img,frame, CV_BGR2GRAY );  //彩色图片转换成黑白图片
        Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片

        // 此处增加直方图均衡化
        Mat frOut = frIn.clone();//定义输出图像
        int gray[256] = {0};
        double gray_prob[256] = {0};//图像的灰度分布概率
        double gray_integral[256] = {0};  //图像的灰度累计密度
        int gray_equal[256] = {0};  //均衡化后的灰度值
        int pixel_sum = 0;//像素总量
        pixel_sum = frIn.cols*frIn.rows;
        for (int i = 0; i < frIn.rows; i++)
        {
            uchar* p = frIn.ptr<uchar>(i);
            for(int j = 0; j < frIn.cols; j++)
            {
                int value = p[j];
                gray[value]++;
            }
        }//计算每个灰度像素个数
        for (int i = 0; i < 256 ; i++)
        {
            gray_prob[i] = ((double)gray[i]/pixel_sum);
        }//计算分布概率
        gray_integral[0] = gray_prob[0];
        for (int i = 1; i < 256; i++)
        {
            gray_integral[i] = gray_integral[i-1] + gray_prob[i];
        }
        for (int i = 0; i < 256 ; i++)
        {
            gray_equal[i] = (uchar)(255 * gray_integral[i] + 0.5);
        }//计算均衡化过后对应灰度值
        for (int i = 0; i < frOut.rows; i++)
        {
            uchar* p = frOut.ptr<uchar>(i);
            for (int j = 0; j < frOut.cols; j++)
            {
                int q = p[j];
                p[j] = gray_equal[q];
            }
        }

        imshow("1",frIn);
        imshow("2",frOut);

        Histogram1D hist;
        Mat histImg1;
        histImg1 = hist.getHistogramImage(frIn);
        Mat histImg2;
        histImg2 = hist.getHistogramImage(frOut);
        imshow("Histogram", histImg1);
        imshow("Histogram_result",histImg2);

        geometry_msgs::Twist cmd_red;

        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;

        pub.publish(cmd_red);

        ros::spinOnce();
//		loop_rate.sleep();
        waitKey(5);

    }


    return 0;
}


