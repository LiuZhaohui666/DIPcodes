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

int H_min = 0;
int H_max = 180;
int S_min = 0;
int S_max = 255;
int V_min = 0;
int V_max = 255;
int red=1;
int orange=0;
int yellow=1;
int green=1;
int cyan=0;
int blue=1;
int purple=0;

int black=0;
int gra=0;
int white=0;

float redN=0;
float orangeN=0;
float yellowN=0;
float greenN=0;
float cyanN=0;
float blueN=0;
float purpleN=0;
float blackN=0;
float graN=0;
float whiteN=0;

using namespace cv;
using namespace std;


//统计颜色数目
void count(Mat inputframe)
{
    Mat hsvframe;
    int red_Num=0;// 1
    int green_Num=0;// 2
    int blue_Num=0;// 3
    int yellow_Num=0;// 4

    cvtColor(inputframe, hsvframe, COLOR_BGR2HSV);

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
                red_Num++;
            }
            //green
            if((H >= 35 && H <= 77) && S>=43 && V >= 46)
            {
                green_Num++;
            }
            //blue
            if((H >= 100 && H <= 124) && S >= 43 && V >= 46)
            {
                blue_Num++;
            }
            //yellow
            if( (H >= 26 && H <= 34) && S >= 43 && V >= 46)
            {
                yellow_Num++;
            }
        }
    }

    //cout<<red_Num<<endl;
    //cout<<green_Num<<endl;
    //cout<<blue_Num<<endl;
    //cout<<yellow_Num<<endl;

    if(red==1)
    {
        redN=(1-(float)red_Num/(inputframe.cols*inputframe.rows))*256;
    }
    if(green==1)
    {
        greenN=(1-(float)green_Num/(inputframe.cols*inputframe.rows))*256;
    }
    if(blue==1)
    {
        blueN=(1-(float)blue_Num/(inputframe.cols*inputframe.rows))*256;
    }
    if(yellow==1)
    {
        yellowN=(1-(float)yellow_Num/(inputframe.cols*inputframe.rows))*256;
    }

    /*cout<<redN<<endl;
    cout<<greenN<<endl;
    cout<<blueN<<endl;
    cout<<yellowN<<endl;*/
    //int NNN=inputframe.cols*inputframe.rows;
    //cout<<NNN<<endl;
}

int main(int argc, char **argv)
{
    //VideoCapture capture;
    //capture.open(0);

    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");
    ros::NodeHandle n;

    // ros::Rate loop_rate(10);
    //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);


    /*if (!capture.isOpened())
    {
        printf("wrong!\n");
        return 0;
    }*/
    //waitKey(1000);
    Mat frame;
    frame=cv::imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/1.jpg");
    int nFrames = 0;
    int frameWidth = frame.rows;
    int frameHeight = frame.cols;

    while (ros::ok())
    {
        Mat out;
        GaussianBlur(frame, out, Size(5, 5),0,0);
        imshow("gauss",out);
        Mat HSV;
        cvtColor(out, HSV, CV_BGR2HSV);
        Mat gray;
        cvtColor(out, gray, CV_BGRA2GRAY);


        namedWindow("Linear Blend", 1);

        createTrackbar( "H_min", "Linear Blend", &H_min, 180);
        createTrackbar( "H_max", "Linear Blend", &H_max, 180);
        createTrackbar( "S_min", "Linear Blend", &S_min, 255);
        createTrackbar( "S_max", "Linear Blend", &S_max, 255);
        createTrackbar( "V_min", "Linear Blend", &V_min, 255);
        createTrackbar( "V_max", "Linear Blend", &V_max, 255);

        /*createTrackbar( "red", "Linear Blend", &red, 1);
        createTrackbar( "green","Linear Blend", &green, 1);
        createTrackbar( "blue","Linear Blend", &blue, 1);
        createTrackbar( "yellow", "Linear Blend", &yellow, 1);
        createTrackbar( "cyan", "Linear Blend", &cyan , 1);
        createTrackbar( "blue", "Linear Blend", &blue, 1);
        createTrackbar( "purple", "Linear Blend", & purple, 1);

        createTrackbar( "black", "Linear Blend", &black, 1);
        createTrackbar( "gray", "Linear Blend", &gra, 1);
        createTrackbar( "white", "Linear Blend", &white, 1);*/

        //on_trackbar();

        Mat HSV_new=HSV.clone();
        vector<Mat> hsv_channels;
        split(HSV_new, hsv_channels);
        Mat H = hsv_channels.at(0);
        Mat S = hsv_channels.at(1);
        Mat V = hsv_channels.at(2);
        Mat frIn=gray.clone();
        for(int i=0;i<H.rows;i++)
        {
            for(int j=0;j<H.cols;j++)
            {
                if(H.at<uchar>(i,j)>H_min&&H.at<uchar>(i,j)<H_max)
                    frIn.at<uchar>(i,j)=255;
                else
                    frIn.at<uchar>(i,j)=0;

                if(S.at<uchar>(i,j)>S_min&&S.at<uchar>(i,j)<S_max)
                    frIn.at<uchar>(i,j)=255;
                else
                    frIn.at<uchar>(i,j)=0;
                if(V.at<uchar>(i,j)>V_min&&V.at<uchar>(i,j)<V_max)
                    frIn.at<uchar>(i,j)=255;
                else
                    frIn.at<uchar>(i,j)=0;
            }
        }
        imshow("colorImage", frIn);

        /*cout<<red<<endl;
        cout<<green<<endl;
        cout<<blue<<endl;
        cout<<yellow<<endl;*/

        //画统计直方图
        count(out);

        int histHeight = 256;

        Mat histImage1 = Mat::zeros(histHeight, 20*10, CV_8UC3);

        rectangle(histImage1, Point(0, histHeight - 1), Point(19, histHeight - redN),
                  Scalar((255,255,255)-(255, 0, 0)));
        rectangle(histImage1, Point(20, histHeight - 1), Point(39,histHeight - orangeN),
                  Scalar(255, 125, 0));
        rectangle(histImage1, Point(40, histHeight - 1), Point(59, histHeight - yellowN),
                  Scalar(255, 255, 0));
        rectangle(histImage1, Point(60, histHeight - 1), Point(79,histHeight - greenN),
                  Scalar(0, 255, 0));
        rectangle(histImage1, Point(80, histHeight - 1), Point(99, histHeight - cyanN),
                  Scalar(0, 255, 255));
        rectangle(histImage1, Point(100, histHeight - 1), Point(119,histHeight - blueN),
                  Scalar(0, 0, 255));
        rectangle(histImage1, Point(120, histHeight - 1), Point(139, histHeight - purpleN),
                  Scalar(255, 0, 255));
        rectangle(histImage1, Point(140, histHeight - 1), Point(159,histHeight - blackN),
                  Scalar(255, 0, 0));
        rectangle(histImage1, Point(160, histHeight - 1), Point(179, histHeight - graN),
                  Scalar(255, 0, 0));
        rectangle(histImage1, Point(180, histHeight - 1), Point(199,histHeight - whiteN),
                  Scalar(255, 0, 0));

        imshow("RGB", histImage1);







        Mat img;
        threshold(gray, img, 50, 255, THRESH_OTSU);//二值化图像
        imshow("2value", img);

        vector< vector<Point> >contours;
        vector<Vec4i> hierarchy;
        findContours(img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
        Mat resultImage = Mat ::zeros(img.size(),CV_8U);
        drawContours(resultImage, contours, -1, Scalar(255, 0, 255));
        imshow("Contours Image",resultImage);


        vector< vector<Point> > contours_ploy(contours.size());
        vector<Rect> rects_ploy(contours.size());
        vector<Point2f> circle_centers(contours.size());
        vector<float> circle_radius(contours.size());
        vector<RotatedRect> RotatedRect_ploy;
        vector<RotatedRect> ellipse_ploy;

        for (size_t i = 0; i< contours.size(); i++)
        {
            approxPolyDP(contours[i], contours_ploy[i], 5, true);
            rects_ploy[i] = boundingRect(contours_ploy[i]);
            minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);

            if (contours_ploy[i].size() >5)
            {
                RotatedRect temp1 = minAreaRect(contours_ploy[i]);
                RotatedRect_ploy.push_back(temp1);

                RotatedRect temp2 = fitEllipse(contours_ploy[i]);
                ellipse_ploy.push_back(temp2);
            }
        }



        Mat draw_rect(img.size(), img.type(), Scalar::all(0)),
            draw_rotateRect(img.size(), img.type(), Scalar::all(0)),
            draw_circle(img.size(), img.type(), Scalar::all(0)),
            draw_ellipse(img.size(), img.type(), Scalar::all(0));

        RNG rng(12345);
        for (size_t i = 0; i<contours.size(); i++)
        {
            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
            rectangle(draw_rect, rects_ploy[i], color, 1, 8);
            circle(draw_circle, circle_centers[i], circle_radius[i], color, 1, 8);
        }
        imshow("draw_rect", draw_rect);
        imshow("draw_circle", draw_circle);

        Point2f pot[4];
        for (size_t i = 0; i<ellipse_ploy.size(); i++)
        {
            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
            ellipse(draw_ellipse, ellipse_ploy[i], color, 1, 8);

            RotatedRect_ploy[i].points(pot);
            for(int j=0; j<4; j++)
            {
                line(draw_rotateRect, pot[j], pot[(j+1)%4], color);
            }
        }
        imshow("draw_ellipse", draw_ellipse);
        imshow("draw_rotateRect", draw_rotateRect);

        Mat img_rec=out(rects_ploy[20]);//截取矩形区域
        imshow("img_rec",img_rec);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
