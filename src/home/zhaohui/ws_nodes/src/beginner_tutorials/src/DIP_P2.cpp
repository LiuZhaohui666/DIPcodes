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
#include<complex.h>
#include<math.h>
using namespace cv;
using namespace std;


//////////////////////滤波//////////////////
// 空域高斯滤波器函数
void Gaussian(Mat *input, Mat *output, double sigma){
    int i,j;
    //Initialize a full value array
    int arr_size=5;//the size of Gaussian normal matrix
    double **array= new double*[arr_size];
    for(i==0;i<arr_size;i++){
        array[i]=new double[arr_size];
    }
    //Gaussian distribution calculation
    int center_i,center_j;
    center_i=center_j=arr_size/2;
    double pi=3.141592653589793;
    double sum=0.0f;
    //Gaussian Function
    for(int i=0;i<arr_size;i++)
    {
        array[i][j]=exp( -(1.0f)* ( ((i-center_i)*(i-center_i)+(j-center_j)*(j-center_j)) / (2.0f*sigma*sigma) ));
        sum+=array[i][j];
    }

    //Normalize
    for(i=0;i<arr_size;i++)
    {
        for(j=0;j<arr_size;j++)
        {
            array[i][j]/=sum;
            //printf(" [%.15f] ", array[i][j]);
        }
        //printf("\n");
    }
    //Part above here within the function is the generation of gaussian cores

    //Part below is processing
    cv::Mat temp=(*input).clone();
    //scan
    int _size=5;
    for (int i = 0; i < (*input).rows; i++) {
        for (int j = 0; j < (*input).cols; j++) {
            if (i > (_size / 2) - 1 && j > (_size / 2) - 1 &&
                i < (*input).rows - (_size / 2) && j < (*input).cols - (_size / 2)) {
                double sum = 0.0;
                for (int k = 0; k < _size; k++) {
                    for (int l = 0;l < _size; l++) {
                        sum += (*input).ptr<uchar>(i-k+(_size/2))[j-l+(_size/2)] * array[k][l];
                    }
                }
                temp.ptr<uchar>(i)[j] = sum;
            }
        }
    }
    (*output) = temp.clone();
}


//未完成的代码
/*double _Complex GU(int image[], int N)
{
    int K = N/2;
    pi=3.141592653589793;
    double _Complex sum=0+0*I;
    for(int x=0;x<K ;x++)
    {
        sum+=image[2*x]*(cos(2*pi*u*x/K)+(-sin(2*pi*u*x/K))*I);
    }
    return sum;
}

double _Complex HU(int image[], int N)
{
    int K = N/2;
    pi=3.141592653589793;
    double _Complex sum=0+0*I;
    for(int x=0;x<K ;x++)
    {
        sum+=image[2*x+1]*(cos(2*pi*u*x/K)+(-sin(2*pi*u*x/K))*I);
    }
    return sum;
}

double _Complex FU(Mat *image,int N)
{
    if (N==2)
    {
        for(;;)
        {

        }
    }
    else
    {
        int N=N/2;
        double _Complex result= FU(image,N);
    }
}*/


// 快速傅里叶变换
/*void fastFuriorTransform(Mat *image) {
    int N = 512;

}*/


// 理想低通滤波器函数
Mat ideal_lbrf_kernel(Mat &scr,float sigma)
{
    Mat ideal_low_pass(scr.size(),CV_32FC1); //，CV_32FC1
    float d0=sigma;//半径D0越小，模糊越大；半径D0越大，模糊越小
    for(int i=0;i<scr.rows ; i++ ){
        for(int j=0; j<scr.cols ; j++ ){
            double d = sqrt(pow((i - scr.rows/2),2) + pow((j - scr.cols/2),2));//分子,计算pow必须为float型
            if (d <= d0){
                ideal_low_pass.at<float>(i,j)=1;
            }
            else{
                ideal_low_pass.at<float>(i,j)=0;
            }
        }
    }
    string name = " idea lowpass";
    imshow(name, ideal_low_pass);
    return ideal_low_pass;
}

// 频率域滤波函数
// src:原图像
// blur:滤波器函数
Mat freqfilt(Mat &scr,Mat &blur)
{
    //***********************DFT*******************
    Mat plane[]={scr, Mat::zeros(scr.size() , CV_32FC1)}; //创建通道，存储dft后的实部与虚部（CV_32F，必须为单通道数）
    Mat complexIm;
    merge(plane,2,complexIm);//合并通道 （把两个矩阵合并为一个2通道的Mat类容器）
    dft(complexIm,complexIm);//进行傅立叶变换，结果保存在自身

    //***************中心化********************
    split(complexIm,plane);//分离通道（数组分离）
//    plane[0] = plane[0](Rect(0, 0, plane[0].cols & -2, plane[0].rows & -2));//这里为什么&上-2具体查看opencv文档
//    //其实是为了把行和列变成偶数 -2的二进制是11111111.......10 最后一位是0
    int cx=plane[0].cols/2;int cy=plane[0].rows/2;//以下的操作是移动图像  (零频移到中心)
    Mat part1_r(plane[0],Rect(0,0,cx,cy));  //元素坐标表示为(cx,cy)
    Mat part2_r(plane[0],Rect(cx,0,cx,cy));
    Mat part3_r(plane[0],Rect(0,cy,cx,cy));
    Mat part4_r(plane[0],Rect(cx,cy,cx,cy));

    Mat temp;
    part1_r.copyTo(temp);  //左上与右下交换位置(实部)
    part4_r.copyTo(part1_r);
    temp.copyTo(part4_r);

    part2_r.copyTo(temp);  //右上与左下交换位置(实部)
    part3_r.copyTo(part2_r);
    temp.copyTo(part3_r);

    Mat part1_i(plane[1],Rect(0,0,cx,cy));  //元素坐标(cx,cy)
    Mat part2_i(plane[1],Rect(cx,0,cx,cy));
    Mat part3_i(plane[1],Rect(0,cy,cx,cy));
    Mat part4_i(plane[1],Rect(cx,cy,cx,cy));

    part1_i.copyTo(temp);  //左上与右下交换位置(虚部)
    part4_i.copyTo(part1_i);
    temp.copyTo(part4_i);

    part2_i.copyTo(temp);  //右上与左下交换位置(虚部)
    part3_i.copyTo(part2_i);
    temp.copyTo(part3_i);

    //*****************滤波器函数与DFT结果的乘积****************
    Mat blur_r,blur_i,BLUR;
    multiply(plane[0], blur, blur_r); //滤波（实部与滤波器模板对应元素相乘）
    multiply(plane[1], blur,blur_i);//滤波（虚部与滤波器模板对应元素相乘）
    Mat plane1[]={blur_r, blur_i};
    merge(plane1,2,BLUR);//实部与虚部合并

    //*********************得到原图频谱图***********************************
    magnitude(plane[0],plane[1],plane[0]);//获取幅度图像，0通道为实部通道，1为虚部，因为二维傅立叶变换结果是复数
    plane[0]+=Scalar::all(1);  //傅立叶变换后的图片不好分析，进行对数处理，结果比较好看
    log(plane[0],plane[0]);    // float型的灰度空间为[0，1])
    normalize(plane[0],plane[0],1,0,CV_MINMAX);  //归一化便于显示
    imshow("Image spectrum",plane[0]);

    idft( BLUR, BLUR);    //idft结果也为复数
    split(BLUR,plane);//分离通道，主要获取通道
    magnitude(plane[0],plane[1],plane[0]);  //求幅值(模)
    normalize(plane[0],plane[0],1,0,CV_MINMAX);  //归一化便于显示
    return plane[0];//返回参数
}

cv::Mat ideal_Low_Pass_Filter(Mat &src, float sigma)
{
    int M = getOptimalDFTSize(src.rows);
    int N = getOptimalDFTSize(src.cols);
    Mat padded;                 //调整图像加速傅里叶变换
    copyMakeBorder(src, padded, 0, M - src.rows, 0, N - src.cols, BORDER_CONSTANT, Scalar::all(0));
    padded.convertTo(padded,CV_32FC1); //将图像转换为flaot型

    Mat ideal_kernel=ideal_lbrf_kernel(padded,sigma);//理想低通滤波器
    Mat result = freqfilt(padded,ideal_kernel);
    imshow("result",result);
    return result;
}


//////////////////////形态学//////////////////
// 膨胀函数
void Dilate(Mat *Src, Mat *Dst){
    Mat temp=(*Src).clone();
    for (int i = 2; i< (*Src).rows-2; i++)
    {
        for (int j = 2; j < (*Src).cols-2; j++)
        {
            if ((*Src).ptr<uchar>(i)[j] == 0 ||
                (*Src).ptr<uchar>(i)[j + 1] == 0 ||
                (*Src).ptr<uchar>(i)[j - 1] == 0||
                (*Src).ptr<uchar>(i+1)[j] == 0||
                (*Src).ptr<uchar>(i-1)[j] == 0||
                (*Src).ptr<uchar>(i)[j + 2] == 0||
                (*Src).ptr<uchar>(i)[j - 2] == 0||
                (*Src).ptr<uchar>(i+2)[j] == 0||
                (*Src).ptr<uchar>(i-2)[j] == 0)
            {
                temp.ptr<uchar>(i)[j] = 0;
            }
        }
    }
    (*Dst) = temp.clone();
}
// 腐蚀函数
void Erode(Mat *Src, Mat *Dst){
    Mat temp=(*Src).clone();
    for (int i = 1; i< (*Src).rows; i++)
    {
        for (int j = 1; j < (*Src).cols; j++)
        {
            if ((*Src).ptr<uchar>(i)[j] == 0 &&
                (*Src).ptr<uchar>(i)[j + 1] == 0 &&
                (*Src).ptr<uchar>(i + 1)[j] == 0&&
                (*Src).ptr<uchar>(i - 1)[j] == 0&&
                (*Src).ptr<uchar>(i)[j-1] == 0)
            {
                temp.ptr<uchar>(i)[j] = 0;
            }
            else
                temp.ptr<uchar>(i)[j] = 255;
        }
    }
    (*Dst) = temp.clone();
}

void GrayToBin(Mat *Src, int threshold)
{
    for(int i=1;i<(*Src).rows-1;i++)
    {
        for(int j=1;j<(*Src).cols-1;j++)
        {
            if((*Src).ptr<uchar>(i)[j]>threshold)
                (*Src).ptr<uchar>(i)[j]=255;
            else
                (*Src).ptr<uchar>(i)[j]=0;
        }
    }
}

int main(int argc, char **argv)
{
    //VideoCapture capture;
    //capture.open(0);//打开zed相机

    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
    ros::NodeHandle n;

    /*if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);*/
    Mat img_1;//read in Gaussian
    Mat img_2;//read in Dilate
    Mat img_3;//read in Erode
    Mat img_4;//read in low_pass
    while (ros::ok())
    {
        Mat frame;//Grey level Gaussian
        Mat frame_2;//Grey level Dilate
        Mat frame_3;//Grey level Erode
        Mat frame_4;//Grey level low_pass

        Mat spatial_domain; //result Gaussian
        Mat p_Dilate;//result Dilate
        Mat p_Erode;//result Erode

        img_1= imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/1.jpg");
        img_2= imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/2.jpg");
        img_3= imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/2.jpg");
        img_4= imread("/home/zhaohui/ws_nodes/src/beginner_tutorials/src/3.jpg");

        /*capture.read(img);
        if(img.empty())
        {
            break;
        }*/
        cvtColor(img_1,frame, CV_BGR2GRAY );  //to Grey level
        cvtColor(img_2,frame_2, CV_BGR2GRAY );  //to Grey level
        cvtColor(img_3,frame_3, CV_BGR2GRAY );  //to Grey level
        cvtColor(img_4,frame_4, CV_BGR2GRAY );  //to Grey level
        GrayToBin(&frame_2,170);//Grey level to Binary
        GrayToBin(&frame_3,170);//Grey level to Binary




        Gaussian(&frame,&spatial_domain,5.0f);// Spatial filter function,parameter:the sigma bigger, the img more smooth and blurred
        //freq_filter();// Frequency domain filter function
        Dilate(&frame_2,&p_Dilate);//Dilate function
        Erode(&frame_3,&p_Erode);//Erode function

        cv::Mat ideal = ideal_Low_Pass_Filter(frame_4, 160);
        ideal = ideal(cv::Rect(0,0, frame_4.cols, frame_4.rows));
        imshow("origin_ideal",frame_4);
        imshow("ideal", ideal);

        imshow("origin_img",frame);
        imshow("p_Binary2",frame_2);
        imshow("p_Binary3",frame_3);

        imshow("spatial_domain",spatial_domain);
        imshow("Dilate",p_Dilate);
        imshow("Erode",p_Erode);

        ros::spinOnce();
        waitKey(5);
    }

    return 0;
}
