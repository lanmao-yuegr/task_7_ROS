#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>

using namespace cv;
using namespace std;

float Point_Distance_2(Point2f p1, Point2f p2);
float Point_Distance_3(Point3f p1, Point3f p2);
RotatedRect Rect_Rotate(RotatedRect rect);

//定义结构：包含轮廓总集、选定轮廓的序号；
typedef struct ContoursRequired
{
    vector<vector<Point>> contours;
    int LightBar_1;
    int LightBar_2;
} ContoursRequired;

//定义结构：包含输出的图像、装甲板角点的像素坐标、装甲板中心的像素坐标；
typedef struct ImageAndPoint
{
    Mat Image;
    vector<Point2f> ImagePoint;
    Point2f Center;
} ImageAndPoint;

//定义探测类
class Detection
{
public:
    Detection();
    ~Detection();
    // 预处理：转变为二值图像；
    Mat PreCompile(Mat Image, String S);
    // 获得所需的轮廓：返回 contours 和轮廓序号 LightBar_1、LightBar_2;
    ContoursRequired GetRequiredContours(Mat Image);
    // 选择装甲板的四个角点；
    vector<Point2f> ChoosePoint(Point2f vertices_1[4], Point2f vertices_2[4]);
    //返回装甲板中心点；
    Point2f GetCenter(vector<Point2f> vertice);
    // 画出装甲板的四个角点和中心点
    ImageAndPoint DrawingImage(Mat Image, Mat Image_Two_Value, ContoursRequired CN);
};