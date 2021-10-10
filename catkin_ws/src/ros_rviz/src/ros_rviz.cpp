#include "ros_rviz.h"


//函数：求二维坐标点的距离；
float Point_Distance_2(Point2f p1, Point2f p2)
{
    float Result = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return Result;
}

//函数：求三维坐标点的距离；
float Point_Distance_3(Point3f p1, Point3f p2)
{
    float Result = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    return Result;
}

//函数：旋转矩形，使矩形的 width 对应较短的边、height 对应较长的边、angle旋转90度
RotatedRect Rect_Rotate(RotatedRect rect)
{
    if (rect.size.width > rect.size.height)
    {
        float tran_tmp = rect.size.width;
        rect.size.width = rect.size.height;
        rect.size.height = tran_tmp;
        rect.angle = rect.angle - 90;
    }
    return rect;
}

Detection::Detection()
{
    //cout << "Object is being created" << endl;
}

Detection::~Detection()
{
    //cout << "Object was destoryed" << endl;
}

Mat Detection::PreCompile(Mat Image, String S)
{
    //二值图片转换的阈值
    float c;
    if (S == "装甲板_1.avi")
    {
        c = 80;
    }
    else if (S == "装甲板_2.avi")
    {
        c = 50;
    }

    //获得灰度图像
    cvtColor(Image, Image, CV_BGR2GRAY);
    //获得二值图像
    for (int i = 0; i < Image.rows; i++)
    {
        for (int j = 0; j < Image.cols; j++)
        {
            Scalar tmp = Image.at<uchar>(i, j);
            if (tmp[0] > c)
            {
                tmp[0] = 255;
            }
            else
            {
                tmp[0] = 0;
            }
            Image.at<uchar>(i, j) = tmp[0];
        }
    }

    //进行开操作去除小白点
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(Image, Image, 2, kernel, Point(-1, -1), 1);
    //返回预处理后的图像
    return Image;
}

ContoursRequired Detection::GetRequiredContours(Mat Image)
{
    //获得轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(Image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    //选择轮廓
    //lightBar 存储两选定轮廓的序号
    int lightBar_1, lightBar_2, break_tmp;
    lightBar_1 = 0;
    lightBar_2 = 0;
    break_tmp = -1;
    ContoursRequired CN;
    vector<Point> contour_tmp;
    vector<RotatedRect> minRect(contours.size());
    RotatedRect rect_tmp;
    RotatedRect rect_tmp_rotated;
    vector<RotatedRect> minRect_rotated(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        minRect[i] = minAreaRect(contours[i]);
    }
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours.size(); j++)
        {
            if (j == i)
            {
                continue;
            }
            //contour_tmp 存储两灯条轮廓（假设）的并集、rect_tmp 存储装甲板（假设）轮廓最小矩形
            contour_tmp = contours[i];
            contour_tmp.insert(contour_tmp.end(), contours[j].begin(), contours[j].end());
            rect_tmp = minAreaRect(contour_tmp);
            //将各轮廓的最小矩形用函数 Rect_Rotate 进行旋转
            rect_tmp_rotated = Rect_Rotate(rect_tmp);
            minRect_rotated[i] = Rect_Rotate(minRect[i]);
            minRect_rotated[j] = Rect_Rotate(minRect[j]);
            //两轮廓相应的最小矩形的角度之差、单个轮廓最小矩形的长宽之差、
            float tmp = minRect_rotated[i].angle - minRect_rotated[j].angle;
            float diff_1 = minRect_rotated[i].size.width - minRect_rotated[i].size.height;
            float diff_2 = minRect_rotated[j].size.width - minRect_rotated[j].size.height;
            //根据两轮廓相应的最小矩形的角度之差、单个轮廓最小矩形的长宽之差、
            //              装甲板最小矩形与单个轮廓最小矩形是否接近垂直 来选择所需的两个轮廓；
            if ((fabs(tmp) < 10 || fabs(tmp) > 80) && fabs(diff_1) > 20 && fabs(diff_2) > 20 && fabs(minRect_rotated[i].angle - rect_tmp_rotated.angle) > 80 && fabs(minRect_rotated[i].angle - rect_tmp_rotated.angle) < 100)
            {
                lightBar_1 = i;
                lightBar_2 = j;
                break_tmp = 0;
                break;
            }
        }
        if (break_tmp == 0)
        {
            break;
        }
    }
    if (lightBar_1 != lightBar_2)
    {
        CN.contours = contours;
        CN.LightBar_1 = lightBar_1;
        CN.LightBar_2 = lightBar_2;
        //返回的是轮廓总集+轮廓序号；
        return CN;
    }
    ContoursRequired NullNum;
    vector<Point> NullPoint = {Point(0, 0)};
    NullNum.contours.push_back(NullPoint);
    NullNum.LightBar_1 = NullNum.LightBar_2 = 0;
    return NullNum;
}

vector<Point2f> Detection::ChoosePoint(Point2f vertices_1[4], Point2f vertices_2[4])
{
    //中间结果：存储灯条端点的像素坐标；
    Point2f vertice[4];
    //输出结果：以左下角的角点为起点，顺时针排序的角点像素坐标
    vector<Point2f> Vertice; //选取较短边的中点作为角点
    if (Point_Distance_2(vertices_1[0], vertices_1[1]) < Point_Distance_2(vertices_1[1], vertices_1[2]))
    {
        vertice[0].x = 0.5 * (vertices_1[0].x + vertices_1[1].x);
        vertice[0].y = 0.5 * (vertices_1[0].y + vertices_1[1].y);
        vertice[1].x = 0.5 * (vertices_1[2].x + vertices_1[3].x);
        vertice[1].y = 0.5 * (vertices_1[2].y + vertices_1[3].y);
    }
    else
    {
        vertice[0].x = 0.5 * (vertices_1[1].x + vertices_1[2].x);
        vertice[0].y = 0.5 * (vertices_1[1].y + vertices_1[2].y);
        vertice[1].x = 0.5 * (vertices_1[0].x + vertices_1[3].x);
        vertice[1].y = 0.5 * (vertices_1[0].y + vertices_1[3].y);
    }
    if (Point_Distance_2(vertices_2[0], vertices_2[1]) < Point_Distance_2(vertices_2[1], vertices_2[2]))
    {
        vertice[2].x = 0.5 * (vertices_2[0].x + vertices_2[1].x);
        vertice[2].y = 0.5 * (vertices_2[0].y + vertices_2[1].y);
        vertice[3].x = 0.5 * (vertices_2[2].x + vertices_2[3].x);
        vertice[3].y = 0.5 * (vertices_2[2].y + vertices_2[3].y);
    }
    else
    {
        vertice[2].x = 0.5 * (vertices_2[1].x + vertices_2[2].x);
        vertice[2].y = 0.5 * (vertices_2[1].y + vertices_2[2].y);
        vertice[3].x = 0.5 * (vertices_2[0].x + vertices_2[3].x);
        vertice[3].y = 0.5 * (vertices_2[0].y + vertices_2[3].y);
    }
    //排序：根据像素坐标的相对位置排序；
    //首先选择最底下的两个角点；
    int tmp[2] = {-1, -1};
    for (int i = 0; i < 2; i++)
    {
        if (vertice[i * 2].y > vertice[i * 2 + 1].y)
        {
            tmp[i] = i * 2;
        }
        else
        {
            tmp[i] = i * 2 + 1;
        }
    }
    //然后判断两点的左右关系，选择左边的角点为第一个点，按照相对位置以顺时针排序压入 Vertice 中；
    if (vertice[tmp[0]].x < vertice[tmp[1]].x)
    {
        Vertice.push_back(vertice[tmp[0]]);
        Vertice.push_back(vertice[(tmp[0] + 1) % 2]);
        Vertice.push_back(vertice[(tmp[1] - 1) % 2 + 2]);
        Vertice.push_back(vertice[tmp[1]]);
    }
    else
    {
        Vertice.push_back(vertice[tmp[1]]);
        Vertice.push_back(vertice[(tmp[1] - 1) % 2 + 2]);
        Vertice.push_back(vertice[(tmp[0] + 1) % 2]);
        Vertice.push_back(vertice[tmp[0]]);
    }
    //返回的是排好序的装甲板角点的像素坐标；
    return Vertice;
}

Point2f Detection::GetCenter(vector<Point2f> vertice)
{
    //根据矩形两对角线相交于中点的原理确定中点像素坐标；
    Point2f P;
    float k1 = (vertice[0].y - vertice[2].y) / (vertice[0].x - vertice[2].x);
    float k2 = (vertice[1].y - vertice[3].y) / (vertice[1].x - vertice[3].x);
    float b1 = vertice[0].y - k1 * vertice[0].x;
    float b2 = vertice[1].y - k2 * vertice[1].x;
    P.x = (b2 - b1) / (k1 - k2);
    P.y = k1 * P.x + b1;
    return P;
}

ImageAndPoint Detection::DrawingImage(Mat Image, Mat Image_Two_Value, ContoursRequired CR)
{
    //选择角点
    Point2f vertices_1[4], vertices_2[4];
    vector<Point2f> vertice;
    RotatedRect rect_1 = minAreaRect(CR.contours[CR.LightBar_1]);
    RotatedRect rect_2 = minAreaRect(CR.contours[CR.LightBar_2]);
    rect_1.points(vertices_1);
    rect_2.points(vertices_2);
    vertice = ChoosePoint(vertices_1, vertices_2);

    //绘制角点以及中心点
    Scalar color_1(0, 0, 255);
    Scalar color_2(0, 255, 0);
    for (int i = 0; i < 4; i++)
    {
        circle(Image, vertice[i], 5, color_1, 2, 8, 0);
        if (i == 0 || i == 1)
        {
            line(Image, vertice[i], vertice[(i + 2) % 4], color_2, 3, 8, 0);
        }
    }
    Point2f R_center = GetCenter(vertice);
    circle(Image, R_center, 5, color_1, 2, 8, 0);

    //返回的是处理过后的图像、角点像素坐标、中心点像素坐标；
    ImageAndPoint IAP;
    IAP.Image = Image;
    IAP.ImagePoint = vertice;
    IAP.Center = R_center;
    return IAP;
}