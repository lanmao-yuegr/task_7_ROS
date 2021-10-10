#include "ros_rviz.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

//相机内外参、装甲板长宽值
Mat Camera_Matrix = (Mat_<float>(3, 3) << 1285.3517927598091, 0., 319.44768628958542, 0., 1279.2339468697937, 239.29354061292258, 0., 0., 1.);
Mat _Distortion_Coefficients = (Mat_<float>(5, 1) << -0.63687295852461456, -1.9748008790347320, 0.030970703651800782, 0.0021944646842516919, 0.);
float Armor_height = 67.5;
float Armor_width = 26.5;


int main(int argc, char **argv)
{
    Mat frame;
    namedWindow("frame", WINDOW_AUTOSIZE);
    Detection Armor;

    // 读取视频文件
    String VideoName = "装甲板_1.avi";
    VideoCapture capture(VideoName);
    if (!capture.isOpened())
    {
        printf("无法打开视频文件！");
        return -1;
    }

    //配置 marker
    ros::init(argc, argv, "Camera_And_Plate");
    ros::NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    ros::Rate r(25);

    //读取每一帧
    while (capture.read(frame))
    {
        Mat fpsImage;
        ImageAndPoint IAP;
        ContoursRequired CR;
        fpsImage = frame;

        //绘制装甲板角点与中心点
        fpsImage = Armor.PreCompile(fpsImage, VideoName);
        CR = Armor.GetRequiredContours(fpsImage);
        IAP = Armor.DrawingImage(frame, fpsImage, CR);
        fpsImage = IAP.Image;

        //求的旋转矩阵与平移向量
        vector<Point3f> ObjectPoint;
        ObjectPoint.push_back(Point3f(0, 0, 0));
        ObjectPoint.push_back(Point3f(0, 0, 26.5));
        ObjectPoint.push_back(Point3f(67.5, 0, 26.5));
        ObjectPoint.push_back(Point3f(67.5, 0, 0));
        vector<Point2f> ImagePoint = IAP.ImagePoint;
        Mat rvec = (Mat_<float>(3, 1) << 0, 0, 0);
        Mat tvec = (Mat_<float>(3, 1) << 0, 0, 0);
        Mat rotM = (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        solvePnP(ObjectPoint, ImagePoint, Camera_Matrix, _Distortion_Coefficients, rvec, tvec, false, 0);
        Rodrigues(rvec, rotM);
        const float PI = 3.141592653;
        float thetaz = atan2(rotM.at<float>(1, 0), rotM.at<float>(0, 0)) / PI * 180;
        float thetay = atan2(-1 * rotM.at<float>(2, 0), sqrt(rotM.at<float>(2, 1) * rotM.at<float>(2, 1) + rotM.at<float>(2, 2) * rotM.at<float>(2, 2))) / PI * 180;
        float thetax = atan2(rotM.at<float>(2, 1), rotM.at<float>(2, 2)) / PI * 180;
        cout << "Z轴欧拉角=" << thetaz << endl;
        cout << "Y轴欧拉角=" << thetay << endl;
        cout << "X轴欧拉角=" << thetax << endl;

        //测距
        Point3f test_Camera_Center;
        Mat test_world = (Mat_<float>(4, 1) << 33.75, 0, 13.25, 1);
        Mat test_rotated = (Mat_<float>(3, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                test_rotated.at<float>(i, j) = rotM.at<float>(i, j);
            }
        }
        for (int i = 0; i < 3; i++)
        {
            test_rotated.at<float>(i, 3) = tvec.at<float>(i, 0);
        }
        Mat test_Camera = test_rotated * test_world;
        test_Camera_Center.x = test_Camera.at<float>(0, 0);
        test_Camera_Center.y = test_Camera.at<float>(1, 0);
        test_Camera_Center.z = test_Camera.at<float>(2, 0);
        float Center_Distance = Point_Distance_3(Point3f(0, 0, 0), test_Camera_Center);

        //显示欧拉角以及装甲板与相机镜头的距离
        String distance = to_string(Center_Distance / 1000);
        distance.append("m");
        String Z_angle = "Z_angle=";
        Z_angle.append(to_string(thetaz));
        String Y_angle = "Y_angle=";
        Y_angle.append(to_string(thetay));
        String X_angle = "X_angle=";
        X_angle.append(to_string(thetax));
        putText(fpsImage, distance, Point(100, 100), FONT_HERSHEY_PLAIN, 1.0, 255, 2);
        putText(fpsImage, Z_angle, Point(100, 120), FONT_HERSHEY_PLAIN, 1.0, 255, 2);
        putText(fpsImage, Y_angle, Point(100, 140), FONT_HERSHEY_PLAIN, 1.0, 255, 2);
        putText(fpsImage, X_angle, Point(100, 160), FONT_HERSHEY_PLAIN, 1.0, 255, 2);
        cout << "相机镜头与装甲板中心的距离=" << Center_Distance / 1000 << "(m)" << endl;

        //rviz仿真
        tf::Quaternion q;
        visualization_msgs::Marker marker;
        q.setRPY(thetax*PI/180, thetay*PI/180, thetaz*PI/180);
        cout << "q.x=" << q.x() << endl;
        cout << "q.y=" << q.y() << endl;
        cout << "q.z=" << q.z() << endl;
        cout << "q.w=" << q.w() << endl;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = tvec.at<float>(0, 0)/100;
        marker.pose.position.y = tvec.at<float>(1, 0)/100;
        marker.pose.position.z = tvec.at<float>(2, 0)/100;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = 0.675;
        marker.scale.y = 0.1;
        marker.scale.z = 0.265;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish(marker);
        r.sleep();
        //显示处理后的图像
        imshow("frame", fpsImage);
        waitKey(40);
    }
}