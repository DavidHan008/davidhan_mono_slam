//
// Created by davidhan on 19-8-1.
//

#include <iostream>

//添加ROS相关头文件
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include "vslamRansac.h"

//添加命名空间
using namespace cv;
using namespace geometry_msgs;


cv_bridge::CvImageConstPtr final_cv_ptr;//最后输出数据的图像
image_transport::Publisher  image_pub;//用ROS的形式发布数据
vslamRansac slam;//实例化SLAM处理数据的结果
bool first=true;//

//定义全局变量用于存储数据

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //1、检查图像的格式，是否为RGB8
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception e)
    {
        ROS_ERROR("图像格式问题：%s",e.what());
        return;
    }
    //2、定义一帧的图像，我们把他显示出来
    cv::Mat mid_frame;//中间的过程帧
    int scale=1;
    cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(cv_ptr->image.size().width/scale,cv_ptr->image.size().height/scale));

    //经历了种种算法

    // 特征点的提取
    if(first)
    {
        slam.captureNewFrame(cv_ptr->image);
        slam.findNewFeatures();
        mid_frame=slam.returnImageDrawed();
    }
    else{
        slam.captureNewFrame(cv_ptr->image,msg->header.stamp.toSec());
        //输入的时间戳是以图像序列的时间戳为准，那么这个时间戳是一个软同步
        std::cout<<"msg->timestamp:"<<msg->header.stamp.toSec()<<std::endl;
        slam.update();
        slam.predict();
        mid_frame=slam.returnImageDrawed();

    }



   // final_cv_ptr=cv_ptr;
    image_pub.publish(final_cv_ptr->toImageMsg());

    // 用于测试图像是否订阅
     std::cout<<"图像订阅发布成功"<<std::endl;


};





int main(int argc,char **argv)
{

    //订阅这个节点
    ros::init(argc, argv, "davidhan_mono_slam");
    ros::NodeHandle node;
    image_transport::Subscriber image_sub;//订阅图像
    image_transport::ImageTransport it(node);
    image_sub= it.subscribe("/usb_cam/image_raw",1,imageCb);
    image_pub = it.advertise("/monoslam/imgproc", 1);
    ros::spin();

    //使用ros的方式发布出去相关的msg

    return 0;
}
