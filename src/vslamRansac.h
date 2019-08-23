//
// Created by davidhan on 19-8-1.
//

#ifndef DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H
#define DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "Patch.hpp"

class vslamRansac {

    cv::Mat frame, old_frame;//定义当前帧和上一帧
    cv::Mat originalImg;//用于显示的new frame
    cv::Mat drawedImg;//用于显示的old frame
    int nInitFeatures;
    int windowsSize;
    std::vector<Patch> patches;
    std::vector<Patch> deleted_patches;
    double old_ts;//上一秒
    float dT;//delt
    int scale;

public:
    void captureNewFrame(cv::Mat newFrame);
    void captureNewFrame(cv::Mat newFrame, double time_stamp);
    void findNewFeatures(int num = -1);
    cv::Mat returnImageDrawed();

    void predict(float v_x = 0, float w_z = 0);
    void update(float v_x = 0, float w_z = 0);
    int addFeature(cv::Point2f pf);



};


#endif //DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H
