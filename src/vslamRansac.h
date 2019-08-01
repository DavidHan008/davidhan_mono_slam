//
// Created by davidhan on 19-8-1.
//

#ifndef DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H
#define DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H

#include <opencv2/opencv.hpp>

class vslamRansac {

    cv::Mat frame, old_frame;//定义当前帧和上一帧
public:
    void captureNewFrame(cv::Mat newFrame);
    void captureNewFrame(cv::Mat newFrame, double time_stamp);
    void findNewFeatures(int num = -1);
    cv::Mat returnImageDrawed();

    void predict(float v_x = 0, float w_z = 0);
    void update(float v_x = 0, float w_z = 0);



};


#endif //DAVIDHAN_MONO_SLAM_VSLAMRANSAC_H
