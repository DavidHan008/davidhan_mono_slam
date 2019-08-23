//
// Created by davidhan on 19-8-1.
//

#include "vslamRansac.h"

void vslamRansac::captureNewFrame(cv::Mat newFrame){
    //调整图像的大小
   // std::cout<<"newFrame.size().width/scale:"<<newFrame.size().width/scale<<std::endl;
    cv::resize(newFrame,newFrame,cv::Size(newFrame.size().width, newFrame.size().height));
    old_frame=frame.clone();
    originalImg=newFrame.clone();
    drawedImg=newFrame.clone();
    //获取第一帧的图像
    cv::cvtColor( newFrame, frame, CV_BGR2GRAY );//并转成灰度图
}


void vslamRansac::captureNewFrame(cv::Mat newFrame, double time_stamp){

    if (old_ts > 0) {
        dT = (time_stamp - old_ts);
        std::cout << "my Dt = " << dT*1000 << std::endl;
    }

    old_ts = time_stamp;
    captureNewFrame(newFrame);
}
void vslamRansac::findNewFeatures(int num) {

    std::cout<<"提取第一帧图像的特征"<<std::endl;
    if (num <= 0) num = nInitFeatures;

    int winSize = 2*windowsSize+1;
    cv::Mat mask = cv::Mat(frame.size(),CV_8UC1 );
    mask.setTo(cv::Scalar(0));
    int estrem = windowsSize;
    cv::Mat(mask, cv::Rect(estrem, estrem,mask.size().width-2*estrem, mask.size().height - 2*estrem)).setTo(cv::Scalar(255));



    for (int i = 0; i < patches.size(); i++) {
        if (patches[i].center.x > windowsSize &&
            patches[i].center.y > windowsSize &&
            patches[i].center.x < mask.size().width - windowsSize &&
            patches[i].center.y < mask.size().height - windowsSize)
        {
            int x = (patches[i].center.x-winSize/2 > 0 ?
                     patches[i].center.x-winSize/2 : 0);
            int y = (patches[i].center.y-winSize/2 > 0 ?
                     patches[i].center.y-winSize/2 : 0);
            int width = winSize -
                        (patches[i].center.x+winSize/2 <= frame.size().width ?
                         0 : frame.size().width - patches[i].center.x-winSize/2);

            int height = winSize -
                         (patches[i].center.y+winSize/2 <= frame.size().height ?
                          0 : frame.size().height - patches[i].center.y-winSize/2);

            cv::Rect roi = cv::Rect(x,y, width, height);
            cv::Mat(mask, roi).setTo(cv::Scalar(0));

        }
    }
    std::vector<cv::Point2f> features;


    cv::goodFeaturesToTrack(frame,features,num,0.01f,5,mask);

    for (int i = 0; i < features.size(); i++) {
        cv::Point2f newFeature = cv::Point2f(features[i].x, features[i].y);
        this->addFeature(newFeature);
    }

    //ROS_INFO("Found %d Feature[s]", features.size());


}
cv::Mat vslamRansac::returnImageDrawed() {

    return drawedImg.clone();

}
void vslamRansac::predict(float v_x, float w_z) {


}
void vslamRansac::update(float v_x, float w_z) {

}

int vslamRansac::addFeature(cv::Point2f pf) {
    if (pf.x > windowsSize/2 && pf.y > windowsSize/2 && pf.x < frame.size().width - windowsSize/2 && pf.y < frame.size().height - windowsSize/2) {

        int pos = mu.size();
        Patch newPat(cv::Mat(frame, cv::Rect(pf.x-windowsSize/2, pf.y-windowsSize/2, windowsSize,windowsSize)), pf, pos);
        patches.push_back(newPat);


        Vector2f hd;
        hd << (float)pf.x, (float)pf.y;

        Vector3f r = mu.segment<3>(0);
        Vector4f q = mu.segment<4>(3);

        MatrixXf Jac_hCHd;
        Vector3f hC = cam.UndistortAndDeproject(hd, Jac_hCHd);

        Matrix3f Rot = quat2rot(q);

        Vector3f hW = Rot*hC;

        float hx = hW(0);
        float hy = hW(1);
        float hz = hW(2);

        float ro = config.rho_0;


        float theta = atan2(hx,hz);
        float phi = atan2(-hy, sqrt(hx*hx+hz*hz));

        // Updating state and Sigma

        VectorXf f(6);
        f << r, theta, phi, ro;
        mu = Concat(mu,f);


        int nOld = Sigma.rows();
        MatrixXf Js = MatrixXf::Zero(nOld+6, nOld+3);
        Js.block(0,0,nOld,nOld) = MatrixXf::Identity(nOld,nOld);
        Js.block<3,3>(nOld,0) = MatrixXf::Identity(3,3);




        MatrixXf Jf_hW = d_f_d_hW(hW);
        MatrixXf J_hW_q = dRq_times_d_dq(q,hC);

        Js.block<6,4>(nOld,3) = Jf_hW*J_hW_q;
        Js.block<6,2>(nOld,nOld) = Jf_hW*Rot*Jac_hCHd;
        Js.block<6,1>(nOld, nOld+2) << 0,0,0,0,0,1;

        MatrixXf S = sigma_pixel_2*MatrixXf::Identity(nOld+3, nOld+3);
        S.block(0,0, nOld, nOld) = Sigma;

        S(nOld + 2, nOld + 2) =  config.sigma_rho_0;

        Sigma = Js*S*Js.transpose();
        return 1;
    }
    return 0;
}