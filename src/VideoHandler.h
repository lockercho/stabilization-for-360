//
//  VideoHandler.hpp
//  stabilization-for-360
//


#ifndef VideoHandler_h
#define VideoHandler_h

#include "Tracker.h"
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include "Equirect2Cubic.h"

using namespace std;

#define FRAME_BUF_SIZE 100

struct R {
    std::vector<std::vector<Eigen::Quaternion<double> > > rot;
    int start;
    int end;
    R(std::vector<std::vector<Eigen::Quaternion<double> > > r, int s, int e) {
        rot = r;
        start = s;
        end = e;
    }
};

class VideoHandler {
    std::string filename = "walk_short.mp4";
    cv::VideoCapture capture;
    bool opened_capture = false;
    cv::Mat frames[FRAME_BUF_SIZE];
    std::vector<R> rotationData;
    
    cv::Mat RGB, YUV;
    Tracker tracker;
    FILE * result_video = NULL;
    int nFrames = 0;
    int nProcessedFrames = 0;
    int lastR = 0;
    
    std::vector<R> allRotations;
    
public:
    std::vector<cv::Mat> tmpMat;
    VideoHandler(std::string filename) {
        this-> filename = filename;
    }
    
    void start();
    int getFrameLength() {
        return (int) nProcessedFrames;
    }
    
    bool isFrameOk(int index) {
        return (int) nProcessedFrames > index;
    }
    
    cv::Mat * getFrame(int index) {
        return &frames[index % FRAME_BUF_SIZE];
    }
    
    std::vector<std::vector<Eigen::Quaternion<double> > > getRotation(int index);
};

#endif /* VideoHandler_h */
