//
//  VideoHandler.cpp
//  stabilization-for-360
//

#include "VideoHandler.h"

void VideoHandler::start() {
    if(!capture.open(filename)) {
        cout<<"Video Not Found"<<endl;
        return ;
    }
    capture.set(CV_CAP_PROP_BUFFERSIZE, 3);
    
    while(true) {
        cv::Mat RGB;
        capture >> RGB;  //Read a frame from the video
        
        // Check if the frame has been read correctly or not
        if(RGB.empty()) {
            cout<<"Capture Finished"<<endl;
            fclose(result_video);
            break;
        }
        cvtColor(RGB, RGB, CV_BGR2RGB);
        
        frames[nFrames++ % FRAME_BUF_SIZE] = RGB;
        
        fprintf(stderr, "nFrames: %d\n", (int)nFrames);
//        usleep(16000);
        
        auto cube_face_size = 256;
        
        Equirect2Cubic myTransForm(RGB.cols, RGB.rows, cube_face_size, cube_face_size);
        
        cv::Mat result[6];
        
        for (int i = 0; i<6; i++) {
            result[i] = cv::Mat(cube_face_size, cube_face_size, RGB.type());
            myTransForm.remapWithMap(RGB, result[i], i);
        }
        this->tmpMat.assign(result, result+6);
        
        std::vector<std::vector<Eigen::Quaternion<double> > > rotation = tracker.Track(result);
        if(rotation.size() > 0) {
            // multiply with last rotation
//            if(allRotations.size() > 0) {
//                for(int i=0 ; i<6 ; i++) {
//                    cv::Mat tmp = rotation[i] * allRotations[allRotations.size()-1].rot[i];
//                    rotation[i] = tmp;
//                }
//            }
            allRotations.push_back(R(rotation, nProcessedFrames, nFrames));
            nProcessedFrames = nFrames;
        }
    }
}

std::vector<std::vector<Eigen::Quaternion<double> > > VideoHandler::getRotation(int index) {
    for(int i=0 ; i<allRotations.size() ; i++ ) {
        if(allRotations[i].start <= index && allRotations[i].end > index) {
            return allRotations[i].rot;
        }
    }
    std::vector<std::vector<Eigen::Quaternion<double> > > dummy;
    return dummy;
}
