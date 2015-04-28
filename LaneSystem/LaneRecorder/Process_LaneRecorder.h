//
//  Process.h
//  LaneRecorder1.1
//
//  Created by LI XUANPENG on 10/13.
//  Copyright (c) 2012 ESIEE-Amiens. All rights reserved.
//

#ifndef LaneRecorder_Process_h
#define LaneRecorder_Process_h

#include "../LaneDetector/DetectLanes.h"
#include "../LaneDetector/TrackLanes.h"
#include "../LaneDetector/LaneDetectorTools.h"
#include "../LaneDetector/GenerateLaneIndicators.h"
#include "CameraConfigure.h"       

#include <deque>
#include <stdio.h>
#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
#include <cassert>
#include <iomanip>

#include <dirent.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

//!IPC
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/ipc.h>
#include "errno.h"

namespace LaneRecorder{
    const int    NUM_LANE_FEATURES  = 17;
    const std::string laneFeatureName[NUM_LANE_FEATURES] = 
    {
        "Frame", 
        "LO", "LATSD", "LATSD_b", 
        "LATMEAN", "LATMEAN_b", 
        "LANEDEV", "LANEDEV_b",
        "LANEX", 
        "TLC", "TLC_2s","TLCF_2s", "TLC_halfs", "TLCF_halfs", "TLC_min",
        "execTime", "pastTime"
    };
    
     void ProcessLaneImage(cv::Mat &laneMat, 
                          const LaneDetector::LaneDetectorConf &laneDetectorConf, 
                          cv::KalmanFilter &laneKalmanFilter, 
                          cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx, 
                          std::vector<cv::Vec2f> &hfLanes, 
                          std::vector<cv::Vec2f> &lastHfLanes, 
                          double & lastLateralOffset,
                          double &lateralOffset, int &isChangeLane,
                          int &detectLaneFlag,  const int &idx, double &execTime, 
                          std::vector<cv::Vec2f> &preHfLanes, int &changeDone,
                          const double &YAW_ANGLE, const double &PITCH_ANGLE); 
    
    
    void InitRecordData(std::ofstream &file, const char* fileName, const std::string *strName, const int &elemNum);
    
    void RecordLaneFeatures(std::ofstream &file, const LaneDetector::LaneFeature &laneFeatures, 
                        const double &execTime, const double &pastTime);
    
    void CodeMsg( const LaneDetector::LaneFeature &laneFeatures, char *str);
}
#endif
