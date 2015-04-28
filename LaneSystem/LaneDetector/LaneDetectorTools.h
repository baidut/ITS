//
//  LaneDetectorTools.h
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 06/06/13.
//  Copyright (c) 2013 ESIEE-Amiens. All rights reserved.
//

#ifndef LaneDetector_LaneDetectorTools_h
#define LaneDetector_LaneDetectorTools_h

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdlib.h>

namespace LaneDetector {
    
    
    void PrintMat(const cv::Mat &mat);
    
    void multiImShow(const std::vector<cv::Mat> &mat, const std::vector<std::string> &winname, cv::Mat &multiMat);
    
    void imShowSub(const std::string &winname, const cv::Mat &mat, 
                   const int &Cols, const int &Rows, const int &winPos);
    
    
}

#endif
