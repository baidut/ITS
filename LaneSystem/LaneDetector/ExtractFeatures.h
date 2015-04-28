//
//  ExtractFeatures.h
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 01/07/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

#ifndef LaneDetector_ExtractFeatures_h
#define LaneDetector_ExtractFeatures_h

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace LaneDetector{	
    //! The methods exist in the API reference:
    //! imgproc: conerHarris, goodFeaturesToTrack
    //! features2d: FAST, MSER, ORB, BRISK, Freak
    //! gpu: FAST etc.
    //! nonfree: SIFT, SURF
    //! legacy: RandomizedTree
    //! ocl: BruteForceMatcher, HOGDescriptor
    void ExtractFeatures(const cv::Mat &mat, 
                         const std::string &detectorMethod, std::vector<cv::KeyPoint> &keypoints,
                         const std::string &extractorMethod, cv::Mat &descriptor, cv::Mat &outMat);

    void MatchFeatures(const std::string &matcherMethod,
                       const cv::Mat &descriptor1,
                       const cv::Mat &descriptor2,
                       std::vector< cv::DMatch > &good_matches);
    
    
    
    //! The function extracts the HOG descriptors.
    void ExtractHOG(const cv::Mat &mat);

    void GetVisualHOG(const cv::Mat &mat, const int &pixels, std::vector<float> &descriptors, cv::Mat &visualHOG);
    
    
    void HOG_trans(const cv::Mat &imageIPM);
}

#endif
