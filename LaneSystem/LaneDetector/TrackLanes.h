//
//  TrackLanes.h
//  LaneDetector1.2
//
//  Created by LI XUANPENG on 09/13.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef LaneDetector_TrackLanes_h
#define LaneDetector_TrackLanes_h

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "LaneDetectorTools.h"
#include "DetectLanes.h"
#include "FittingCurve.h"

namespace LaneDetector{	
    /********************************************************************/
    /* Kalman Filter */
    /********************************************************************/
    void InitLaneKalmanFilter(cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx);
    
    /*
	 * This function tracks lanes in the input image. 
     * Input is the parameters from hough transform: rho and theta.
     * Output is the predicted lane. 
	 * \param image the input image
	 * \param kalman filter
     * \param process noise
     * \param measurement
     * \param lane detected in current frame
     * \param lane detected from last frame
     * \param lane tracked
	 */
	void TrackLanes_KF(const cv::Mat &laneMat, 
                       cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, 
                       const std::vector<cv::Vec2f> &hfLanes, const std::vector<cv::Vec2f> &lastHfLanes, 
                       std::vector<cv::Vec2f> &preHfLanes, std::vector<cv::Vec2f> &postHfLanes,
                       const double &PITCH_ANGLE);
    
    /********************************************************************/
    /* Particle Filter */
    /********************************************************************/
    //! struct of particles
    typedef struct particle {
        double weight;
        double a0;
        double a1;
        double a2;
    }PARTICLE_LANE;
    
    bool particle_cmp(const PARTICLE_LANE &p1, const PARTICLE_LANE &p2);
    
    void TrackLanes_Particle(const cv::Mat &ipmMat, 
                             const LaneDetectorConf &laneDetectorConf,
                             const int &samplingNum, 
                             cv::Mat &leftCoefs, cv::Mat &rightCoefs,
                             std::vector<cv::Point2d> &leftSampledPoints, 
                             std::vector<cv::Point2d> &rightSampledPoints, 
                             double &laneWidth);
}

#endif
