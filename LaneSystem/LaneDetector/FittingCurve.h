//
//  FittingCurve.h
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 29/08/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

#ifndef  LaneDetector_FittingCurve_h
#define  LaneDetector_FittingCurve_h

#include <opencv2/opencv.hpp>
#include <vector>

namespace LaneDetector{	
    void ExtractPointSet(const cv::Mat &img, std::vector<cv::Point2d> &pointSet);
    
    void IPMDrawCurve(const cv::Mat& coefs, cv::Mat &img, std::vector<cv::Point2d> &sampledPoints, const cv::Scalar &color);
    
    /*********************************************************************************/
    /* Least Squares Methods to Fitting Curve */
    /********************************************************************************/
    void FittingCurve_LS(const std::vector<cv::Point2d> &pointSet, const int &termNum, cv::Mat &coefs);
    
    void FittingCurve_WLS(const std::vector<cv::Point2d> &pointSet, const int &termNum, std::vector<double> &weights, cv::Mat &coefs);
    
    
    /*********************************************************************************/
    /* RANSAC Methods to Fitting Curve */
    /*********************************************************************************/
    void FittingCurve_RANSAC(const std::vector<cv::Point2d> &pointSet, 
                             const int &termNum, const int &minDataNum, 
                             const int &iterNum, const double &thValue,
                             const int & closeDataNum, cv::Mat &coefs, const cv::Mat &img);
    
    void SolveCubicFuntion(const cv::Mat &coefs, cv::Point2d &p, std::vector<double> &roots);
}


#endif
