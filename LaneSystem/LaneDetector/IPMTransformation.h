//
//  IPMTransformation.h
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 21/06/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//
//  /* The work is based on the MATLAB code from Eric Johnson and Randy Hamburger */
//  /* More information can be referenced from our notes */
//

#ifndef LaneDetector_IPMTransformation_h
#define LaneDetector_IPMTransformation_h
#include "LaneDetectorTools.h"
#include "DetectLanes.h"
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace LaneDetector{	 
    //! The function creates matrices giving the x and y locations in the world
    //! frame where the pixels in the bottom portion of the camera image map
    //! by using the IPM equations.
    //  \param xMap, yMap : p x n the matrices giving the x and y coordinates where 
    //                      the pixels in the last p rows of the image map in the 
    //                      world coordinate frame. (meter)
    //  \param rHorizon : based on the pitch angle, some region in raw image could be cropped.
    void IPMpixelsToWorld(LaneDetectorConf &laneDetectorConf, cv::Mat &xMap, cv::Mat &yMap);
    
    void IPMworldToPixels(const LaneDetectorConf &laneDetectorConf, const cv::Point2d &inPoint, cv::Point2d &outPoint);
    
    
    typedef struct _interpMap{
        cv::Mat pixels[4];
        cv::Mat weights[4];
    }InterpMap;
    
    //! The function finds the weights needed to create an IPM image with evenly 
    //! spaced pixels by linearly interpolating between intensity/color values in
    //! the original image. The interpolation weights are calculated based on the 
    //! locations where the image pixels map in the world coordinates so that they 
    //! will be based on the true physical distances. Areas in the IPM image which 
    //! are not visible in the original image are given weights of 0 so that they 
    //! will be black in the IPM image.
    //  \param xGrid : 1 x Nx vector giving the x values corresponding to the columns
    //                 of the IPM image.
    //  \param yGrid : 1 x Ny vector giving the y values corresponding to the rows of 
    //                 the IPM image. 
    //  \param interpMap.pixels : Ny x Nx x 4 array where each element gives a linear
    //                            index to a pixel in the original image.
    //  \param interpMap.weights : Ny x Nx x 4 array giving the weights associated with 
    //                             each of the original image pixels specified in the
    //                             corresponding location in the .pixels field.
    void IPMgetInterpMap(const cv::Mat &xMap, const cv::Mat &yMap,
                         LaneDetectorConf &laneDetectorConf, 
                         InterpMap &interpMap, cv::Mat &ipmMask);
    
    
    //! The function gets the row bounds in the xMap once the x visibility of a point
    //! in the IPM image is established.
    void IPMgetRowBounds(const int &xRow, const int &mCropped, int &r12, int &r34);
    
    
    //! The function checks the y direction visibility of a point in the IPM image and 
    //! returns the column bounds in the yMap if it is visible. 
    void IPMgetColBounds(const double &y, const cv::Mat &yMap, 
                         const int &r12, const int &r34,
                         std::vector<double> &cVec);
    
    
    //! The function convers the row, column locations we have found in the xMap, yMap
    //! matrices into equivalent linear indices in the original image.
    void IPMgetIndices(const int &m, const int &mCropped, const int &n, 
                       const std::vector<double> &rVec, const std::vector<double> &cVec,
                       std::vector<double> &iVec);
    
    //! The function gets the interpolation weightings for a given IPM pixels.
    void IPMgetWeights(const double &x, const double &y, 
                       const std::vector<double> &rVec, const std::vector<double> &cVec,
                       const cv::Mat &xMap, const cv::Mat &yMap, std::vector<double> &wVec);
    
    //! The function applies the inverse-perspective interpolation mapping to 
    //! an RBG camera image and returns a color and grayscale IPM image
    //  \param image : CV_64F 
    //  \param ipmMat : grayscale IPM image
    void IPMgetWorldImage(const cv::Mat &image, const LaneDetectorConf &laneDetectorConf, const InterpMap &interpMap, cv::Mat &ipmMat);  
    
    
    //! This function plot the points and lines from IPM process
    void DrawMarkingFromIPM(cv::Mat &laneMat, 
                            const std::vector<cv::Point2d> &leftSampledPoints, const std::vector<cv::Point2d> &rightSampledPoints, 
                            const LaneDetectorConf &laneDetectorConf);
}

#endif
