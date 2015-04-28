//
//  IPMTransformation.cpp
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 21/06/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

#include "IPMTransformation.h"


extern const int WIN_COLS;
extern const int WIN_ROWS;

namespace LaneDetector{	
    void IPMpixelsToWorld(LaneDetectorConf &laneDetectorConf, cv::Mat &xMap, cv::Mat &yMap)
    {
        double m = laneDetectorConf.m;
        double n = laneDetectorConf.n;
        double h = laneDetectorConf.h;
        double alphaTot = laneDetectorConf.alphaTot;
        double theta0 = laneDetectorConf.theta0;
        
        double den = sqrt(pow(m-1, 2) + pow(n-1, 2));
        double alpha_u = atan( (n-1)/den * tan(alphaTot));
        double alpha_v = atan( (m-1)/den * tan(alphaTot));
        laneDetectorConf.alpha_u = alpha_u;
        laneDetectorConf.alpha_v = alpha_v;
        laneDetectorConf.rHorizon = (m-1)/2*(1 - tan(theta0)/tan(alpha_v)) + 1 +  m * 0.05 ;
        
        /**************************/
        /* Get xMap yMap Matrices */
        /**************************/
        int mCropped = cvRound(m - laneDetectorConf.rHorizon) + 1;
        xMap = cv::Mat::zeros(mCropped, cvRound(n), CV_64F);
        yMap = cv::Mat::zeros(mCropped, cvRound(n), CV_64F);
        
        for (int r = 0; r < mCropped ; r++)
        {
            double rOrig = r + laneDetectorConf.rHorizon - 1;
            double rFactor = (1 - 2*(rOrig-1)/(m-1)) * tan(alpha_v);
            double numerator = 1 + rFactor * tan(theta0);
            double denominator = tan(theta0) - rFactor;
            cv::Mat result = cv::Mat::ones(1, cvRound(n), CV_64F) * h * (numerator / denominator);
            result.copyTo(xMap.row(r));
            
            for (int c = 0; c < n; c++) 
            {
                numerator = (1 - 2*(c-1)/(n-1))*tan(alpha_u);
                denominator = sin(theta0) - rFactor*cos(theta0);
                yMap.at<double>(r,c) = h * (numerator / denominator);
            }
        } 
    }// IPMpixelsToWorld
    
    void IPMworldToPixels(const LaneDetectorConf &laneDetectorConf, const cv::Point2d &inPoint, cv::Point2d &outPoint)
    {
        double m = laneDetectorConf.m;
        double n = laneDetectorConf.n;
        double h = laneDetectorConf.h;
        double theta0 = laneDetectorConf.theta0;
        double alpha_u = laneDetectorConf.alpha_u;
        double alpha_v = laneDetectorConf.alpha_v;
        
//        double x_max = laneDetectorConf.ipmX_max;
        double x_min = laneDetectorConf.ipmX_min;
//        double y_max = laneDetectorConf.ipmY_max;
        double y_min = laneDetectorConf.ipmY_min;
        
        double u = inPoint.x, v = inPoint.y;
        
        //! Translate the IPM pixel(u, v) into world(x, y);
        double x = u / laneDetectorConf.ipmStep + x_min;
        double y = -(v / laneDetectorConf.ipmStep + y_min + 0.1) ;//!left is plus, right is minus.
        
        double r = (m-1)/2.0 * ( 1 - (x*tan(theta0)-h)/(h*tan(theta0)+x)/tan(alpha_v) ) + 1;
        double c = (n-1)/2.0 * ( 1 - y/(h*sin(theta0)+x*cos(theta0))/tan(alpha_u) ) + 1;
        //printf("(%f, %f) Mapping To (%f, %f)\n", x, y, c, r);
        
        outPoint = cv::Point2d(c, r);
    }// end IPMworldToPixels
    
    
    void IPMgetInterpMap(const cv::Mat &xMap, const cv::Mat &yMap, 
                         LaneDetectorConf &laneDetectorConf,
                         InterpMap &interpMap, cv::Mat &ipmMask)
    {
        std::vector<double> xGrid, yGrid;
        
        double m = laneDetectorConf.m;
        double xRange_max;
        double xRange_min;
        cv::minMaxLoc(xMap, &xRange_min, &xRange_max, 0, 0);
        //! Only focus on the effective range, because too far distance is not available.
        xRange_max = xRange_max > laneDetectorConf.ipmX_max ? laneDetectorConf.ipmX_max : xRange_max;
        laneDetectorConf.ipmX_min = xRange_min;
        
        double yRange_max = laneDetectorConf.ipmY_max;
//        double yRange_min = laneDetectorConf.ipmY_min;
        double mIPM = laneDetectorConf.mIPM; 
        double step = 1/laneDetectorConf.ipmStep;
        
        int n = xMap.cols;
        int mCropped  = xMap.rows;
        
        int xNum = cvRound((xRange_max - xRange_min)/step);
        for (int i = 0; i < xNum; i++) {
            xGrid.push_back(xRange_min + step*i);
        }
        
        int yNum = cvRound(mIPM);
        for (int j = 0; j < yNum; j++) {
            yGrid.push_back(yRange_max - step*j);
        }
        
        int nRows = (int)yGrid.size();
        int nCols = (int)xGrid.size();
        for (int k = 0; k < 4; k++) {
            interpMap.pixels[k] = cv::Mat::ones(nRows, nCols, CV_64F);
            interpMap.weights[k] = cv::Mat::zeros(nRows, nCols, CV_64F);
        }
        
        std::vector<double> xVec;
        for (int i = 0; i < xMap.rows; i++) {
            xVec.push_back(xMap.at<double>(i, 0));
        }
        double xMinVis = xVec.back();
        
        
        for(int c = 0; c < nCols; c++) 
        {
            double x = xGrid.at(c);
            
            int flagL = 0, flagR = 0;
            for (int i = 0; i < (int)xVec.size(); i++) {
                if (xVec.at(i) >= x) {
                    flagL++;
                }
            }
            if (x >= xMinVis){
                flagR++;
            }
            int xRow = flagL*flagR;
            
            if (xRow > 0)
            {
                int r12, r34;
                IPMgetRowBounds(xRow, mCropped, r12, r34);
                
                for (int r = 0; r < nRows; r++) 
                {
                    double y = yGrid.at(r);
                    std::vector<double> cVec;
                    IPMgetColBounds(y, yMap, r12, r34, cVec);
                    if (cVec.at(0) > 0)
                    {
                        std::vector<double> rVec;
                        rVec.push_back(r12);
                        rVec.push_back(r12);
                        rVec.push_back(r34);
                        rVec.push_back(r34);
                        
                        std::vector<double> iVec;
                        IPMgetIndices(m, mCropped, n, rVec, cVec, iVec);
                        
                        std::vector<double> wVec;
                        IPMgetWeights(x, y, rVec, cVec, xMap, yMap, wVec);
                        
                        for (int i = 0; i < 4; i++) 
                        {
                            interpMap.pixels[i].at<double>(r,c) = iVec.at(i);
                            interpMap.weights[i].at<double>(r,c) = wVec.at(i);
                        }
                    }
                }// for       
            }// if
        }// for
        
        interpMap.weights[0].copyTo(ipmMask);
        double *p = NULL;
        for (int m = 0; m < nRows; m++) 
        {
            p = ipmMask.ptr<double>(m);
            for (int n = 0; n < nCols; n++) 
            {
                p[n] = p[n] == 0 ? 0 : 255;
            }
        }
        ipmMask.convertTo(ipmMask, CV_8U);
        
    }// IPMgetInterpMap
    
    
    void IPMgetRowBounds(const int &xRow, const int &mCropped, int &r12, int &r34)
    {
        if (xRow < mCropped) {
            r12 = xRow;
            r34 = xRow - 1;
        } else {
            r12 = xRow - 1;
            r34 = xRow - 2;
        }
    }// IPMgetRowBounds
    
    void IPMgetColBounds(const double &y, const cv::Mat &yMap, 
                         const int &r12, const int &r34,
                         std::vector<double> &cVec)
    {
        //! Init cVec
        for(int i = 0; i < 4; i++)
            cVec.push_back(0);
        
        std::vector<double> yVec12;
        for (int i = 0; i < yMap.cols; i++) {
            yVec12.push_back(yMap.at<double>(r12, i));
        }
        
        int flagL = 0, flagR = 0;
        for (int i = 0; i < (int)yVec12.size(); i++) {
            if (yVec12.at(i) >= y) {
                flagL++;
            }
        }
        if (y >= yVec12.back()){
            flagR++;
        }
        int yCol12 = flagL*flagR;
        
        if (yCol12 > 0){
            std::vector<double> yVec34;
            for(int i = 0; i <yMap.cols; i++) {
                yVec34.push_back(yMap.at<double>(r34, i));
            }
            
            int flagL = 0, flagR = 0;
            for (int i = 0; i < (int)yVec34.size(); i++) {
                if (yVec34.at(i) >= y) {
                    flagL++;
                }
            }
            if (y >= yVec34.back()){
                flagR++;
            }
            int yCol34 = flagL*flagR;
            
            if (yCol34 > 0) {
                if (yCol12 < (int)yVec12.size()) {
                    cVec.at(0) = yCol12;
                    cVec.at(1) = yCol12 + 1;
                } else {
                    cVec.at(0) = yCol12 - 1;
                    cVec.at(1) = yCol12;
                }
                if(yCol34 < (int)yVec34.size()) {
                    cVec.at(2) = yCol34;
                    cVec.at(3) = yCol34 + 1;
                } else {
                    cVec.at(2) = yCol34 - 1;
                    cVec.at(3) = yCol34;
                }
            }
        }
    }// IPMgetColBounds
    
    void IPMgetIndices(const int &m, const int &mCropped, const int &n, 
                       const std::vector<double> &rVec, const std::vector<double> &cVec,
                       std::vector<double> &iVec)
    {
        if(!iVec.empty())
            iVec.clear();
        
        int shift = m - mCropped;
        
        std::vector<double> rOrig;
        for(int i = 0; i < (int)rVec.size(); i++) {
            rOrig.push_back(rVec.at(i) + shift);
            iVec.push_back((rOrig.at(i)+1)*n + cVec.at(i)+1);//sub2ind (MATLAB)
        }
    }// IPMgetIndices
    
    void IPMgetWeights(const double &x, const double &y, 
                       const std::vector<double> &rVec, const std::vector<double> &cVec,
                       const cv::Mat &xMap, const cv::Mat &yMap, std::vector<double> &wVec)
    {
        std::vector<double> px, py;
        for (int p = 0; p < 4; p++) {
            px.push_back(xMap.at<double>(cvRound(rVec.at(p)), cvRound(cVec.at(p))));
            py.push_back(yMap.at<double>(cvRound(rVec.at(p)), cvRound(cVec.at(p))));
        }
        
        double d12y = py.at(0) - py.at(1);
        double d1y = py.at(0) - y;
        double d2y = y - py.at(1);
        
        double d34y = py.at(2) - py.at(3);
        double d3y = py.at(2) - y;
        double d4y = y - py.at(3);
        
        double d13x = px.at(2) - px.at(0);
        double d3x = px.at(2) - x;
        double d1x = x - px.at(0);
        
        wVec.push_back(d3x*d2y/(d13x*d12y));
        wVec.push_back(d3x*d1y/(d13x*d12y));
        wVec.push_back(d1x*d4y/(d13x*d34y));
        wVec.push_back(d1x*d3y/(d13x*d34y));  
        
    }// IPMgetWeights
    

    void IPMgetWorldImage(const cv::Mat &image, const LaneDetectorConf &laneDetectorConf, const InterpMap &interpMap, cv::Mat &ipmMat)
    {
        int nRows = interpMap.pixels[0].rows;
        int nCols = interpMap.pixels[0].cols;
        ipmMat = cv::Mat::zeros(nRows, nCols, CV_64F);
        
#if 0
        //! Show the cropped image.
        cv::Rect mask(0, laneDetectorConf.rHorizon, image.cols, image.rows - laneDetectorConf.rHorizon);
        cv::Mat roi = image(mask);
        cv::imshow("roi", roi);
#endif
        
        //int iRows = image.rows;
        int iCols = image.cols;
        cv::Mat imageF;
        image.convertTo(imageF, CV_64F);
        
        
        //More efficient method to scan the matrix
        cv::Mat newPixels, temp;
        for (int j = 0; j < 4; j++)
        {
            newPixels = cv::Mat::zeros(nRows, nCols, CV_64F);
            
            int m, n;
            double *p = NULL;
            double *q = NULL;
            for (m = 0; m < nRows; m++)
            {
                cv::Mat pixel = interpMap.pixels[j];
                p = pixel.ptr<double>(m);
                q = newPixels.ptr<double>(m);
                for (n = 0; n < nCols; n++) {
                    int indice = cvRound(p[n]);
                    int col =  cvRound(indice % iCols);
                    int row = cvRound(indice / iCols);
                    col = col < imageF.cols ? col : imageF.cols - 1;
                    row = row < imageF.rows ? row : imageF.rows - 1;
//                    cout << "Col : Row " << col << " " << row << endl;
//                    cout << "imageF " << imageF.cols << " " << imageF.rows << endl;
                    q[n] = imageF.at<double>(row, col);
                }
            }
            
            temp = cv::Mat::zeros(nRows, nCols, CV_64F);
            cv::multiply(newPixels, interpMap.weights[j], temp);
            ipmMat += temp;
        }
        
        double *b = NULL;
        for (int m = 0; m < nRows; m++) 
        {
            b = ipmMat.ptr<double>(m);
            for (int n = 0; n < nCols; n++) 
            {
                b[n] = b[n] < 0 ? 0 : b[n];
                b[n] = b[n] > 255 ? 255 : b[n];
            }
        }
        
        ipmMat.convertTo(ipmMat, CV_8U);
        
    }// IPMgetWorldImage

    
/**********************************************************/
//!Get pixel coordinates 
/**********************************************************/
//    void IPMgetPixelCoordinates(const cv::Point2d &inPoints, 
//                                const cv::Mat &xMap, const cv::Mat &yMap,
//                                const LaneDetectorConf &laneDetectorConf, 
//                                cv::Point2d &outPoints)
//    {
//        int xOut, yOut;//xOut -> Point_Y（非等差）, yOut -> Point_X(等差)
//        int u = 0,v = 0;
//        int xFlag = 0, yFlag = 0;
//        double xWorld = inPoints.x / laneDetectorConf.ipmStep + laneDetectorConf.ipmX_min;
////        std::cout << "xWorld: " << xWorld << " ";
//        
//        double minDis0 = abs(xWorld - xMap.at<double>(u,0));
//        for(u = 1; u < xMap.rows; u++)
//        {
////            std::cout << " xMap_X: " << xMap.at<double>(u, 0) << std::endl;
//            
//            //Find the minimum distance
//            double minDis = abs(xWorld - xMap.at<double>(u,0));
////            std::cout << minDis0 << " ^ " << minDis << std::endl;
//            if (minDis >= minDis0 || u == xMap.rows - 1)
//            {   
//                xOut = u;//the last one (u-1);
//                xFlag = 1;
//                break;
//            }
//            minDis0 = minDis;
//        }
//        
//        double yWorld = laneDetectorConf.ipmY_max - inPoints.y / laneDetectorConf.ipmStep;
////        std::cout << "yWorld: " << yWorld << std::endl;
//        
//        CV_Assert(xFlag == 1);
//        double yTh = (yMap.at<double>(xOut, 0) - yMap.at<double>(xOut, yMap.cols-1)) / double(yMap.cols);
//        for(; v < yMap.cols; v++)
//        {
////            std::cout << " yMap_Y: " << yMap.at<double>(xOut, v) << std::endl;
//            if(abs(yWorld - yMap.at<double>(xOut, v)) <= yTh)//meter
//            {
////                std::cout << "Find Y Coordinate" << std::endl;
//                yOut = v;
//                yFlag = 1;
//                break;
//            }
//            yOut = v;//Only equal to yMap.cols-1. 
//        }
//        
//        double yTemp;
//        if(yWorld < yMap.at<double>(xOut, yMap.cols-1))
//        {
//            //From big to small
//            yTemp = yMap.at<double>(xOut, yMap.cols-1) - yTh;
//            do{
//                yOut++;
//                yTemp -= yTh;
//            }while(yWorld < yTemp);
//            yFlag = 1;
//        }
//        else if(yWorld > yMap.at<double>(xOut, 0))
//        {
//            // From small to big
//            yOut = 0;
//            yTemp = yMap.at<double>(xOut, 0) + yTh;
//            do{
//                yOut--;
//                yTemp += yTh;
//            }while(yWorld > yTemp);
//            yFlag = 1;    
//        }
//    
//        //! Offset with rHorizon
//        xOut += cvRound(laneDetectorConf.rHorizon) ;
//        //yOut += 5;//?
////        std::cout << "xFlag: " << xFlag << " yFlag: " << yFlag << std::endl;
//        
//        CV_Assert(xFlag == 1 && yFlag == 1);
//        outPoints = cv::Point(yOut, xOut);
//        
//    }//end IPMgetPixelCoordinates
    
    
    void DrawMarkingFromIPM(cv::Mat &laneMat, 
                            const std::vector<cv::Point2d> &leftSampledPoints, const std::vector<cv::Point2d> &rightSampledPoints, 
                            const LaneDetectorConf &laneDetectorConf)
    {
        std::vector<cv::Point2d> leftPoints, rightPoints;
        int sampledPointsNum = (int)leftSampledPoints.size();
        for(int i = 0; i < sampledPointsNum; i++)
        {
            cv::Point2d leftPoint;
            IPMworldToPixels(laneDetectorConf, leftSampledPoints[i], leftPoint);
            leftPoints.push_back(leftPoint);
            
            cv::Point2d rightPoint;
            IPMworldToPixels(laneDetectorConf, rightSampledPoints[i], rightPoint);
            rightPoints.push_back(rightPoint);
        }
        
        for(int j = 0; j < sampledPointsNum - 1; j++)
        {
            //! left lane marking
            cv::circle(laneMat, leftPoints[j+1], 3, CV_RGB(255, 255, 0));
            cv::line(laneMat, leftPoints[j], leftPoints[j+1], CV_RGB(255, 0, 0), 2);
            
            //! right lane marking
            cv::circle(laneMat, rightPoints[j+1], 3, CV_RGB(255, 0, 255));
            cv::line(laneMat, rightPoints[j], rightPoints[j+1], CV_RGB(0, 255, 0), 2);
        }
    }
    
    
}//namespace LaneDetector
