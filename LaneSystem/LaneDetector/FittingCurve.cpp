//
//  FittingCurve.cpp
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 29/08/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//

// \************************************\
//  Polynomial Curves by the Least Squares Method for Curve Fitting
//  
// 
// /************************************/

#include <cmath>
#include <iostream>
#include "FittingCurve.h"
#include "LaneDetectorTools.h"

namespace LaneDetector{	
    //! This function relies on the least squares method to fitting the potential curve or line based on the degree number. 
    // \pointSet:  the set of observed points 
    // \termNum: the number of terms like x = a0 + a1*y + a2*y^2
    // \coef: the coefficient of the polynomial
    void FittingCurve_LS(const std::vector<cv::Point2d> &pointSet, const int &termNum, cv::Mat &coefs)
    {
//        double startTime = (double)getTickCount();
        
        cv::Mat innerProductXX = cv::Mat::zeros(termNum, termNum, CV_64F);
        cv::Mat innerProductXY = cv::Mat::zeros(termNum, 1, CV_64F);
        
        for(int i = 0; i < termNum; i++)
        {
            double termXX = 0;
            double termXY = 0;
            for(int m = 0; m < (int)pointSet.size(); m++)
            {
                double pointX = pointSet.at(m).x; //horizontal axes
                double pointY = pointSet.at(m).y; //vertical axes
//                cout << "pointX: " << pointX << " pointY: " << pointY << endl;
                for(int j = 0; j < termNum; j++)
                {
                    termXX = pow(pointX, i) * pow(pointX, j);
//                    cout << "termXX : " << termXX << endl;
                    innerProductXX.at<double>(i,j) += termXX;
                }
                
                termXY += pow(pointX, i) * pointY;
            }
            
            innerProductXY.at<double> (i,0) = termXY; 
        }
        
        
        cv::Mat invertedXX = cv::Mat(termNum, termNum, CV_64F);
        double isSingular = cv::invert(innerProductXX, invertedXX);
        
        if(isSingular != 0)
            coefs = invertedXX * innerProductXY; 
        //PrintMat(coefs);
        
        
//        double costTime = ((double)getTickCount() - startTime)/getTickFrequency();
//        printf("LS Curve Fitting Needs %.2f msec about %.2f Hz\n", costTime*1000, 1/costTime);
        
    }//end FittingCurve
    
    
    //! This function execute the weighted Least Squares method
    void FittingCurve_WLS(const std::vector<cv::Point2d> &pointSet, const int &termNum, std::vector<double> &weights, cv::Mat &coefs)
    {
//        double startTime = (double)getTickCount();

        cv::Mat innerProductXX = cv::Mat::zeros(termNum, termNum, CV_64F);
        cv::Mat innerProductXY = cv::Mat::zeros(termNum, 1, CV_64F);
        
        for(int i = 0; i < termNum; i++)
        {
            double termXX = 0;
            double termXY = 0;
            for(int m = 0; m < (int)pointSet.size(); m++)
            {
                double pointX = pointSet.at(m).x;
                double pointY = pointSet.at(m).y;
//                cout << "pointX : " << pointX << " pointY : " << pointY << endl;
                for(int j = 0; j < termNum; j++)
                {
                    termXX = pow(pointX, i) * pow(pointX, j) * weights[m];
//                    cout << "termXX : " << termXX << endl;
                    innerProductXX.at<double>(i,j) += termXX;
                }
                
                termXY += pow(pointX, i) * pointY * weights[m];
            }
            
            innerProductXY.at<double>(i,0) = termXY; 
        }
        
        cv::Mat invertedXX = cv::Mat(termNum, termNum, CV_64F);
        double isSingular = cv::invert(innerProductXX, invertedXX);
        
        if(isSingular != 0);
        coefs = invertedXX * innerProductXY; 
        
        
//        double costTime = ((double)getTickCount() - startTime)/getTickFrequency();
//        printf("WLS Curve Fitting Needs %.2f msec about %.2f Hz\n", costTime*1000, 1/costTime);
        
    }//end FittingCurve
    


    //! This functioin extract the feature points.
    // \thMat: the input image(mat) in threshold pattern
    // \pointSet: the output set of points
    void ExtractPointSet(const cv::Mat &img, std::vector<cv::Point2d> &pointSet)
    {
        CV_Assert(img.type()==CV_64F || img.type()==CV_32F || img.type()==CV_8U);
        //PrintMat(thMat);
        
        cv::Mat thMat = img.clone();
        //! First rows, then cols
        if(thMat.type() == CV_64F)
        {
            double *p = NULL;
            for (int m = 0; m < thMat.rows; m++) 
            {
                p = thMat.ptr<double>(m);
                for (int n = 0; n < thMat.cols; n++) 
                {
                    if(p[n] == 1)
                        pointSet.push_back(cv::Point2d(n, m));
                }
            }

        }
        else if(thMat.type() == CV_32F)
        {
            float *p = NULL;
            for (int m = 0; m < thMat.rows; m++) 
            {
                p = thMat.ptr<float>(m);
                for (int n = 0; n < thMat.cols; n++) 
                {
                    if(p[n] == 1)
                        pointSet.push_back(cv::Point2d(n, m));
                }
            } 
        }
        else if(thMat.type() == CV_8U)
        {
            uchar *p = NULL;
            for (int m = 0; m < thMat.rows; m++) 
            {
                p = thMat.ptr<uchar>(m);
                for (int n = 0; n < thMat.cols; n++) 
                {
                    if((int)p[n] > 0) 
                    {
                        pointSet.push_back(cv::Point2d(n, m));
                    }
                    
                }
            }
        }
        else
            std::cerr << "Unknown Mat Type! " << std::endl;
    }//end ExtractPointSet
    
    
    
    // \coefs: the coefficients of the polynomial, stored in a row of Mat
    void IPMDrawCurve(const cv::Mat& coefs, cv::Mat &img, std::vector<cv::Point2d> &sampledPoints, const cv::Scalar &color)
    {
        sampledPoints.clear();
        
        double width = img.cols;
        int terms = coefs.rows;

        double pointX = 0;
        double sampledPointsNum = 5;
        
        for (int i = 0; i <= sampledPointsNum; i++) {
            pointX =  (double)i * width/sampledPointsNum;
            double pointY = 0;
            for(int k = 0; k < terms; k++ )
            {
                pointY += coefs.at<double>(k,0) * pow(pointX, k);
            }
            
            sampledPoints.push_back(cv::Point2d(pointX, pointY));
//            cout << "pointX : "<< pointX << " pointY : " << pointY << endl;
            
            cv::circle(img, sampledPoints[i], 3, CV_RGB(255, 255, 0));
        }
        
        for (int i = 0; i < sampledPointsNum; i++) {
            cv::line(img, sampledPoints[i], sampledPoints[i+1], color);
        }
        
    }//end IPMDrawCurve
    
    
       
    
    //! Fitting line or Curve via RANSAC has constraint in some conditions.
    // \minDataNum:   the minimum number of data required to fit the model. 
    // \closeDataNum: the number of close data values required to assert that a model fits well to data.
    // \iterNum:      the number of iterations performed by the algorithm.
    // \thValue:      a threshold value for determining when a datum fits a model.
    void FittingCurve_RANSAC(const std::vector<cv::Point2d> &pointSet, 
                             const int &termNum, const int &minDataNum, 
                             const int &iterNum, const double &thValue,
                             const int &closeDataNum, cv::Mat &coefs, const cv::Mat &img)
    {
        double startTime = (double)cv::getTickCount();

        std::vector<cv::Point2d> bestPointSet;   //data points from which this model has been estimated
        double bestError = INFINITY;        //the error of this model relative to the data
        double errorSum, minErrorSum;       //Prevent the sampling number smaller than closeDataNum
        int errorFlag = 0; 
        
        cv::Mat maybeModel = cv::Mat::zeros(termNum, 1, CV_64F);
        cv::Mat img2 = img.clone();
        
        for(int iter = 0; iter < iterNum; iter++)
        {
            std::vector<cv::Point2d> tempPointSet = pointSet;
            errorSum = 0;
            
            //! Generate a set of maybe_inliers
            std::vector<cv::Point2d> maybeInliersPointSet;
            uint64 seed = iter + 1; 
            cv::RNG rng(seed);
            for(int i = 0; i < minDataNum; i++)
            {
                int n = rng.uniform(0, (int)tempPointSet.size()-1);
                maybeInliersPointSet.push_back(tempPointSet.at(n));
                tempPointSet.erase(tempPointSet.begin()+n);
                if (tempPointSet.size() == 0)
                    break;
            }//end for
            
            
            //! Fitting maybe_inliers to model by Least Squares Method
            FittingCurve_LS(maybeInliersPointSet, termNum, maybeModel);
            bestPointSet = maybeInliersPointSet; //Random Sampling Model
#if 0
            for(int i=0; i<bestPointSet.size(); i++)
            {
                cv::circle(img2, bestPointSet.at(i), 1, CV_RGB(125, 125, 0));
            }
            std::vector<cv::Point2d> sampledPoints;
            IPMDrawCurve(maybeModel, img2, sampledPoints, CV_RGB(100, 0, 0) );
            //cv::imshow("RANSAC", img2);
            //cv::waitKey();
#endif
            
            
            //! Different from the linear modeling, the non-linear asks for computation of the distance between point and a polynomial curve.
            //! deviration of distance function d{l(x)}/d{x} = 0
            //! It is: 4*a2*x^3 + 6*a1*a2*x^2 + 2*(1+2*a2*(a0-y0)+a1^2)*x + 2*(a1*(-y0+a0)-x0) = 0 
            //! Solve this cubic funtion (polynomial of degree three)
            double a0, a1, a2;
            if (termNum == 2) {
                //!Linear RANSAC
                a0 = maybeModel.at<double>(0,0);
                a1 = maybeModel.at<double>(1,0);
                for (int i = 0; i < tempPointSet.size(); i++) {
                    int x = cvRound(tempPointSet.at(i).x);
                    int y = cvRound(tempPointSet.at(i).y);
                    double error = std::pow(std::abs(a1*x - y + a0)/sqrt(a1*a1+1), 2);
                    errorSum += error;
                    if (error < thValue) {
                        bestPointSet.push_back(tempPointSet.at(i));
                    }
                }//end for
            }
            else if (termNum == 3) {
                //!NonLinear RANSAC
                a0 = maybeModel.at<double>(0,0);
                a1 = maybeModel.at<double>(1,0);
                a2 = maybeModel.at<double>(2,0);
                for (int i = 0; i < tempPointSet.size(); i++) {
                    int x0 = cvRound(tempPointSet.at(i).x);
                    int y0 = cvRound(tempPointSet.at(i).y);
                    double error = std::pow(std::abs(y0 - (a0+(a1+a2*x0)*x0)), 2);
                    errorSum += error;
                    //cout << "Outliers Error: " << error << endl;
                    if (error < thValue) {
                        bestPointSet.push_back(tempPointSet.at(i));
                    }
                }//end for
            }
            else {
                std::cerr << "Only fitting line of degree 1 or 2." << std::endl;
            }

            //! First sampling as initialization
            if(iter == 0) minErrorSum = errorSum;
//            cout << "errorSum: " << errorSum << " minError: " << minErrorSum << endl;
            //! Prevent the situation that all maybePoint number smaller than the close data number
            if(errorSum < minErrorSum)
            {
                minErrorSum = errorSum;
                if(errorFlag == 0)
                    coefs = maybeModel; //! Only works when all sampling doesn't have a good model. 
            }
            
            //! this implies that we may have found a good model;
            //! now test how good it is
            if((int)bestPointSet.size() > closeDataNum)
            {
                double thisError = 0;
                //cout << "Error Update!" << endl;
                cv::Mat thisModel = cv::Mat::zeros(termNum, 1, CV_64F);
                FittingCurve_LS(bestPointSet, termNum, thisModel);
                
                if (termNum == 2) {
                    a0 = thisModel.at<double>(0,0);
                    a1 = thisModel.at<double>(1,0);
                    for(int i = 0; i < bestPointSet.size(); i++) 
                    {
                        int x = cvRound(bestPointSet.at(i).x);
                        int y = cvRound(bestPointSet.at(i).y);
                        thisError += pow((a1*x - y + a0)/sqrt(a1*a1+1), 2);
                    }//end for
                }
                else if(termNum == 3) {
                    a0 = thisModel.at<double>(0,0);
                    a1 = thisModel.at<double>(1,0);
                    a2 = thisModel.at<double>(2,0);
                    for(int i = 0; i < bestPointSet.size(); i++) 
                    {
                        int x0 = cvRound(bestPointSet.at(i).x);
                        int y0 = cvRound(bestPointSet.at(i).y);
                        thisError += std::pow(std::abs(y0 - (a0+(a1+a2*x0)*x0)), 2);
                    }//end for
                }
                else {
                    std::cerr << "Only fitting line of degree 1 or 2." << std::endl;
                }
                
                //! The line above contains the bug. 
                //! This_error should be replaced by a score that is either the size of the consensus set, 
                //! or the robust error norm computed on ALL samples (not just the consensus set).
//                cout << "thisError: " << thisError << " bestError: " << bestError << endl;
                if(thisError < bestError)
                {
                    //! we have found a model which is better than any of the previous ones;
                    //! keep it until a better one is found)
                    //! best_consensus_set := consensus_set;
                    coefs = thisModel; //bestModel := thisModel
                    bestError = thisError;//bestError := thisError
                    errorFlag = 1;
                }//end if

            }//end if
            
            
//            std::vector<cv::Point2d> sampledPoints2;
//            IPMDrawCurve(coefs, img2, sampledPoints2, CV_RGB(0, 255, 0) );
//            cv::imshow("RANSAC", img2);
            
        }//end for iteration
        
        
        double costTime = ((double)cv::getTickCount() - startTime)/cv::getTickFrequency();
        printf("RANSAC Line Fitting Needs %.2f msec about %.2f Hz\n", costTime*1000, 1/costTime);
        
    }//end FittingCurve_RANSAC
    
    
    //! Not finished
    void SolveCubicFuntion(const cv::Mat &coefs, cv::Point2d &p, std::vector<double> &roots)
    {
        double a0 = coefs.at<double>(0,0);
        double a1 = coefs.at<double>(1,0);
        double a2 = coefs.at<double>(2,0);
        double x0 = p.x;
        double y0 = p.y;
        
        double a = 4*a2;
        double b = 6*a1*a2;
        double c = 2*(1+2*a2*(a0-y0)+a1*a1);
        double d = 2*(a1*(-y0+a0)-x0);

        //!Calculate the delta to determine the roots
        double delta0 = pow(b*c/(6*a*a) - b*b*b/(27*a*a*a) - d/(2*a), 2);
        double delta1 = pow(c/(3*a) - b*b/(9*a*a), 3);
        double delta = delta0 + delta1;
        std::cout << "delta: " << delta << std::endl;

        //! TOO SLOW to calculate the roots
        if (delta > 0) {
            //! Function has one real root and two complex roots
            //! roots = ?
        } 
        else if(delta == 0)
        {
            //! Function has three real roots
            if(delta0 == 0)
            {
                //! the three real roots are same
                //! roots = ?
            }
            else 
            {
                //! two real roots are same
                //! roots = ?
            } 
        }
        else
        {
            //! Function has three different roots
            //! roots = ?
        }
    }
    
}//namespace LaneDetector
