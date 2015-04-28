//
//  LaneDetectorTools.cpp
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 06/06/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//
#include "LaneDetectorTools.h"

namespace LaneDetector {
    void PrintMat(const cv::Mat &mat)
    {
        
        for(int i = 0; i < mat.rows; i++) {
            for(int j = 0; j < mat.cols; j++) {
                if(mat.type() == CV_64F)
                    std::cout << mat.at<double>(i,j) <<' ';
                else if(mat.type() == CV_32F)
                    std::cout << mat.at<float>(i,j) << ' ';
                else
                    std::cout << (int)mat.at<uchar>(i,j) << ' ';
            }
            std::cout << std::endl;
        }
    }
   
    //! The image in vector should be in the same size.
    void multiImShow(const std::vector<cv::Mat> &mat, const std::vector<std::string> &winname, cv::Mat &multiMat)
    {
        if (mat.empty()) {
            std::cerr << "ERROR: Please put image in vector!" << std::endl;
            exit(EXIT_FAILURE);
        } 
        else {
            int imRows = mat.at(0).rows;
            int imCols = mat.at(0).cols;
            int imNum = (int)mat.size();
            int shift = 5;
            
            if (imNum * imCols >= 2000)
            {
                multiMat = cv::Mat::ones(imNum/2 * (imRows + shift), imNum/2 * (imCols + shift), CV_8U);
                
                for (int i = 0; i < 2 ; i++) {
                    for(int j = 0; j < imNum/2; j++) {
                        multiMat(cv::Rect(j * (imCols + shift), i * (imRows + shift), imCols, imRows)) += mat.at((i * imNum/2 + j));
                    }
                } 
            } else {
                multiMat = cv::Mat::zeros(imRows, imNum * (imCols + shift), CV_8U);
                
                for (int i = 0; i < imNum ; i++) {
                    multiMat(cv::Rect(i * (imCols + shift), 0, mat.at(i).cols, mat.at(i).rows)) += mat.at(i);
                }
            }
            
            
            
            std::string title = "TITLE: ";
            for (int s = 0; s < (int)winname.size(); s++) {
               title += winname.at(s);
            }
            
            cv::imshow(title, multiMat); 
        }
    }//end multiImShow
    
    
    //! This function works like the same one in MATLAB
    void imShowSub(const std::string &winname, const cv::Mat &mat, 
                 const int &Cols, const int &Rows, const int &winPos)
    {
        int width = cvRound((double)1200 / (double)Cols);
        int height = cvRound((double)750 / (double)Rows);
        
        CV_Assert(winPos <= Rows * Cols);
        int col = (winPos - 1) % Cols;
        int yWin = height * (winPos-1-col) / Cols + 30;
        int xWin = width * col;//Add offset 
        
        //std::cout << "xWin: " << xWin << " yWin: " << yWin << std::endl;
        cv::imshow(winname, mat);
        cv::moveWindow(winname, xWin, yWin);
    }//end imShowSub
    
    
    
    

}//namespace LaneDetector
