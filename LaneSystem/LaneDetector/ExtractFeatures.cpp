//
//  ExtractFeatures.cpp
//  LaneDetector1.2
//
//  Created by XUANPENG LI on 01/07/13.
//  Copyright (c) 2013 __MyCompanyName__. All rights reserved.
//
#include "ExtractFeatures.h"

namespace LaneDetector {
    void ExtractFeatures(const cv::Mat &mat, 
                         const std::string &detectorMethod, std::vector<cv::KeyPoint> &keypoints,
                         const std::string &extractorMethod, cv::Mat &descriptor, cv::Mat &outMat)
    {    
        //! Common API of Feature Detector
        //! The function supports the following methods:
        //! "FAST" -- FastFeatureDetector
        //! "STAR" -- StarFeatureDetector
        //! "SIFT" -- SIFT(nonfree module)
        //! "SURF" -- SURF(nonfree module)
        //! "ORB"  -- ORB
        //! "BRISK" -- BRISK
        //! "MSER" -- MSER
        //! "GFTT" -- GoodFeatureToTrackDetector
        //! "HARRIS" -- GoodFeaturesToTrackDetector with Harris detector enabled
        //! "Dense" -- DenseFeatureDetector
        //! "SimpleBlob" -- SimpleBlobDetector
        //! A combined format is supported as "GridFAST" and "PyramidSTAR".
        
        if (detectorMethod == "SIFT" || detectorMethod == "SURF")
            cv::initModule_nonfree();
        
        cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(detectorMethod);
        detector->detect(mat, keypoints);
        
        cv::drawKeypoints(mat, keypoints, outMat);
        
        
        std::cout<< detectorMethod << "--Features Size: " << keypoints.size() << std::endl;
        
        //! Some methods cannot support the Common Extractor
        //! The current version supports the following types:
        //! "SIFT" -- SIFT
        //! "SURF" -- SURF
        //! "ORB" -- ORB
        //! "BRISK" -- BRISK
        //! "BRIEF" -- BriefDescriptorExtractor
        //! "Opponent" -- OpponentColorDescriptorExtractor 
        if(extractorMethod == "SIFT" || extractorMethod == "SURF" || extractorMethod == "ORB" || extractorMethod == "BRISK" || extractorMethod == "BRIEF" || extractorMethod == "Opponent") {
            cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(extractorMethod);
            extractor->compute(mat, keypoints, descriptor);
        }
        else {
            std::cerr << "Please input the correct extractor name!" <<std::endl;
            exit(EXIT_FAILURE);
        }
    }// ExtractFeatures
    
    
    //! Descriptor matcher can support the following matcher:
    //! "BruteForce" (it uses L2)
    //! "BruteForce-L1"
    //! "BruteForce-Hamming"
    //! "BruteForce-Hamming(2)"
    //! "FlannBased"
    void MatchFeatures(const std::string &matcherMethod,
                       const cv::Mat &descriptor1,
                       const cv::Mat &descriptor2,
                       std::vector< cv::DMatch > &good_matches)
    {
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matcherMethod);
#if 0       
        std::vector< cv::DMatch > matches;
        matcher->match( descriptor1, descriptor2, matches );
        double max_dist = 0; double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptor1.rows; i++ )
        { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
        
        //-- Filter only "good" matches (i.e. whose distance is less than 2*min_dist )
        //-- PS.- radiusMatch can also be used here.
        for( int i = 0; i < descriptor1.rows; i++ )
        { 
            if( matches[i].distance <= 3*min_dist )
                good_matches.push_back( matches[i]); 
            
        }

#else
        //! The matches are returned in the distance increasing order,
        //! the number is according to k.
        std::vector<std::vector< cv::DMatch> > matchesVec;
        matcher->knnMatch(descriptor1, descriptor2, matchesVec, 3);
        
        std::cout << "matches size : " << matchesVec.size() << std::endl;
        for( int i = 0; i < descriptor1.rows; i++ ) 
        {
            good_matches.push_back(matchesVec[i][0]);
        }
#endif

    }// MatchFeatures
    
    void ExtractHOG(const cv::Mat &mat)
    {
        int pixelsCell = 8;
        
        int cols = mat.cols - mat.cols % pixelsCell;
        int rows = mat.rows - mat.rows % pixelsCell;
        cv::Mat resizedMat;
        resize(mat, resizedMat, cv::Size(cols, rows));
        std::cout << "Cols: " << resizedMat.cols << " Rows: " << resizedMat.rows << std::endl;
        
        //! We have used different window size!
        cv::HOGDescriptor hog = cv::HOGDescriptor(cv::Size(resizedMat.cols,resizedMat.rows), cv::Size(pixelsCell * 2, pixelsCell * 2), cv::Size(pixelsCell, pixelsCell), cv::Size(pixelsCell, pixelsCell), 9, 0, -1, 0, 0.2, 0);
        // All following parameters are default setting.
        // Size(64,128),    //winSize(width, height)
        // Size(16,16),     //blockSize
        // Size(8,8),       //blockStride
        // Size(8,8),       //cellSize
        // 9,               //nbins,
        // 0,               //derivAper,
        // -1,              //winSigma,
        // 0,               //histogramNormType,
        // 0.2,             //L2HysThresh,
        // 0                //gammal correction.
        // HOG descriptor length = #Blocks * #CellsPerBlock * #BinsPerCell
        //                       = (64/8 - 1) * (128/8 - 1) * (2 * 2) * 9
        //                       = 420(cells including overlapped) * 9
        //                       = 3780
        
        std::vector<float> descriptors;
        std::vector<cv::Point> locations;
        // \winStride: the scanning inverval between the continuous two windows.
        // \padding: the margin along the side of image.
        hog.compute(resizedMat, descriptors, cv::Size(0,0), cv::Size(0,0), locations);
        
        std::cout << "HOG descriptor size is " << hog.getDescriptorSize() << std::endl;
        std::cout << "Found " << descriptors.size() << " descriptor values" << std::endl;
        //std::cout << "Nr of locations specified : " << locations.size() << std::endl;
        
        cv::Mat visualHOG;
        GetVisualHOG(resizedMat, pixelsCell, descriptors, visualHOG);
        cv::imshow("visualHOG", visualHOG);
    } // ExtractHOG
    
    //! It is needed to crop and resize the image in 128 x 64, fixed in a window size.
    //! Because it is aimed to detect the people within such size in a picture.
    void GetVisualHOG(const cv::Mat &mat, const int &pixels, std::vector<float> &descriptors, cv::Mat &visualHOG)
    {
        int zoom        = 1; // Scale of draw cells 
//        int blockSize   = pixelsCell * 2;//16 default
        int cellSize    = pixels;//8 default
        int binSize     = 9;
        float radRangeEachBin = (float)M_PI/(float)binSize;
        
        cv::Mat colorMat;
        cv::cvtColor(mat, visualHOG, cv::COLOR_GRAY2BGR);

        
        // In a individual window 
        // Data structure: 9 orientation / gradient strenghts for each cell 
        int cellsX = mat.cols / cellSize;     //8 cells per col in window size
        int cellsY = mat.rows / cellSize;     //16 cells per row in window size
        
        float*** gradientStrengths = new float**[cellsY];
        int** cellUpdateCounter = new int*[cellsY];
        for(int celly = 0; celly < cellsY; celly++)
        {
            gradientStrengths[celly] = new float*[cellsX];
            cellUpdateCounter[celly] = new int[cellsX];
            for (int cellx = 0; cellx < cellsX; cellx++)
            {
                gradientStrengths[celly][cellx] = new float[binSize];
                cellUpdateCounter[celly][cellx] = 0;
                
                for(int bin = 0; bin < binSize; bin++)
                {
                    gradientStrengths[celly][cellx][bin] = 0;
                }
            }
        }
        
        // Num row of blocks = num row of cells - 1
        // since there is a new block on each cell, overlapping blocks.
        int blocksX = cellsX - 1;
        int blocksY = cellsY - 1;
        
        // compute gradient strengths per cell
        int descriptorDataIdx = 0;
        for(int blockx = 0; blockx < blocksX; blockx++)
        {
            for(int blocky = 0; blocky < blocksY; blocky++)
            {
                //4 cells per block...
                for(int cellNr = 0; cellNr < 4; cellNr++)
                {
                    int cellx = blockx;
                    int celly = blocky;
                    if (cellNr == 1) celly++;
                    if (cellNr == 2) cellx++;
                    if (cellNr == 3)
                    {
                        celly++;
                        cellx++;
                    }
                    
                    for(int bin = 0; bin < binSize; bin++)
                    {
                        float gradientStrength = descriptors[descriptorDataIdx];
                        descriptorDataIdx++ ;
                        
                        gradientStrengths[celly][cellx][bin] += gradientStrength;
                    }// for (all bins)
                        
                    // note: overlapping blocks lead to multiple updates of this sum!
                    // we therefore keep track how often a cell was updated,
                    // to compute average gradient strengths.
                    cellUpdateCounter[celly][cellx]++;
                    
                }// for (all cells)
                
            }// for (all block x pos)
        }// for (all block y pos)
        
        // compute average gradient strengths
        for(int celly = 0; celly < cellsY; celly++)
        {
            for(int cellx = 0; cellx < cellsX; cellx++)
            {
                float NrUpdateEachCell = (float)cellUpdateCounter[celly][cellx];
                
                //compute average gradient strengths for each gradient bin direction
                for(int bin = 0; bin < binSize; bin++)
                {
                    gradientStrengths[celly][cellx][bin] /= NrUpdateEachCell;
                }
            }
        }
        
        std::cout << "descriptorDataIdx : " << descriptorDataIdx << std::endl;
        
        //draw cells;
        for (int celly = 0; celly < cellsY; celly++)
        {
            for(int cellx = 0; cellx < cellsX; cellx++)
            {
                int drawX = cellx * cellSize;
                int drawY = celly * cellSize;
                
                int mx = drawX + cellSize/2;
                int my = drawY + cellSize/2;
                
                cv::rectangle(visualHOG, cv::Point(drawX * zoom, drawY * zoom), cv::Point((drawX + cellSize) * zoom, (drawY + cellSize)) * zoom, CV_RGB(100,100,100), 1);
                
                // draw in each cell all 9 gradient strengths
                for(int bin = 0; bin < binSize; bin ++)
                {
                    float currentGradStrength = gradientStrengths[celly][cellx][bin];
                    
                    // no line to draw?
                    if (currentGradStrength == 0)
                        continue;
                    
                    float currRad = bin * radRangeEachBin + radRangeEachBin/2;
                    
                    float dirVecX = std::cos( currRad );
                    float dirVecY = std::sin( currRad );
                    float maxVecLen = cellSize/2;
                    float scale = 2.5f;// just a visualization scale, to see the lines better
                    
                    // compute line coordinates
                    float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                    float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                    float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                    float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
                    
                    // draw gradient visulization
                    cv::line(visualHOG, cv::Point(x1, y1) * zoom, cv::Point(x2, y2) * zoom, CV_RGB(0, 255, 0), 1); 
                } // for (all bins)
                
            }// for (cellx)
        } // for (celly)
        
        // don't forget to free memory allocated by helper data structures!
        for(int celly = 0; celly < cellsY; celly++)
        {
            for(int cellx = 0; cellx < cellsX; cellx++)
            {
                delete[] gradientStrengths[celly][cellx];
            }
            delete[] gradientStrengths[celly];
            delete[] cellUpdateCounter[celly];
        }
        delete[] gradientStrengths;
        delete[] cellUpdateCounter;
        
    } // GetVisualHOG
    
    
    /// HOG
    void HOG_trans(const cv::Mat &imageIPM)
    {
        int binSize = 9;
        // I don't know what it means.
        cv::HOGDescriptor hogtest = cv::HOGDescriptor(cv::Size(1,1), cv::Size(1,1), cv::Size(1,1), cv::Size(1,1),binSize);
        
        std::vector<float>descriptors;
        hogtest.compute(imageIPM, descriptors);
        std::cout << "HOG descriptor size is " << hogtest.getDescriptorSize() << std::endl;
        std::cout << "Found " << descriptors.size() << " descriptor values" << std::endl;
        
        int IPMrows = imageIPM.rows;
        int IPMcols = imageIPM.cols;
        cv::Mat imageHOG_tmp(IPMrows, IPMcols, CV_32F);
        cv::Mat imageHOG_01(IPMrows, IPMcols, CV_32F);
        
        
        for (int i=0; i<IPMrows; i++)
        {
            for (int j=0; j<IPMcols; j++)
            {
                for(int bin = 0; bin < binSize; bin++)
                {
                    imageHOG_tmp.at<float>(i, j) += descriptors.at(i*binSize*(IPMcols) + binSize*j + bin);
                }
                imageHOG_tmp.at<float>(i,j) /= binSize; 
                           
            }
        }
        
        double maxPixel, minPixel;
        cv::minMaxLoc(imageHOG_tmp, &minPixel, &maxPixel);
        std::cout << "min " << minPixel << " max " << maxPixel << std::endl;
        
        for (int i=0; i<IPMrows; i++)
        {
            for (int j=0; j<IPMcols; j++)
            {
                if (imageHOG_tmp.at<float>(i,j) > 0.5)
                    imageHOG_01.at<float>(i,j) = imageHOG_tmp.at<float>(i,j);
                else
                    imageHOG_01.at<float>(i,j) = 0;
            }
        }
        cv::imshow("temp", imageHOG_tmp);
        
        cv::Mat imageHOG_02 = cv::Mat(imageHOG_01.rows, imageHOG_02.cols, CV_8UC1);
        cv::threshold(imageHOG_01, imageHOG_02, 1, 255, cv::THRESH_BINARY);
        cv::imshow("HOG", imageHOG_01);   
        cv::imshow("HOG_th", imageHOG_02);
    }
    
}//namespace LaneDetector
