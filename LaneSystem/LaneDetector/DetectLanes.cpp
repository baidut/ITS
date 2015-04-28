////
//  detectlane.cpp
//  LaneDetector1.2
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//  Modified on 09/13

#include "DetectLanes.h"

extern const double COEF;

extern const int    WIN_COLS;
extern const int    WIN_ROWS;

extern const int    DEBUG_LANE = 0;
extern const int    DEBUG_HOUGH = 0;

extern const int    TH_KALMANFILTER; 

namespace LaneDetector{
    void IPMPreprocess(const cv::Mat &ipmMat, const LaneDetectorConf &laneDetectorConf, cv::Mat &thMat)
    {
        imShowSub("0.Raw", ipmMat, WIN_COLS, WIN_ROWS, 1);
        // cv::Mat multiImage;
        // std::vector<cv::Mat> ipmMatT;
        // std::vector<std::string> winName;
        // ipmMatT.push_back(ipmMat); winName.push_back("IPM_Raw ");


        // //! Reduce the noise.
        // cv::blur(ipmMat, ipmMat, Size(3,3));

        // //! filter2D
        // cv::Mat filterMat;
        // cv::Mat kernel  = (cv::Mat_<double>(3,3)<< 1, -2, 1,
        //                                    2, -4, 2,
        //                                    1, -2, 1 );
        // cv::Mat kernelSobel  = (cv::Mat_<double>(3,3)<< -1, 0, 1,
        //                                    -2, 0, 2,
        //                                    -1, 0, 1 );//This kernel has the same effect like Sobel
        // cv::Mat kernelScharr  = (cv::Mat_<double>(3,3)<< -3, 0, 3,
        //                                    -10, 0, 10,
        //                                    -3, 0, 3 );//This kernel has the same effect like Scharr
        // cv::filter2D(ipmMat, filterMat, -1, kernel);
        // //ipmMatT.push_back(filterMat); winName.push_back("filter2D_0 ");

        // cv::Mat filterMat1;
        // cv::Mat kernel1  = (cv::Mat_<double>(3,3)<< 1, -2, 1,
        //                                     2, -3, 2,
        //                                     1, -2, 1 );
        // cv::filter2D(ipmMat, filterMat1, -1, kernel1);
        // //ipmMatT.push_back(filterMat1); winName.push_back("filter2D_1 ");

        // cv::Mat filterMat2;
        // cv::Mat kernel2  = (cv::Mat_<double>(3,3)<< 1, -2, 1,
        //                                     2, -5, 2,
        //                                     1, -2, 1 );
        // cv::filter2D(ipmMat, filterMat2, -1, kernel2);
        // //ipmMatT.push_back(filterMat2); winName.push_back("filter2D_-1 ");

        // cv::Mat filterMat3;
        // cv::Mat kernel3  = (cv::Mat_<double>(3,3)<< 2, -4, 2,
        //                                     4, -8, 4,
        //                                     2, -4, 2 );
        // cv::filter2D(ipmMat, filterMat3, -1, kernel3);
        // //ipmMatT.push_back(filterMat3); winName.push_back("filter2D_0_2 ");

        // cv::Mat filterMat4;
        // cv::Mat kernel4 = (cv::Mat_<double>(1,3) << -2, 0, 2);
        // cv::filter2D(ipmMat, filterMat4, -1, kernel4);
        // ipmMatT.push_back(filterMat4); winName.push_back("Center ");

        // //! Sobel
        // cv::Mat sobelMat;
        // cv::Sobel(ipmMat, sobelMat, -1, 1, 0, 3);
        // cv::imshow("sobel_x3", sobelMat);

        // cv::Mat sobelMat1;
        // cv::Sobel(ipmMat, sobelMat1, -1, 0, 1, 3);
        // cv::imshow("sobel_y3", sobelMat1);

        // cv::Mat sobelMat2;
        // cv::Sobel(ipmMat, sobelMat2, -1, 1, 0, 5);
        // cv::imshow("sobel_x5", sobelMat2);

        // cv::Mat sobelMat3;
        // cv::Sobel(ipmMat, sobelMat3, -1, 1, 1, 3);
        // cv::imshow("sobel_xy3", sobelMat3);
        // //! Scharr
        // cv::Mat scharrMat;
        // cv::Scharr(ipmMat, scharrMat, -1, 1, 0);
        // cv::imshow("Scharr_x3", scharrMat);

        // cv::waitKey();

        // //! Canny
        // cv::Mat cannyMat;
        // cv::Canny(ipmMat, cannyMat, 10, 50);imshow("Canny", cannyMat);
        // ipmMatT.push_back(cannyMat); winName.push_back("Canny(10:100) ");
        // cv::Mat cannyMat1;
        // cv::Canny(ipmMat, cannyMat1, 10, 70);
        // ipmMatT.push_back(cannyMat1); winName.push_back("Canny(10:70) ");
        // cv::Mat cannyMat2;
        // cv::Canny(ipmMat, cannyMat2, 10, 50);
        // ipmMatT.push_back(cannyMat2); winName.push_back("Canny(10:50) ");
        // cv::Mat cannyMat3;
        // cv::Canny(ipmMat, cannyMat3, 10, 30);
        // ipmMatT.push_back(cannyMat3); winName.push_back("Canny(10:30) ");

        // //! Laplacian
        // cv::Mat lapMat;
        // cv::Laplacian(ipmMat, lapMat, -1);
        // ipmMatT.push_back(lapMat); winName.push_back("Laplacian ");

        // //! Threshold
        // cv::Mat thMat, thMat1, thMat2, thMat3, thMat4,thMat5;
        // //medianBlur(filterMat, thMat, 3);
        // cv::GaussianBlur(filterMat3, thMat, Size(3,3), 1);
        // cv::threshold(thMat, thMat, 10, 255, THRESH_BINARY_INV);
        // //        ipmMatT.push_back(thMat); winName.push_back("filter2D02G_10 ");

        // //medianBlur(filterMat, thMat1, 3);
        // cv::GaussianBlur(filterMat3, thMat1, Size(3,3), 1);
        // cv::threshold(thMat1, thMat1, 30, 255, THRESH_BINARY_INV);
        // //        ipmMatT.push_back(thMat1); winName.push_back("filter2D02G_30 ");

        // //medianBlur(filterMat, thMat2, 3);
        // cv::GaussianBlur(filterMat3, thMat2, Size(3,3), 1);
        // cv::threshold(thMat2, thMat2, 50, 255, THRESH_BINARY_INV);
        // ipmMatT.push_back(thMat2); winName.push_back("filter2D02G_50 ");

        // //medianBlur(filterMat3, thMat3, 3);
        // cv::GaussianBlur(filterMat3, thMat3, Size(3,3), 1);
        // cv::threshold(thMat3, thMat3, 75, 255, THRESH_BINARY_INV);
        // //        ipmMatT.push_back(thMat3); winName.push_back("filter2D02G_75 ");

        // cv::GaussianBlur(filterMat4, thMat4, Size(3,3), 1);
        // cv::threshold(thMat4, thMat4, 20, 255, THRESH_BINARY_INV);
        // ipmMatT.push_back(thMat4); winName.push_back("filterCenter_75 ");

        // //medianBlur(sobelMat, thMat5, 3);
        // cv::GaussianBlur(sobelMat, thMat5, Size(3,3), 1);
        // cv::threshold(thMat5, thMat5, 20, 255, THRESH_BINARY_INV);
        // ipmMatT.push_back(thMat5); winName.push_back("sobel_x3 ");

        // cv::Mat ipmMatGG;
        // cv::GaussianBlur(ipmMatG, ipmMatGG, Size(3,3), 1);
        // imshow("1.1 Gauss x2", ipmMatGG);

        // cv::Mat dogMat2 = ipmMatG - ipmMatGG;
        // EnhanceContrast_LCE(dogMat2, dogMat2);
        // dogMat2 = cv::Mat::ones(ipmMat.size(), CV_8U)* 255 - dogMat2;
        // cv::Canny(dogMat2, dogMat2, 10, 100);
        // cv::imshow("2.1 DoG x2", dogMat2);

        // cv::Mat tempMat = dogMat + dogMat2;
        // imShowSub("DoG_Comb", tempMat, 3, 3, 7);  

        // //!filter2D(center kernel) + LCE
        // cv::Mat filterMat;
        // cv::Mat kernel  = (cv::Mat_<double>(3,1)<< -1, 0, 1 );
        // cv::filter2D(ipmMat, filterMat, -1, kernel);
        // EnhanceContrast_LCE(filterMat, filterMat);
        // filterMat = cv::Mat::ones(ipmMat.size(), CV_8U)* 255 - filterMat;
        
        /*
         * Get the contours in IPM image
         * It can be optimized using ipmMask
         */
        cv::Point2f p1, p2, p3, p4;
        //P1 (left top)
        for(int m = 0; m < ipmMat.rows; m++)
        {
            if ((int)ipmMat.at<uchar>(m, 0) != 0) {
                p1.y = m + 1;// Add offset
                p1.x = 0;
                break;
            }   
        }
        CV_Assert(p1.y != 0);
        
        //P2 (left bottom)
        for(int m = ipmMat.rows - 1; m > 0; m--)
        {
            
            if((int)ipmMat.at<uchar>(m, 0) != 0) {
                p2.y = m - 1;
                p2.x = 0;
                break;
            }
        }
        CV_Assert(p2.y != 0);
        
        //P3 & P4 (right top & bottom)
        if ((int)ipmMat.at<uchar>(0, ipmMat.cols-1) != 0)
        {
            for(int n = ipmMat.cols-2; n > 0; n--)
            {
                if ((int)ipmMat.at<uchar>(0, n) == 0){
                    p4.x = n + 1;
                    p4.y = 0;
                    p3.x = n + 1;
                    p3.y = ipmMat.rows - 1;
                    break;
                }
            }
        } 
        else 
        {
            for(int m = 1; m < ipmMat.rows; m++)
            {
                if((int)ipmMat.at<uchar>(m, ipmMat.cols-1) != 0)
                {
                    p4.x = ipmMat.cols - 1;
                    p4.y = m + 1;
                    p3.x = ipmMat.cols - 1;
                    p3.y = ipmMat.rows - 1 - (m + 1);
                    break;
                } 
            }
        }
        CV_Assert(p3.x!=0 && p4.x!=0);
        
        std::vector<cv::Point2f> contour;
        contour.push_back(p1);
        contour.push_back(p2);
        contour.push_back(p3);
        contour.push_back(p4);
        
        #if 0    
        cv::Mat ipmCopy = ipmMat.clone();
        cv::line(ipmCopy, p1, p2, Scalar(255));
        cv::line(ipmCopy, p2, p3, Scalar(255));
        cv::line(ipmCopy, p3, p4, Scalar(255));
        cv::line(ipmCopy, p4, p1, Scalar(255));
        cv::imshow("ipm", ipmCopy);//cv::waitKey();
        #endif

        /*
         * The steps of edge detection
         */
        //cv::equalizeHist( ipmMat, ipmMat );
        //EnhanceContrast_LCE(ipmMat, ipmMat);
        cv::Mat sobelMat;
        cv::Sobel(ipmMat, sobelMat, -1, 0, 1, 3);
        for(int i = 0; i < ipmMat.rows; i++) {
            for(int j = 0; j < ipmMat.cols; j++) {
                if (pointPolygonTest(contour, cv::Point2f(j,i), false) <= 0)
                {
                    sobelMat.at<uchar>(i,j) = 0;
                }
            }
        }
        imShowSub("1.Sobel", sobelMat, WIN_COLS, WIN_ROWS, 2);
        
        //!DoG
        cv::Mat gaussMat;
        double sigmaX = 1.0, sigmaY = 1.0;
        cv::GaussianBlur(sobelMat, gaussMat, cv::Size(3,3), sigmaX, sigmaY);
        imShowSub("2.Gauss", gaussMat, WIN_COLS, WIN_ROWS, 3);
        
        cv::Mat dogMat = sobelMat - gaussMat;
        //! Work in Contour????
        for(int i = 0; i < ipmMat.rows; i++) {
            for(int j = 0; j < ipmMat.cols; j++) {
                if (pointPolygonTest(contour, cv::Point2f(j,i), false) <= 0)
                {
                    dogMat.at<uchar>(i,j) = 0;
                }
            }
        }
        imShowSub("3.DoG", dogMat, WIN_COLS, WIN_ROWS, 4);
        
        //! Local Contrast Enhancement
        cv::Mat lceMat;
        EnhanceContrast_LCE(dogMat, lceMat);
        imShowSub("4.LCE", lceMat, WIN_COLS, WIN_ROWS, 5);
        
        
        cv::threshold(lceMat, thMat, 50, 255, cv::THRESH_BINARY);
        //Canny(lceMat, thMat, 10, 100);  
        imShowSub("5.Threshold", thMat, WIN_COLS, WIN_ROWS, 6);   
    }//end IPMPreprocess
    
    
    void IPMDetectLanes(const cv::Mat &ipmMat, 
                        const LaneDetectorConf &laneDetectorConf, 
                        std::vector<Lane> &leftIPMLanes, std::vector<Lane> &rightIPMLanes,
                        cv::Mat &leftCoefs, cv::Mat &rightCoefs,
                        std::vector<cv::Point2d> &leftSampledPoints, std::vector<cv::Point2d> &rightSampledPoints,
                        double &laneWidth)
    {  
        /* Preprossing Step */
        cv::Mat thMat;
        IPMPreprocess(ipmMat, laneDetectorConf, thMat);
        
        /* Set the ROI, considering the noise in the initialization step */
        #if 1

            double interval  = 3.0; //meter
            int yROI = cvRound((double)ipmMat.rows/2 -  interval * laneDetectorConf.ipmStep);
            
            for (int i = 0; i < yROI; i++)
            {
                thMat.row(i) = cv::Scalar(0);
            }
            for(int j = thMat.rows - 1; j > thMat.rows - yROI; j--)
            {
                thMat.row(j) = cv::Scalar(0);
            }
            // int xROI = 0;
            // int wROI = ipmMat.cols;
            // int hROI = cvRound(2 * interval * laneDetectorConf.ipmStep);
            // Rect roi = Rect(xROI,yROI,wROI,hROI);
            // thMat = thMat(roi);

        #endif
        
        cv::Mat colorMat = ipmMat.clone();
        cv::cvtColor(colorMat, colorMat, cv::COLOR_GRAY2RGB);
        cv::line(colorMat, cv::Point(0, colorMat.rows/2), cv::Point(colorMat.cols, colorMat.rows/2), CV_RGB(0, 0, 255));
        imShowSub("6.Division", colorMat, WIN_COLS, WIN_ROWS, 7);
        
        /* Hough Transform Step (Optional) */        
        #if 0
        //! Standard Hough Transform.
        std::vector<cv::Vec2f> hfLanesCandi;
        int maxLineGap = 50;
        cv::HoughLines(thMat, hfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, maxLineGap); 
       
        std::vector<cv::Vec2f> leftHfLanes;
        std::vector<cv::Vec2f> rightHfLanes;
        //printf("Num of hfLinesCandi: %d\n", (int)hfLinesCandi.size());
        for (size_t i = 0; i < hfLanesCandi.size(); i++)
        {
            double thetaCandi = (double)hfLanesCandi[i][1];
            double rhoCandi = (double)hfLanesCandi[i][0];
            // std::cout << "Theta: " << thetaCandi << ", Rho: " << rhoCandi << std::endl;
            
            if(cos(thetaCandi) < cos(CV_PI/2 - CV_PI/30)  && cos(thetaCandi) > cos(CV_PI/2 + CV_PI/30))// && rhoCandi > laneDetectorConf.mIPM * 0.3 && rhoCandi < laneDetectorConf.mIPM * 0.7
            {   
                if (rhoCandi < thMat.rows/2 - 10)
                    leftHfLanes.push_back(hfLanesCandi.at(i)); 
                else
                    rightHfLanes.push_back(hfLanesCandi.at(i));                
                //hfLanes.push_back(Vec2f(hfLanesCandi.at(i)[0], hfLanesCandi.at(i)[1]));
            }
        }
        
        double rho, theta;
        int lanesNum;
        if (!leftHfLanes.empty()) {
            // sort in sequence
            sort(leftHfLanes.begin(), leftHfLanes.end(), sort_smaller);
            // Find correct line from list of HoughLine;  
            lanesNum = (int)leftHfLanes.size();
            // Matched lines may be the middle of all candidate lines.
            rho = leftHfLanes.at(lanesNum/2)[0];
            theta = leftHfLanes.at(lanesNum/2)[1];
            
            leftHfLanes.clear();
            leftHfLanes.push_back(Vec2f(rho, theta));
            HfLanetoLane(thMat, leftHfLanes, leftIPMLanes);
        }
        
        if (!rightHfLanes.empty()) {
            // sort in sequence
            sort(rightHfLanes.begin(), rightHfLanes.end(), sort_smaller);
            // Find correct line from list of HoughLine;  
            lanesNum = (int)rightHfLanes.size();
            rho = rightHfLanes.at(lanesNum/2)[0];
            theta = rightHfLanes.at(lanesNum/2)[1];
            
            rightHfLanes.clear();
            rightHfLanes.push_back(Vec2f(rho, theta));
            HfLanetoLane(thMat, rightHfLanes, rightIPMLanes);
        } 
        
        //! Draw the candidated lanes
        std::vector<Lane> drawLanes;
        if (!leftHfLanes.empty()) {
            HfLanetoLane(thMat, leftHfLanes, drawLanes);
            for(size_t i = 0; i < leftHfLanes.size(); i++) {
                cv::line(colorMat, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(255, 0, 0), 1);
            }
            drawLanes.clear();
        }
        
        if(!rightHfLanes.empty()) {
            HfLanetoLane(thMat, rightHfLanes, drawLanes);
            for(size_t i = 0; i < drawLanes.size(); i++) {
                cv::line(colorMat, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0, 255, 0), 1); 
            }
            drawLanes.clear();
        }
        #endif
                
        
        #if 0       
        //! Probabilistic Hough Transform.
        std::vector<cv::Vec4i> phfLanesCandi;
        double MinLength = 5;
        double MaxGap = 300;
        cv::HoughLinesP( thMat, phfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 10, MinLength, MaxGap );
        
        for( size_t i = 0; i < hfLanesCandi.size(); i++ )
        {
            cv::line( colorMat, cv::Point(phfLanesCandi[i][0], phfLanesCandi[i][1]),
                 cv::Point(phfLanesCandi[i][2], phfLanesCandi[i][3]), CV_RGB(0,0,255), 1);
        }
        
        #endif
        
        /* Curve Fitting Step */
        #if 1
        std::vector<cv::Point2d> leftPointSet, rightPointSet;
        
        int termNum = 3;
        int minDataNum = 20;
        int iterNum = 100;
        double thValue  = 1;
        
        cv::Rect leftROI = cv::Rect(0, 0, thMat.cols, thMat.rows/2);
        cv::Mat leftThMat = thMat(leftROI);
        ExtractPointSet(leftThMat, leftPointSet);
        if(!leftPointSet.empty())
        {
            int leftCloseDataNum = 50;
            // FittingCurve_LS(leftPointSet, termNum, leftCoefs);PrintMat(leftCoefs);
            FittingCurve_RANSAC(leftPointSet, termNum, minDataNum, iterNum, thValue, leftCloseDataNum, leftCoefs, colorMat);
            IPMDrawCurve(leftCoefs, colorMat, leftSampledPoints, CV_RGB(255, 0, 0));
        }
        
        cv::Rect rightROI = cv::Rect(0, thMat.rows/2, thMat.cols, thMat.rows/2);
        cv::Mat rightThMat = thMat(rightROI);
        ExtractPointSet(rightThMat, rightPointSet);
        if(!rightPointSet.empty())
        {
            for(int i = 0; i < (int)rightPointSet.size(); i++)
            {
                rightPointSet.at(i).y += thMat.rows/2;
            }
            int rightCloseDataNum = 80;
            // FittingCurve_LS(rightPointSet, termNum, rightCoefs);
            FittingCurve_RANSAC(rightPointSet, termNum, minDataNum, iterNum, thValue, rightCloseDataNum, rightCoefs, colorMat);
            IPMDrawCurve(rightCoefs, colorMat, rightSampledPoints, CV_RGB(0, 255, 0));
        }
        
        MeasureLaneWidth(leftSampledPoints, rightSampledPoints, laneDetectorConf, laneWidth);
        
        imShowSub("7.Init Tracking", colorMat, WIN_COLS, WIN_ROWS, 8); 
        #endif  
    }//end IPMDetectLane
    
    
    /*
	 * This function detects lanes in the input image using IPM
	 * transformation and the input camera parameters. The returned lines
	 * are in a vector of Line objects, having start and end point in
	 * input image frame.
	 * \param input image
	 * \param original image
	 * \param camera information
	 * \param lane detect configuration  
	 */
	void DetectLanes(const cv::Mat &laneMat,
                     const LaneDetectorConf &laneDetectorConf, 
                     const int &offsetX,
                     const int &offsetY,
                     std::vector<cv::Vec2f> &hfLanes,
                     std::vector<cv::Vec2f> &pHfLanes, 
                     const int &laneKalmanIdx,
                     const double &isChangeLane)
	{
        cv::Mat roiMat = laneMat(cv::Rect(offsetX, offsetY, laneMat.cols-2*offsetX, laneMat.rows-offsetY));
 
        /* Edge Detection */
        FilterLanes(roiMat, laneDetectorConf);
        
        /*
         * Utilize the last tracked lane position to predict the current ones
         * If change lanes, the ROI region will not work; 
         * It should redefine the ROI
         * First Left, Second Right
         */
        cv::Mat lLaneMask = cv::Mat(roiMat.rows, roiMat.cols, CV_8U, cv::Scalar::all(0));
        cv::Mat rLaneMask = cv::Mat(roiMat.rows, roiMat.cols, CV_8U, cv::Scalar::all(0));
        
        int isROI = 0;// Indicator whether Hough execute in the ROI
        if(laneKalmanIdx > TH_KALMANFILTER && isChangeLane == 0 && pHfLanes.size() == 2)
        {
            // std::cout << "Set Up ROI" << std::endl;

            /* Predict the current possible area from last tracked range */
            const int proStartXRange = laneDetectorConf.top_range; //!pixel
            const int proEndXRange = laneDetectorConf.bottom_range; //!pixel
            // std::cout << proStartXRange << std::endl;
            // std::cout << proEndXRange << std::endl;

            std::vector<Lane> lastLanes;
            HfLanetoLane(roiMat, pHfLanes, lastLanes);
            
            /* The Coordinate reversed to the normal xy coordinates */
            int lStartX = cvRound(lastLanes[0].startPoint.x);
            double lY = lastLanes[0].endPoint.y;
            int lEndX = 0;
            if(pHfLanes[0][1] > 0) {
                lEndX = cvRound((lY - roiMat.rows) * tan(pHfLanes[0][1]));
            } else {
                lEndX = roiMat.cols - cvRound((lY - roiMat.rows) * -tan(pHfLanes[0][1]));
            }
                
            /* Reversed to normal xy coordinates & Move to right edge of image */
            int rStartX = cvRound(lastLanes[1].startPoint.x);
            double rY = lastLanes[1].endPoint.y;
            int rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * tan(CV_PI - pHfLanes[1][1]));
            if(pHfLanes[1][1] > 0) {
                rEndX = cvRound((rY - roiMat.rows) * tan(pHfLanes[1][1]));
            } else {
                rEndX = roiMat.cols - cvRound((rY - roiMat.rows) * -tan(pHfLanes[1][1]));
            }

            cv::Point lPoly[1][4], rPoly[1][4];
            int npoly[] = {4};

            /* Left Side */
            lPoly[0][0] = cv::Point(lStartX+proStartXRange, 0);
            lPoly[0][1] = cv::Point(lStartX-proStartXRange, 0);
            lPoly[0][2] = cv::Point(lEndX-proEndXRange, roiMat.rows);
            lPoly[0][3] = cv::Point(lEndX+proEndXRange, roiMat.rows);
            const cv::Point *plPoly[1] = {lPoly[0]};
            cv::fillPoly(lLaneMask, plPoly, npoly, 1, cv::Scalar(255));
            
            /* Right Side */
            rPoly[0][0] = cv::Point(rStartX+proStartXRange, 0);
            rPoly[0][1] = cv::Point(rStartX-proStartXRange, 0);
            rPoly[0][2] = cv::Point(rEndX-proEndXRange, roiMat.rows);
            rPoly[0][3] = cv::Point(rEndX+proEndXRange, roiMat.rows);
            const cv::Point *prPoly[1] = {rPoly[0]};
            cv::fillPoly(rLaneMask, prPoly, npoly, 1, cv::Scalar(255));
            
            // cv::imshow("lLaneMask", lLaneMask);
            // cv::moveWindow("lLaneMask", 0, 0);
            // cv::imshow("rLaneMask", lLaneMask + rLaneMask);
            // cv::moveWindow("rLaneMask", 0, 400);
            
            isROI = 1;
        }
        
        
        /* Lane Extraction */
		GetHfLanes(roiMat, laneDetectorConf, hfLanes, lLaneMask, rLaneMask, isROI, isChangeLane);
    }
	
	/**
	 * This function extract the edge via filter (sobel or canny)
	 */
	void FilterLanes(cv::Mat &laneMat, const LaneDetectorConf &laneDetectorConf)
	{
        cv::Mat mat1, mat2, mat3, finalMat;
        int filterType = SOBEL_FILTER_2;
		switch (filterType) //laneDetectorConf.filterType
        {
            case SOBEL_FILTER_1:
                cv::Sobel(laneMat, mat1, CV_8U, 1, 1, 3);              
                if(DEBUG_LANE)imShowSub("1.Sobel", mat1, WIN_COLS, WIN_ROWS, 1);
                
                cv::medianBlur(mat1, mat2, 3);                      
                if(DEBUG_LANE)imShowSub("2.Blur", mat2, WIN_COLS, WIN_ROWS, 2);
                
                cv::threshold(mat2, finalMat, 40, 255, cv::THRESH_BINARY);  
                if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 3);
    
                break;
                
			case SOBEL_FILTER_2:
                cv::Sobel(laneMat, mat1, CV_8U, 1, 0, 3);
                if(DEBUG_LANE)imShowSub("1.Sobel_X", mat1, WIN_COLS, WIN_ROWS, 1);
                
                cv::Sobel(mat1, mat2, CV_8U, 0, 1, 3);
                if(DEBUG_LANE)imShowSub("2.Sobel_Y", mat2, WIN_COLS, WIN_ROWS, 2);
                
                cv::medianBlur(mat2, mat3, 3);
                if(DEBUG_LANE)imShowSub("3.Median", mat3, WIN_COLS, WIN_ROWS, 3);
                
                cv::threshold(mat3, finalMat, 100, 255, cv::THRESH_BINARY);
                if(DEBUG_LANE)imShowSub("4.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);

				break;
                
            case SOBEL_FILTER_3:
                cv::Sobel(laneMat, mat1, CV_8U, 0, 1, 3);
                if(DEBUG_LANE)imShowSub("1.Sobel_Y", mat1, WIN_COLS, WIN_ROWS, 1);
                
                cv::Sobel(mat1, mat2, CV_8U, 1, 0, 3);
                if(DEBUG_LANE)imShowSub("2.Sobel_X", mat2, WIN_COLS, WIN_ROWS, 2);
                
                cv::medianBlur(mat2, mat3, 3);
                if(DEBUG_LANE)imShowSub("3.Median", mat3, WIN_COLS, WIN_ROWS, 3);
                
                cv::threshold(mat3, finalMat, 150, 255, cv::THRESH_BINARY);
                if(DEBUG_LANE)imShowSub("4.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);
  
				break;
                
			case CANNY_FILTER:
				//cvCanny demands the 8UC1(0~255) 
				cv::Canny(laneMat, finalMat, 10, 100, 3);
                if(DEBUG_LANE)imShowSub("1.Canny", finalMat, WIN_COLS, WIN_ROWS, 1);
         
				break;
                
			case LAPLACE_FILTER:
				cv::Laplacian(laneMat, mat1, CV_8U, 3);
                if(DEBUG_LANE)imShowSub("1.Laplacian", mat1, WIN_COLS, WIN_ROWS, 1);
                
                cv::medianBlur(mat1, mat2, 3);
                if(DEBUG_LANE)imShowSub("2.Median", mat2, WIN_COLS, WIN_ROWS, 2);
                
                cv::threshold(mat2, finalMat, 30, 255, cv::THRESH_BINARY);
                if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 3);
                
				break;
                
            case DOG_FILTER:
                cv::GaussianBlur(laneMat, mat1, cv::Size(3,3), 1);
                mat2 = laneMat - mat1;
                if(DEBUG_LANE)imShowSub("1.DOG", mat2, WIN_COLS, WIN_ROWS, 1);
                
               // cv::GaussianBlur(mat2, mat3, cv::Size(3,3), 1);
               // if(DEBUG_LANE)imShowSub("2.Blur", mat3, WIN_COLS, WIN_ROWS, 2);
                
                cv::threshold(mat2, finalMat, 5, 255, cv::THRESH_BINARY);
                if(DEBUG_LANE)imShowSub("3.Threshold", finalMat, WIN_COLS, WIN_ROWS, 4);
                
                break;
                
            default:
                break;
		}
        
        finalMat.copyTo(laneMat);  
    }//end FilterLanes
    
    

	/*
     * This function extracts lines from the passed infiltered and thresholded
	 * image
	 * \param the input thresholded filtered image
	 * \param a vector of lines
	 * \param the configure of lines structure
	 */
	void GetHfLanes(cv::Mat &laneMat, 
                    const LaneDetectorConf &laneDetectorConf, 
                    std::vector<cv::Vec2f> &hfLanes,
                    const cv::Mat &lLaneMask,
                    const cv::Mat &rLaneMask,
                    const int &isROI, const int &isChangeLane)
	{
        std::vector<cv::Vec2f> hfLanesCandi; 
        
        std::vector<cv::Vec2f> leftHfLanesCandi;
        std::vector<cv::Vec2f> leftHfLanes;
        std::vector<cv::Vec2f> rightHfLanesCandi;
        std::vector<cv::Vec2f> rightHfLanes;
        
        std::vector<cv::Point2d> lLanePts;
        std::vector<cv::Point2d> rLanePts;
        
        //std::cout << "isROI: " << isROI << std::endl;

        if(!isROI)
        {
           /*
            * Detect the lane in whole image
            * \param isChangeLane indicates the direction for ROI
            */
            cv::HoughLines(laneMat, hfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0); 
            
            /* Adjust the parameters when theta > CV_PI/2 */
            for(size_t i = 0; i < hfLanesCandi.size(); i++) {
                hfLanesCandi[i][1] = hfLanesCandi[i][0] > 0 ? hfLanesCandi[i][1] : hfLanesCandi[i][1]-(float)CV_PI;
                hfLanesCandi[i][0] = std::abs(hfLanesCandi[i][0]);
            }
           

            for (size_t i = 0; i < hfLanesCandi.size(); i++)
            {
                double thetaCandi = hfLanesCandi[i][1];
                double rhoCandi = hfLanesCandi[i][0];
                
                //! Solution considering for changing lanes
                if(rhoCandi > laneDetectorConf.rhoMin) {
                    if(isChangeLane == 0) {
                        //! car keep in the center
                        if( thetaCandi >= 0 && thetaCandi < laneDetectorConf.thetaMax) 
                            leftHfLanes.push_back(hfLanesCandi.at(i)); // line in region from 0 to 90 degree
                        else if(thetaCandi > -laneDetectorConf.thetaMax && thetaCandi < 0)
                            rightHfLanes.push_back(hfLanesCandi.at(i));// line in region from -90 to 0 degree
                    } 
                    else if(isChangeLane == 1) {
                        //!car towards right
                        if(thetaCandi <= laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
                            leftHfLanes.push_back(hfLanesCandi.at(i));
                        else if(thetaCandi < -laneDetectorConf.thetaMin && thetaCandi >  -laneDetectorConf.thetaMax)
                            rightHfLanes.push_back(hfLanesCandi.at(i));
                    }
                    else if(isChangeLane == -1) {
                        //!car towards left
                        if(thetaCandi >= laneDetectorConf.thetaMin && thetaCandi < laneDetectorConf.thetaMax)
                            leftHfLanes.push_back(hfLanesCandi.at(i));
                        else if(thetaCandi < laneDetectorConf.thetaMin && thetaCandi > -laneDetectorConf.thetaMin)
                            rightHfLanes.push_back(hfLanesCandi.at(i));
                    }
                }
            }
            
            /* 
             * Sort in sequence
             * Find correct line from list of HoughLine 
             * Matched lines may be the middle of all candidate lines
             */
            int lanesNum;
            float rho, theta;
            if (!leftHfLanes.empty()) {
                sort(leftHfLanes.begin(), leftHfLanes.end(), sort_smaller);
                lanesNum = (int)leftHfLanes.size();
                rho = leftHfLanes.at(lanesNum/2)[0];
                theta = leftHfLanes.at(lanesNum/2)[1];
                hfLanes.push_back(cv::Vec2f(rho, theta));
            }
            if (!rightHfLanes.empty()) {
                sort(rightHfLanes.begin(), rightHfLanes.end(), sort_smaller);
                lanesNum = (int)rightHfLanes.size();
                rho = rightHfLanes.at(lanesNum/2)[0];
                theta = rightHfLanes.at(lanesNum/2)[1];
                hfLanes.push_back(cv::Vec2f(rho, theta));
            }   
            
            
            /* Draw the lane candidates of Hough transform */
            if (DEBUG_HOUGH) {
                std::vector<Lane> drawLanes;
                cv::Mat laneMatRGB;
                cv::cvtColor(laneMat, laneMatRGB, cv::COLOR_GRAY2RGB);
                
                if (!leftHfLanes.empty()) {
                    HfLanetoLane(laneMat, leftHfLanes, drawLanes);
                    // cv::threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//draw
                    for(size_t i = 0; i < drawLanes.size(); i++) {
                        cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
                    }
                }
                
                if(!rightHfLanes.empty()) {
                    drawLanes.clear();
                    HfLanetoLane(laneMat, rightHfLanes, drawLanes);
                    for(size_t i = 0; i < drawLanes.size(); i++)
                        cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
                }
                
                 imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
                //cv::moveWindow("Hough Lanes Candi", 790, 0);
            }
        } 
        else 
        {
            /* Detect the lane in the ROI, set by the tracked lane */
            //std::cout << "Detect Lanes in ROI" << std::endl;
            
            /* Threshold of distance value to fix the most fitted hfLane */
            const int TH_DIS = 10;
            
            /* Left ROI */
            cv::Mat lLaneMat = laneMat & lLaneMask;
            cv::HoughLines(lLaneMat, leftHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0); 


                
            for(size_t i = 0; i < leftHfLanesCandi.size(); i++) {
                leftHfLanesCandi[i][1] = leftHfLanesCandi[i][0] > 0 ? leftHfLanesCandi[i][1] : leftHfLanesCandi[i][1]-(float)CV_PI;
                leftHfLanesCandi[i][0] = std::abs(leftHfLanesCandi[i][0]);
                
                if(leftHfLanesCandi[i][0] > laneDetectorConf.rhoMin && 
                    leftHfLanesCandi[i][1] < laneDetectorConf.thetaMax &&
                    leftHfLanesCandi[i][1] > -laneDetectorConf.thetaMin)
                    leftHfLanes.push_back(leftHfLanesCandi[i]);   
            }
            
            ExtractPointSet(lLaneMat, lLanePts);
                
            if(!leftHfLanes.empty()) {
                std::vector<Lane> leftLanes;
                HfLanetoLane(laneMat, leftHfLanes, leftLanes);
                
                int pos = 0;
                int maxFitNum = 0;
                //! Lane Candidates
                for(size_t i = 0; i < leftLanes.size(); i++)
                {
                    double xIntercept = leftLanes[i].startPoint.x;
                    double yIntercept = leftLanes[i].endPoint.y;
                    int fitNum = 0;
                    
                    //! Points 
                    for(size_t j = 0; j < lLanePts.size(); j++) {
                        double dis = std::abs(xIntercept*lLanePts[j].y + yIntercept*lLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);
                        
                        if(dis  < TH_DIS) 
                            fitNum ++;
                    }
                    
                    if (maxFitNum < fitNum) {
                        maxFitNum = fitNum; 
                        pos = i;
                    }
                }
                
                hfLanes.push_back(leftHfLanes[pos]);
            }
            
            
            //! Right ROI
            cv::Mat rLaneMat = laneMat & rLaneMask;
            cv::HoughLines(rLaneMat, rightHfLanesCandi, laneDetectorConf.rhoStep, laneDetectorConf.thetaStep, 20, 0 ,0); 
            for(size_t i = 0; i < rightHfLanesCandi.size(); i++) {
                rightHfLanesCandi[i][1] = rightHfLanesCandi[i][0] > 0 ? rightHfLanesCandi[i][1] : rightHfLanesCandi[i][1]-(float)CV_PI;
                rightHfLanesCandi[i][0] = std::abs(rightHfLanesCandi[i][0]);
                
                if(rightHfLanesCandi[i][0] > laneDetectorConf.rhoMin && 
                   rightHfLanesCandi[i][1] > -laneDetectorConf.thetaMax &&
                   rightHfLanesCandi[i][1] < laneDetectorConf.thetaMin)
                    rightHfLanes.push_back(rightHfLanesCandi[i]);   
            }
            
            ExtractPointSet(rLaneMat, rLanePts);
            
            if(!rightHfLanes.empty()) 
            {
                std::vector<Lane> rightLanes;
                HfLanetoLane(laneMat, rightHfLanes, rightLanes);
                
                int pos = 0;
                int maxFitNum = 0;
                //! Lane Candidates
                for(int i = 0; i < (int)rightLanes.size(); i++)
                {
                    double xIntercept = rightLanes[i].startPoint.x;
                    double yIntercept = rightLanes[i].endPoint.y;
                    int fitNum = 0;
                    //! Points 
                    for(int j = 0; j < (int)rLanePts.size(); j++)
                    {
                        double dis = std::abs(xIntercept*rLanePts[j].y + yIntercept*rLanePts[j].x - xIntercept*yIntercept) / (xIntercept * xIntercept + yIntercept * yIntercept);
                        
                        if(dis  < TH_DIS) 
                            fitNum ++;
                    }
                    
                    if (maxFitNum < fitNum)
                    {
                        maxFitNum = fitNum; 
                        pos = i;
                    }
                }
                hfLanes.push_back(rightHfLanes[pos]);
            }
            
            //! Draw the lane candidates of Hough transform
            if (DEBUG_HOUGH) {
                std::vector<Lane> drawLanes;
                cv::Mat laneMatRGB;
                cv::cvtColor(laneMat, laneMatRGB, cv::COLOR_GRAY2RGB);
                
                if (!leftHfLanes.empty()) {
                    HfLanetoLane(laneMat, leftHfLanes, drawLanes);
                    // cv::threshold(laneMatRGB, laneMatRGB, 0, 255, 1);//1
                    for(size_t i = 0; i < drawLanes.size(); i++)
                        cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(0,0,200), 1);
                }
                if(!rightHfLanes.empty()) {
                    drawLanes.clear();
                    HfLanetoLane(laneMat, rightHfLanes, drawLanes);
                    for(size_t i = 0; i < drawLanes.size(); i++)
                        cv::line(laneMatRGB, drawLanes[i].startPoint, drawLanes[i].endPoint, CV_RGB(200,0,0), 1);
                }
                 imShowSub("Hough Lanes Candi", laneMatRGB,  WIN_COLS, WIN_ROWS, 5);
            }
            
            /* Debug: draw the ROI */
            if(DEBUG_HOUGH) {
                cv::Mat roiMat = lLaneMat + rLaneMat;
                cv::imshow("roiMat", roiMat);
            }
        }

	}// end GetHFLanes
    
    
    /* big to small */
    bool sort_smaller(const cv::Vec2f &lane1, const cv::Vec2f &lane2){
        return lane1[1] > lane2[1];
    }


    void GetLateralOffset(const cv::Mat &laneMat, 
                          const double &leftPoint, 
                          const double &rightPoint, 
                          double &lateralOffset)
    {
        // std::cout << "LeftPoint :" << leftPoint << " RightPoint :" << rightPoint << std::endl;
        double laneCenter = (leftPoint + rightPoint)/2;
        double carCenter = (laneMat.cols)/2;//point of car center seen as mid of x(point) in image
        double carOffset = carCenter - laneCenter;//- left + right

        double distance = std::abs(leftPoint - rightPoint);//lane width in pixel 
        double LANE_WIDTH = 3.5;
        double CAR_WIDTH = 1.7;
        double maxLateralOffset = (CAR_WIDTH/LANE_WIDTH) * distance/2.0;
        lateralOffset = carOffset / maxLateralOffset;
        lateralOffset = lateralOffset > 1 ? 1 : lateralOffset;
        lateralOffset = lateralOffset < -1 ? -1 : lateralOffset;

		// std::cout << "GetLateralOffset >> maxLateralOffset :" << maxLateralOffset << std::endl;
    }//end GetLateralOffset

    enum database{ KIT = 1, ESIEE };
    
    // Default setting due to lack of enough information
	void InitlaneDetectorConf(const cv::Mat &laneMat, LaneDetectorConf &laneDetectorConf, const int database)
	{
        /* run IPM */
        laneDetectorConf.isIPM = 1; //1 open, 0 close
        /* Parameters of configuration of camera */
        laneDetectorConf.m = laneMat.rows * COEF;    //Rows (height of Image)
        laneDetectorConf.n = laneMat.cols * COEF;     //Columns (width of Image)
        laneDetectorConf.h = 1.15;              //Distance of camera above the ground (meters)
        laneDetectorConf.alphaTot = atan(3/12.5); //HALF viewing angle
        
        //! \param 6.7 for lane(data_130326)
        //! \param 5.5 for lane(data_121013)
        // laneDetectorConf.theta0 = CV_PI*(5.5/180);   //Camera tilted angle below the horizontal(positive)
        
        //! \params for lane (data_130710)
        laneDetectorConf.theta0 = CV_PI * (8.5/180.0); //the pitch angle
        
        laneDetectorConf.ipmX_max = 60.0;  //meters
        laneDetectorConf.ipmY_max = 12.0;  //meters
        laneDetectorConf.ipmY_min = -12.0; //meters
        laneDetectorConf.ipmStep = 8;      //pixels per meter
        laneDetectorConf.mIPM = (laneDetectorConf.ipmY_max - laneDetectorConf.ipmY_min) * laneDetectorConf.ipmStep;
        
		laneDetectorConf.kernelWidth = 2;
		laneDetectorConf.kernelHeight = 2;
		
		laneDetectorConf.groupingType = GROUPING_TYPE_HOUGH_LINES;
		laneDetectorConf.filterType = DOG_FILTER;
        
        laneDetectorConf.rhoMin  = 30;
		laneDetectorConf.rhoStep = 1;
		
		laneDetectorConf.thetaStep = CV_PI/180;

        switch(database){
            case KIT:
                laneDetectorConf.thetaMin = CV_PI * 0.25;//45 degree
                laneDetectorConf.thetaMax = CV_PI * 0.36; //72 degree
                laneDetectorConf.top_range = 20;
                laneDetectorConf.bottom_range = 70;

                laneDetectorConf.vpTop = laneMat.rows * 0.2 * COEF;
                laneDetectorConf.vpBottom = laneMat.rows * 0.6 * COEF;
                laneDetectorConf.distCornerMin = laneMat.cols * 0.2 * COEF;
                laneDetectorConf.distCornerMax = laneMat.cols * 0.5 * COEF;
                break;

            case ESIEE:
                laneDetectorConf.thetaMin = CV_PI / 4;//45 degree
                laneDetectorConf.thetaMax = CV_PI * 0.5; // / 9 * 4; //80 degree
                laneDetectorConf.top_range = 20;
                laneDetectorConf.bottom_range = 170;

                laneDetectorConf.vpTop = -laneMat.rows * 0.2 * COEF;
                laneDetectorConf.vpBottom = laneMat.rows * 0.3 * COEF;
                laneDetectorConf.distCornerMin = laneMat.cols * 0.5 * COEF;
                laneDetectorConf.distCornerMax = laneMat.cols * COEF * 2.5;
                break;

            default:
                break;
        }
     


	}//end InitlaneDetectorConf
    
    
    //! Local Constrast Enhancement based on Unshapen Masking(USM)
    //! This creates a local contrast mask which maps larger-scale 
    //! transitions than the small-scale edges which are mapped 
    //! when sharpening an image.
    void EnhanceContrast_LCE(const cv::Mat &inMat, cv::Mat &outMat, const int &threshold, const int &amount)
    {
        if(inMat.type() == CV_8U)//Gray
        {
            cv::Mat tempMat = inMat.clone();
            cv::Mat blurredMat;
            cv::GaussianBlur(tempMat, blurredMat, cv::Size(), 3);
            cv::Mat lowContrastMask = abs(tempMat - blurredMat) < threshold;
            //std::cout << "Threshold_LEC : " << threshold << std::endl;
            outMat = tempMat + (tempMat - blurredMat) * amount;
            
            //! Copy the contrast mask of original mat.
            tempMat.copyTo(outMat, lowContrastMask); 
            
        }
        else if(inMat.type() == CV_8UC3)//RGB
        {
            cv::Mat hsvMat;
            cv::cvtColor(inMat, hsvMat, cv::COLOR_RGB2HSV);
            cv::Mat vMat(hsvMat.size(), CV_8U);
            
            uchar *p = NULL;
            cv::Vec3b *q = NULL;
            for(int m = 0; m < vMat.rows; m++)
            {
                p = vMat.ptr<uchar>(m);
                q = hsvMat.ptr<cv::Vec3b>(m);
                for(int n = 0; n < vMat.cols; n++)
                {
                    cv::Vec3b pixels = q[n];
                    p[n] = pixels[2];
                }
            }
            cv::imshow("V-channel",vMat);
        }
    }//end EnhanceContrast
    
    
    
    void HfLanetoLane(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes, std::vector<Lane> &lanes)
    {
        lanes.clear();
        
        for( size_t i = 0; i < hfLanes.size(); i++)
        {
            cv::Point2d Pt1, Pt2;
            double rho = hfLanes.at(i)[0];
            double theta = hfLanes.at(i)[1];
             
            if( theta > 0 && theta < CV_PI/2 )
            {
                //! Pt1
                Pt1.y = 0; 
                Pt1.x = rho/cos(theta);
                // if(Pt1.x > laneMat.cols)
                // {
                //    Pt1.y = (Pt1.x - laneMat.cols) / tan(theta);
                //    Pt1.x = laneMat.cols;
                   
                // }
                //! Pt2
                Pt2.x = 0;
                Pt2.y = rho/sin(theta);
                // if(Pt2.y > laneMat.rows)
                // {
                //    Pt2.x = (Pt2.y - laneMat.rows) * tan(theta);
                //    Pt2.y = laneMat.rows;
                // }
            }
            else if(theta == CV_PI/2)
            {
                //! Pt1
                Pt1.x = laneMat.cols;
                Pt1.y = rho;
                //! Pt2
                Pt2.x = 0;
                Pt2.y = rho;
                
            }
            else if(theta == 0)
            {
                //!Pt1
                Pt1.y = 0;
                Pt1.x = std::abs(rho);
                //!Pt2
                Pt2.y = laneMat.rows;
                Pt2.x = std::abs(rho);
            }
            else if(theta > -CV_PI/2 && theta < 0)
            {
                //!Pt1
                Pt1.y = 0;
                Pt1.x = rho / cos(theta);
//                if(Pt1.x < 0)
//                {
//                    Pt1.y = std::abs(Pt1.x) * tan(theta - CV_PI/2);
//                    Pt1.x = 0;
//                }
                
                //!Pt2
                Pt2.x = laneMat.cols;
                Pt2.y = (laneMat.cols - Pt1.x) / -tan(theta);
//                if(Pt2.y > laneMat.rows )
//                {
//                    Pt2.x = laneMat.rows - (Pt2.y - laneMat.rows) * tan(CV_PI - theta);
//                    Pt2.y = laneMat.rows;
//                }
            }
    
            //std::cout << "ToLane: " << Pt1 << "," << Pt2 << std::endl;
            Lane tempLane = {Pt1, Pt2};//StartPt, EndPt
            lanes.push_back(tempLane);
        }
    }
    
    
    void GetMarkerPoints(const cv::Mat &laneMat, const std::vector<cv::Vec2f> &hfLanes, 
        cv::Point2d &vp, cv::Point2d &corner_l, cv::Point2d &corner_r, const int offsetX, const int offsetY)
    {
        std::vector<Lane> lanes;
        
        HfLanetoLane(laneMat, hfLanes, lanes);
    
        std::vector<double> k;// slope
        for (std::vector<Lane>::const_iterator iter = lanes.begin(); iter != lanes.end(); ++iter )
        {
            k.push_back((iter->startPoint.y - iter->endPoint.y)/(iter->startPoint.x - iter->endPoint.x));
        }
        
        
        if(!k.empty()) {
            vp.x = (k.at(0)*lanes.at(0).startPoint.x - k.at(1)*lanes.at(1).startPoint.x 
                    - lanes.at(0).startPoint.y + lanes.at(1).startPoint.y) / (k.at(0)-k.at(1)) + offsetX;
            
            vp.y = (k.at(0)*lanes.at(1).startPoint.y - k.at(1)*lanes.at(0).startPoint.y 
                    + k.at(0)*k.at(1)*lanes.at(0).startPoint.x - k.at(0)*k.at(1)*lanes.at(1).startPoint.x) / (k.at(0)-k.at(1)) + offsetY;
            
            corner_l.y = laneMat.rows;
            corner_r.y = laneMat.rows;
            corner_l.x = (1/k.at(0))*(corner_l.y-vp.y)+vp.x;
            corner_r.x = (1/k.at(1))*(corner_r.y-vp.y)+vp.x; 
        }
       
    }
    
    void DrawPreROI(cv::Mat &laneMat, 
                    const int offsetX,
                    const int offsetY,
                    const std::vector<cv::Vec2f> &pHfLanes, 
                    const int &laneKalmanIdx,
                    const int &isChangeLane,
                    const LaneDetectorConf &laneDetectorConf)
    {
        cv::Point2d  vp, corner_l, corner_r;
        std::vector<LaneDetector::Lane> pLanes;
        
        if(laneKalmanIdx > TH_KALMANFILTER && !pHfLanes.empty()) {
            //! ROI from predicted lanes
            GetMarkerPoints(laneMat, pHfLanes, vp, corner_l, corner_r, offsetX, offsetY);
            
            //! Kalman Predicted Lanes in Next Frame
            cv::circle(laneMat, cv::Point2d(vp.x, vp.y), 1, CV_RGB(255,0,0));
            cv::line(laneMat, cv::Point2d(vp.x, vp.y), cv::Point2d(corner_l.x, corner_l.y), CV_RGB(200, 0, 200), 2);
            cv::line(laneMat, cv::Point2d(vp.x, vp.y), cv::Point2d(corner_r.x, corner_r.y), CV_RGB(0, 200, 200), 2);
            
            if (!isChangeLane) {
                const int proStartXRange = laneDetectorConf.top_range;
                const int proEndXRange = laneDetectorConf.bottom_range;
                HfLanetoLane(laneMat, pHfLanes, pLanes);
                
                //! The Coordinate reversed to normal xy coordinates & Move to left edge of image
                int lStartX = cvRound(pLanes[0].startPoint.x);
                double lY = pLanes[0].endPoint.y;
                int lEndX;
                if(pHfLanes[0][1] > 0) {
                    lEndX = cvRound((lY - (laneMat.rows-offsetY)) * tan(pHfLanes[0][1]));
                } else {
                    lEndX = laneMat.cols - cvRound((lY - (laneMat.rows-offsetY)) * -tan(pHfLanes[0][1]));
                }
                
                cv::line(laneMat, cv::Point(lStartX + proStartXRange + offsetX, offsetY), 
                        cv::Point(lStartX - proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);
                cv::line(laneMat, cv::Point(lStartX - proStartXRange + offsetX, offsetY), 
                        cv::Point(lEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
                cv::line(laneMat, cv::Point(lEndX - proEndXRange + offsetX, laneMat.rows), 
                        cv::Point(lEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(100, 0, 100), 2);
                cv::line(laneMat, cv::Point(lEndX + proEndXRange + offsetX, laneMat.rows), 
                        cv::Point(lStartX + proStartXRange + offsetX, offsetY), CV_RGB(100, 0, 100), 2);
                
                
                //! Reversed to normal xy coordinates & Move to right edge of image
                int rStartX = cvRound(pLanes[1].startPoint.x);
                double rY = pLanes[1].endPoint.y;
                int rEndX;
                if(pHfLanes[1][1] > 0) {
                    rEndX = cvRound((rY - (laneMat.rows-offsetY)) * tan(pHfLanes[1][1]));
                } else {
                    rEndX = laneMat.cols - cvRound((rY - (laneMat.rows-offsetY)) * -tan(pHfLanes[1][1]));
                }
                
                cv::line(laneMat, cv::Point(rStartX + proStartXRange + offsetX, offsetY), 
                                cv::Point(rStartX - proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);
                cv::line(laneMat, cv::Point(rStartX - proStartXRange + offsetX, offsetY), 
                                cv::Point(rEndX - proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
                cv::line(laneMat, cv::Point(rEndX - proEndXRange + offsetX, laneMat.rows), 
                                cv::Point(rEndX + proEndXRange + offsetX, laneMat.rows), CV_RGB(0, 100, 100), 2);
                cv::line(laneMat, cv::Point(rEndX + proEndXRange + offsetX, laneMat.rows), 
                                cv::Point(rStartX + proStartXRange + offsetX, offsetY), CV_RGB(0, 100, 100), 2);
            
                // //! Fill the poly
                // int npoly[] = {4};
                // //! Left Side
                // cv::Point lPoly[1][4];
                // lPoly[0][0] = cv::Point(lStartX+proStartXRange, offsetY);
                // lPoly[0][1] = cv::Point(lStartX-proStartXRange, offsetY);
                // lPoly[0][2] = cv::Point(lEndX-proEndXRange, laneMat.rows);
                // lPoly[0][3] = cv::Point(lEndX+proEndXRange, laneMat.rows);
                // const cv::Point *plPoly[1] = {lPoly[0]};
                // cv::fillPoly(laneMat, plPoly, npoly, 1, CV_RGB(0, 100, 100));

                // //! Right Side
                // cv::Point rPoly[1][4];
                // rPoly[0][0] = cv::Point(rStartX+proStartXRange, offsetY);
                // rPoly[0][1] = cv::Point(rStartX-proStartXRange, offsetY);
                // rPoly[0][2] = cv::Point(rEndX-proEndXRange, laneMat.rows);
                // rPoly[0][3] = cv::Point(rEndX+proEndXRange, laneMat.rows);
                // const cv::Point *prPoly[1] = {rPoly[0]};
                // cv::fillPoly(laneMat, prPoly, npoly, 1, CV_RGB(0, 100, 100));

                // cv::Point poly[1][3];
                // poly[0][0] = cv::Point(vp.x, vp.y);
                // poly[0][1] = cv::Point(corner_l.x, corner_l.y);
                // poly[0][2] = cv::Point(corner_r.x, corner_r.y);
                // const cv::Point *ppoly[1] = {poly[0]};
                // int npoly[] = {3};
                // cv::fillPoly(laneMat, ppoly, npoly, 1, cv::Scalar(255, 0, 0));
            }
        }
    }
    
    
    void DrawMarker(cv::Mat &laneMat, 
                    const int offsetX,
                    const int offsetY,
                    const std::vector<cv::Vec2f> &hfLanes, 
                    const double &lateralOffset)
    {
        if ((int)hfLanes.size() == 2){
            cv::Point2d  vp, corner_l, corner_r;
            GetMarkerPoints(laneMat, hfLanes, vp, corner_l, corner_r, offsetX, offsetY);
            
            // Draw center marker of image
            cv::line(laneMat, cv::Point2d(laneMat.cols * 0.5, laneMat.rows * 0.95 ), cv::Point2d(laneMat.cols * 0.5, laneMat.rows-1), cv::Scalar(0,0,0), 2);
            
            // lane center point
            const double markerLO = (double)laneMat.cols/3.0;
            cv::Point2d centerPt;
            centerPt.x = laneMat.cols * 0.5 + lateralOffset * markerLO;
            centerPt.y = laneMat.rows;
            
            cv::Point  pt[1][4];
            pt[0][0] = cv::Point2d(laneMat.cols * 0.5, laneMat.rows * 0.95);
            pt[0][1] = cv::Point2d(laneMat.cols * 0.5, laneMat.rows);
            pt[0][2] = cv::Point2d(centerPt.x, centerPt.y);
            pt[0][3] = cv::Point2d(centerPt.x, laneMat.rows * 0.95);
            
            const cv::Point * ppt[1] = {pt[0]};
            int npt[] = {4};
            if (std::abs(lateralOffset) < 0.6 ){
                //green
                cv::fillPoly(laneMat, ppt, npt, 1, cv::Scalar(0,255,100));
            }
            else if(std::abs(lateralOffset) >= 0.6 && std::abs(lateralOffset) < 1){
                //yellow
                cv::fillPoly(laneMat, ppt, npt, 1, cv::Scalar(0,255,255));
            }
            else {
                //red
                cv::fillPoly(laneMat, ppt, npt, 1, cv::Scalar(0,0,255));
            }
            
            char *text_lateralOffset = new char[50];
            sprintf(text_lateralOffset, "%3.1f%%", lateralOffset*100);//%3.1f%%
            cv::putText(laneMat, text_lateralOffset, cv::Point2d((centerPt.x + laneMat.cols * 0.5)/2, (centerPt.y + laneMat.rows*0.97)/2), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,0));
            delete text_lateralOffset;
        }
    }// end DrawMarker
    
    void MeasureLaneWidth(const std::vector<cv::Point2d> &leftSampledPoints, const std::vector<cv::Point2d> &rightSampledPoints, const LaneDetectorConf &laneDetectorConf, double &laneWidth)
    {
        int sampledPointsNum = (int)leftSampledPoints.size();
        double disSum = 0.0;
        for (int i = 0 ; i < sampledPointsNum; i++) 
        {
            disSum += std::abs(leftSampledPoints[i].y - rightSampledPoints[i].y);
        }
        double distance  = disSum / sampledPointsNum; //pixels
        laneWidth = distance / laneDetectorConf.ipmStep;
        printf("@Two lane markings distance : %.2f, about %.2f meter.\n", distance , laneWidth);
    }
    
    
}//namespace LaneDetector

