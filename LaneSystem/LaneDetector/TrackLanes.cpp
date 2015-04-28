//
//  TrackLanes.cpp
//  LaneDetector1.2
//
//  Created by LI XUANPENG on 09/13.
//  Copyright (c) 2012 ESIEE-Amiens. All rights reserved.
//
// !Tip: KalmanFilter demands a <float> pattern
//       Corresponding to KalmanFilter, Measure and State Mat must be a <float> pattern
//
//  Modified on 09/13
#include "TrackLanes.h"


extern const int    WIN_COLS;
extern const int    WIN_ROWS;

#define dtof(d) ((float)d)

namespace LaneDetector {
    /*
     * Kalman Filter Tracking
     * This function initialize the kalman filter with related params.
     * \param kalman kernel
     * \param current state: x(k)
     * \param process noise
     * \param measurement: y(k)
     */
    void InitLaneKalmanFilter(cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, int &laneKalmanIdx)
    {
        laneKalmanFilter.transitionMatrix = (cv::Mat_<float>(8,8) << 
                                              1,0,1,0,0,0,0,0,// rho_l, delta_rho_l
                                              0,1,0,1,0,0,0,0,// theta_l, delta_theta_l
                                              0,0,1,0,0,0,0,0,// delta_rho_l
                                              0,0,0,1,0,0,0,0,// delta_theta_l
                                              0,0,0,0,1,0,1,0,// rho_r, delta_rho_r
                                              0,0,0,0,0,1,0,1,// theta_r, delta_theta_r
                                              0,0,0,0,0,0,1,0,// delta_rho_r
                                              0,0,0,0,0,0,0,1);// delta_theta_r)
        
        cv::setIdentity(laneKalmanFilter.measurementMatrix);                        //A
        cv::setIdentity(laneKalmanFilter.processNoiseCov, cv::Scalar::all(1e-5));       //Q
        cv::setIdentity(laneKalmanFilter.measurementNoiseCov, cv::Scalar::all(1e-1));   //R
        cv::setIdentity(laneKalmanFilter.errorCovPost, cv::Scalar::all(1));             //P
        
        cv::randn(laneKalmanFilter.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
        
        /* Reset the State and Measure Mat */
        laneKalmanMeasureMat = cv::Mat::zeros(8, 1, CV_32F);
        // std::cout << "Reset Measure >> " << std::endl;
        // PrintMat(laneKalmanMeasureMat);

        laneKalmanIdx = 0;
    } 
    
    
    void TrackLanes_KF(const cv::Mat &laneMat, 
                       cv::KalmanFilter &laneKalmanFilter, cv::Mat &laneKalmanMeasureMat, 
                       const std::vector<cv::Vec2f> &hfLanes, const std::vector<cv::Vec2f> &lastHfLanes, 
                       std::vector<cv::Vec2f> &preHfLanes, std::vector<cv::Vec2f> &postHfLanes, 
                       const double &PITCH_ANGLE)
    {
        const int subX = 0;
        const int subY = cvRound(laneMat.rows * PITCH_ANGLE);
        
        cv::Mat subMat = laneMat(cv::Rect(subX, subY, laneMat.cols, laneMat.rows-subY));
        
        
        // Get value of current state: {rho, theta, delta_rho, delta_theta}x2
        switch ((int)hfLanes.size()) {
            case 2:
            {      
                laneKalmanMeasureMat.at<float>(0) = dtof(hfLanes.at(0)[0]);//rho of lane 1
                laneKalmanMeasureMat.at<float>(1) = dtof(hfLanes.at(0)[1]);//theta of lane 1
                laneKalmanMeasureMat.at<float>(4) = dtof(hfLanes.at(1)[0]);//rho of lane 2
                laneKalmanMeasureMat.at<float>(5) = dtof(hfLanes.at(1)[1]);//theta of lane 2
                
                switch ((int)lastHfLanes.size()) {
                    case 2:
                        laneKalmanMeasureMat.at<float>(2) = dtof(hfLanes.at(0)[0])-dtof(lastHfLanes.at(0)[0]);//delta_rho of lane 1
                        laneKalmanMeasureMat.at<float>(3) = dtof(hfLanes.at(0)[1])-dtof(lastHfLanes.at(0)[1]);//delta_theta of lane 1
                        laneKalmanMeasureMat.at<float>(6) = dtof(hfLanes.at(1)[0])-dtof(lastHfLanes.at(1)[0]);//delta_rho of lane 2
                        laneKalmanMeasureMat.at<float>(7) = dtof(hfLanes.at(1)[1])-dtof(lastHfLanes.at(1)[1]);//delta_theta of lane 2
                        break;
                        
                    case 1:
                        if (lastHfLanes.at(0)[1] < CV_PI/2 && lastHfLanes.at(0)[1] >= 0) {
                            // last left line 
                            laneKalmanMeasureMat.at<float>(2) = dtof(hfLanes.at(0)[0])-dtof(lastHfLanes.at(0)[0]);//delta_rho
                            laneKalmanMeasureMat.at<float>(3) = dtof(hfLanes.at(0)[1])-dtof(lastHfLanes.at(0)[1]);//delta_theta
                            laneKalmanMeasureMat.at<float>(6) = 0;//no change in rho
                            laneKalmanMeasureMat.at<float>(7) = 0;//no change in theta 
                        }
                        else {
                            // last right line
                            laneKalmanMeasureMat.at<float>(2) = 0;
                            laneKalmanMeasureMat.at<float>(3) = 0;
                            laneKalmanMeasureMat.at<float>(6) = dtof(hfLanes.at(1)[0])-dtof(lastHfLanes.at(0)[0]);
                            laneKalmanMeasureMat.at<float>(7) = dtof(hfLanes.at(1)[1])-dtof(lastHfLanes.at(0)[1]);
                            
                        }
                        break;
                    case 0:
                        laneKalmanMeasureMat.at<float>(2) = 0;
                        laneKalmanMeasureMat.at<float>(3) = 0;
                        laneKalmanMeasureMat.at<float>(6) = 0;
                        laneKalmanMeasureMat.at<float>(7) = 0;
                        break;
                    default:
                        break;
                }
            }
                break;
                
            case 1:
            {
                if (hfLanes[0][1] >= 0) //! left lanes
                {   
                    //! Predict the left one
                    laneKalmanMeasureMat.at<float>(0) = dtof(hfLanes[0][0]);
                    laneKalmanMeasureMat.at<float>(1) = dtof(hfLanes[0][1]);
                    switch ((int)lastHfLanes.size()) {
                        case 2:
                            laneKalmanMeasureMat.at<float>(2) = dtof(hfLanes[0][0])-dtof(lastHfLanes[0][0]);
                            laneKalmanMeasureMat.at<float>(3) = dtof(hfLanes[0][1])-dtof(lastHfLanes[0][1]);
                            break;
                            
                        case 1:
                            if (lastHfLanes[0][1] >= 0) 
                            {   //last left lanes
                                laneKalmanMeasureMat.at<float>(2) = dtof(hfLanes[0][0])-dtof(lastHfLanes[0][0]);
                                laneKalmanMeasureMat.at<float>(3) = dtof(hfLanes[0][1])-dtof(lastHfLanes[0][1]);
                            } else {
                                laneKalmanMeasureMat.at<float>(2) = 0;
                                laneKalmanMeasureMat.at<float>(3) = 0;
                            }
                            break;
                            
                        default:
                            break;
                    }
                    
                    //! Predict the right one
                    laneKalmanMeasureMat.at<float>(6) = 0;
                    laneKalmanMeasureMat.at<float>(7) = 0;
                    
                }
                else //! right lane
                {
                    //! Predict the right one
                    laneKalmanMeasureMat.at<float>(4) = dtof(hfLanes[0][0]);
                    laneKalmanMeasureMat.at<float>(5) = dtof(hfLanes[0][1]);
                    
                    switch ((int)lastHfLanes.size()) {
                        case 2:
                            laneKalmanMeasureMat.at<float>(6) = dtof(hfLanes[0][0])-dtof(lastHfLanes[1][0]);
                            laneKalmanMeasureMat.at<float>(7) = dtof(hfLanes[0][1])-dtof(lastHfLanes[1][1]);
                        break;
                            
                        case 1:
                            if (lastHfLanes[0][1] < 0 ) 
                            {
                                laneKalmanMeasureMat.at<float>(6) = dtof(hfLanes[0][0])-dtof(lastHfLanes[0][0]);
                                laneKalmanMeasureMat.at<float>(7) = dtof(hfLanes[0][1])-dtof(lastHfLanes[0][1]);
                            } else {
                                laneKalmanMeasureMat.at<float>(6) = 0;
                                laneKalmanMeasureMat.at<float>(7) = 0;
                            }
                            break;
                            
                        default:
                            break;
                    }
                    
                    //! Predict the left one
                    laneKalmanMeasureMat.at<float>(2) = 0;
                    laneKalmanMeasureMat.at<float>(3) = 0;
                }
            }
                break;
            
            default:
                break; 
        }
        
//        std::cout << "Lane Kalman Measure Mat:" << std::endl;
//        PrintMat(laneKalmanMeasureMat);
        
        // 2.kalman prediction
        const cv::Mat laneKalmanStatePreMat = laneKalmanFilter.predict(); 
//        cout << "laneKalmanPredictMat: " << endl;
//        for (int i = 0; i < 8; i++)cout << laneKalmanPredictMat.at<float>(i) << endl;
        
        // Predict param of left and right lines (statePre)
        cv::Vec2f leftPreHfLane = cv::Vec2f(laneKalmanStatePreMat.at<float>(0), laneKalmanStatePreMat.at<float>(1));
        //printf("left line predict: rho %f, theta %f.\n", leftPredictLane.rho, leftPredictLane.theta);
        cv::Vec2f rightPreHfLane = cv::Vec2f(laneKalmanStatePreMat.at<float>(4),laneKalmanStatePreMat.at<float>(5));
        //printf("right line predict: rho %f, theta %f.\n", rightPredictLane.rho, rightPredictLane.theta);
        
        preHfLanes.push_back(leftPreHfLane);
        preHfLanes.push_back(rightPreHfLane);
        
        
        // 3.update with measurement for Kalman Gain (K), error covariance (P), and statePost(x(k)) 
        laneKalmanFilter.correct(laneKalmanMeasureMat);
//        cout << "State Post Mat" << endl;
//        for(int i = 0; i < 8; i++) cout << laneKalmanFilter.statePost.at<float>(i) << endl;
        
        // Correct param of left and right lines (statePost)
        cv::Vec2f leftPostHfLane = cv::Vec2f(laneKalmanFilter.statePost.at<float>(0), laneKalmanFilter.statePost.at<float>(1));
        //printf("left line predict: rho %f, theta %f.\n", leftPredictLine.rho, leftPredictLine.theta);
        cv::Vec2f rightPostHfLane = cv::Vec2f(laneKalmanFilter.statePost.at<float>(4),laneKalmanFilter.statePost.at<float>(5));
        
        postHfLanes.push_back(leftPostHfLane);
        postHfLanes.push_back(rightPostHfLane);
    }
    
    
    
    /****************************************************************/
    // Particle Filter Tracking
    /****************************************************************/
    //! Based on a 2-degree polynomial curve fitting model
    double SIGMA0 = 0.5;
    double SIGMA1 = 0.005;
    double SIGMA2 = 0.00001;
    
    bool particle_cmp(const PARTICLE_LANE &p1, const PARTICLE_LANE &p2)
    {
        return p1.weight > p2.weight;
    }
    
    //! For every tracking, the particles don't record for the next frame, 
    //! but they are constructed by the parameters \leftCoefs and \rightCoefs.
    void TrackLanes_Particle(const cv::Mat &ipmMat, 
                             const LaneDetectorConf &laneDetectorConf,
                             const int &samplingNum, 
                             cv::Mat &leftCoefs, cv::Mat &rightCoefs, 
                             std::vector<cv::Point2d> &leftSampledPoints, 
                             std::vector<cv::Point2d> &rightSampledPoints,
                             double &laneWidth)
    {  
        cv::Mat thMat = ipmMat.clone();
        LaneDetector::IPMPreprocess(thMat, laneDetectorConf, thMat);
        
        //! Just for drawing
        cv::Mat colorMat = thMat.clone();
        cv::cvtColor(colorMat, colorMat, cv::COLOR_GRAY2BGR);
        
        //! From last frame
        double a0_l = leftCoefs.at<double>(0,0);
        double a1_l = leftCoefs.at<double>(1,0);
        double a2_l = leftCoefs.at<double>(2,0);
        double a0_r = rightCoefs.at<double>(0,0);
        double a1_r = rightCoefs.at<double>(1,0);
        double a2_r = rightCoefs.at<double>(2,0);
        
        cv::RNG rng;
        std::vector<double> samplingA0_l, samplingA1_l, samplingA2_l;
        std::vector<double> samplingA0_r, samplingA1_r, samplingA2_r;
        for(int i = 0; i < samplingNum; i++) {
            //!Left Lane Sampling Cooefficients
            samplingA0_l.push_back(a0_l + rng.gaussian(SIGMA0));
            samplingA1_l.push_back(a1_l + rng.gaussian(SIGMA1));
            samplingA2_l.push_back(a2_l + rng.gaussian(SIGMA2));
            //!Right Lane Sampling Cooefficients
            samplingA0_r.push_back(a0_r + rng.gaussian(SIGMA0));
            samplingA1_r.push_back(a1_r + rng.gaussian(SIGMA1));
            samplingA2_r.push_back(a2_r + rng.gaussian(SIGMA2));
            
        }//end for
        
        
        std::vector<PARTICLE_LANE> leftParticles, rightParticles;
        cv::Mat colorMat2 = colorMat.clone();
        //! the inside loop has highest priority level.
        for(int i = 0; i < samplingNum; i++) {
            for(int j = 0; j < samplingNum; j++) {
                for(int k = 0; k < samplingNum; k++) {
                    PARTICLE_LANE leftParticle = {0, samplingA0_l[i], samplingA1_l[j], samplingA2_l[k]};
                    leftParticles.push_back(leftParticle);
                    PARTICLE_LANE rightParticle = {0, samplingA0_r[i], samplingA1_r[j], samplingA2_r[k]};
                    rightParticles.push_back(rightParticle);
                    
                    std::vector<cv::Point2d> sampledPoints;
                    cv::Mat coefs_l = (cv::Mat_<double>(3,1) << samplingA0_l[i], samplingA1_l[j], samplingA2_l[k]);
                    IPMDrawCurve(coefs_l, colorMat2, sampledPoints, CV_RGB(100,100,0));
                    std::vector<cv::Point2d> sampledPoints2;
                    cv::Mat coefs_r = (cv::Mat_<double>(3,1) << samplingA0_r[i], samplingA1_r[j], samplingA2_r[k]);
                    IPMDrawCurve(coefs_r, colorMat2, sampledPoints2, CV_RGB(0,100,100));
                }
            }
        }//end for
        imShowSub("6. Candidate Sampling", colorMat2, WIN_COLS, WIN_ROWS, 7);
        
        
        
        CV_Assert(thMat.type() == CV_8U || thMat.type() == CV_64F || thMat.type() == CV_32F);
        std::vector<cv::Point2d> pointSet;
        ExtractPointSet(thMat, pointSet);
        
        
        /* Calculate the weight of each particle */
        double startTime = (double)cv::getTickCount();
        
        int PARTICLE_NUM = (int)leftParticles.size();// The same to rightParticles.
        
        
        double disTh = 1;
        for(int i = 0; i < PARTICLE_NUM; i++){
            int leftFitPointNum = 0, rightFitPointNum = 0;
            for(int m = 0; m < (int)pointSet.size(); m++){
                double x0 = pointSet[m].x;
                double y0 = pointSet[m].y;
                double y_l = leftParticles[i].a0 + leftParticles[i].a1*x0 + leftParticles[i].a2*x0*x0;
                if(std::abs(y_l-y0) <= disTh)
                    leftFitPointNum++;
                
                double y_r = rightParticles[i].a0 + rightParticles[i].a1*x0 + rightParticles[i].a2*x0*x0;
                if(std::abs(y_r-y0) <= disTh)
                    rightFitPointNum++;
            }
            //cout << "Left Fitted Point Number: " << leftFitPointNum << endl;
            leftParticles.at(i).weight = leftFitPointNum;
            rightParticles.at(i).weight = rightFitPointNum;
        }//end 
        
        /* Normalize the weights */ 
        // double sum_l = 0.0, sum_r = 0.0;

        // for(int i = 0; i < PARTICLE_NUM; i++)
        // {
        //    sum_l += leftParticles[i].weight;
        //    sum_r += rightParticles[i].weight;
        // }

        // for(int j = 0; j < PARTICLE_NUM; j++) {
        //    leftParticles[j].weight /= sum_l;
        //    rightParticles[j].weight /= sum_r;
        // }
        
        sort(leftParticles.begin(), leftParticles.end(), particle_cmp);
        sort(rightParticles.begin(), rightParticles.end(), particle_cmp);
        
        /* Sampling Importance Resampling based on weights */ 
        // PARTICLE_LANE newLeftParticles[PARTICLE_NUM];
        // PARTICLE_LANE newRightParticles[PARTICLE_NUM];
        // //! Left
        // int np_l, k_l = 0;        
        // for(int i = 0; i < PARTICLE_NUM; i++)
        // {
        //    np_l = cvRound(leftParticles[i].weight * (double)PARTICLE_NUM);
        //    for(int j = 0; j < np_l; j++)
        //    {
        //        //! Copy the particles of higher importance 
        //        newLeftParticles[k_l++] = leftParticles[i];
        //        if(k_l == PARTICLE_NUM)
        //            goto LEFTEXITOUT;
        //    }
        // }
        // //! Complement the particles of highest importance        
        // while (k_l < PARTICLE_NUM) {
        //    newLeftParticles[k_l++] = leftParticles[0];
        // }

        // LEFTEXITOUT:
        // for(int i=0; i < PARTICLE_NUM; i++)
        // {
        //    leftParticles[i] = newLeftParticles[i];
        // }
        // sort(rightParticles.begin(), rightParticles.end(), particle_cmp);

        // //! Right 
        // int np_r, k_r = 0;
        // for(int i = 0; i < PARTICLE_NUM; i++)
        // {
        //    np_r = cvRound(rightParticles[i].weight * (double)PARTICLE_NUM);
        //    for(int j = 0; j < np_r; j++)
        //    {
        //        //! Copy the particles of higher importance
        //        newRightParticles[k_r++] = rightParticles[i];
        //        if(k_r == PARTICLE_NUM)
        //            goto RIGHTEXITOUT;
        //    }
        // }
        // //! Complement the particles of higher importance
        // while (k_r < PARTICLE_NUM) {
        //    newRightParticles[k_r++] = rightParticles[0];
        // }

        // RIGHTEXITOUT:
        // for(int i=0; i < PARTICLE_NUM; i++)
        // {
        //    rightParticles[i] = newRightParticles[i];
        // }
        // sort(rightParticles.begin(), rightParticles.end(), particle_cmp);

        
        /* Model Tracking Based on Weighted Average Method */ 
        // double A0_l, A1_l, A2_l;
        // double A0_r, A1_r, A2_r;
        // sum_l = 0.0, sum_r = 0.0;

        // for(int i = 0; i < PARTICLE_NUM; i++)
        // {
        //    sum_l += leftParticles[i].weight;
        //    sum_r += rightParticles[i].weight;
        // }

        // for(int j = 0; j < PARTICLE_NUM; j++) {
        //    leftParticles[j].weight /= sum_l;
        //    rightParticles[j].weight /= sum_r;
        // }

        // for(int i = 0; i < PARTICLE_NUM; i++)
        // {
        //    cout << "weight, a0, a1, a2: " << leftParticles[i].weight << "," <<leftParticles[i].a0 << "," << leftParticles[i].a1 << "," << leftParticles[i].a2 << endl;
           
        //    //!left
        //    A0_l += leftParticles[i].a0 * leftParticles[i].weight;
        //    A1_l += leftParticles[i].a1 * leftParticles[i].weight;
        //    A2_l += leftParticles[i].a2 * leftParticles[i].weight;
        //    A0_l += leftParticles[i].a0;
        //    A1_l += leftParticles[i].a1;
        //    A2_l += leftParticles[i].a2;
        // }
        // A0_l /= PARTICLE_NUM;
        // A1_l /= PARTICLE_NUM;
        // A2_l /= PARTICLE_NUM;

        // cout <<  "A0_l, A1_l, A2_l" << A0_l << "," << A1_l << "," << A2_l << endl;
        // std::vector<cv::Point2d> sampledPoints;
        // cv::Mat coefs_l = (cv::Mat_<double>(3,1) << A0_l, A1_l, A2_l);
        // IPMDrawCurve(coefs_l, colorMat, sampledPoints, CV_RGB(100,100,0));
        // cv::imshow("left", colorMat); cv::waitKey();

        // for(int i = 0; i < k_r; i++)
        // {
        //    //!right
        //    //! Method 2, Weighted Average
        //    A0_r += rightParticles[i].a0 * rightParticles[i].weight;
        //    A1_r += rightParticles[i].a1 * rightParticles[i].weight;
        //    A2_r += rightParticles[i].a2 * rightParticles[i].weight;
        //    //! Method 1, Average
        //    A0_r += rightParticles[i].a0;
        //    A1_r += rightParticles[i].a1;
        //    A2_r += rightParticles[i].a2;
        // }
        // A0_r /= PARTICLE_NUM;
        // A1_r /= PARTICLE_NUM;
        // A2_r /= PARTICLE_NUM;

        // std::vector<Point2d> sampledPoints2;
        // cv::Mat coefs_r = (cv::Mat_<double>(3,1) << A0_r, A1_r, A2_r);
        // IPMDrawCurve(coefs_r, colorMat, sampledPoints2, CV_RGB(0,100,100));
        // cv::imshow("right", colorMat); cv::waitKey();
        
        
        /* Show the first step tracking result */ 
        /* Weighted Average Results */
        // cv::Mat leftCoefs_temp = (cv::Mat_<double>(3,1) << A0_l, A1_l; A2_l);
        // cv::Mat rightCoefs_temp = (cv::Mat_<double>(3,1) << A0_r, A1_r, A2_r);
    
        /* Max Weighted Results */
        cv::Mat leftCoefs_temp = (cv::Mat_<double>(3,1) << leftParticles[0].a0, leftParticles[0].a1, leftParticles[0].a2);
        
        cv::Mat rightCoefs_temp = (cv::Mat_<double>(3,1) << rightParticles[0].a0, rightParticles[0].a1, rightParticles[0].a2);
        
        std::vector<cv::Point2d> leftSampledPoints_temp, rightSampledPoints_temp;
        IPMDrawCurve(leftCoefs_temp, colorMat, leftSampledPoints_temp, CV_RGB(100,100,0));
        IPMDrawCurve(rightCoefs_temp, colorMat, rightSampledPoints_temp, CV_RGB(100,100,0));
        
        double laneWidth_temp;
        MeasureLaneWidth(leftSampledPoints_temp, rightSampledPoints_temp, laneDetectorConf, laneWidth_temp);
       
        imShowSub("7.Tracking_1", colorMat, WIN_COLS, WIN_ROWS, 8);

        /*
         * Utilize the inherent property, the same distance between two lanes
         * First use the last measured laneWidth to generate the candidated ones
         * Compare the candidated lane marking group which can fit points most
         * Strategies: 1.the fitting point number too big or too small      
         */  
        /* Left to Right */
        double a0_r2l = rightParticles[0].a0 - laneWidth * laneDetectorConf.ipmStep;
        /* Right to Left */
        double a0_l2r = leftParticles[0].a0 + laneWidth * laneDetectorConf.ipmStep;

        /* The most fitted lane models above record the fitted points number */
        int rightFitPointNum2_l = 0, leftFitPointNum2_r = 0;
        for(int m = 0; m < (int)pointSet.size(); m++){
            double x0 = pointSet[m].x;
            double y0 = pointSet[m].y;
            double y_l2r = a0_l2r + leftParticles[0].a1*x0 + leftParticles[0].a2*x0*x0;
            if(std::abs(y_l2r-y0) <= disTh)
                rightFitPointNum2_l++;
            
            double y_r2l = a0_r2l + rightParticles[0].a1*x0 + rightParticles[0].a2*x0*x0;
            if(std::abs(y_r2l-y0) <= disTh)
                leftFitPointNum2_r++;
        }
        //cout << "Left Fitted Point Number: " << leftFitPointNum << endl;
        
        double maxFitNum_l = leftParticles[0].weight;
        double maxFitNum_r = rightParticles[0].weight;
        
    ORI:    int fitSum0 = cvRound(maxFitNum_l + maxFitNum_r);
        
    LtoR:   int fitSum1 = cvRound(maxFitNum_l + rightFitPointNum2_l);
        
    RtoL:   int fitSum2 = cvRound(leftFitPointNum2_r + maxFitNum_r);
        
        int maxSum = fitSum0 >= fitSum1 ? fitSum0 : fitSum1;
        maxSum = maxSum >= fitSum2 ? maxSum : fitSum2;
        
        char * text_ORI = new char[50];
        sprintf(text_ORI, "ORIN: %d, L: %d, R: %d, laneWidth: %.3f", fitSum0, (int)maxFitNum_l, (int)maxFitNum_r, laneWidth_temp);
        cv::putText(colorMat, text_ORI, cv::Point(5, 10), cv::FONT_HERSHEY_SIMPLEX, 0.35, CV_RGB(0, 255, 0));
        delete text_ORI;
        
        char *text_LtoR = new char[50];
        sprintf(text_LtoR, "LtoR: %d, L: %d, R: %d", fitSum1, (int)maxFitNum_l, (int)rightFitPointNum2_l);
        cv::putText(colorMat, text_LtoR, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.35, CV_RGB(0, 255, 0));
        delete text_LtoR;
        
        char *text_RtoL = new char[50];
        sprintf(text_RtoL, "RtoL: %d, L: %d, R: %d", fitSum2, (int)leftFitPointNum2_r, (int)maxFitNum_r);
        cv::putText(colorMat, text_RtoL, cv::Point(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.35, CV_RGB(0, 255, 0));
        delete text_RtoL;


        char *text_fit = new char[50];
        // Finite State Machine
        if (maxSum == fitSum0 && laneWidth_temp >= 2.5 && laneWidth_temp <= 4.2 ) {
            //! Prevent the fitting point number too small
            sprintf(text_fit, "ORI");
            leftCoefs = (cv::Mat_<double>(3,1) << leftParticles[0].a0, leftParticles[0].a1, leftParticles[0].a2);
            
            rightCoefs = (cv::Mat_<double>(3,1) << rightParticles[0].a0, rightParticles[0].a1, rightParticles[0].a2);
        } 
        else if(maxSum == fitSum1) {
            sprintf(text_fit, "L -> R");
            leftCoefs = (cv::Mat_<double>(3,1) << leftParticles[0].a0, leftParticles[0].a1, leftParticles[0].a2);
            
            rightCoefs = (cv::Mat_<double>(3,1) << a0_l2r, leftParticles[0].a1, leftParticles[0].a2);
        }
        else {
            sprintf(text_fit, "R -> L");
            leftCoefs = (cv::Mat_<double>(3,1) << a0_r2l, rightParticles[0].a1, rightParticles[0].a2);
            
            rightCoefs = (cv::Mat_<double>(3,1) << rightParticles[0].a0, rightParticles[0].a1, rightParticles[0].a2);
        }
        
        IPMDrawCurve(leftCoefs, colorMat, leftSampledPoints, CV_RGB(255,0,0));
        IPMDrawCurve(rightCoefs, colorMat, rightSampledPoints, CV_RGB(0,255,0));
    
        cv::putText(colorMat, text_fit, cv::Point(5, 40), cv::FONT_HERSHEY_SIMPLEX, 0.35, CV_RGB(0, 255, 0));
        delete text_fit;
        
               
        
        //! Update the estimated lane width
        MeasureLaneWidth(leftSampledPoints, rightSampledPoints, laneDetectorConf, laneWidth);
        char *text_width = new char[50];
        sprintf(text_width, "LaneWidth: %.3f", laneWidth);
        cv::putText(colorMat, text_width, cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 0.35, CV_RGB(0, 255, 0));
        delete text_width;
        
        
        imShowSub("8.Tracking_2", colorMat, WIN_COLS, WIN_ROWS, 9);

        
        double costTime = ((double)cv::getTickCount() - startTime)/cv::getTickFrequency();
        printf("Particle Filter Tracking with %.2f msec about %.2f Hz\n", costTime*1000, 1/costTime);
 
    }//end TrackLanes_Particle

}//namespace LaneDetector
