//
//  GenerateLaneIndicators.h
//  LaneDetector1.2
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//

#ifndef LaneDetector_GenerateLaneIndicators_h
#define LaneDetector_GenerateLaneIndicators_h

#include <cmath>
#include <vector>
#include <iostream>
#include <deque>
#include <opencv2/opencv.hpp>

namespace LaneDetector{
    typedef struct _LaneFeature{
        int frame;
        double lateralOffset;       //Current lateral offset
        double LATSD;               //Standard deviation of lateral offset
        double LATSD_Baseline;      //Baseline of LATSD
        double LATMEAN;             //Average of lateral offset by EWMA
        double LATMEAN_Baseline;    //Baseline of LATMEAN
        double LANEDEV;             //standard deviation of LATMEAN by EWVAR
        double LANEDEV_Baseline;    //Baseline of LATDEV
        double LANEX;               //Fraction of lane exits
        //double LANEX_Baseline;      //Baseline of LANEX;
        double TLC;                 //Simple time of lane crossing
        int    TLC_2s;              //Number of times that TLC fall below 0.5s
        double TLCF_2s;             //Fraction of TLC below TLC_2s 
        int    TLC_halfs;           //Number of times that TLC fall below 2s
        double TLCF_halfs;          //Fraction of TLC below TLC_halfs
        double TLC_min;             //Global minimum of TLC over a given time interval
        double TOT;                 //Time on task
    } LaneFeature;

    /*
    * laneFeatures initialize
    */
    void InitlaneFeatures(LaneFeature &laneFeatures);
    
    typedef struct _InfoCar{
        double lateralOffset;   //lane lateral offset
        double intervalTime;
        double winTime;         //the time of fixed window
    }InfoCar;
    
    //! current TLC status  
    typedef struct _InfoTLC{
        double TLC;
        double intervalTime;
        double winTime;
    }InfoTLC;
    
    bool TLC_cmp(const InfoTLC &val1, const InfoTLC &val2);
    bool LATSD_cmp(const double &val1, const double &val2);
   
    void GetEWMA(const int &sampleIdx, int &muWindowSize, const double &inputParam, double &outputEWMA);
    
    void GetEWVAR(const int &sampleIdx, int &sigmaWindowSize, const double &inputParam, const double &inputEWMAParam, double &outputEWVAR);
    
    void GetStandardDeviation(const std::deque<InfoCar> &inputParam, double &deviation);
    
    void GetMeanSquareError(const std::deque<InfoCar> &inputParam, double &deviation);
    
	void GetLaneBaseline(const int &sampleIdx, 
                         const int &SAMPLING_TIME,
                         int &muWindowSize, int &sigmaWindowSize, 
                         const double &lateralOffset, 
                         std::vector<double> &LATSD_Baseline,
                         std::deque<InfoCar> &lateralOffsetDeque, 
                         std::deque <InfoCar> &LANEXDeque, 
                         std::deque<InfoTLC> &TLCDeque,
                         LaneFeature &laneFeatures,
                         const double &intervalTime);
    
    void GenerateLaneIndicators(const int &sampleIdx, 
                          const int &TIME_SAMPLING_WINDOW,
                          int &muWindowSize, int &sigmaWindowSize,
                          const double &lateralOffset,
                          std::deque<InfoCar> &lateralOffsetDeque,
                          std::deque <InfoCar> &LANEXDeque, 
                          std::deque<InfoTLC> &TLCDeque, 
                          LaneFeature &laneFeatures,
                          const double &intervalTime);
}

#endif
