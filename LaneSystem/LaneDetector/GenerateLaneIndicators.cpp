//
//  GenerateLaneIndicators.cpp
//  LaneDetector1.2
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
// 
//  /* 
//   * Lane and Vehicle Parameters
//   * Mean Lane Width on Freeway: 3.5 meter
//   * Mean Vehicle Width: 1.8 meter
//   * Tolerant offset: 0 ~ (+/-)0.85 meter
//   */
// 
//  Modified on 03/13

#include "GenerateLaneIndicators.h"

extern const int FRAME_START;
extern const int NUM_WINDOW_EWM;

namespace LaneDetector{
    void InitlaneFeatures(LaneFeature &laneFeatures)
    {
        laneFeatures.frame = 0;
        laneFeatures.lateralOffset = 0;
        laneFeatures.LATSD = 0;
        laneFeatures.LATSD_Baseline = 0;
        laneFeatures.LATMEAN = 0;
        laneFeatures.LATMEAN_Baseline = 0;
        laneFeatures.LANEDEV = 0;
        laneFeatures.LANEDEV_Baseline = 0;
        laneFeatures.LANEX = 0;
        laneFeatures.TLC = 0;
        laneFeatures.TLC_2s = 0;
        laneFeatures.TLCF_2s = 0;
        laneFeatures.TLC_halfs = 0;
        laneFeatures.TLCF_halfs = 0;
        laneFeatures.TLC_min = 0;
        laneFeatures.TOT = 0;
    }


    /******************************************************************************************/
    //! EWMA, EWVAR is just calculated according to the number of sampling.
    //! Exponentially Weighted Moving Average
    // \param sampleIdx: start with 1, and increase one by one sampling 
    // \param muWindowSize: start with 5, and increase one by one sampling
    // \param outputEWMA: each sampling input as last result, then update it
    //! Due to no triggering event, it will lead to a higher weight on the previous parameter, 
    //! But lower weight on the current parameter. 
    //! The problem is same in the EWVAR. 
    /******************************************************************************************/
    void GetEWMA(const int &sampleIdx, int &muWindowSize, const double &inputParam, double &outputEWMA)
    {
        if(sampleIdx < NUM_WINDOW_EWM) {
            outputEWMA = (double)(outputEWMA * (sampleIdx-1) + inputParam)/(double)sampleIdx;//!Average 
        }
        else {
            muWindowSize ++; 
            double muFactor = (double)(muWindowSize - 1)/(double)(muWindowSize);
            outputEWMA = muFactor * outputEWMA + (1 - muFactor) * inputParam;
        }
    }//end GetEWMA
    
    void GetEWVAR(const int &sampleIdx, int &sigmaWindowSize, const double &inputParam, const double &outputEWMA, double &outputEWVAR)
    {
        /// Start with window size about 5, increase by 1 for every sample
        if(sampleIdx < NUM_WINDOW_EWM) {
            outputEWVAR = (double)(outputEWVAR * (sampleIdx-1) + inputParam)/(double)sampleIdx;
        }
        else {
            sigmaWindowSize++;
            double sigmaFactor = (double)(sigmaWindowSize - 1)/(double)(sigmaWindowSize);
            outputEWVAR = sigmaFactor * outputEWVAR + (1 - sigmaFactor) * (inputParam - outputEWMA) * (inputParam - outputEWMA);//outputEWMA: use current EWMA
        }
    }//end GetEWVAR
    
    
    void GetStandardDeviation(const std::deque<InfoCar> &inputParam, double &deviation)
    {
        double mu = 0, inputSum = 0, deviationSum = 0;
        for(std::deque<InfoCar>::const_iterator iter = inputParam.begin(); iter != inputParam.end(); ++iter)
        {
            inputSum += iter->lateralOffset;
        }
        mu = inputSum/(double)inputParam.size();
        
        for(std::deque<InfoCar>::const_iterator iter = inputParam.begin(); iter != inputParam.end(); ++iter)
        {
            deviationSum = deviationSum + (iter->lateralOffset - mu) * (iter->lateralOffset - mu);
        }
        if (inputParam.size() > 1) {
            deviation  = sqrt(deviationSum/(double)(inputParam.size()-1));
        } else {
            deviation = 0;
        }
    }//end GetStandardDeviation
    
    void GetMeanSquareError(const std::deque<InfoCar> &inputParam, double &deviation)
    {
        double mu = 0, deviationSum = 0;
        for(std::deque<InfoCar>::const_iterator iter = inputParam.begin(); iter != inputParam.end(); ++iter)
        {
            deviationSum = deviationSum + (iter->lateralOffset - mu) * (iter->lateralOffset - mu);
        }
        if (inputParam.size() > 1) {
            deviation  = sqrt(deviationSum/(double)(inputParam.size()-1));
        } else {
            deviation = 0;
        }
    }//end GetMeanSquareError
    
    
    bool TLC_cmp(const InfoTLC &val1, const InfoTLC &val2) {
        return val1.TLC > val2.TLC;
    }
    
    bool LATSD_cmp(const double &val1, const double &val2) {
        return val1 > val2;
    }
    /*******************************************************************************/
    // This function access to the baseline of lateral offset related parameters.
    /*******************************************************************************/
    void GetLaneBaseline(const int &sampleIdx, 
                         const int &TIME_SAMPLING_WINDOW,
                         int &muWindowSize, int &sigmaWindowSize, 
                         const double &lateralOffset,
                         std::vector<double> &LATSD_Baseline,
                         std::deque<InfoCar> &lateralOffsetDeque, 
                         std::deque<InfoCar> &LANEXDeque, 
                         std::deque<InfoTLC> &TLCDeque,
                         LaneFeature &laneFeatures,
                         const double &intervalTime)
    {
        laneFeatures.frame = sampleIdx;
        laneFeatures.lateralOffset = lateralOffset;
        
/*********************************************************/
//! lateralOffsetDeque
/*********************************************************/
        InfoCar infoLO;
        infoLO.lateralOffset = lateralOffset;
        infoLO.intervalTime  = intervalTime;
        if(!lateralOffsetDeque.empty()) {
            infoLO.winTime = lateralOffsetDeque.back().winTime + intervalTime;
        }
        else {
            infoLO.winTime = intervalTime;
        }
        while (infoLO.winTime > TIME_SAMPLING_WINDOW && !lateralOffsetDeque.empty()) {
            infoLO.winTime -= lateralOffsetDeque.front().intervalTime;
            lateralOffsetDeque.pop_front();
        }
        lateralOffsetDeque.push_back(infoLO);
        
/******************************************************************/
//! Need to reset \param muWindowSize and \param sigmaWindowSize.
/******************************************************************/
        GetEWMA(sampleIdx, muWindowSize, lateralOffset , laneFeatures.LATMEAN);
        GetEWVAR(sampleIdx, sigmaWindowSize, lateralOffset , laneFeatures.LATMEAN, laneFeatures.LANEDEV);
        
/******************************************************************/
//! Standard Deviation of LATSD for Baseline
/******************************************************************/
        GetStandardDeviation(lateralOffsetDeque, laneFeatures.LATSD);
        LATSD_Baseline.push_back(laneFeatures.LATSD);
        sort(LATSD_Baseline.begin(), LATSD_Baseline.end(), LATSD_cmp);
        
/*******************************************************************************/
//! Considering the sampling Time of LANEX different that of lateralOffset 
//! Rebuild the InfoCar struct
/*******************************************************************************/
        InfoCar infoLANEX;
        infoLANEX.lateralOffset = lateralOffset;
        infoLANEX.intervalTime  = intervalTime;
        
        if(!LANEXDeque.empty()) {
            infoLANEX.winTime = LANEXDeque.back().winTime + intervalTime;
        }
        else {
            infoLANEX.winTime = intervalTime;
        }
        
        while (infoLANEX.winTime > TIME_SAMPLING_WINDOW) {
            infoLANEX.winTime -= LANEXDeque.front().intervalTime;
            LANEXDeque.pop_front();
        }
        LANEXDeque.push_back(infoLANEX);
        
        double sumTime_LANEX = 0;
        for(std::deque<InfoCar>::size_type i = 0; i != LANEXDeque.size(); ++i)
        {
            if(LANEXDeque[i].lateralOffset == 1)
            {
                sumTime_LANEX += LANEXDeque[i].intervalTime;
            }
        }
        double LANEX = sumTime_LANEX / LANEXDeque.back().winTime;
        

        if(sampleIdx > 1) {
/**************************************************************/
//! Add TLC into deque
//! Simple TLC 
/**************************************************************/
            double TLC = 0, TLC_min = 0;
            int TLC_2s = 0, TLC_halfs = 0;
            double TLCF_2s = 0, TLCF_halfs = 0;
            
            CV_Assert((int)lateralOffsetDeque.size() > 1);
            double lastLateralOffset = lateralOffsetDeque.at(lateralOffsetDeque.size()-2).lateralOffset;
            double lateralVelocity = lateralOffset - lastLateralOffset;
            
            double TLC_Frame = 0;
            if (lateralVelocity < 0)//direct to left
                TLC_Frame = std::abs((1 + lateralOffset) / lateralVelocity);
            else if(lateralVelocity > 0)//direct to right
                TLC_Frame = std::abs((1 - lateralOffset) / lateralVelocity);
            else
                TLC_Frame = 10000;//Max TLC shows a safe deviation 
            TLC = TLC_Frame / intervalTime;//sec
            TLC = TLC < 1000 ? TLC : 1000;
            
            InfoTLC infoTLC;
            infoTLC.TLC = TLC;
            infoTLC.intervalTime = intervalTime;
            if(!TLCDeque.empty()) {
                infoTLC.winTime = TLCDeque.back().winTime + intervalTime;
            }
            else {
                infoTLC.winTime = intervalTime;
            }
            while (infoTLC.winTime > TIME_SAMPLING_WINDOW) {
                infoTLC.winTime -= TLCDeque.front().intervalTime;
                TLCDeque.pop_front();
            }
            TLCDeque.push_back(infoTLC);
            
            
            //! Number of times that TLC below a threshold
            for(std::deque<double>::size_type i = 0; i != TLCDeque.size(); i++) {
                //!TLC with signal falling below 2s in a given time interval
                if(TLCDeque[i].TLC < 2.0) {
                    TLC_2s ++;
                }
                //!TLC with signal falling below 0.5s in a given time interval
                if(TLCDeque[i].TLC < 0.5) {
                    TLC_halfs ++;
                }
            }
            //! Fraction of TLC_2s and TLC_halfs
            TLCF_2s = (double)TLC_2s / (double)TLCDeque.size();
            TLCF_halfs = (double)TLC_halfs / (double)TLCDeque.size();
                        
            //! Global minimum of TLC over a given time interval;
            std::deque<InfoTLC> cmpTLCDeque = TLCDeque;
            sort(cmpTLCDeque.begin(), cmpTLCDeque.end(), TLC_cmp);
            TLC_min = cmpTLCDeque.back().TLC;
            std::deque<InfoTLC>::iterator iter = cmpTLCDeque.end();
            while (TLC_min == 0) { 
                --iter;
                TLC_min = iter->TLC;
            }
            
/**************************************************************/            
//! Calculate the Baseline
/**************************************************************/
            //!Baseline of LATSD should be the median value.
            laneFeatures.LATSD_Baseline = LATSD_Baseline.at(cvRound(LATSD_Baseline.size()/2.0));
            //!Others
            laneFeatures.LATMEAN_Baseline = laneFeatures.LATMEAN_Baseline > laneFeatures.LATMEAN ? laneFeatures.LATMEAN_Baseline : laneFeatures.LATMEAN;
            laneFeatures.LANEDEV_Baseline = laneFeatures.LANEDEV_Baseline > laneFeatures.LANEDEV ? laneFeatures.LANEDEV_Baseline : laneFeatures.LANEDEV;
            laneFeatures.LANEX = LANEX;
            laneFeatures.TLC = TLC;
            laneFeatures.TLC_2s = TLC_2s;
            laneFeatures.TLC_halfs = TLC_halfs;
            laneFeatures.TLC_min = TLC_min;
            laneFeatures.TLCF_2s = TLCF_2s;
            laneFeatures.TLCF_halfs = TLCF_halfs;
        } 
        else {
            laneFeatures.LATMEAN_Baseline   = 0;
            laneFeatures.LANEDEV_Baseline   = 0;
            laneFeatures.LATSD_Baseline     = 0;
            laneFeatures.LANEX              = 0;
            laneFeatures.TLC                = 0;
            laneFeatures.TLC_2s             = 0;
            laneFeatures.TLC_halfs          = 0;
            laneFeatures.TLC_min            = 0;
            laneFeatures.TLCF_2s            = 0;
            laneFeatures.TLCF_halfs         = 0;
        }//end if
    }//end GetLaneBaseline
    
    
    /***************************************************************************************/
    // This function builds the mass function based on lane features
    // \param lateralOffset: current lateral offset  
    // \param intervalTime: the interval time between the two consective frames
    //! Assign the mass from lane source 
    //! Standard deviation reflects the extent of wave about car. It will show the fatigue.
    /***************************************************************************************/
	void GenerateLaneIndicators(const int &sampleIdx, 
                          const int &TIME_SAMPLING_WINDOW,
                          int &muWindowSize, int &sigmaWindowSize,
                          const double &lateralOffset,
                          std::deque<InfoCar> &lateralOffsetDeque,
                          std::deque <InfoCar> &LANEXDeque, 
                          std::deque<InfoTLC> &TLCDeque, 
                          LaneFeature &laneFeatures,
                          const double &intervalTime)
    {
        double LATMEAN=laneFeatures.LATMEAN, LANEDEV=laneFeatures.LANEDEV, LATSD=0, LANEX=0, TLC=0, TLC_min=0;
        int TLC_2s = 0, TLC_halfs = 0;
        double TLCF_2s = 0, TLCF_halfs = 0;
        
/*********************************************************/
//! lateralOffsetDeque
/*********************************************************/
        InfoCar infoLO;
        infoLO.lateralOffset = lateralOffset;
        infoLO.intervalTime  = intervalTime;
        if(!lateralOffsetDeque.empty()) {
            infoLO.winTime = lateralOffsetDeque.back().winTime + intervalTime;
        }
        else {
            infoLO.winTime = intervalTime;
        }
        while (infoLO.winTime > TIME_SAMPLING_WINDOW && !lateralOffsetDeque.empty()) {
            infoLO.winTime -= lateralOffsetDeque.front().intervalTime;
            lateralOffsetDeque.pop_front();
        }
        lateralOffsetDeque.push_back(infoLO);
        
        
/******************************************************************/
//! Need to reset \param muWindowSize and \param sigmaWindowSize.
/******************************************************************/        
        GetEWMA(sampleIdx, muWindowSize, lateralOffset, LATMEAN);
        GetEWVAR(sampleIdx, sigmaWindowSize, lateralOffset, LATMEAN, LANEDEV);
        GetStandardDeviation(lateralOffsetDeque, LATSD);
        std::cout << "LO: " << lateralOffset << "LATSD: " << LATSD << std::endl;
/******************************************************************************/
//! Considering the sampling Time of LANEX different that of lateralOffset 
//! Rebuild the InfoCar struct
/******************************************************************************/
        InfoCar infoLANEX;
        infoLANEX.lateralOffset = lateralOffset;
        infoLANEX.intervalTime  = intervalTime;
        if(!LANEXDeque.empty()) {
            infoLANEX.winTime = LANEXDeque.back().winTime + intervalTime;
        }
        else {
            infoLANEX.winTime = intervalTime;
        }
        while (infoLANEX.winTime > TIME_SAMPLING_WINDOW) {
            infoLANEX.winTime -= LANEXDeque.front().intervalTime;
            LANEXDeque.pop_front();
        }
        LANEXDeque.push_back(infoLANEX);
        
        double sumTime_LANEX = 0, sumTime = 0;
        for(std::deque<InfoCar>::size_type i = 0; i != LANEXDeque.size(); ++i)
        {
            if(LANEXDeque[i].lateralOffset == 1)
            {
                sumTime_LANEX += LANEXDeque[i].intervalTime;
            }
            sumTime += LANEXDeque[i].intervalTime;
        }
        LANEX = sumTime_LANEX / sumTime;
        
        
/*****************************************/
//! Time-to-Lane-Crossing    
/*****************************************/
        //! Simple TLC 
        CV_Assert((int)lateralOffsetDeque.size() > 1);
        double lastLateralOffset = lateralOffsetDeque.at(lateralOffsetDeque.size()-2).lateralOffset;
        double lateralVelocity = lateralOffset - lastLateralOffset;
        
        double TLC_Frame = 0;
        if (lateralVelocity < 0)//direct to left
            TLC_Frame = std::abs((1 + lateralOffset) / lateralVelocity);
        else if(lateralVelocity > 0)//direct to right
            TLC_Frame = std::abs((1 - lateralOffset) / lateralVelocity);
        else
            TLC_Frame = 10000;//Max TLC shows a safe deviation 
        TLC = TLC_Frame * intervalTime;//sec
        TLC = TLC < 1000 ? TLC : 1000;
        
        InfoTLC infoTLC;
        infoTLC.TLC = TLC;
        infoTLC.intervalTime = intervalTime;
        if(!TLCDeque.empty()) {
            infoTLC.winTime = TLCDeque.back().winTime + intervalTime;
        }
        else {
            infoTLC.winTime = intervalTime;
        }
        while (infoTLC.winTime > TIME_SAMPLING_WINDOW) {
            infoTLC.winTime -= TLCDeque.front().intervalTime;
            TLCDeque.pop_front();
        }
        TLCDeque.push_back(infoTLC);
       
        
        //! Number of times that TLC below a threshold
        for(std::deque<double>::size_type i = 0; i != TLCDeque.size(); ++i)
        {
            //!TLC with signal falling below 2s in a given time interval
            if(TLCDeque[i].TLC < 2.0)
            {
                TLC_2s ++;
            }
            //!TLC with signal falling below 0.5s in a given time interval
            if(TLCDeque[i].TLC < 0.5)
            {
                TLC_halfs ++;
            }
        }
        //! Fraction of TLC_2s and TLC_halfs
        TLCF_2s = (double)TLC_2s / (double)TLCDeque.size();
        TLCF_halfs = (double)TLC_halfs / (double)TLCDeque.size();
        
        //! Global minimum of TLC over a given time interval;
        std::deque<InfoTLC> cmpTLCDeque = TLCDeque;
        sort(cmpTLCDeque.begin(), cmpTLCDeque.end(), TLC_cmp);
        TLC_min = cmpTLCDeque.back().TLC;
        std::deque<InfoTLC>::iterator iter = cmpTLCDeque.end();
        while (TLC_min == 0) { 
            --iter;
            TLC_min = iter->TLC;
        }
        
        
        //!Update the LaneFeature struct
        laneFeatures.frame          = sampleIdx;
        laneFeatures.lateralOffset  = lateralOffset;
        laneFeatures.LATMEAN        = LATMEAN;  //EWMA
        laneFeatures.LANEDEV        = LANEDEV;  //EWVAR
        laneFeatures.LATSD          = LATSD;
        laneFeatures.LANEX          = LANEX;
        laneFeatures.TLC            = TLC;
        laneFeatures.TLC_2s         = TLC_2s;
        laneFeatures.TLC_halfs      = TLC_halfs;
        laneFeatures.TLC_min        = TLC_min;
        laneFeatures.TLCF_2s        = TLCF_2s;
        laneFeatures.TLCF_halfs     = TLCF_halfs;
	}//end GenerateLaneIndicators
    
}//namespace LaneDetector




