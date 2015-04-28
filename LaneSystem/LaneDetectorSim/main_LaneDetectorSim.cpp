//
//  main.cpp
//  LaneDetectorSim1.2
//
//  Created by Xuanpeng Li on 09/13.
//  Copyright (c) 2013 ESIEE-AMIENS. All rights reserved.
//
#include "main_LaneDetectorSim.h"
#include "Process_LaneDetectorSim.h"      
#include <stdexcept>

#ifdef __cplusplus

/* Time */
extern const double SAMPLING_TIME       = 60;   //sec for sampling lane features
// extern const double SAMPLING_FREQ       = 8.42;  //Hz for camera: 8.4 for my database
extern const double TIME_BASELINE       = 300;  //sec (300)
extern const int    NUM_WINDOW_EWM      = 5;    //EWMA, EWVAR Init (times)
/* Size of Image */
extern const double COEF                = 1;
/* Multi-Image Show */
extern const int    WIN_COLS            = 3;
extern const int    WIN_ROWS            = 3;

/* Run applicaiton */
extern const int    IMAGE_RECORD        = 0;

#ifdef __APPLE__
/* Record docs */
extern const char   LANE_RECORD_FILE[]  = "/Users/xuanpengli/DriveAssist/Data/LaneFeatures_22-03-2014_13h05m12s.txt";
extern const char   FILE_LANE_FEATURE[] = "/Users/xuanpengli/DriveAssist/Sim_Record/Sim_LaneFeatures_22-03-2014_13h05m12s.txt";
// extern const char   LANE_RECORD_FILE[]  = "/Users/xuanpengli/DriveAssist/Data/LaneFeatures_16-03-2014_16h37m38s.txt"; // record when capturing
// extern const char   FILE_LANE_FEATURE[] = "/Users/xuanpengli/DriveAssist/Sim_Record/Sim_LaneFeatures_16-03-2014_16h37m38s.txt";
// extern const char   FILE_LANE_FEATURE[] = "/Users/xuanpengli/DriveAssist/Sim_Record/Sim_LaneFeatures_10-07-2013_18h30m21s.txt";
extern const char   LANE_RECORD_IMAGE[]  = "/Users/xuanpengli/Desktop/lane_pic/lane_%d.png";
/* Data Source */
extern const char   LANE_RAW_NAME[]     = "/Users/xuanpengli/DriveAssist/Data/LaneRaw_22-03-2014_13h05m12s/lane_%d.jpg";
// extern const char   LANE_RAW_NAME[]     = "/Users/xuanpengli/DriveAssist/KIT/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/%010d.png";
// extern const char   LANE_RAW_NAME[]     = "/Users/xuanpengli/DriveAssist/Data/LaneRaw_16-03-2014_16h37m38s/lane_%d.jpg";
// extern const char   LANE_RAW_NAME[]     = "/Users/xuanpengli/DriveAssist/Data/LaneRaw_10-07-2013_18h30m21s/lane_%d.jpg";
extern const char   KEY_PATH[] 			= "/Users/xuanpengli/DriveAssist/key.txt";

#elif defined __linux 
/* Record docs */
extern const char   LANE_RECORD_FILE[]  = "/home/lixp/Developer/Data/LaneFeatures_22-03-2014_13h05m12s.txt";
extern const char   FILE_LANE_FEATURE[] = "/home/lixp/Developer/Sim_Record/Sim_LaneFeatures_22-03-2014_13h05m12s.txt";
extern const char   LANE_RECORD_IMAGE[]  = "/home/lixp/Developer/Sim_Record/IMAGE/lane_%d.png";
/* Data Source */
extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/Data/LaneRaw_22-03-2014_13h05m12s/lane_%d.jpg";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/KIT/2011_09_26/2011_09_26_drive_0015_sync/image_00/data/%010d.png";
// extern const char   LANE_RAW_NAME[]     = "/home/lixp/Developer/Data/LaneRaw_10-07-2013_18h30m21s/lane_%d.jpg";
extern const char   KEY_PATH[] 			= "/home/lixp/Developer/key.txt";
#endif

extern const int    TH_KALMANFILTER     = 1; //frames

/* Frame 10-07-2013_18h30m21s */
// extern const int    FRAME_START         = 1840;	 	//18:34:00
// extern const int    FRAME_STOP1         = 14007;    //18:57:59
// extern const int    FRAME_RESTART1      = 15031;    //19:00:00
// extern const int    FRAME_STOP2         = 17507;    //19:04:59
// extern const int    FRAME_RESTART2      = 19497;    //19:09:00
// extern const int    FRAME_END           = 20994; 	//19:11:59


namespace LaneDetectorSim {
    
	int Process(int argc, const char* argv[])
	{
        if(argc < 9)
            std::cout << "Not enough parameters" << std::endl;

		int	LANE_DETECTOR 	= atoi(argv[1]);
		int	LANE_ANALYZER 	= atoi(argv[2]);
		int	SEND_DATA     	= atoi(argv[3]);
		int	DATA_RECORD   	= atoi(argv[4]);
		int	StartFrame		= atoi(argv[5]); // FRAME_START
		int EndFrame		= atoi(argv[6]); // FRAME_END
        double YAW_ANGLE    = atof(argv[7]); // yaw - X
        double PITCH_ANGLE  = atof(argv[8]); // pitch - Y

        std::cout << "/*************************************/" << std::endl;
        std::cout << "Input LANE_DETECTOR" << LANE_DETECTOR << std::endl;
        std::cout << "Input LANE_ANALYZER" << LANE_ANALYZER << std::endl;
        std::cout << "Input SEND_DATA" << SEND_DATA << std::endl;
        std::cout << "Input DATA_RECROD" << DATA_RECORD << std::endl;
        std::cout << "Input StartFrame" << StartFrame << std::endl;
        std::cout << "Input EndFrame" << EndFrame << std::endl;
        std::cout << "Input YAW_ANGLE" << YAW_ANGLE << std::endl;
        std::cout << "Input PITCH_ANGLE" << PITCH_ANGLE << std::endl;
        std::cout << "/*************************************/" << std::endl;


        int  idx            = StartFrame;  //index for image sequence
        int  sampleIdx      = 1;    //init sampling index
        char laneImg[100];

        double initTime         = (double)cv::getTickCount();
        double intervalTime     = 0;
        double execTime         = 0;  // Execute Time for Each Frame
        double pastTime         = 0;
        double lastStartTime    = (double)cv::getTickCount();
        char key;
        double delay = 1;
        
        std::ofstream laneFeatureFile;
        if (DATA_RECORD){
            InitRecordData(laneFeatureFile, FILE_LANE_FEATURE, laneFeatureName, NUM_LANE);
        }
        
        /* Parameters for Lane Detector */
        cv::Mat laneMat;
        LaneDetector::LaneDetectorConf laneDetectorConf; 
        std::vector<cv::Vec2f> hfLanes;
        std::vector<cv::Vec2f> lastHfLanes;
        std::vector<cv::Vec2f> preHfLanes;
        
        std::vector<double> LATSDBaselineVec;
        std::deque<LaneDetector::InfoCar> lateralOffsetDeque;
        std::deque<LaneDetector::InfoCar> LANEXDeque;
        std::deque<LaneDetector::InfoTLC> TLCDeque;
        LaneDetector::LaneFeature laneFeatures;
        double lastLateralOffset = 0;
        double lateralOffset     = 0;    // Lateral Offset
        int    detectLaneFlag    = -1;   // init state -> normal state 0
        int    isChangeLane      = 0;    // Whether lane change happens
        int    changeDone        = 0;    // Finish lane change
        int    muWindowSize      = 5;    // Initial window size: 5 (sample)
        int    sigmaWindowSize   = 5;    // Initial window size: 5 (sample)
        std::vector<float> samplingTime;

        /* Initialize Lane Kalman Filter */
        cv::KalmanFilter laneKalmanFilter(8, 8, 0);//(rho, theta, delta_rho, delta_theta)x2
        cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0));//(rho, theta, delta_rho, delta_theta)
        int    laneKalmanIdx     = 0;    //Marker of start kalmam

        InitlaneFeatures(laneFeatures);
        GetSamplingTime(LANE_RECORD_FILE, samplingTime);

        if (LANE_DETECTOR) { 
             /* Lane detect and tracking */ 
            sprintf(laneImg, LANE_RAW_NAME , idx);
            laneMat = cv::imread(laneImg);
            LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 2); // KIT 1, ESIEE 2
            LaneDetector::InitLaneKalmanFilter(laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx);
        }
        
        /* Inter-process communication */
        key_t ipckey;
        int mq_id;
        struct { 
            long type; 
            char text[1024]; 
        } laneMsg;

        if (SEND_DATA) 
        {
            /* Generate the ipc key */
            ipckey = ftok(KEY_PATH, 'a');
            if(ipckey == -1){
                printf("Key Error: %s\n", strerror(errno));
                exit(1);
            }
            
            mq_id = msgget(ipckey, 0);
            if (mq_id == -1) { 
                //MQ doesn't exit
                mq_id = msgget(ipckey, IPC_CREAT | IPC_EXCL | 0666);
                printf("LaneDetector creates a new MQ %d\n", mq_id);
            }
            else {
                //MQ does exit
                mq_id = msgget(ipckey, IPC_EXCL | 0666);
                printf("LaneDetector uses an existed MQ %d\n", mq_id);
            }
            //printf("Lane identifier is %d\n", mq_id);
            if(mq_id == -1) {  
                throw std::logic_error("Can't build pipeline"); 
            }
            //printf("This is the LaneDetectSim process, %d\n", getpid());
        }
        
		double delayTime;
		if(idx == 1)
			delayTime = 0;
		else
			delayTime = samplingTime.at(idx - 2);
        
        /* Entrance of Process */
        while (idx <= EndFrame)
        {
            double startTime = (double)cv::getTickCount();

            /* Lane detect and tracking */ 
            sprintf(laneImg, LANE_RAW_NAME , idx);
            laneMat = cv::imread(laneImg);//imshow("laneMat", laneMat);

            if (LANE_DETECTOR)
            {
                ProcessLaneImage(laneMat, laneDetectorConf, startTime, 
                    laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, 
                    hfLanes, lastHfLanes, lastLateralOffset, lateralOffset, 
                    isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone, YAW_ANGLE, PITCH_ANGLE);
            }
            
        
            intervalTime = (startTime - lastStartTime)/ cv::getTickFrequency();//get the time between two continuous frames
            lastStartTime = startTime;
            // cout << "intervalTime: "<< intervalTime << endl;
            
            /* Generate lane indicators */
            if (LANE_DETECTOR && LANE_ANALYZER)
            {
                /// First init the baseline, then get lane mass 
                if( pastTime < TIME_BASELINE + delayTime){
                    /// Get lane baseline
                    LaneDetector::GetLaneBaseline(sampleIdx, SAMPLING_TIME,
                                    muWindowSize, sigmaWindowSize,
                                    lateralOffset, LATSDBaselineVec,
                                    lateralOffsetDeque, LANEXDeque,
                                    TLCDeque, laneFeatures, intervalTime);
                    //idx_baseline = sampleIdx;
                }
                else {
                    //sampleIdx -= idx_baseline;
                    LaneDetector::GenerateLaneIndicators(sampleIdx, SAMPLING_TIME,
                                     muWindowSize, sigmaWindowSize,
                                     lateralOffset,
                                     lateralOffsetDeque,
                                     LANEXDeque, TLCDeque, 
                                     laneFeatures, intervalTime);
                    //! LATSD
                    char *text_LATSD = new char[30];
                    sprintf(text_LATSD, "L1. LATSD: %.4f", laneFeatures.LATSD);
                    cv::putText(laneMat, text_LATSD, cv::Point(0, 70), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_LATSD;
                    //! LATMEAN
                    char *text_LATMEAN = new char[30];
                    sprintf(text_LATMEAN, "L2. LATMEAN: %.4f", laneFeatures.LATMEAN);
                    cv::putText(laneMat, text_LATMEAN, cv::Point(0, 80), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_LATMEAN;
                    //! LANEDEV
                    char *text_LANEDEV = new char[30];
                    sprintf(text_LANEDEV, "L3. LANEDEV: %.4f", laneFeatures.LANEDEV);
                    cv::putText(laneMat, text_LANEDEV, cv::Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_LANEDEV;
                    //! LANEX
                    char *text_LANEX = new char[30];
                    sprintf(text_LANEX, "L4. LANEX: %.4f", laneFeatures.LANEX);
                    cv::putText(laneMat, text_LANEX, cv::Point(0, 100), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_LANEX;
                    //! TLC
                    char *text_TLC = new char[30];
                    sprintf(text_TLC, "L5. TLC: %.4f", laneFeatures.TLC);
                    cv::putText(laneMat, text_TLC, cv::Point(0, 110), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLC;
                    //! TLC_2s
                    char *text_TLC_2s = new char[30];
                    sprintf(text_TLC_2s, "L6. TLC_2s: %d", laneFeatures.TLC_2s);
                    cv::putText(laneMat, text_TLC_2s, cv::Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLC_2s;
                    
                    char *text_TLCF_2s = new char[50];
                    sprintf(text_TLCF_2s, "L7. Fraction_TLC_2s: %f", laneFeatures.TLCF_2s);
                    cv::putText(laneMat, text_TLCF_2s, cv::Point(0, 130), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLCF_2s;
                    //! TLC_halfs
                    char *text_TLC_halfs = new char[30];
                    sprintf(text_TLC_halfs, "L8. TLC_halfs: %d", laneFeatures.TLC_halfs);
                    cv::putText(laneMat, text_TLC_halfs, cv::Point(0, 140), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLC_halfs;
                    
                    char *text_TLCF_halfs = new char[50];
                    sprintf(text_TLCF_halfs, "L9. Fraction_TLC_halfs: %f", laneFeatures.TLCF_halfs);
                    cv::putText(laneMat, text_TLCF_halfs, cv::Point(0, 150), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLCF_halfs;
                    //! TLC_min
                    char *text_TLC_min = new char[30];
                    sprintf(text_TLC_min, "L10. TLC_min: %.4f", laneFeatures.TLC_min);
                    cv::putText(laneMat, text_TLC_min, cv::Point(0, 160), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                    delete text_TLC_min;
                }//end if
            }//end Generate Lane Indicators
            
            if(IMAGE_RECORD){
                char *text = new char[100];
                sprintf(text, LANE_RECORD_IMAGE, idx);
                cv::imwrite(text, laneMat);
                delete text;
            }
            
            /* Send the datas as string to fusion center */
            if(LANE_DETECTOR & SEND_DATA) {
                char *str = new char[1024];
                memset(str, 0, 1024);
                CodeMsg(laneFeatures, str);
                
                strcpy(laneMsg.text, str);//!!! overflow 
                laneMsg.type = 1;
                //printf("Data sends: %s\n", laneMsg.text);
                
                //! 0 will cause a block/ IPC_NOWAIT will close the app.
                if(msgsnd(mq_id, &laneMsg, sizeof(laneMsg), 0) == -1)
                {  
                    throw std::runtime_error("LaneDetectSim: msgsnd failed!");  
                }  
                delete str;
            }
            
            /* Adjust the interval time within fixed frequency */
			double execFreq; 
            do {
                execFreq = 1.0 / (((double)cv::getTickCount() - startTime)/cv::getTickFrequency());
                pastTime = ((double)cv::getTickCount() - initTime)/cv::getTickFrequency() + delayTime;
            }while ( pastTime < samplingTime.at(idx-1) );
            // }while(execFreq > SAMPLING_FREQ);
            // }while(pastTime < (double)sampleIdx/SAMPLING_FREQ);
            
            /* Record lane features */
            if (DATA_RECORD) {
                RecordLaneFeatures(laneFeatureFile, laneFeatures, execTime, pastTime);
            }//end if

            char *text = new char[30];
            sprintf(text, "past time: %.2f sec", pastTime);
            cv::putText(laneMat, text, cv::Point(0, laneMat.rows-5), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0,255,0));
            
            sprintf(text, "Adjusted Freq: %.2f Hz", execFreq);
            cv::putText(laneMat, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
            delete text;

            cv::imshow("Lane System", laneMat);
            // cv::moveWindow("Lane System", 790, 30);
            key = cv::waitKey(delay);
            if (key == 'q' || key == 'Q' || 27 == (int)key) //Esc q\Q\key to stop
                break;
            else if(key == 's' || key == 'S')
                    delay = 0;
            else
                delay = 1;

            /* Update the sampling index */
            sampleIdx++;//update the sampling index
            idx++;

            // if(idx == FRAME_STOP1)
            //     idx = FRAME_RESTART1;
            // if(idx == FRAME_STOP2)
            //     idx = FRAME_RESTART2;
        }//end while loop 

        laneFeatureFile.close();
        cv::destroyAllWindows();
        
        return 0;
    }
}//FusedCarSurveillanceSim

#endif //__cplusplus

using LaneDetectorSim::Process;
int main(int argc, const char * argv[])
{
    return Process(argc, argv);
}
