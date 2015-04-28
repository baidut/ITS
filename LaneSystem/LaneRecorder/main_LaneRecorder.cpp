//
//  main.cpp
//  LaneRecorder1.1
//
//  Created by Xuanpeng Li on 10/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//  
#include "main_LaneRecorder.h"
#include "Process_LaneRecorder.h"
#include <stdexcept>

#ifdef __cplusplus

/* Time */
extern const int        SAMPLING_TIME           = 60;
extern const int        NUM_WINDOW_EWM          = 5;    //EWMA, EWVAR Init (times)
/* Size of Image */
extern const double     COEF                    = 0.75;
extern const int        WIDTH                   = 640;
extern const int        HEIGHT                  = 480;
/// Multi-Image Show
extern const int        WIN_COLS                = 3;
extern const int        WIN_ROWS                = 3;
/// Frame
extern const int        FRAME_START             = 1; 
extern const int        FRAME_END               = INFINITY;

#ifdef __APPLE__
/// Record images
extern const char       FILE_TEMP_NAME[]        = "/Users/xuanpengli/DriveAssist/Data/snap/snap.png";  
extern const char       LANE_PATH_NAME[]        = "/Users/xuanpengli/DriveAssist/Data/LaneRaw_%02d-%02d-%d_%02dh%02dm%02ds";
extern const char       LANE_IMG_NAME[]         = "/lane_%d.jpg";
/// Record docs
extern const char       FILE_LANE_FEATURES[]    = "/Users/xuanpengli/DriveAssist/Data/LaneFeatures_%02d-%02d-%d_%02dh%02dm%02ds.txt";
extern const char       KEY_PATH[]              = "/Users/xuanpengli/DriveAssist/key.txt";

#elif defined __linux
/// Record images
extern const char       FILE_TEMP_NAME[]        = "/home/lixp/Developer/Data/snap/snap.png";  
extern const char       LANE_PATH_NAME[]        = "/home/lixp/Developer/Data/LaneRaw_%02d-%02d-%d_%02dh%02dm%02ds";
extern const char       LANE_IMG_NAME[]         = "/lane_%d.jpg";
/// Record docs
extern const char       FILE_LANE_FEATURES[]    = "/home/lixp/Developer/Record/LaneFeatures_%02d-%02d-%d_%02dh%02dm%02ds.txt";
extern const char       KEY_PATH[]              = "/home/lixp/Developer/key.txt";
#endif

extern const int        TH_KALMANFILTER         = 1;
namespace LaneRecorder{
	int Process(int argc, const char* argv[])
	{
        std::cout << "/*************************************/" << std::endl;
        std::cout << "Input LANE_IMG_RECORD" << std::endl;
        std::cout << "Input LANE_DETECTOR" << std::endl;
        std::cout << "Input DATA_RECORD" << std::endl;
        std::cout << "Input SEND_DATA" << std::endl;
        std::cout << "Input SAMPLING_FREQ" << std::endl;
        std::cout << "Input COMPRESSION_RATE" << std::endl;
        std::cout << "Input YAW_ANGLE" << std::endl;
        std::cout << "Input PITCH_ANGLE" << std::endl;
        std::cout << "Input TIME_BASELINE" << std::endl;
        std::cout << "Input LANE_ANALYSER" << std::endl;
        std::cout << "/*************************************/" << std::endl;
        if(argc < 11)
            std::cout << "Not enough parameters" << std::endl;

        /// Debug Marker
        int LANE_IMG_RECORD     = atoi(argv[1]);//Record the lane image
        int LANE_DETECTOR       = atoi(argv[2]);//Whether detect Lane
        int DATA_RECORD         = atoi(argv[3]);//Record the lane features
        int SEND_DATA           = atoi(argv[4]);//Whether send data
        int SAMPLING_FREQ       = atoi(argv[5]);//Sampling Frequency
        int COMPRESSION_RATE    = atoi(argv[6]);//Compression rate for recording images
        double YAW_ANGLE        = atof(argv[7]);//yaw - X
        double PITCH_ANGLE      = atof(argv[8]);//pitch - Y
        double TIME_BASELINE    = atof(argv[9]);//sec 
        int LANE_ANALYSER       = atoi(argv[10]);
        
//        std::cout << "LANE_IMG_RECORD: " << LANE_IMG_RECORD << std::endl;
//        std::cout << "DATA_RECORD: " << DATA_RECORD << std::endl;
//        std::cout << "SAMPLING_FREQ: " << SAMPLING_FREQ << endl;
//        std::cout << "COMPRESSION_RATE: " << COMPRESSION_RATE << std::endl;
//        std::cout << "LANE_DETECTOR: " << LANE_DETECTOR << std::endl;
//        std::cout << "NUM_WINDOW_SAMPLE: " << NUM_WINDOW_SAMPLE << std::endl;
        std::cout << "/*****************************************/" << std::endl;
        std::cout << "Press q  / Q / ESC to quit " << std::endl;
        std::cout << "Press s / S to stop " << std::endl;
        std::cout << "/*****************************************/" << std::endl;

        // Init parameters
        int idx             = FRAME_START; //index for image sequence
        int sampleIdx       = 1;
        
        //! Get the current time
        time_t t;
        time(&t);
        tm *tmTime = localtime(&t);
        char currentTime[50];
        
        sprintf(currentTime,"%02d-%02d-%d_%02dh%02dm%02ds",tmTime->tm_mday,tmTime->tm_mon+1,tmTime->tm_year+1900, tmTime->tm_hour,tmTime->tm_min, tmTime->tm_sec);  
        std::cout << "Current Time: " << currentTime << std::endl;
        
        //! Init the Record file
        std::ofstream laneFeatureFile;
        char *laneFeaturePath = new char[100];
        if (DATA_RECORD){ 
            sprintf(laneFeaturePath, FILE_LANE_FEATURES, tmTime->tm_mday,tmTime->tm_mon+1,tmTime->tm_year+1900, tmTime->tm_hour,tmTime->tm_min, tmTime->tm_sec);
            InitRecordData(laneFeatureFile, laneFeaturePath, laneFeatureName, NUM_LANE_FEATURES);
        }
        
        //! Create a new dir for recording the images
        char * laneRawImgPath = new char[100];
        DIR *pDir = NULL;
        if(LANE_IMG_RECORD){
            do{
                sprintf(laneRawImgPath, LANE_PATH_NAME, tmTime->tm_mday,tmTime->tm_mon+1,tmTime->tm_year+1900, tmTime->tm_hour,tmTime->tm_min, tmTime->tm_sec);
                pDir = opendir(laneRawImgPath);
            }while (pDir != NULL);
            mkdir(laneRawImgPath, S_IRWXU);
        }
        
        /* Parameters for Lane Detector */
        cv::Mat laneMat;
        LaneDetector::LaneDetectorConf laneDetectorConf; 
        std::vector<cv::Vec2f> hfLanes;
        std::vector<cv::Vec2f> lastHfLanes;
        std::vector<cv::Vec2f> preHfLanes;
        //lane features
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
        
        // Initialize Lane Kalman Filter
        cv::KalmanFilter laneKalmanFilter(8, 8, 0); //(rho, theta, delta_rho, delta_theta)x2
        cv::Mat laneKalmanMeasureMat(8, 1, CV_32F, cv::Scalar::all(0)); //(rho, theta, delta_rho, delta_theta)
        int    laneKalmanIdx     = 0;    //Marker of start kalmam

        LaneDetector::InitlaneFeatures(laneFeatures);
        if (LANE_DETECTOR) {
            LaneDetector::InitlaneDetectorConf(laneMat, laneDetectorConf, 1); // KIT 1, ESIEE 2
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
        
        /* Entrance of Process */
        double initTime         = (double)cv::getTickCount();
        double intervalTime     = 0;
        double execTime         = 0;  // Execute Time for Each Frame
        double pastTime         = 0;
        double lastStartTime    = (double)cv::getTickCount();
        char key;
        double delay = 1;

        //! Camera parameters
        tCamera Camera;
        tPvErr errCode;
        // initialize the PvAPI
        if((errCode = PvInitialize()) != ePvErrSuccess)
        { 
            printf("PvInitialize err: %u\n", errCode);
        }
        else
        {
            //IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more info.
            memset(&Camera,0,sizeof(tCamera));
            
            // wait for a camera to be plugged in
            WaitForCamera();
            
            // get first camera found
            if(CameraGet(&Camera))
            {
                // open camera
                if(CameraSetup(&Camera))
                {
                    // start camera streaming
                    if(CameraStart(&Camera))
                    {
                        // Start Time in the process
                        while(idx <= FRAME_END)
                        {   
                            double startTime = (double)cv::getTickCount();

                            /*  Lane detection starts   */
                            if (WaitForTrigger() && CameraSnap(&Camera)) 
                            {
                                /* Read the saved images from the camera */
                                laneMat = cv::imread(Camera.Filename);

                                // Reduce the size of raw image
                                cv::resize(laneMat, laneMat, cv::Size(WIDTH*COEF, HEIGHT*COEF), CV_INTER_AREA);
                                
                                /* Record the resized raw image */
                                if (LANE_IMG_RECORD) {
                                    std::vector<int> compression_params;
                                    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                                    compression_params.push_back(COMPRESSION_RATE);
                                    
                                    char * rawLaneName = new char[50];
                                    char * tempName = new char[100];
                                    sprintf(rawLaneName, LANE_IMG_NAME, idx);
                                    strcpy(tempName, laneRawImgPath);
                                    strcat(tempName, rawLaneName);
                                    cv::imwrite(tempName, laneMat, compression_params);
                                    delete tempName;
                                    delete rawLaneName;
                                }    
                                
                                if (LANE_DETECTOR) {
                                    ProcessLaneImage(laneMat, laneDetectorConf, 
                                        laneKalmanFilter, laneKalmanMeasureMat, laneKalmanIdx, 
                                        hfLanes, lastHfLanes, lastLateralOffset, lateralOffset, 
                                        isChangeLane, detectLaneFlag,  idx, execTime, preHfLanes, changeDone,
                                        YAW_ANGLE, PITCH_ANGLE);
                                }
                            }

                            /* Calculate the running time for every sampling */  
                            pastTime = ((double)cv::getTickCount() - initTime)/cv::getTickFrequency();
                            // printf("@Lane Sampling passes %f sec\n", pastTime);
                            char *text_pastTime = new char[50];
                            sprintf(text_pastTime, "LaneDetector Time: %.2f sec", pastTime);
                            cv::putText(laneMat, text_pastTime, cv::Point(0, laneMat.rows-5), cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0));
                            delete text_pastTime;
                            
                            intervalTime = (startTime - lastStartTime)/ cv::getTickFrequency();//get the time between two continuous frames
                            lastStartTime = startTime;

                            /* Generate lane indicators */
                            if (LANE_DETECTOR && LANE_ANALYSER) {
                                /// First init the baseline, then get lane mass 
                                if( pastTime < TIME_BASELINE ){
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
                             
                            /* Record lane features */
                            if (DATA_RECORD) {
                                laneFeatures.frame = idx;
                                RecordLaneFeatures(laneFeatureFile, laneFeatures, execTime, pastTime);
                            }//end if
                            
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
                                //cv::waitKey(1);
                                execFreq = 1.0 / (((double)cv::getTickCount() - startTime)/cv::getTickFrequency());
                            }while ( execFreq >= SAMPLING_FREQ );

                            char *text = new char[30];
                            sprintf(text, "Process: %.2f Hz", execFreq);
                            cv::putText(laneMat, text, cv::Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(0, 255, 0));
                            delete text;

                            cv::imshow("Lane", laneMat);
                            key = cv::waitKey(delay);
                            if (key == 'q' || key == 'Q' || 27 == (int)key) //Esc q\Q\key to stop
                                break;
                            else if(key == 's' || key == 'S' )
                                delay = 0;
                            else
                                delay = 1;

                            sampleIdx++;    //update the sampling index
                            idx++;

                        }//end while loop 
                        
                        laneFeatureFile.close();
                        cv::destroyAllWindows();
                    }//if CameraStart
                    
                    CameraStop(&Camera);
                }//if CameraSetup
                
                CameraUnsetup(&Camera);
            }//if CameraGet
        }        
        
        PvUnInitialize();
        
        return 0;
    }
}//end LaneRecorder

#endif //__cplusplus

using LaneRecorder:: Process;
int main (int argc, const char * argv[])
{
    return Process(argc, argv);
}
