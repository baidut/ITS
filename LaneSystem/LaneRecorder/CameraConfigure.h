//
//  CameraConfigure.h
//  LaneRecorder1.1
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//

#ifndef LaneRecorder_CameraConfigure_h
#define LaneRecorder_CameraConfigure_h

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#ifndef _WINDOWS
#define strncpy_s(dest,len,src,count) strncpy(dest,src,count)
#define sprintf_s(dest,len,format,args...) sprintf(dest,format,args)
#define sscanf_s sscanf
#define strcpy_s(dst,len,src) strcpy(dst,src)
#define _strdup(src) strdup(src)
#define strtok_s(tok,del,ctx) strtok(tok,del)
#endif

#include <unistd.h>
#include <time.h>
#include "../PvApi.h"
#include "../ImageLib.h"
#include <iostream>
#include <opencv2/opencv.hpp>

namespace LaneRecorder{
    ///Camera Calibration info
    typedef struct _CameraInfo
    {
        ///focal length in x and y
        cv::Point2f focalLength;
        ///optical center coordinates in image frame (origin is (0,0) at top left)
        cv::Point2f opticalCenter;
        ///height of camera above ground
        double cameraHeight;
        ///pitch angle in radians (+ve downwards)
        double pitch;
        ///yaw angle in radians (+ve clockwise)
        double yaw;
        ///width of images
        double imageWidth;
        ///height of images
        double imageHeight;
    }CameraInfo;
    
    /**
     * This function does initialization of Camera Info due to lack of 
     * Camera Setting
     */
    void InitCameraInfoDefault(CameraInfo *cameraInfo);
    

    
    /**
     * Structure and function related with PROSILICA WEBCAMERA
     *
     */    
    typedef struct _Camera
    {
        unsigned long   UID;
        tPvHandle       Handle;
        tPvFrame        Frame;
        tPvUint32       Counter;
        char            Filename[50];
    }tCamera; // camera frame struct
    
    void Sleep(unsigned int time);
    void WaitForCamera();
    bool WaitForTrigger();
    bool CameraGet(tCamera* Camera);
    bool CameraSetup(tCamera* Camera);
    void CameraUnsetup(tCamera* Camera);
    bool CameraStart(tCamera* Camera);
    void CameraStop(tCamera* Camera);
    bool CameraSnap(tCamera* Camera);
}

#endif
