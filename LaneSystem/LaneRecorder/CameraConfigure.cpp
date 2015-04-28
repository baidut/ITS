//
//  CameraConfigure.cpp
//  LaneRecorder1.1
//
//  Created by Xuanpeng Li on 03/13.
//  Copyright (c) 2012 ESIEE-AMIENS. All rights reserved.
//
#include "CameraConfigure.h"
extern const char  FILE_TEMP_NAME[]; 

namespace LaneRecorder{
    /**
     * This function does initialization of Camera Info due to lack of 
     * Camera Setting
     */
    void InitCameraInfoDefault(CameraInfo *cameraInfo)
    {
        // focal length
        cameraInfo->focalLength.x = 300;
        cameraInfo->focalLength.y = 300;
        // optical center
        cameraInfo->opticalCenter.x = 300;
        cameraInfo->opticalCenter.y = 256.5352;
        // height of the camera in mm
        cameraInfo->cameraHeight = 1500;
        // pitch of the camera
        cameraInfo->pitch = 14.0 * CV_PI/180;
        // yaw of the camera
        cameraInfo->yaw = 0 * CV_PI/180;
        // image width and height
        cameraInfo->imageWidth = 640;
        cameraInfo->imageHeight = 480;
    }
    
    void Sleep(unsigned int time)
    {
        struct timespec t,r;
        
        t.tv_sec    = time / 1000;
        t.tv_nsec   = (time % 1000) * 1000000;
        
        while(nanosleep(&t,&r)==-1)
            t = r;
    }
      
    
    //Wait for camera to be plugged in
    void WaitForCamera()
    {
        printf("Waiting for a camera ");
        while(PvCameraCount() == 0)
        {
            printf(".");
            Sleep(250);
        }
        printf("\n");
    }
    
    // wait for the user to press q
    // return value: true == snap, false == quit
    bool WaitForTrigger()
    {
        char c;
        
        do
        {
            c = 's';
            Sleep(50);
            
        } while(c != 'q' && c != 's');
        
        return c == 's';
    }
    
    // get the first camera found
    // return value: true == success, false == fail
    bool CameraGet(tCamera* Camera)
    {
        tPvUint32 count,connected;
        tPvCameraInfoEx list;
        
        //regardless if connected > 1, we only set UID of first camera in list
        count = PvCameraListEx(&list,1,&connected, sizeof(tPvCameraInfoEx));
        if(count == 1)
        {
            Camera->UID = list.UniqueId;
            printf("Got camera %s\n",list.SerialNumber);
            return true;
        }
        else
        {
            printf("CameraGet: Failed to find a camera\n");
            return false;
        }
    }
    
    // open camera, allocate memory
    // return value: true == success, false == fail
    bool CameraSetup(tCamera* Camera)
    {
        tPvErr errCode;
        unsigned long FrameSize = 0;
        
        //open camera
        if ((errCode = PvCameraOpen(Camera->UID,ePvAccessMaster,&(Camera->Handle))) != ePvErrSuccess)
        {
            if (errCode == ePvErrAccessDenied)
                printf("PvCameraOpen returned ePvErrAccessDenied:\nCamera already open as Master, or camera wasn't properly closed and still waiting to HeartbeatTimeout.");
            else
                printf("PvCameraOpen err: %u\n", errCode);
            return false;
        }
        
        // Calculate frame buffer size
        if((errCode = PvAttrUint32Get(Camera->Handle,"TotalBytesPerFrame",&FrameSize)) != ePvErrSuccess)
        {
            printf("CameraSetup: Get TotalBytesPerFrame err: %u\n", errCode);
            return false;
        }
        
        // allocate image buffer
        Camera->Frame.ImageBuffer = new char[FrameSize];
        if(!Camera->Frame.ImageBuffer)
        {
            printf("CameraSetup: Failed to allocate buffers.\n");
            return false;
        }
        Camera->Frame.ImageBufferSize = FrameSize;
        
        return true;
    }
    
    // close camera, free memory.
    void CameraUnsetup(tCamera* Camera)
    {
        tPvErr errCode;
        
        if((errCode = PvCameraClose(Camera->Handle)) != ePvErrSuccess)
        {
            printf("CameraUnSetup: PvCameraClose err: %u\n", errCode);
        }
        else
        {
            printf("Camera closed.");
        }
        
        // free image buffer
        delete [] (char*)Camera->Frame.ImageBuffer;
    }
    
    // setup and start streaming
    // return value: true == success, false == fail
    bool CameraStart(tCamera* Camera)
    {
        tPvErr errCode;
        
        // NOTE: This call sets camera PacketSize to largest sized test packet, up to 8228, that doesn't fail
        // on network card. Some MS VISTA network card drivers become unresponsive if test packet fails.
        // Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See network card properties
        // for max allowable PacketSize/MTU/JumboFrameSize.
        if((errCode = PvCaptureAdjustPacketSize(Camera->Handle,8228)) != ePvErrSuccess)
        {
            printf("CameraStart: PvCaptureAdjustPacketSize err: %u\n", errCode);
            return false;
        }
        
        // start driver capture stream
        if((errCode = PvCaptureStart(Camera->Handle)) != ePvErrSuccess)
        {
            printf("CameraStart: PvCaptureStart err: %u\n", errCode);
            return false;
        }
        
        // queue frame
        if((errCode = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frame),NULL)) != ePvErrSuccess)
        {
            printf("CameraStart: PvCaptureQueueFrame err: %u\n", errCode);
            // stop driver capture stream
            PvCaptureEnd(Camera->Handle);
            return false;
        }
        
        // set the camera in software trigger, continuous mode, and start camera receiving triggers
        if((PvAttrEnumSet(Camera->Handle,"FrameStartTriggerMode","Software") != ePvErrSuccess) ||
           (PvAttrEnumSet(Camera->Handle,"AcquisitionMode","Continuous") != ePvErrSuccess) ||
           (PvCommandRun(Camera->Handle,"AcquisitionStart") != ePvErrSuccess))
        {
            printf("CameraStart: failed to set camera attributes\n");
            // clear queued frame
            PvCaptureQueueClear(Camera->Handle);
            // stop driver capture stream
            PvCaptureEnd(Camera->Handle);
            return false;
        }
        
        return true;
    }
    
    
    
    // stop streaming
    void CameraStop(tCamera* Camera)
    {
        tPvErr errCode;
        
        //stop camera receiving triggers
        if ((errCode = PvCommandRun(Camera->Handle,"AcquisitionStop")) != ePvErrSuccess)
            printf("AcquisitionStop command err: %u\n", errCode);
        else
            printf("Camera stopped.\n");
        
        //clear queued frames. will block until all frames dequeued
        if ((errCode = PvCaptureQueueClear(Camera->Handle)) != ePvErrSuccess)
            printf("PvCaptureQueueClear err: %u\n", errCode);
        else
            printf("Queue cleared.\n");
        
        //stop driver stream
        if ((errCode = PvCaptureEnd(Camera->Handle)) != ePvErrSuccess)
            printf("PvCaptureEnd err: %u\n", errCode);
        else
            printf("Driver stream stopped.\n");
    }
    
    
    // trigger and save a frame from the camera
    // return value: true == success, false == fail
    bool CameraSnap(tCamera* Camera)
    {        
        //cout << "Snap Camera..." << endl;
        tPvErr errCode;
        
        //software trigger camera
        // printf("Triggering camera.\n");
        if((errCode = PvCommandRun(Camera->Handle,"FrameStartTriggerSoftware")) != ePvErrSuccess)
        {
            printf("CameraSnap: FrameStartTriggerSoftware err: %u\n", errCode);
            return false;
        }
        
        //wait for frame to return from camera to host. short timeout here to show ~time for return
        //unsigned long Timeout,     Wait timeout (in milliseconds); use PVINFINITE for no timeout
        while(PvCaptureWaitForFrameDone(Camera->Handle,&(Camera->Frame),PVINFINITE) == ePvErrTimeout)
        printf("Waiting for frame to return to host...\n");
        
        //check returned Frame.Status
        if(Camera->Frame.Status == ePvErrSuccess)
        {
            sprintf_s(Camera->Filename,sizeof(Camera->Filename),FILE_TEMP_NAME,++Camera->Counter);

            //save image
            if(!ImageWriteTiff(Camera->Filename,&(Camera->Frame)))
            {
                printf("ImageWriteTiff fail.\n");
                return false;
            }
        }
        else
        {
            if (Camera->Frame.Status == ePvErrDataMissing)
                printf("Dropped packets. Possible improper network card settings:\nSee GigE Installation Guide.");
            else
                printf("Frame.Status error: %u\n",Camera->Frame.Status);
        }
        
        //requeue frame
        if((errCode = PvCaptureQueueFrame(Camera->Handle,&(Camera->Frame),NULL)) != ePvErrSuccess)
        {
            printf("CameraSnap: PvCaptureQueueFrame err: %u\n", errCode);
            return false;
        }
        return true;
    }
    
}
