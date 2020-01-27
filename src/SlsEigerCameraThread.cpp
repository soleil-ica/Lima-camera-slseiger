//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2018
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

/*************************************************************************/
/*! 
 *  \file   SlsEigerCameraThread.h
 *  \brief  SlsEiger detector acquisition thread class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "lima/Exceptions.h"
#include "SlsEigerCameraThread.h"
#include "SlsEigerCamera.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima::SlsEiger;

#include <cmath>

// define it if you want an absolute timestamp in the frame data.
// let it commented if you want a relative timestamp in the frame data (starting at 0).
//#define CAMERA_THREAD_USE_ABSOLUTE_TIMESTAMP

/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraThread::CameraThread(CameraFrames * in_frames_manager, Camera * in_cam) : m_cam(in_cam), m_frames_manager(in_frames_manager)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::CameraThread - BEGIN";
    m_force_stop = false;
    DEB_TRACE() << "CameraThread::CameraThread - END";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraThread::~CameraThread()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::~CameraThread";
}

/************************************************************************
 * \brief starts the thread
 ************************************************************************/
void CameraThread::start()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::start - BEGIN";
    CmdThread::start();
    waitStatus(CameraThread::Idle);
    DEB_TRACE() << "CameraThread::start - END";
}

/************************************************************************
 * \brief inits the thread
 ************************************************************************/
void CameraThread::init()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::init - BEGIN";
    setStatus(CameraThread::Idle);
    DEB_TRACE() << "CameraThread::init - END";
}

/************************************************************************
 * \brief aborts the thread
 ************************************************************************/
void CameraThread::abort()
{
	DEB_MEMBER_FUNCT();
    CmdThread::abort();
}

/************************************************************************
 * \brief command execution
 * \param cmd command indentifier
************************************************************************/
void CameraThread::execCmd(int cmd)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::execCmd - BEGIN";
    int status = getStatus();

    try
    {
        switch (cmd)
        {
            case CameraThread::StartAcq:
                if (status == CameraThread::Idle)
                    execStartAcq();
                break;

            default:
                break;
        }
    }
    catch (...)
    {
    }

    DEB_TRACE() << "CameraThread::execCmd - END";
}

/************************************************************************
 * \brief execute the stop command
 ************************************************************************/
void CameraThread::execStopAcq()
{
    DEB_MEMBER_FUNCT();

    if(getStatus() == CameraThread::Running)
    {
        m_force_stop = true;

        // Waiting for thread to finish or to be in error
        waitNotStatus(CameraThread::Running);
    }
}

/************************************************************************
 * \brief execute the start command
 ************************************************************************/
void CameraThread::execStartAcq()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "executing StartAcq command...";

    const double end_acq_sleep_time_sec = 0.5; // sleep the thread in seconds
    const double no_frame_sleep_time_sec = 0.1; // sleep the thread in seconds

    m_force_stop = false;

    // the thread is running a new acquisition (it frees the Camera::startAcq method)
    setStatus(CameraThread::Running);

    // starting acquisition
    if(!startAcquisition())
    {
        setStatus(CameraThread::Error);

        // after a live mode, we need to restore the previous data : frames number, trigger mode, latency time
        m_cam->restoreDataAfterLiveMode();

        return;
    }

    // setting the start timestamp
    lima::Timestamp start_timestamp = lima::Timestamp::now();
    registerStartTimestamp(start_timestamp);

    // Main acquisition loop
    // m_force_stop can be set to true by the execStopAcq call to abort an acquisition
    // m_force_stop can be set to true also with an error hardware camera status
    // the loop can also end if the number of 
    while(!m_force_stop && !allFramesAcquired())
    {
        // treating all complete frames
        treatNotMergedFrames(start_timestamp);

        // checking if the hardware acquisition is running and if there is no more frame to treat in the containers
        if(!isAcquisitionRunning())
        {
            lima::Sleep(end_acq_sleep_time_sec); // sleep the thread in seconds because a data callback can be activated

            if(m_frames_manager->NoMoreFrameToTreat())
            {
                // we stop the treatment - seems we have lost frame(s)
                break;
            }
        }
        else
        // hardware acquisition is running, we are waiting for new frames not using the cpu during this time
        {
            lima::Sleep(no_frame_sleep_time_sec); // sleep the thread in seconds
        }
    }

    // acquisition was aborted
    if(m_force_stop || !allFramesAcquired())
    {
        stopAcquisition();

        // treating all complete frames which were not treated yet
        treatNotMergedFrames(start_timestamp);

        // problem occured during the acquisition ?
        if(!m_force_stop)
        {
            std::size_t lost_frames_nb = getLostFramesNb();

            // checking again because sometimes the camera status can change to idle before we receive the frame
            if(lost_frames_nb > 0)
            {
                setStatus(CameraThread::Error);

                std::size_t received  ;
                std::size_t not_merged;
                std::size_t treated   ;
                
                m_frames_manager->getNbFrames(received, not_merged, treated );

                // after a live mode, we need to restore the previous data : frames number, trigger mode, latency time
                m_cam->restoreDataAfterLiveMode();

                DEB_TRACE() << "frames received   (" << received   << ")";
                DEB_TRACE() << "frames not merged (" << not_merged << ")";
                DEB_TRACE() << "frames treated    (" << treated    << ")";

                std::stringstream temp_stream;
                temp_stream << "Lost " << lost_frames_nb << " frame(s) during real time acquisition!";
                std::string error_text = temp_stream.str();
                manageError(error_text);
                return;
            }
        }
    }
    else
    // no problem occured during the acquisition
    {
        // stopping receiver listening mode
        if(!stopAcquisition())
        {
            setStatus(CameraThread::Error);

            // after a live mode, we need to restore the previous data : frames number, trigger mode, latency time
            m_cam->restoreDataAfterLiveMode();

            std::string error_text = "Could not stop real time acquisition!";
            manageError(error_text);
            return;
        }
    }

    // waiting for an idle or error hardware status
    for(;;)
    {
        // checking if the hardware acquisition is running
        if(!isAcquisitionRunning())
        {
            break;
        }
        else
        {
            lima::Sleep(end_acq_sleep_time_sec); // sleep the thread in seconds
        }
    }

    // change the thread status only if the thread is not in error
    if(getStatus() == CameraThread::Running)
    {
        setStatus(CameraThread::Idle);

        // after a live mode, we need to restore the previous data : frames number, trigger mode, latency time
        m_cam->restoreDataAfterLiveMode();
    }
}

/************************************************************************
 * \brief treat all not merged frames
 * \param in_start_timestamp start timestamp
 * \param in_buffer_mgr buffer manager
************************************************************************/
void CameraThread::treatNotMergedFrames(lima::Timestamp in_start_timestamp)
{
    DEB_MEMBER_FUNCT();

    const double sleep_time_sec = 0.000001; // sleep the thread in seconds (1µs)

    yat::SharedPtr<CameraFrame> not_merged_frame      ;
    char *                      image_buffer          ; 
    std::size_t                 image_buffer_byte_size;

    // getting the first not merged frame from the frames container
    m_frames_manager->getFirstNotMerged(not_merged_frame);

    while(not_merged_frame != NULL)
    {
        // get the information about the corresponding image buffer
        getImageBuffer(not_merged_frame, image_buffer, image_buffer_byte_size);

        // build the image
        m_frames_manager->buildImage(not_merged_frame, image_buffer, image_buffer_byte_size);
        
        TreatImageBufferReady(not_merged_frame, in_start_timestamp);

        // the not merged frame has been treated, so we move it to the final container
        m_frames_manager->moveFirstNotMergedToTreated();

        lima::Sleep(sleep_time_sec); // sleep the thread in seconds to help changing thread and avoid blocking acquisition callbacks

        // getting the first not merged frame from the frames container
        m_frames_manager->getFirstNotMerged(not_merged_frame);
    }
}

/************************************************************************
 * \brief start the acquisition
 * \return true if ok, else false
************************************************************************/
bool CameraThread::startAcquisition()
{
    bool result = true;

    // starting receiver listening mode
    if(!m_cam->startReceiver())
    {
        std::string errorText = "Can not start the receiver listening mode!";
        REPORT_EVENT(errorText);
        result = false;
    }
    else
    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(!m_cam->startAcquisition())
    {
        m_cam->stopReceiver();

        std::string errorText = "Can not start real time acquisition!";
        REPORT_EVENT(errorText);
        result = false;
    }

    return result;
}

/************************************************************************
 * \brief stop the acquisition
 * \return true if ok, else false
************************************************************************/
bool CameraThread::stopAcquisition()
{
    // checking if the hardware acquisition is running or in error
    if(isAcquisitionRunning())
    {
        // stop detector acquisition
        m_cam->stopAcquisition();
    }

    // stop receiver listening mode
    return m_cam->stopReceiver();
}

/************************************************************************
 * \brief check if the acquisition is running 
 * \return true if ok, else false
************************************************************************/
bool CameraThread::isAcquisitionRunning()
{
    SlsEiger::Status status = m_cam->getDetectorStatus();
    return ((status == SlsEiger::Waiting) || (status == SlsEiger::Running));
}

/************************************************************************
 * \brief check if all frames were acquired
 * \return true if ok, else false
************************************************************************/
bool CameraThread::allFramesAcquired()
{
    return (m_cam->getNbAcquiredFrames() == m_cam->getInternalNbFrames());
}

/************************************************************************
 * \brief get the number of lost frames during the acquisition
 * \return number of lost frames
************************************************************************/
std::size_t CameraThread::getLostFramesNb()
{
    std::size_t lost_frames_nb = m_cam->getInternalNbFrames() - m_cam->getNbAcquiredFrames();
    return lost_frames_nb;
}

/************************************************************************
 * \brief Register the start timestamp
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThread::registerStartTimestamp(lima::Timestamp in_start_timestamp)
{
    m_cam->getStdBufferCbMgr().setStartTimestamp(in_start_timestamp);
}

/************************************************************************
 * \brief get the image buffer data to copy the frame into
 * \param in_frame current frame
 * \param out_image_buffer pointer to the destination image buffer 
 * \param out_image_buffer_byte_size size of the image buffer in bytes
************************************************************************/
void CameraThread::getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                  char *      & out_image_buffer            , 
                                  std::size_t & out_image_buffer_byte_size  )
{
    DEB_MEMBER_FUNCT();

    StdBufferCbMgr & buffer_mgr  = m_cam->getStdBufferCbMgr();
    lima::FrameDim   frame_dim   = buffer_mgr.getFrameDim();

    out_image_buffer_byte_size = frame_dim.getMemSize();
    out_image_buffer = static_cast<char *>(buffer_mgr.getFrameBufferPtr(in_frame->getIndex()));
}

/************************************************************************
 * \brief Treat a new ready image buffer
 * \param in_frame current frame
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThread::TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
                                         Timestamp in_start_timestamp)
{
    DEB_MEMBER_FUNCT();

    HwFrameInfoType frame_info;
    double frame_timestamp_sec;

    frame_info.acq_frame_nb = in_frame->getIndex();

    // converting the timestamp which is in 10MHz to seconds
    frame_timestamp_sec = static_cast<double>(in_frame->getTimestamp()) / 10000000.0;

    #ifdef CAMERA_THREAD_USE_ABSOLUTE_TIMESTAMP
        frame_info.frame_timestamp = in_start_timestamp + Timestamp(frame_timestamp_sec);
    #else
        frame_info.frame_timestamp = Timestamp(frame_timestamp_sec);
    #endif

    //Pushing the image buffer through Lima 
    DEB_TRACE() << "New Frame Ready [ " << frame_info.acq_frame_nb << ", " << frame_timestamp_sec << " ]";

    m_cam->getStdBufferCbMgr().newFrameReady(frame_info);
}

/************************************************************************
 * \brief Manage an incomming error
 * \param in_error_text text which describes the error
************************************************************************/
void CameraThread::manageError(std::string & in_error_text)
{
    REPORT_EVENT(in_error_text);
}

//========================================================================================
