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
CameraThread::CameraThread(yat::SharedPtr<CameraFrames> in_frames_manager) : m_frames_manager(in_frames_manager)
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
    abort();
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
	DEB_TRACE() << "DONE";
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

    int status = getStatus();

    if (status == CameraThread::Running)
        m_force_stop = true;
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

                DEB_TRACE() << "frames received   (" << received   << ")";
                DEB_TRACE() << "frames not merged (" << not_merged << ")";
                DEB_TRACE() << "frames treated    (" << treated    << ")";

                std::stringstream temp_stream;
                temp_stream << "Lost " << lost_frames_nb << " frames during real time acquisition!";
                std::string error_text = temp_stream.str();
                manageError(error_text);
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

            std::string error_text = "Could not stop real time acquisition!";
            manageError(error_text);
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
    char *                     image_buffer          ; 
    std::size_t                image_buffer_byte_size;

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

//========================================================================================
