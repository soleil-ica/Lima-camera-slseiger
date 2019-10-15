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
 *  \file   SlsEigerCameraThreadLima.h
 *  \brief  SlsEiger detector acquisition thread class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "SlsEigerCameraThreadLima.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima::SlsEiger;

#include <cmath>

/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraThreadLima::CameraThreadLima(yat::SharedPtr<CameraFrames> in_frames_manager, yat::SharedPtr<Camera> in_cam) : 
                                   m_cam(in_cam), CameraThread(in_frames_manager)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThreadLima::~CameraThreadLima";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraThreadLima::~CameraThreadLima()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThread::~CameraThread";
}

/************************************************************************
 * \brief start the acquisition
 * \return true if ok, else false
************************************************************************/
bool CameraThreadLima::startAcquisition()
{
    bool result = true;

    // starting receiver listening mode
    if(m_cam->startReceiver() == slsDetectorDefs::FAIL)
    {
        std::string errorText = "Can not start the receiver listening mode!";
        REPORT_EVENT(errorText);
        result = false;
    }
    else
    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(m_cam->startAcquisition() == slsDetectorDefs::FAIL)
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
bool CameraThreadLima::stopAcquisition()
{
    // checking if the hardware acquisition is running or in error
    if(isAcquisitionRunning())
    {
        // stop detector acquisition
        m_cam->stopAcquisition();
    }

    // stop receiver listening mode
    return (m_cam->stopReceiver() != slsDetectorDefs::FAIL);
}

/************************************************************************
 * \brief check if the acquisition is running 
 * \return true if ok, else false
************************************************************************/
bool CameraThreadLima::isAcquisitionRunning()
{
    Camera::Status status = m_cam->getDetectorStatus();
    return ((status == Camera::Waiting) || (status == Camera::Running));
}

/************************************************************************
 * \brief check if all frames were acquired
 * \return true if ok, else false
************************************************************************/
bool CameraThreadLima::allFramesAcquired()
{
    return (m_cam->getNbAcquiredFrames() == m_cam->getInternalNbFrames());
}

/************************************************************************
 * \brief get the number of lost frames during the acquisition
 * \return number of lost frames
************************************************************************/
std::size_t CameraThreadLima::getLostFramesNb()
{
    std::size_t lost_frames_nb = m_cam->getInternalNbFrames() - m_cam->getNbAcquiredFrames();
    return lost_frames_nb;
}

/************************************************************************
 * \brief Register the start timestamp
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThreadLima::registerStartTimestamp(lima::Timestamp in_start_timestamp)
{
    m_cam->getBufferCtrlObj().setStartTimestamp(in_start_timestamp);
}

/************************************************************************
 * \brief get the image buffer data to copy the frame into
 * \param in_frame current frame
 * \param out_image_buffer pointer to the destination image buffer 
 * \param out_image_buffer_byte_size size of the image buffer in bytes
************************************************************************/
void CameraThreadLima::getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                      char *      & out_image_buffer           , 
                                      std::size_t & out_image_buffer_byte_size )
{
    DEB_MEMBER_FUNCT();

    StdBufferCbMgr & buffer_mgr  = m_cam->getBufferCtrlObj();
    lima::FrameDim   frame_dim   = buffer_mgr.getFrameDim();

    out_image_buffer_byte_size = frame_dim.getMemSize();
    out_image_buffer = static_cast<char *>(buffer_mgr.getFrameBufferPtr(in_frame->getIndex()));
}

/************************************************************************
 * \brief Treat a new ready image buffer
 * \param in_frame current frame
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThreadLima::TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
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

    m_cam->getBufferCtrlObj().newFrameReady(frame_info);
}

/************************************************************************
 * \brief Manage an incomming error
 * \param in_error_text text which describes the error
************************************************************************/
void CameraThreadLima::manageError(std::string & in_error_text)
{
    REPORT_EVENT(in_error_text);
}

//========================================================================================
