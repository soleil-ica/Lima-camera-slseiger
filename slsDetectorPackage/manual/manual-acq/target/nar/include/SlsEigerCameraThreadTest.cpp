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
 *  \file   SlsEigerCameraThreadTest.h
 *  \brief  SlsEiger detector acquisition thread class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "SlsEigerCameraThreadTest.h"

using namespace lima::SlsEiger;

#include <cmath>

/************************************************************************
 * \brief constructor
 ************************************************************************/
CameraThreadTest::CameraThreadTest(yat::SharedPtr<CameraFrames> in_frames_manager,
                                   yat::SharedPtr<StreamNexus>  in_stream_nexus  ,
                                   slsDetectorUsers * in_detector, 
                                   std::size_t in_nb_frames) : 
                                   m_nb_frames(in_nb_frames), m_detector(in_detector), m_stream_nexus(in_stream_nexus), CameraThread(in_frames_manager)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThreadTest::~CameraThreadTest";

#ifdef EIGER_DETECTOR_SIMULATION
    m_acquisition_ended = false;
#endif
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraThreadTest::~CameraThreadTest()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "CameraThreadTest::~CameraThreadTest";

    m_not_saved_frames.clear();
}

/************************************************************************
 * \brief start the acquisition
 * \return true if ok, else false
************************************************************************/
bool CameraThreadTest::startAcquisition()
{
    DEB_MEMBER_FUNCT();
    bool result = true;

    m_not_saved_frames.clear();

#ifndef EIGER_DETECTOR_SIMULATION
    // starting receiver listening mode
    if(m_detector->startReceiver() == slsDetectorDefs::FAIL)
    {
        std::string errorText = "Can not start the receiver listening mode!";
        DEB_TRACE() << errorText;
        result = false;
    }
    else
    // starting real time acquisition in non blocking mode
    // returns OK if all detectors are properly started, FAIL otherwise
    if(m_detector->startAcquisition() == slsDetectorDefs::FAIL)
    {
        m_detector->stopReceiver();

        std::string errorText = "Can not start real time acquisition!";
        DEB_TRACE() << errorText;
        result = false;
    }
#endif

    return result;
}

/************************************************************************
 * \brief stop the acquisition
 * \return true if ok, else false
************************************************************************/
bool CameraThreadTest::stopAcquisition()
{
#ifndef EIGER_DETECTOR_SIMULATION
    // checking if the hardware acquisition is running or in error
    if(isAcquisitionRunning())
    {
        // stop detector acquisition
        m_detector->stopAcquisition();
    }

    // stop receiver listening mode
    return (m_detector->stopReceiver() != slsDetectorDefs::FAIL);
#else
    return true;
#endif
}

#ifdef EIGER_DETECTOR_SIMULATION
/************************************************************************
 * \brief the acquisition is ended
************************************************************************/
void CameraThreadTest::flagAcquisitionAsEnded()
{
    m_acquisition_ended = true;
}
#endif

/************************************************************************
 * \brief check if the acquisition is running 
 * \return true if ok, else false
************************************************************************/
bool CameraThreadTest::isAcquisitionRunning()
{
#ifndef EIGER_DETECTOR_SIMULATION
    int status = m_detector->getDetectorStatus();
    return (!((status == slsDetectorDefs::IDLE) || (status == slsDetectorDefs::STOPPED) || (status == slsDetectorDefs::ERROR)));
#else
    return !m_acquisition_ended;
#endif
}

/************************************************************************
 * \brief check if all frames were acquired
 * \return true if ok, else false
************************************************************************/
bool CameraThreadTest::allFramesAcquired()
{
    return (m_nb_frames == m_frames_manager->getNbTreatedFrames());
}

/************************************************************************
 * \brief get the number of lost frames during the acquisition
 * \return number of lost frames
************************************************************************/
std::size_t CameraThreadTest::getLostFramesNb()
{
    std::size_t lost_frames_nb = (m_nb_frames - m_frames_manager->getNbTreatedFrames());
    return lost_frames_nb;
}

/************************************************************************
 * \brief Register the start timestamp
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThreadTest::registerStartTimestamp(lima::Timestamp in_start_timestamp)
{
    m_start_timestamp = in_start_timestamp;
}

/************************************************************************
 * \brief get the image buffer data to copy the frame into
 * \param in_frame current frame
 * \param out_image_buffer pointer to the destination image buffer 
 * \param out_image_buffer_byte_size size of the image buffer in bytes
************************************************************************/
void CameraThreadTest::getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                      char *      & out_image_buffer           , 
                                      std::size_t & out_image_buffer_byte_size )
{
    DEB_MEMBER_FUNCT();

    std::size_t frame_byte_size = m_frames_manager->getFrameSizex() * m_frames_manager->getFrameSizey() * m_frames_manager->getColorDepthBytesNb();

    yat::SharedPtr<CameraFramePart> frame_part = new CameraFramePart(0, 0, 0, in_frame->getTimestamp(), NULL, frame_byte_size);
    yat::SharedPtr<CameraFrame>     frame      = new CameraFrame(in_frame->getIndex());
    frame->setTimestamp(in_frame->getTimestamp());

    frame->addPart(frame_part);

    out_image_buffer = reinterpret_cast<char *>(frame_part->getDataWritePointer());
    out_image_buffer_byte_size = frame_byte_size;

    m_not_saved_frames.push_back(frame);
}

/************************************************************************
 * \brief Treat a new ready image buffer
 * \param in_frame current frame
 * \param in_start_timestamp start timestamp
************************************************************************/
void CameraThreadTest::TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
                                             Timestamp in_start_timestamp)
{
    DEB_MEMBER_FUNCT();
    m_stream_nexus->update(m_not_saved_frames);
}

/************************************************************************
 * \brief Manage an incomming error
 * \param in_error_text text which describes the error
************************************************************************/
void CameraThreadTest::manageError(std::string & in_error_text)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << in_error_text;
}

//========================================================================================
