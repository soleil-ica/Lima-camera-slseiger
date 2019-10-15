//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2019
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
 *  \brief  SlsEiger detector acquisition thread class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSEIGERCAMERATHREADTEST_H
#define SLSEIGERCAMERATHREADTEST_H

#include "lima/HwBufferMgr.h"
#include "lima/HwEventCtrlObj.h"

#include "SlsEigerCompatibility.h"
#include "SlsEigerCameraThread.h"

#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

#include "StreamNexus.h"

//#define EIGER_DETECTOR_SIMULATION
#define EIGER_DETECTOR_SIMULATION_LOST_FRAME_PART

#define EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE    (0)
#define EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_UNCOMPLETE  (1) 
#define EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_LOST        (2) 

// distribution of type of frame parts
#define EIGER_DETECTOR_SIMULATION_FRAME_PART_DISTRIBUTION_NB_ELEMENTS (10)

static const int EIGER_DETECTOR_SIMULATION_FRAME_PART_DISTRIBUTION[EIGER_DETECTOR_SIMULATION_FRAME_PART_DISTRIBUTION_NB_ELEMENTS] = 
{
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_COMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_UNCOMPLETE,
    EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_LOST,
};

namespace lima
{
    namespace SlsEiger
    {
        /***********************************************************************
         * \class CameraThreadTest
         * \brief used to handle some specific tasks (startAcq, stopAcq, ...)
         ***********************************************************************/

        class CameraThreadTest : public CameraThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraThreadTest", "SlsEiger");

        public:
            // constructor
            CameraThreadTest(yat::SharedPtr<CameraFrames> in_frames_manager,
                             yat::SharedPtr<StreamNexus> in_stream_nexus,
                             slsDetectorUsers * in_detector, 
                             std::size_t in_nb_frames);

            // destructor
            virtual ~CameraThreadTest();

        #ifdef EIGER_DETECTOR_SIMULATION
            // the acquisition is ended - used to simulate the end of acquisition
            void flagAcquisitionAsEnded();
        #endif

        protected:
            // start the acquisition
            virtual bool startAcquisition();

            // stop the acquisition
            virtual bool stopAcquisition();

            // check if the acquisition is running 
            virtual bool isAcquisitionRunning();

            // check if all frames were acquired
            virtual bool allFramesAcquired();

            // get the number of lost frames during the acquisition
            virtual std::size_t getLostFramesNb();

            // Register the start timestamp
            virtual void registerStartTimestamp(lima::Timestamp in_start_timestamp);

            // get the image buffer data to copy the frame into
            virtual void getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                        char *      & out_image_buffer           , 
                                        std::size_t & out_image_buffer_byte_size );

            // Treat a new ready image buffer
            virtual void TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
                                               Timestamp in_start_timestamp);

            // Manage an incomming error
            virtual void manageError(std::string & in_error_text);

        private:
            // direct access to the sls detector
            slsDetectorUsers * m_detector;

            // number of frames to acquire
            std::size_t m_nb_frames;

            lima::Timestamp m_start_timestamp;

            FramesListContainer m_not_saved_frames;
            yat::SharedPtr<StreamNexus> m_stream_nexus;

        #ifdef EIGER_DETECTOR_SIMULATION
            bool m_acquisition_ended;
        #endif
        };
    }
}
#endif // SLSEIGERCAMERATHREADTEST_H

/*************************************************************************/
