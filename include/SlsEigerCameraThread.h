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
 *  \file   SlsEigerCameraThread.h
 *  \brief  SlsEiger detector acquisition thread class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSEIGERCAMERATHREAD_H
#define SLSEIGERCAMERATHREAD_H

#include "lima/Debug.h"
#include "lima/Constants.h"
#include "lima/ThreadUtils.h"
#include "lima/Timestamp.h"
#include "lima/HwBufferMgr.h"
#include "lima/HwEventCtrlObj.h"

#include "SlsEigerCompatibility.h"
#include "SlsEigerCameraFrames.h"

/*************************************************************************/
#define REPORT_EVENT(desc)  {   \
                                Event *my_event = new Event(Hardware,Event::Info, Event::Camera, Event::Default,desc); \
                                m_cam->getEventCtrlObj()->reportEvent(my_event); \
                            }
/*************************************************************************/

namespace lima
{
    namespace SlsEiger
    {
        // pre-defines the Camera class
        class Camera;

        /***********************************************************************
         * \class CameraThread
         * \brief used to handle some specific tasks (startAcq, stopAcq, ...)
         ***********************************************************************/

        class CameraThread : public CmdThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "SlsEiger");

        public:
			// Status
            enum
			{ 
				Idle    = MaxThreadStatus, // ready to manage acquisition
                Running                  , // acquisition is running 
                Error                    , // unexpected error
			};

            // Cmd
            enum
            { 
                StartAcq = MaxThreadCmd, // command used to start acquisition
            };

            // constructor
            CameraThread(CameraFrames * in_frames_manager, Camera * in_cam);

            // destructor
            virtual ~CameraThread();

            // starts the thread
            virtual void start();

            // aborts the thread
            virtual void abort();

            // execute the stop command
            void execStopAcq();

        protected:
            // start the acquisition
            bool startAcquisition();

            // stop the acquisition
            bool stopAcquisition();

            // check if the acquisition is running 
            bool isAcquisitionRunning();

            // check if all frames were acquired
            bool allFramesAcquired();

            // get the number of lost frames during the acquisition
            std::size_t getLostFramesNb();

            // Register the start timestamp
            void registerStartTimestamp(lima::Timestamp in_start_timestamp);

            // get the image buffer data to copy the frame into
            void getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                char *      & out_image_buffer            , 
                                std::size_t & out_image_buffer_byte_size  );

            // Treat a new ready image buffer
            void TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
                                       lima::Timestamp in_start_timestamp);

            // Manage an incomming error
            void manageError(std::string & in_error_text);

        protected:
            // inits the thread
            virtual void init();

            // command execution
            virtual void execCmd(int cmd);

        private:
            // execute the start command
            void execStartAcq();

            // treat all not merged frames
            void treatNotMergedFrames(lima::Timestamp in_start_timestamp);

        private :
            volatile bool m_force_stop;

            Camera       * m_cam           ; // direct access to camera
            CameraFrames * m_frames_manager; // direct access to frame manager
        };
    }
}
#endif // SLSEIGERCAMERATHREAD_H

/*************************************************************************/