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

#include "SlsEigerCompatibility.h"
#include "SlsEigerCameraFrames.h"

/*************************************************************************/

namespace lima
{
    namespace SlsEiger
    {
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
            CameraThread(yat::SharedPtr<CameraFrames> in_frames_manager);

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
            virtual bool startAcquisition() = 0;

            // stop the acquisition
            virtual bool stopAcquisition() = 0;

            // check if the acquisition is running 
            virtual bool isAcquisitionRunning() = 0;

            // check if all frames were acquired
            virtual bool allFramesAcquired() = 0;

            // get the number of lost frames during the acquisition
            virtual std::size_t getLostFramesNb() = 0;

            // Register the start timestamp
            virtual void registerStartTimestamp(lima::Timestamp in_start_timestamp) = 0;

            // get the image buffer data to copy the frame into
            virtual void getImageBuffer(const yat::SharedPtr<CameraFrame> in_frame,
                                        char *      & out_image_buffer           , 
                                        std::size_t & out_image_buffer_byte_size ) = 0;

            // Treat a new ready image buffer
            virtual void TreatImageBufferReady(const yat::SharedPtr<CameraFrame> in_frame,
                                               lima::Timestamp in_start_timestamp) = 0;

            // Manage an incomming error
            virtual void manageError(std::string & in_error_text) = 0;

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

        protected:
            yat::SharedPtr<CameraFrames> m_frames_manager;

        private :
            volatile bool m_force_stop;
        };
    }
}
#endif // SLSEIGERCAMERATHREAD_H

/*************************************************************************/