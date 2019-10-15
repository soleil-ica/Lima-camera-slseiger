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
 *  \file   SlsEigerCameraThreadLima.h
 *  \brief  SlsEiger detector acquisition thread class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSEIGERCAMERATHREADLIMA_H
#define SLSEIGERCAMERATHREADLIMA_H

#include "lima/HwBufferMgr.h"
#include "lima/HwEventCtrlObj.h"

#include "SlsEigerCompatibility.h"
#include "SlsEigerCameraThread.h"

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
        /**********************************************************************/
        // defines the SlsEiger camera class for direct access
        class Camera
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsEiger");

        public :
            // status values
            enum Status
            {
                Idle   , // ready to start acquisition
                Waiting, // waiting for trigger or gate signal  
                Running, // acquisition is running 
                Error  , // acquisition stopped externally, fifo full or unexpected error 
            };

            //==================================================================
            // Related to BufferCtrl object
            //==================================================================
            /*******************************************************************
             * \brief Gets the internal buffer manager
             * \return the internal buffer manager
             *******************************************************************/
            StdBufferCbMgr & getBufferCtrlObj()
            {
                DEB_MEMBER_FUNCT();
                return m_buffer_ctrl_obj.getBuffer();
            }

            //==================================================================
            // Related to event control object
            //==================================================================
            // Gets the Lima event control object
            HwEventCtrlObj * getEventCtrlObj()
            {
                DEB_MEMBER_FUNCT();
                return &m_event_ctrl_obj;
            }

            //------------------------------------------------------------------
            // acquisition management
            //------------------------------------------------------------------
            // start receiver listening mode
            int startReceiver() {return 0;};

            // stop receiver listening mode
            int stopReceiver() {return 0;};

            // start detector real time acquisition in non blocking mode
            int startAcquisition() {return 0;};

            // stop detector real time acquisition
            int stopAcquisition() {return 0;};

            //------------------------------------------------------------------
            // status management
            //------------------------------------------------------------------
            // returns the current detector status
            Camera::Status getDetectorStatus() {return Status::Idle;};

            //------------------------------------------------------------------
            // Acquired frames management
            //------------------------------------------------------------------
            // Gets the number of acquired frames
            uint64_t getNbAcquiredFrames() const {return 0;};

            // Gets the internal number of frames (for thread access)
            uint64_t getInternalNbFrames() {return 0;};

        private :
            //------------------------------------------------------------------
            // Lima buffer control object
            //------------------------------------------------------------------
            SoftBufferCtrlObj m_buffer_ctrl_obj;

            //------------------------------------------------------------------
            // Lima event control object
            //------------------------------------------------------------------
            HwEventCtrlObj m_event_ctrl_obj;
        };

        /***********************************************************************
         * \class CameraThreadLima
         * \brief used to handle some specific tasks (startAcq, stopAcq, ...)
         ***********************************************************************/

        class CameraThreadLima : public CameraThread
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraThreadLima", "SlsEiger");

        public:
            // constructor
            CameraThreadLima(yat::SharedPtr<CameraFrames> in_frames_manager, yat::SharedPtr<Camera> in_cam);

            // destructor
            ~CameraThreadLima();

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
            // direct access to camera
            yat::SharedPtr<Camera> m_cam;
        };
    }
}
#endif // SLSEIGERCAMERATHREADLIMA_H

/*************************************************************************/