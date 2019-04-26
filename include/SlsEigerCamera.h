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
 *  \file   SlsEigerCamera.h
 *  \brief  SlsEiger detector hardware class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSEIGERCAMERA_H
#define SLSEIGERCAMERA_H

#include "lima/Debug.h"
#include "SlsEigerCompatibility.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"
#include "lima/HwSyncCtrlObj.h"
#include "lima/HwEventCtrlObj.h"

/**********************************************************************/
namespace lima
{
    namespace SlsEiger
    {
        // pre-defines the Detector class
        class Detector;

        /***********************************************************************
         * \class Camera
         * \brief Hardware control object interface
         ***********************************************************************/

        class LIBSLSEIGER_API Camera
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "SlsEiger");
            friend class Interface;

        public:
            // status values
            enum Status
            {
                Idle   , // ready to start acquisition
                Waiting, // waiting for trigger signal
                Running, // acquisition is running 
                Error  , // acquisition stopped externally, fifo full or unexpected error 
            };

            // clock divider values
            enum ClockDivider
            {
                FullSpeed     , 
                HalfSpeed     , 
                QuarterSpeed  , 
                SuperSlowSpeed,
            };

            //==================================================================
            // constructor
            Camera(const std::string & in_config_file_name      ,  // complete path to the configuration file
                   const double        in_readout_time_sec      ,  // readout time in seconds
                   const long          in_receiver_fifo_depth   ,  // Number of frames in the receiver memory
                   const int           in_bit_depth             ,  // bit depth (8, 16, 32)
                   const long          in_frame_packet_number_8 ,  // Number of packets we should get in each receiver frame for 8 bits mode
                   const long          in_frame_packet_number_16,  // Number of packets we should get in each receiver frame for 16 bits mode
                   const long          in_frame_packet_number_32); // Number of packets we should get in each receiver frame for 32 bits mode

            // destructor (no need to be virtual)
            ~Camera();

            //==================================================================
            // Related to HwInterface
            //==================================================================
                //------------------------------------------------------------------
                // acquisition management
                //------------------------------------------------------------------
                // prepares the acquisition
                void prepareAcq();

                // starts the acquisition (start/snap)
                void startAcq();

                // stops the acquisition
                void stopAcq();

                // Acquisition data management
/*                void acquisitionDataReady(const int      in_receiver_index,
                                          const uint64_t in_frame_index   ,
                                          const uint32_t in_packet_number ,
                                          const uint64_t in_timestamp     ,
                                          const char *   in_data_pointer  ,
                                          const uint32_t in_data_size     );*/

                //------------------------------------------------------------------
                // status management
                //------------------------------------------------------------------
                // returns the current camera status
                // can not be const because internal member is updated during the call
                Camera::Status getStatus();

                //------------------------------------------------------------------
                // Acquired frames management
                //------------------------------------------------------------------
                // Gets the number of acquired frames
                uint64_t getNbAcquiredFrames() const;

                // get the number of frames in the containers
                void getNbFrames(size_t & out_received  , 
                                 size_t & out_not_merged,
                                 size_t & out_treated   ) const;

            //==================================================================
            // Related to HwDetInfoCtrlObj
            //==================================================================
                //------------------------------------------------------------------
                // image size management
                //------------------------------------------------------------------
                // Gets the maximum image width
                unsigned short getMaxWidth() const;

                // Gets the maximum image height
                unsigned short getMaxHeight() const;

                // Gets the image width
                unsigned short getWidth() const;

                // Gets the image height
                unsigned short getHeight() const;

                //------------------------------------------------------------------
                // current image type management
                //------------------------------------------------------------------
                // gets the default image type
                lima::ImageType getDefImageType() const;

                // gets the current image type
                lima::ImageType getImageType() const;

                // sets the current image type
                void setImageType(lima::ImageType in_type);

                //------------------------------------------------------------------
                // pixel size management
                //------------------------------------------------------------------
                // gets the pixel size
                void getPixelSize(double & out_x_size, double & out_y_size) const;

                //------------------------------------------------------------------
                // detector info management
                //------------------------------------------------------------------
                // gets the detector type
                std::string getDetectorType() const;

                // gets the detector model
                std::string getDetectorModel() const;

                // gets Detector Firmware Version
                std::string getDetectorFirmwareVersion() const;

                 // gets Detector Software Version
                std::string getDetectorSoftwareVersion() const;

                 // gets Detector This Software Version
                std::string getDetectorThisSoftwareVersion() const;

            //==================================================================
            // Related to HwSyncCtrlObj
            //==================================================================
                //------------------------------------------------------------------
                // trigger mode management
                //------------------------------------------------------------------
                // Checks the trigger mode validity
                bool checkTrigMode(lima::TrigMode in_trig_mode) const;

                // Sets the trigger mode
                void setTrigMode(lima::TrigMode in_mode);

                // Gets the trigger mode
                // can not be const because internal members are updated during the call
                lima::TrigMode getTrigMode();

                //------------------------------------------------------------------
                // exposure time management
                //------------------------------------------------------------------
                // Gets the exposure time
                // can not be const because internal members are updated during the call
                double getExpTime();

                // Sets the exposure time
                void setExpTime(double in_exp_time);

                //------------------------------------------------------------------
                // latency time management
                //------------------------------------------------------------------
                // Gets the latency time
                // can not be const because internal members are updated during the call
                double getLatencyTime();

                // Sets the latency time
                void setLatencyTime(double in_latency_time);

                //------------------------------------------------------------------
                // number of frames management
                //------------------------------------------------------------------
                // Sets the number of frames
                void setNbFrames(int64_t in_nb_frames);

                // Gets the number of frames
                // can not be const because internal member is updated during the call
                int64_t getNbFrames();

                //------------------------------------------------------------------
                // valid ranges management
                //------------------------------------------------------------------
                // Gets exposure and latency ranges
                HwSyncCtrlObj::ValidRangesType getValidRanges() const;

            //==================================================================
            // Related to BufferCtrl object
            //==================================================================
                // Gets the internal buffer manager
                HwBufferCtrlObj * getBufferCtrlObj();

            //==================================================================
            // Related to commands (put & get)
            //==================================================================
                // Executes a set command
                std::string setCmd(const std::string & in_command, int in_module_index = -1);

                // Executes a get command
                std::string getCmd(const std::string & in_command, int in_module_index = -1);

            //==================================================================
            // Related to specifics attributes
            //==================================================================
                //==================================================================
                // threshold energy management
                //==================================================================
                // Gets the threshold energy in eV
                int getThresholdEnergy();

                // Sets the threshold energy in eV
                void setThresholdEnergy(int in_threshold_energy_eV);

                //------------------------------------------------------------------
                // clock divider management
                //------------------------------------------------------------------
                // Gets the clock divider
                lima::SlsEiger::Camera::ClockDivider getClockDivider();

                // Sets the clock divider
                void setClockDivider(lima::SlsEiger::Camera::ClockDivider in_clock_divider);

                //==================================================================
                // delay after trigger management
                //==================================================================
                // Gets the maximum delay after trigger of all modules (in seconds)
                double getMaxDelayAfterTriggerAllModules(void);

                // Gets the delay after trigger of one module (in seconds)
                double getDelayAfterTrigger(int in_module_index);

                // Sets the delay after trigger (in seconds)
                void setDelayAfterTrigger(const double & in_delay_after_trigger_sec);

            //==================================================================
            // Related to event control object
            //==================================================================
                // Gets the Lima event control object
                HwEventCtrlObj* getEventCtrlObj();

        private:
            //==================================================================
            // Specifics methods management
            //==================================================================
                // Gets the internal number of frames (for thread access)
                uint64_t getInternalNbFrames();
/*
                // Gets the frame manager const access
                const CameraFrames & getFrameManager() const;

                // Gets the frame manager access
                CameraFrames & getFrameManager();
*/
            //------------------------------------------------------------------
            // acquisition management
            //------------------------------------------------------------------
                // start receiver listening mode
                bool startReceiver();

                // stop receiver listening mode
                bool stopReceiver();

                // start detector real time acquisition in non blocking mode
                bool startAcquisition();

                // stop detector real time acquisition
                bool stopAcquisition();

        private:
            //friend class CameraThread; // for getFrameManager(), getInternalNbFrames() and m_buffer_ctrl_obj accesses

            //------------------------------------------------------------------
            // Lima event control object
            //------------------------------------------------------------------
            HwEventCtrlObj m_event_ctrl_obj;

            //------------------------------------------------------------------
            // Lima buffer control object
            //------------------------------------------------------------------
            SoftBufferCtrlObj   m_buffer_ctrl_obj;

            // Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
            Detector * m_detector;








            //------------------------------------------------------------------
            // camera stuff
            //------------------------------------------------------------------
            // complete config file path
            std::string m_config_file_name;

            // readout time in seconds
            double m_readout_time_sec;

            // Number of frames in the receiver memory
            long m_receiver_fifo_depth;

            // Number of packets we should get in each receiver frame
            uint32_t m_frame_packet_number;

            // exposure time
            double m_exposure_time;

            // latency time
            double m_latency_time;

            // trigger mode label from sls sdk
            std::string m_trig_mode_label;

            // trigger mode 
            lima::TrigMode m_trig_mode;

            // number of frames per cycle from sls sdk
            int64_t m_nb_frames_per_cycle;

            // number of cycle from sls sdk
            int64_t m_nb_cycles;

            // number total of frames
            int64_t m_nb_frames;

            // last known status
            Camera::Status m_status;
            
            // delay after trigger in seconds
            double m_delay_after_trigger;

            // clock divider
            Camera::ClockDivider m_clock_divider;

            // treshold energy
            int m_threshold_energy_eV; 

            //------------------------------------------------------------------
            // main acquisition thread
            //------------------------------------------------------------------
            //CameraThread m_thread;

            //------------------------------------------------------------------
            // frames manager
            //------------------------------------------------------------------
            //CameraFrames m_frames_manager;
        };
    }
}
#endif // SLSEIGERCAMERA_H

/*************************************************************************/