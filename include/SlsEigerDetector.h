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

/********************************************************************************/
/*! 
 *  \file   SlsEigerDetector.h
 *  \brief  detector class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSEIGERDETECTOR_H
#define SLSEIGERDETECTOR_H

#include "lima/Debug.h"
#include "SlsEigerCompatibility.h"
#include "lima/Constants.h"

// LOCAL
#include "SlsEigerReceivers.h"

/**********************************************************************/
// defines the SLS slsDetectorUsers class
// Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
class slsDetectorUsers;

namespace lima
{
    namespace SlsEiger
    {
        static const int SLS_GET_VALUE = -1; // special value used to call set/get sls methods into get mode

        static const std::string SLS_TRIGGER_MODE_AUTO    = "auto"   ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_TRIGGER_MODE_TRIGGER = "trigger"; // there is no triggers enums in the current sls sdk

        static const std::string SLS_GAIN_MODE_DYNAMIC   = "dynamicgain" ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_GAIN_MODE_LOW       = "lowgain"     ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_GAIN_MODE_MEDIUM    = "mediumgain"  ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_GAIN_MODE_HIGH      = "highgain"    ; // there is no triggers enums in the current sls sdk
        static const std::string SLS_GAIN_MODE_VERY_HIGH = "veryhighgain"; // there is no triggers enums in the current sls sdk

        /***********************************************************************
         * \class Detector
         * \brief class used to control an Eiger Psi detector
         ***********************************************************************/

        class Detector
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Detector", "SlsEiger");

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

            // trigger mode values
            enum TriggerMode 
            { 
                TRIGGER_INTERNAL_SINGLE    = 0,
                TRIGGER_EXTERNAL_SINGLE    ,
                TRIGGER_EXTERNAL_MULTIPLE  ,
            };

            //==================================================================
            // constructor
            explicit Detector(const std::string & in_config_file_name      ,
                              const double        in_readout_time_sec      ,
                              const long          in_receiver_fifo_depth   ,
                              const int           in_bit_depth             ,
                              const long          in_frame_packet_number_8 ,
                              const long          in_frame_packet_number_16,
                              const long          in_frame_packet_number_32);

            // destructor (needs to be virtual)
            virtual ~Detector();

            // gets the number of modules
            int getModulesNb(void);

            //------------------------------------------------------------------
            // image size management
            //------------------------------------------------------------------
            // Gets the maximum image width
            int getMaxWidth() const;

            // Gets the maximum image height
            int getMaxHeight() const;

            // Gets the image width
            int getWidth() const;

            // Gets the image height
            int getHeight() const;

            // Gets the bit depth
            int getBitDepth();

            // Sets the bit depth (8, 16, 32)
            void setBitDepth(const int & in_bit_depth);

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

            //------------------------------------------------------------------
            // trigger mode management
            //------------------------------------------------------------------
            // Sets the trigger mode
            void setTriggerMode(const Detector::TriggerMode & in_trigger_mode);

            // Gets the trigger mode
            // can not be const because internal members are updated during the call
            Detector::TriggerMode getTriggerMode(void);

            //------------------------------------------------------------------
            // number of frames management
            //------------------------------------------------------------------
            // Sets the number of frames
            void setNbFrames(int64_t in_nb_frames);

            // Gets the number of frames
            // can not be const because internal member is updated during the call
            int64_t getNbFrames();

            // Gets the internal number of frames (for thread access)
            uint64_t getInternalNbFrames();

            //------------------------------------------------------------------
            // clock divider management
            //------------------------------------------------------------------
            // Gets the clock divider
            Detector::ClockDivider getClockDivider();

            // Sets the clock divider
            void setClockDivider(Detector::ClockDivider in_clock_divider);

            //==================================================================
            // delay after trigger management
            //==================================================================
            // Gets the maximum delay after trigger of all modules (in seconds)
            double getMaxDelayAfterTriggerAllModules(void);

            // Gets the delay after trigger of one module (in seconds)
            double getDelayAfterTrigger(int in_module_index);

            // Sets the delay after trigger (in seconds)
            void setDelayAfterTrigger(const double & in_delay_after_trigger);

            //==================================================================
            // threshold energy management
            //==================================================================
            // Gets the threshold energy in eV
            int getThresholdEnergy();

            // Sets the threshold energy in eV
            void setThresholdEnergy(int in_threshold_energy_eV);

            //==================================================================
            // times management
            //==================================================================
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

            //==================================================================
            // Related to commands (put & get)
            //==================================================================
            // Executes a set command
            std::string setCmd(const std::string & in_command, int in_module_index = -1);

            // Executes a get command
            std::string getCmd(const std::string & in_command, int in_module_index = -1);

            //==================================================================
            // acquisition management
            //==================================================================
            // start receiver listening mode
            bool startReceiver();

            // stop receiver listening mode
            bool stopReceiver();

            // start detector real time acquisition in non blocking mode
            bool startAcquisition();

            // stop detector real time acquisition
            bool stopAcquisition();

            //==================================================================
            // status management
            //==================================================================
            // returns the current detector status
            Detector::Status getStatus();

        private:
            // creates an autolock mutex for sdk methods access
            lima::AutoMutex sdkLock() const;

            // cleans the shared memory used by the camera
            void cleanSharedMemory();

            // converts a version id to a string
            static std::string convertVersionToString(int64_t in_version);

            // Updates trigger data (mode, frame, cycle) using camera data 
            void updateTriggerData();

            // Updates exposure & latency times using camera data 
            void updateTimes();

            // Converts a standard string to args arguments
            void convertStringToArgs(const std::string & in_command,
                                     char  * *         & out_argv  ,
                                     int               & out_argc  );
            // Releases args arguments
            void releaseArgs(char * * & in_out_argv,
                             int      & in_out_argc);

            // inits the detector while setting the configuration file name
            void init(const std::string & in_config_file_name);

        private:
            // Class for detector functionalities to embed the detector controls in the users custom interface e.g. EPICS, Lima etc.
            slsDetectorUsers * m_detector_control;

            // Controller class for detector receivers functionalities
            Receivers * m_detector_receivers;

            // last known status
            Detector::Status m_status;

            //------------------------------------------------------------------
            // configuration
            //------------------------------------------------------------------
            // complete config file path
            std::string m_config_file_name;

            // readout time in seconds
            double m_readout_time_sec;

            // Number of frames in the receiver memory
            long m_receiver_fifo_depth;

            // Number of packets we should get in each receiver frame for 8 bits mode
            uint32_t m_frame_packet_number_8;

            // Number of packets we should get in each receiver frame for 16 bits mode
            uint32_t m_frame_packet_number_16;

            // Number of packets we should get in each receiver frame for 32 bits mode
            uint32_t m_frame_packet_number_32;

            //------------------------------------------------------------------
            // image management
            //------------------------------------------------------------------
            // current bit depth
            int m_bit_depth;

            // maximum width
            int m_max_width ;

            // maximum height
            int m_max_height;

            // current width
            int m_width ;

            // current height
            int m_height;

            //------------------------------------------------------------------
            // detector info management
            //------------------------------------------------------------------
            // detector type
            std::string m_detector_type;

            // detector model
            std::string m_detector_model;

            // detector firmware version
            std::string m_detector_firmware_version;

            // detector software version
            std::string m_detector_software_version;

            // detector this software version
            std::string m_detector_this_software_version;

            // number of modules
            int m_modules_nb;

            // number total of frames
            int64_t m_nb_frames;

            // trigger mode label from sls sdk
            std::string m_trigger_mode_label;

            // trigger mode 
            Detector::TriggerMode m_trigger_mode;

            // number of frames per cycle from sls sdk
            int64_t m_nb_frames_per_cycle;

            // number of cycle from sls sdk
            int64_t m_nb_cycles;

            // clock divider
            Detector::ClockDivider m_clock_divider;

            // delay after trigger in seconds for each modules
            std::vector<double> m_delays_after_trigger;

            // treshold energy
            int m_threshold_energy_eV; 

            // exposure time
            double m_exposure_time;

            // latency time
            double m_latency_time;

            //------------------------------------------------------------------
            // mutex stuff
            //------------------------------------------------------------------
            // used to protect the concurrent access to sdk methods
            // mutable keyword is used to allow const methods even if they use this class member
            mutable lima::Cond m_sdk_cond;
        };
    } // SlsEiger
} // lima
#endif // SLSEIGERDETECTOR_H

/*************************************************************************/