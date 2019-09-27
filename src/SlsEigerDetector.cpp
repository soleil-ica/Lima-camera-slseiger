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

/*************************************************************************************/
/*! 
 *  \file   SlsEigerDetector.h
 *  \brief  detector class implementation
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

// SYSTEM
#include <cstdlib>

// LIMA
#include "lima/Exceptions.h"

// LOCAL
#include "SlsEigerDetector.h"
#include "SlsEigerCamera.h"

// SLS SDK
#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima::SlsEiger;

/************************************************************************
 * \brief constructor
 * \param in_camera 
 * \param in_config_file_name complete path to the configuration file
 * \param in_readout_time_sec readout time in seconds
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_frame_packet_number_8 Number of packets we should get in each receiver frame for 8 bits mode
 * \param in_frame_packet_number_16 Number of packets we should get in each receiver frame for 16 bits mode
 * \param in_frame_packet_number_32 Number of packets we should get in each receiver frame for 32 bits mode
 ************************************************************************/
Detector::Detector(Camera            * in_camera                ,
                   const std::string & in_config_file_name      ,
                   const double        in_readout_time_sec      ,
                   const long          in_receiver_fifo_depth   ,
                   const int           in_bit_depth             ,
                   const long          in_frame_packet_number_8 ,
                   const long          in_frame_packet_number_16,
                   const long          in_frame_packet_number_32)
{
    m_camera        = in_camera;
    m_modules_nb    = 0;
    m_max_width     = 0;
    m_max_height    = 0;
    m_width         = 0; 
    m_height        = 0;
    m_exposure_time = 0.0;
    m_latency_time  = 0.0;

    m_parallel_mode           = lima::SlsEiger::ParallelMode::Parallel;
    m_overflow_mode           = false;
    m_sub_frame_exposure_time = 0.0;
    m_threshold_energy_eV     = 4500.0; // default value
    m_gain_mode               = lima::SlsEiger::GainMode::undefined;

    m_count_rate_correction_activation = false;
    m_count_rate_correction_ns         = 0    ;

    m_temperature_labels.resize(lima::SlsEiger::Temperature::hw_size);

    m_temperature_labels[hw_fpga]    = SLS_TEMP_FPGA;
    m_temperature_labels[hw_fpgaext] = SLS_TEMP_FPGAEXT;
    m_temperature_labels[hw_10ge]    = SLS_TEMP_10GE;
    m_temperature_labels[hw_dcdc]    = SLS_TEMP_DCDC;
    m_temperature_labels[hw_sodl]    = SLS_TEMP_SODL;
    m_temperature_labels[hw_sodr]    = SLS_TEMP_SODR;
    m_temperature_labels[hw_fpgafl]  = SLS_TEMP_FPGA2;
    m_temperature_labels[hw_fpgafr]  = SLS_TEMP_FPGA3;

    m_readout_time_sec       = in_readout_time_sec   ;
    m_receiver_fifo_depth    = in_receiver_fifo_depth;
    m_bit_depth              = in_bit_depth          ;
    m_frame_packet_number_8  = static_cast<uint32_t>(in_frame_packet_number_8 );
    m_frame_packet_number_16 = static_cast<uint32_t>(in_frame_packet_number_16);
    m_frame_packet_number_32 = static_cast<uint32_t>(in_frame_packet_number_32);

    // important for the calls of updateTimes, updateTriggerData in the init method.
    m_status = SlsEiger::Status::Idle; 

    m_detector_type                  = "undefined";
    m_detector_model                 = "undefined";
    m_detector_firmware_version      = "undefined";
    m_detector_software_version      = "undefined";
    m_detector_this_software_version = "undefined";

    init(in_config_file_name);
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Detector::~Detector()
{
    DEB_DESTRUCTOR();

    // releasing the detector control instance
    DEB_TRACE() << "Detector::~Detector - releasing the detector control instance";

    if(m_detector_control != NULL)
    {
        m_detector_control->setReceiverOnline(slsDetectorDefs::OFFLINE_FLAG);
        m_detector_control->setOnline(slsDetectorDefs::OFFLINE_FLAG);

        delete m_detector_control;
        m_detector_control = NULL;
    }

    // releasing the controller class for detector receivers functionalities
    DEB_TRACE() << "Detector::~Detector - releasing the controller class for detector receivers functionalities";
    
    if(m_detector_receivers != NULL)
    {
        delete m_detector_receivers;
        m_detector_receivers = NULL;
    }
}

/*******************************************************************
 * \brief gets the number of modules
 *******************************************************************/
int Detector::getModulesNb(void)
{
    return m_modules_nb;
}

/************************************************************************
 * \brief creates an autolock mutex for sdk methods access
 ************************************************************************/
lima::AutoMutex Detector::sdkLock() const
{
    return lima::AutoMutex(m_sdk_cond.mutex());
}

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void Detector::cleanSharedMemory()
{
    std::string cmd = "rm /dev/shm/slsDetectorPackage*;";
    std::system(cmd.c_str());
}

/************************************************************************
 * \brief inits the detector while setting the configuration file name
 * \param in_config_file_name complete path to the configuration file
 ************************************************************************/
void Detector::init(const std::string & in_config_file_name)
{
    DEB_MEMBER_FUNCT();

    const int detector_flow_control_10G     = 1; // 1 sets, 0 unsets
    const int detector_ten_gigabit_ethernet = 1; // 1 sets, 0 unsets
    const int detector_enable_gap_pixels    = 0; // enable 1 sets, 0 unsets

    int result;

    // before, cleaning the shared memory
    // cleanSharedMemory();

    // initing the class attributes
    m_config_file_name = in_config_file_name;

    // creating the controller class for detector receivers functionalities
    // CameraReceivers needs direct access to camera (*this)
    m_detector_receivers = new Receivers(*this);

    // creating the receivers (two for eiger)
    m_detector_receivers->init(m_config_file_name);

    // set the number of modules
    m_modules_nb = m_detector_receivers->getNumberOfReceivers();

    // creating the detector control instance
    int id = 0;

    m_detector_control = new slsDetectorUsers(result, id);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "slsDetectorUsers constructor failed! Could not initialize the camera!";
    }

    // configuration file is used to properly configure advanced settings in the shared memory
    result = m_detector_control->readConfigurationFile(m_config_file_name);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "readConfigurationFile failed! Could not initialize the camera!";
    }

    // Setting the detector online
    m_detector_control->setOnline(slsDetectorDefs::ONLINE_FLAG);

    // Connecting to the receiver
    m_detector_control->setReceiverOnline(slsDetectorDefs::ONLINE_FLAG);

    // setting the bit depth of the detector
    m_detector_control->setBitDepth(m_bit_depth);

    // getting the bit depth of the camera
    if(m_detector_control->setBitDepth(SLS_GET_VALUE) != m_bit_depth)
    {
        THROW_HW_FATAL(ErrorType::Error) << "setBitDepth failed! Could not initialize the detector!";
    }

    // network speed
    m_detector_control->setTenGigabitEthernet(detector_ten_gigabit_ethernet);

    if(m_detector_control->setTenGigabitEthernet(SLS_GET_VALUE) != detector_ten_gigabit_ethernet)
    {
        THROW_HW_FATAL(ErrorType::Error) << "setTenGigabitEthernet failed! Could not initialize the detector!";
    }

    m_detector_control->setFlowControl10G(detector_flow_control_10G);

    if(m_detector_control->setFlowControl10G(SLS_GET_VALUE) != detector_flow_control_10G)
    {
        THROW_HW_FATAL(ErrorType::Error) << "setFlowControl10G failed! Could not initialize the detector!";
    }

    m_detector_control->enableGapPixels(detector_enable_gap_pixels);

    if(m_detector_control->enableGapPixels(SLS_GET_VALUE) != detector_enable_gap_pixels)
    {
        THROW_HW_FATAL(ErrorType::Error) << "enableGapPixels failed! Could not initialize the detector!";
    }

    // getting the maximum detector size
    result = m_detector_control->getMaximumDetectorSize(m_max_width, m_max_height);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "getMaximumDetectorSize failed! Could not initialize the detector!";
    }

    // getting the detector size to be sure
    int x0;
    int y0;

    result = m_detector_control->getDetectorSize(x0, y0, m_width, m_height);

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_FATAL(ErrorType::Error) << "getDetectorSize failed! Could not initialize the detector!";
    }

    // computing the number of packets we should receive in each frame part (sls callback)
    if(m_bit_depth == 8)
    {
        m_frame_packet_number = m_frame_packet_number_8;
    }
    else
    if(m_bit_depth == 16)
    {
        m_frame_packet_number = m_frame_packet_number_16;
    }
    else
    if(m_bit_depth == 32)
    {
        m_frame_packet_number = m_frame_packet_number_32;
    }

    // disabling the file write by the camera
    m_detector_control->enableWriteToFile(slsDetectorDefs::DISABLED);

    // setting the receiver fifo depth (number of frames in the receiver memory)
    m_detector_control->setReceiverFifoDepth(m_receiver_fifo_depth);

    // initing the internal copy of parallel mode
    getParallelMode();

    // initing the internal copy of overflow mode
    getOverflowMode();

    // initing the internal copy of sub frame exposure time
    getSubFrameExposureTime();

    // setting of the default configuration and loading the settings (trimbits, dacs, iodelay etc) to the detector
    // Also, initing the the internal copies of gain mode and threshold energy
    setGainMode(lima::SlsEiger::GainMode::standard);

    // initing the internal copies of exposure & latency times
    updateTimes();

    // initing the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // allocating the cache for temperatures
    m_temperatures.resize(m_modules_nb);

    for(int module_index = 0 ; module_index < m_modules_nb ; module_index++)
    {
        m_temperatures[module_index].resize(lima::SlsEiger::Temperature::hw_size, 0);
    }

    // initing some const data
    // Module firmware version does not exist for Eiger
    m_detector_type                  = m_detector_control->getDetectorDeveloper();
    m_detector_model                 = m_detector_control->getDetectorType();
    m_detector_firmware_version      = convertVersionToString(m_detector_control->getDetectorFirmwareVersion());
    m_detector_software_version      = convertVersionToString(m_detector_control->getDetectorSoftwareVersion());
    m_detector_this_software_version = convertVersionToString(m_detector_control->getThisSoftwareVersion());

    // logging some versions informations
    DEB_TRACE() << "Detector Type : " << getDetectorType();
    DEB_TRACE() << "Detector Model : " << getDetectorModel();
    DEB_TRACE() << "Detector Firmware Version : " << getDetectorFirmwareVersion();
    DEB_TRACE() << "Detector Software Version : " << getDetectorSoftwareVersion();
    DEB_TRACE() << "Detector This Software Version : " << getDetectorThisSoftwareVersion();
}

//------------------------------------------------------------------
// image size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the maximum image width
 * \return maximum image width
 *******************************************************************/
int Detector::getMaxWidth() const
{
    return m_max_width;
}

/*******************************************************************
 * \brief Gets the maximum image height
 * \return maximum image height
 *******************************************************************/
int Detector::getMaxHeight() const
{
    return m_max_height;
}

/*******************************************************************
 * \brief Gets the image width
 * \return image width
 *******************************************************************/
int Detector::getWidth() const
{
    return m_width;
}

/*******************************************************************
 * \brief Gets the image height
 * \return image height
 *******************************************************************/
int Detector::getHeight() const
{
    return m_height;
}

/*******************************************************************
 * \brief Gets the bit depth
 * \return bit depth (8, 16, 32)
 *******************************************************************/
int Detector::getBitDepth()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 
        
        m_bit_depth = m_detector_control->setBitDepth(SLS_GET_VALUE);
    }

    return m_bit_depth;
}

/*******************************************************************
 * \brief Sets the bit depth (8, 16, 32)
 * \param in_bit_depth needed bit depth
*******************************************************************/
void Detector::setBitDepth(const int & in_bit_depth)
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    m_detector_control->setBitDepth(in_bit_depth);
}

/*******************************************************************
 * \brief Gets the number of packets we should receive during the acquisition
 * \return number of packets which depends of the bit depth (8, 16, 32)
 *******************************************************************/
uint32_t Detector::getFramePacketNumber()
{
    return m_frame_packet_number;
}

/*******************************************************************
 * \brief Gets the readout time 
 * \return readout time in seconds
 *******************************************************************/
double Detector::getReadoutTimeSec()
{
    return m_readout_time_sec;
}

//------------------------------------------------------------------
// detector info management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the detector type
 * \return detector type
 *******************************************************************/
std::string Detector::getDetectorType() const
{
    return m_detector_type;
}

/*******************************************************************
 * \brief gets the detector model
 * \return detector model
 *******************************************************************/
std::string Detector::getDetectorModel() const
{
    return m_detector_model;
}

/*******************************************************************
 * \brief gets Detector Firmware Version
 * \return Detector Firmware Version
 *******************************************************************/
std::string Detector::getDetectorFirmwareVersion() const
{
    return m_detector_firmware_version;
}

/*******************************************************************
 * \brief gets Detector Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Detector::getDetectorSoftwareVersion() const
{
    return m_detector_software_version;
}

/*******************************************************************
 * \brief gets Detector This Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Detector::getDetectorThisSoftwareVersion() const
{
    return m_detector_this_software_version;
}

/*******************************************************************
 * \brief converts a version id to a string
 * \return version in string format (uppercase & hexa)
 *******************************************************************/
std::string Detector::convertVersionToString(int64_t in_version)
{
    std::stringstream tempStream;
    tempStream << "0x" << std::uppercase << std::hex << in_version;
    return tempStream.str();
}

//------------------------------------------------------------------
// trigger mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Updates trigger data (mode, frame, cycle) using camera data 
 *******************************************************************/
void Detector::updateTriggerData()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            // getting the current trigger mode index
            int trigger_mode_index = m_detector_control->setTimingMode(SLS_GET_VALUE);

            // converting trigger mode index to trigger mode label
            m_trigger_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);
            
            // reading the number of frames per cycle
            m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(SLS_GET_VALUE);

            // reading the number of cycles
            m_nb_cycles = m_detector_control->setNumberOfCycles(SLS_GET_VALUE);
        }

        // computing the number of frames
        m_nb_frames = m_nb_cycles * m_nb_frames_per_cycle;

        // computing the trigger mode
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_AUTO)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_BURST)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_TRIGGER)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_GATING)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE;
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "updateTriggerData : This camera trigger mode is not managed: (" << m_trigger_mode_label << ")";
        }
    }
}

/*******************************************************************
 * \brief Sets the trigger mode
 * \param in_trigger_mode needed trigger mode 
 *******************************************************************/
void Detector::setTriggerMode(const lima::SlsEiger::TriggerMode & in_trigger_mode)
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // converting the detector trigger mode to the sdk trigger mode
    std::string trig_mode;

    switch (in_trigger_mode)
    {       
        case lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE:
            trig_mode = SLS_TRIGGER_MODE_AUTO;
            break;

        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
            trig_mode = SLS_TRIGGER_MODE_BURST;
            break;

        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
            trig_mode = SLS_TRIGGER_MODE_TRIGGER;
            break;

        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE:
            trig_mode = SLS_TRIGGER_MODE_GATING;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "setTriggerMode : Cannot change the Trigger mode of the camera, this mode is not managed: (" << in_trigger_mode << ")";
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // initing the number of frames per cycle and  number of cycles 
        // to avoid problems during the trigger mode change.
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(1);
        m_nb_cycles           = m_detector_control->setNumberOfCycles(1);

        // conversion of trigger mode label to trigger mode index
        int trigger_mode_index = slsDetectorUsers::getTimingMode(trig_mode);

        // apply the trigger change
        m_detector_control->setTimingMode(trigger_mode_index);

        // converting trigger mode index to trigger mode label
        m_trigger_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);

        // change the frames per cycle and the cycles values
        int64_t nb_frames_per_cycle;
        int64_t nb_cycles          ;

        switch (in_trigger_mode)
        {       
            case lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE:
            case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
                nb_frames_per_cycle = m_nb_frames;
                nb_cycles           = 1LL;
                break;

            case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
            case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE:
                nb_frames_per_cycle = 1LL;
                nb_cycles           = m_nb_frames;
                break;

            default:
                break;
        }

        // setting the number of cycles
        m_nb_cycles = m_detector_control->setNumberOfCycles(nb_cycles);

        // setting the number of frames per cycle
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(nb_frames_per_cycle);

        // computing the trigger mode
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_AUTO)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_BURST)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_TRIGGER)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_GATING)
        {
            m_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE;
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "setTriggerMode : This camera trigger Mode is not managed! (" << m_trigger_mode_label << ")";
        }
    }
}

/*******************************************************************
 * \brief Gets the trigger mode
 * \return trigger mode
 *******************************************************************/
lima::SlsEiger::TriggerMode Detector::getTriggerMode(void)
{
    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    return m_trigger_mode;
}

//------------------------------------------------------------------
// number of frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the number of frames
 * \return number of frames
 *******************************************************************/
int64_t Detector::getNbFrames()
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    return m_nb_frames;
}

/*******************************************************************
 * \brief Gets the internal number of frames (for thread access)
 * \return number of frames
 *******************************************************************/
uint64_t Detector::getInternalNbFrames()
{
    return static_cast<uint64_t>(m_nb_frames);
}

/*******************************************************************
 * \brief Sets the number of frames
 * \param in_nb_frames number of needed frames
*******************************************************************/
void Detector::setNbFrames(int64_t in_nb_frames)
{
    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    int64_t nb_frames_per_cycle;
    int64_t nb_cycles          ;

    switch (m_trigger_mode)
    {       
        case lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE:
        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
            nb_frames_per_cycle = in_nb_frames;
            nb_cycles           = 1LL;
            break;

        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
        case lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE:
            nb_frames_per_cycle = 1LL;
            nb_cycles           = in_nb_frames;
            break;

        default:
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock();  

        // setting the number of cycles
        m_nb_cycles = m_detector_control->setNumberOfCycles(nb_cycles);

        // setting the number of frames per cycle
        m_nb_frames_per_cycle = m_detector_control->setNumberOfFrames(nb_frames_per_cycle);
    }

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();
}

//------------------------------------------------------------------
// clock divider management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the clock divider
 * \return clock divider
 *******************************************************************/
lima::SlsEiger::ClockDivider Detector::getClockDivider()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        int clock_divider;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 
            
            clock_divider = m_detector_control->setClockDivider(SLS_GET_VALUE);
        }

        m_clock_divider = static_cast<lima::SlsEiger::ClockDivider>(clock_divider);
    }

    return m_clock_divider;
}

/*******************************************************************
 * \brief Sets the clock divider
 * \param in_clock_divider needed clock divider
*******************************************************************/
void Detector::setClockDivider(lima::SlsEiger::ClockDivider in_clock_divider)
{
    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_detector_control->setClockDivider(static_cast<int>(in_clock_divider));
    }

    // updating the internal data
    getClockDivider();
}

//------------------------------------------------------------------
// parallel mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the parallel mode 
 * \return the parallel mode
 *******************************************************************/
lima::SlsEiger::ParallelMode Detector::getParallelMode()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        int parallel_mode;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 
            
            parallel_mode = m_detector_control->setParallelMode(SLS_GET_VALUE);
        }

        m_parallel_mode = static_cast<lima::SlsEiger::ParallelMode>(parallel_mode);
    }

    return m_parallel_mode;
}

/*******************************************************************
 * \brief Sets the parallel mode
 * \param in_parallelMode needed parallel mode
*******************************************************************/
void Detector::setParallelMode(lima::SlsEiger::ParallelMode in_parallel_mode)
{
    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_detector_control->setParallelMode(static_cast<int>(in_parallel_mode));
    }

    // updating the internal data
    getParallelMode();
}

//------------------------------------------------------------------
// overflow mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the overflow mode
 * \return the overflow mode
 *******************************************************************/
bool Detector::getOverflowMode()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        int overflow_mode;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 
            
            overflow_mode = m_detector_control->setOverflowMode(SLS_GET_VALUE);
        }

        m_overflow_mode = (overflow_mode == 1);
    }

    return m_overflow_mode;
}

/*******************************************************************
 * \brief Sets the overflow mode
 * \param in_overflow_mode needed overflow mode
*******************************************************************/
void Detector::setOverflowMode(bool in_overflow_mode)
{
    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_detector_control->setOverflowMode((in_overflow_mode) ? 1 : 0);
    }

    // updating the internal data
    getOverflowMode();
}

//------------------------------------------------------------------
// sub frame exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the sub frame exposure time
 * \return sub frame exposure time
 *******************************************************************/
double Detector::getSubFrameExposureTime()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        double sub_frame_exposure_time;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            sub_frame_exposure_time = m_detector_control->setSubFrameExposureTime(SLS_GET_VALUE, true); // in seconds
        }

        m_sub_frame_exposure_time = sub_frame_exposure_time;
    }

    return m_sub_frame_exposure_time;
}

/*******************************************************************
 * \brief Sets the sub frame exposure time
 * \param in_sub_frame_exposure_time needed sub frame exposure time
*******************************************************************/
void Detector::setSubFrameExposureTime(double in_sub_frame_exposure_time)
{
    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_detector_control->setSubFrameExposureTime(in_sub_frame_exposure_time, true); // in seconds
    }

    // updating the internal data
    getSubFrameExposureTime();
}

//------------------------------------------------------------------
// threshold energy management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the threshold energy in eV
 * \return threshold energy in eV
 *******************************************************************/
int Detector::getThresholdEnergy()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        int threshold_energy_eV;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            threshold_energy_eV = m_detector_control->getThresholdEnergy();
        }

        if(threshold_energy_eV == slsDetectorDefs::FAIL)
        {
            THROW_HW_ERROR(ErrorType::Error) << "getThresholdEnergy failed!";
        }

        m_threshold_energy_eV = threshold_energy_eV;
    }

    return m_threshold_energy_eV;
}

/*******************************************************************
 * \brief Sets the threshold energy in eV
 * \param in_threshold_energy_eV needed threshold energy in eV
*******************************************************************/
void Detector::setThresholdEnergy(int in_threshold_energy_eV)
{
    DEB_MEMBER_FUNCT();

    int result;

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // setting the current gain mode index and loading the settings (trimbits, dacs, iodelay etc) to the detector
        int result = m_detector_control->setThresholdEnergy(in_threshold_energy_eV, 1, -1); // 1 -> loads the trimbits, -1 -> uses current setting
    }

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_ERROR(ErrorType::Error) << "setThresholdEnergy failed!";
    }

    // updating the internal data
    getThresholdEnergy();
}

//------------------------------------------------------------------
// times management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Updates exposure & latency times using camera data 
 *******************************************************************/
void Detector::updateTimes()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        double exposure_period;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            m_exposure_time = m_detector_control->setExposureTime  (SLS_GET_VALUE, true); // in seconds
            exposure_period = m_detector_control->setExposurePeriod(SLS_GET_VALUE, true); // in seconds
        }

        // compute latency time
        // exposure period = exposure time + latency time
        m_latency_time = exposure_period - m_exposure_time;
    }
}

//------------------------------------------------------------------
// exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the exposure time
 * \return exposure time
 *******************************************************************/
double Detector::getExpTime()
{
    // updating the internal copies of exposure & latency times
    updateTimes();
    return m_exposure_time;
}

/*******************************************************************
 * \brief Sets the exposure time
 * \param in_exp_time needed exposure time
*******************************************************************/
void Detector::setExpTime(double in_exp_time)
{
    double exposure_time  ;
    double exposure_period;

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // if we change the exposure time, we need to update also the exposure period
        // exposure period = exposure time + latency time
        exposure_time   = m_detector_control->setExposureTime  (in_exp_time, true); // in seconds
        exposure_period = m_detector_control->setExposurePeriod(exposure_time + m_latency_time, true); // in seconds
    }

    // updating the internal copies of exposure & latency times
    updateTimes();
}

//------------------------------------------------------------------
// latency time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the latency time
 * \return latency time
 *******************************************************************/
double Detector::getLatencyTime()
{
    // updating the internal copies of exposure & latency times
    updateTimes();
    return m_latency_time;
}

/*******************************************************************
 * \brief Sets the latency time
 * \param in_latency_time needed latency time
*******************************************************************/
void Detector::setLatencyTime(double in_latency_time)
{
    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // exposure period = exposure time + latency time
        m_detector_control->setExposurePeriod(m_exposure_time + in_latency_time, true); // in seconds
    }

    // updating the internal copies of exposure & latency times
    updateTimes();
}

//------------------------------------------------------------------
// gain mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the gain mode
 * \return gain mode
 *******************************************************************/
lima::SlsEiger::GainMode Detector::getGainMode(void)
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 

            // getting the current gain mode index
            int gain_mode_index = m_detector_control->setSettings(SLS_GET_VALUE);

            // converting gain mode index to gain mode label
            m_gain_mode_label = slsDetectorUsers::getDetectorSettings(gain_mode_index);
        }

        // computing the trigger mode
        if(m_gain_mode_label == SLS_GAIN_MODE_UNDEFINED)
        {
            m_gain_mode = lima::SlsEiger::GainMode::undefined;
        }
        else
        if(m_gain_mode_label == SLS_GAIN_MODE_STANDARD)
        {
            m_gain_mode = lima::SlsEiger::GainMode::standard;
        }
        else
        if(m_gain_mode_label == SLS_GAIN_MODE_LOW)
        {
            m_gain_mode = lima::SlsEiger::GainMode::low;
        }
        else
        if(m_gain_mode_label == SLS_GAIN_MODE_MEDIUM)
        {
            m_gain_mode = lima::SlsEiger::GainMode::medium;
        }
        else
        if(m_gain_mode_label == SLS_GAIN_MODE_HIGH)
        {
            m_gain_mode = lima::SlsEiger::GainMode::high;
        }
        else
        if(m_gain_mode_label == SLS_GAIN_MODE_VERY_HIGH)
        {
            m_gain_mode = lima::SlsEiger::GainMode::very_high;
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "getGainMode : This camera gain mode is not managed: (" << m_gain_mode_label << ")";
        }
    }

    return m_gain_mode;
}

/*******************************************************************
 * \brief Sets the gain mode
 * \param in_gain_mode needed gain mode 
 *******************************************************************/
void Detector::setGainMode(lima::SlsEiger::GainMode in_gain_mode)
{
    DEB_MEMBER_FUNCT();

    // converting the detector gain mode to the sdk gain mode
    std::string gain_mode;

    switch (in_gain_mode)
    {       
        case lima::SlsEiger::GainMode::standard:
            gain_mode = SLS_GAIN_MODE_STANDARD;
            break;

        case lima::SlsEiger::GainMode::low:
            gain_mode = SLS_GAIN_MODE_LOW;
            break;

        case lima::SlsEiger::GainMode::medium:
            gain_mode = SLS_GAIN_MODE_MEDIUM;
            break;

        case lima::SlsEiger::GainMode::high:
            gain_mode = SLS_GAIN_MODE_HIGH;
            break;

        case lima::SlsEiger::GainMode::very_high:
            gain_mode = SLS_GAIN_MODE_VERY_HIGH;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "setGainMode : Cannot change the Trigger mode of the camera, this mode is not managed: (" << in_gain_mode << ")";
            break;
    }

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        // conversion of gain mode label to gain mode index
        int gain_mode_index = slsDetectorUsers::getDetectorSettings(gain_mode);

        if(gain_mode_index == -1)
        {
            THROW_HW_ERROR(ErrorType::Error) << "setGainMode failed!";
        }

        // setting the current gain mode index and loading the settings (trimbits, dacs, iodelay etc) to the detector
        DEB_TRACE() << "setThresholdEnergy  " << m_threshold_energy_eV << "," << gain_mode_index;

        int threshold_energy_eV = m_detector_control->setThresholdEnergy(m_threshold_energy_eV, 1, gain_mode_index); // 1 -> loading the trimbits

        if(threshold_energy_eV == slsDetectorDefs::FAIL)
        {
            THROW_HW_ERROR(ErrorType::Error) << "setGainMode failed!";
        }
    }

    // updating the internal data
    getGainMode();
    getThresholdEnergy();
}

//------------------------------------------------------------------
// count rate correction management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the activation of count rate correction
 * \return count rate correction activation
 *******************************************************************/
bool Detector::getCountRateCorrectionActivation()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_count_rate_correction_activation = (m_detector_control->enableCountRateCorrection(SLS_GET_VALUE) != 0);
    }

    return m_count_rate_correction_activation;
}

/*******************************************************************
 * \brief Sets the activation of count rate correction
 * \param in_count_rate_correction_activated  
*******************************************************************/
void Detector::setCountRateCorrectionActivation(bool in_count_rate_correction_activation)
{
    DEB_MEMBER_FUNCT();

    // protecting the sdk concurrent access
    {
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_detector_control->enableCountRateCorrection((in_count_rate_correction_activation) ? 1 : 0);
    }

    // updating the internal data
    getCountRateCorrectionActivation();
}

/*******************************************************************
 * \brief Gets the count rate correction in ns
 * \return count rate correction in eV (0 disabled else default value)
 *******************************************************************/
int Detector::getCountRateCorrection()
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_count_rate_correction_ns = m_detector_control->enableCountRateCorrection(SLS_GET_VALUE);
    }

    return m_count_rate_correction_ns;
}

//------------------------------------------------------------------
// temperature management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the temperature in millidegree Celsius of hardware element 
          for a specific module
 * \param  in_temperature_type type of hardware to be monitored
 * \param in_module_index module index (starts at 0)
 * \return temperature in millidegree Celsius
 *******************************************************************/
int Detector::getTemperature(lima::SlsEiger::Temperature in_temperature_type, int in_module_index)
{
    DEB_MEMBER_FUNCT();

    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == SlsEiger::Status::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_temperatures[in_module_index][in_temperature_type] = 
            m_detector_control->getADC(m_temperature_labels[in_temperature_type], in_module_index);
    }

    return m_temperatures[in_module_index][in_temperature_type];
}

//==================================================================
// Related to commands (put & get)
//==================================================================
/*******************************************************************
 * \brief Converts a standard string to args arguments
 * \param in_command command in command line format
 * \param out_argv output c-strings c-array
 * \param out_argc output number of arguments of out_argv
 *******************************************************************/
void Detector::convertStringToArgs(const std::string & in_command,
                                   char  * *         & out_argv  ,
                                   int               & out_argc  )
{
	DEB_MEMBER_FUNCT();

    out_argv = NULL;
    out_argc = 0   ;

    // filling a string vector with the command line elements
    std::vector<std::string> elements;
    std::stringstream ss(in_command);

	while (ss) 
    {
        std::string element;
		ss >> element;

        if(element.size() > 0)
        {
            elements.push_back(element);
        }
	}

    // setting argc value
    out_argc = elements.size();

    // allocating argv array
	out_argv = new char * [out_argc];
    
    // filling argv array
	for (int element_index = 0; element_index < out_argc; element_index++)
    {
        out_argv[element_index] = new char[elements[element_index].size() + 1]; // adding the allocation of end of c-string 
        strcpy(out_argv[element_index], elements[element_index].c_str()); // copying the string including the eos
    }
}

/*******************************************************************
 * \brief Releases args arguments
 * \param in_out_argv output c-strings c-array
 * \param in_out_argc output number of arguments of out_argv
 *******************************************************************/
void Detector::releaseArgs(char * * & in_out_argv,
                           int      & in_out_argc)
{
	DEB_MEMBER_FUNCT();

    if(in_out_argv != NULL)
    {
        // releasing the c_strings array content
        for (int element_index = 0; element_index < in_out_argc; element_index++)
        {
            delete [] in_out_argv[element_index];
        }

        // releasing the c_strings array
        delete [] in_out_argv;

        in_out_argv = NULL;
        in_out_argc = 0   ;
    }
}

/*******************************************************************
 * \brief Executes a set command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Detector::setCmd(const std::string & in_command, int in_module_index)
{
    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        result = m_detector_control->putCommand(argc, argv, in_module_index);
    }

    releaseArgs(argv, argc);
    return result;
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Detector::getCmd(const std::string & in_command, int in_module_index)
{
    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        result = m_detector_control->getCommand(argc, argv, in_module_index);
    }

    releaseArgs(argv, argc);
    return result;
}

//------------------------------------------------------------------
// acquisition management
//------------------------------------------------------------------
/************************************************************************
 * \brief start receiver listening mode
 * \return true if ok, else false
 ************************************************************************/
bool Detector::startReceiver()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    // reset the number of caught frames in the sdk
    m_detector_control->resetFramesCaughtInReceiver();

    return (m_detector_control->startReceiver() != slsDetectorDefs::FAIL);
}

/************************************************************************
 * \brief stop receiver listening mode
 * \return true if ok, else false
 ************************************************************************/
bool Detector::stopReceiver()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return (m_detector_control->stopReceiver() != slsDetectorDefs::FAIL);
}

/************************************************************************
 * \brief start detector real time acquisition in non blocking mode
 * \return true if ok, else false
 ************************************************************************/
bool Detector::startAcquisition()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return (m_detector_control->startAcquisition() != slsDetectorDefs::FAIL);
}

/************************************************************************
 * \brief stop detector real time acquisition
 * \return true if ok, else false
 ************************************************************************/
bool Detector::stopAcquisition()
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    return (m_detector_control->stopAcquisition() != slsDetectorDefs::FAIL);
}

/************************************************************************
 * \brief Acquisition data management
 * \param m_receiver_index receiver index
 * \param in_frame_index frame index (starts at 1)
 * \param in_pos_x horizontal position of the part in the final image (starts at 0)
 * \param in_pos_y vertical position of the part in the final image (starts at 0)
 * \param in_packet_number number of packets caught for this frame
 * \param in_timestamp time stamp in 10MHz clock
 * \param in_data_pointer frame image pointer
 * \param in_data_size frame image size 
 ************************************************************************/
void Detector::acquisitionDataReady(const int      in_receiver_index,
                                    uint64_t       in_frame_index   ,
                                    const int      in_pos_x         ,
                                    const int      in_pos_y         ,
                                    const uint32_t in_packet_number ,
                                    const uint64_t in_timestamp     ,
                                    const char *   in_data_pointer  ,
                                    const uint32_t in_data_size     )
{
    // the detector will manage the new frame
    m_camera->acquisitionDataReady(in_receiver_index, 
                                   in_frame_index   ,
                                   in_pos_x         ,
                                   in_pos_y         ,
                                   in_packet_number ,
                                   in_timestamp     ,
                                   in_data_pointer  ,
                                   in_data_size     );
}

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current detector status
 * \return current hardware status
 ************************************************************************/
lima::SlsEiger::Status Detector::getStatus()
{
    DEB_MEMBER_FUNCT();

    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    SlsEiger::Status result;

    // In auto mode:
    // Detector starts with idle to running and at end of acquisition, returns to idle.
    //
    // In trigger mode:
    // Detector starts with idle to waiting and stays in waiting until it gets a trigger.
    // Upon trigger, it switches to running and goes back to waiting again until it gets a trigger.
    // Upon end of acquisition (after it get n triggers and m frames as configured), it will go to idle.
    //
    // If one explicitly calls stop acquisition:
    // Detectors goes from running to idle.
    //
    // If there is fifo overflow:
    // the detector will go to stopped state and stay there until the fifos are cleared.
    // It should be concidered as an error.    

    // getting the detector status
    int state = m_detector_control->getDetectorStatus();

    // ready to start acquisition or acquisition stopped externally
    if((state == slsDetectorDefs::runStatus::IDLE   ) ||
       (state == slsDetectorDefs::runStatus::STOPPED) ||
       (state == slsDetectorDefs::runStatus::RUN_FINISHED))
    {
        result = SlsEiger::Status::Idle;
    }
    else
    // waiting for trigger or gate signal
    if(state == slsDetectorDefs::runStatus::WAITING)
    {
        result = SlsEiger::Status::Waiting;
    }
    else
    // waiting for trigger or gate signal or acquisition is running 
    // TRANSMITTING : acquisition running and data in memory 
    if((state == slsDetectorDefs::runStatus::RUNNING) ||
       (state == slsDetectorDefs::runStatus::TRANSMITTING))
    {
        result = SlsEiger::Status::Running;
    }
    else
    // fifo full or unexpected error 
    if(state == slsDetectorDefs::runStatus::ERROR)
    {
        result = SlsEiger::Status::Error;
    }
    else
    // impossible state 
    {
        THROW_HW_ERROR(ErrorType::Error) << "Detector::getDetectorStatus failed! An unknown state was returned!";
    }

    // making a copy of the latest read status
    m_status = result;

    return result;
}

//========================================================================================
