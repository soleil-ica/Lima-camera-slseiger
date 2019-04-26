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

// SLS SDK
#include <slsDetectorUsers.h>
#include <sls_detector_defs.h>

using namespace lima::SlsEiger;

/************************************************************************
 * \brief constructor
 * \param in_config_file_name complete path to the configuration file
 * \param in_readout_time_sec readout time in seconds
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_frame_packet_number_8 Number of packets we should get in each receiver frame for 8 bits mode
 * \param in_frame_packet_number_16 Number of packets we should get in each receiver frame for 16 bits mode
 * \param in_frame_packet_number_32 Number of packets we should get in each receiver frame for 32 bits mode
 ************************************************************************/
Detector::Detector(const std::string & in_config_file_name      ,
                   const double        in_readout_time_sec      ,
                   const long          in_receiver_fifo_depth   ,
                   const int           in_bit_depth             ,
                   const long          in_frame_packet_number_8 ,
                   const long          in_frame_packet_number_16,
                   const long          in_frame_packet_number_32)
{
    m_modules_nb    = 0;
    m_max_width     = 0;
    m_max_height    = 0;
    m_width         = 0; 
    m_height        = 0;
    m_exposure_time = 0.0;
    m_latency_time  = 0.0;
    m_threshold_energy_eV = 0.0;

    m_readout_time_sec       = in_readout_time_sec   ;
    m_receiver_fifo_depth    = in_receiver_fifo_depth;
    m_bit_depth              = in_bit_depth          ;
    m_frame_packet_number_8  = static_cast<uint32_t>(in_frame_packet_number_8 );
    m_frame_packet_number_16 = static_cast<uint32_t>(in_frame_packet_number_16);
    m_frame_packet_number_32 = static_cast<uint32_t>(in_frame_packet_number_32);

    // important for the calls of updateTimes, updateTriggerData in the init method.
    m_status = Detector::Status::Idle; 

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

    // creating the receivers (just one for Jungfrau)
    m_detector_receivers->init(m_config_file_name);

    // set the number of modules
    m_modules_nb = m_detector_receivers->getNumberOfReceivers();

    // initing the delay after trigger container
    for(int module_index = 0 ; module_index < m_modules_nb ; module_index++)
    {
    	m_delays_after_trigger.push_back(0.0);
    }

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

    // disabling the file write by the camera
    m_detector_control->enableWriteToFile(slsDetectorDefs::DISABLED);

    // setting the receiver fifo depth (number of frames in the receiver memory)
    m_detector_control->setReceiverFifoDepth(m_receiver_fifo_depth);

    // initing the internal copies of exposure & latency times
    updateTimes();

    // initing the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // initing some const data
    // Module firmware version does not exist for Jungfrau
    m_detector_type             = m_detector_control->getDetectorDeveloper();
    m_detector_model            = m_detector_control->getDetectorType();
    m_detector_firmware_version = convertVersionToString(m_detector_control->getDetectorFirmwareVersion());
    m_detector_software_version = convertVersionToString(m_detector_control->getDetectorSoftwareVersion());
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
    if(m_status == Detector::Status::Idle)
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
    if(m_status == Detector::Status::Idle)
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
            m_trigger_mode = Detector::TriggerMode::TRIGGER_INTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_TRIGGER)
        {
            if(m_nb_cycles == 1LL)
            {
                m_trigger_mode = Detector::TriggerMode::TRIGGER_EXTERNAL_SINGLE;
            }
            else
            {
                m_trigger_mode = Detector::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE;
            }
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
void Detector::setTriggerMode(const Detector::TriggerMode & in_trigger_mode)
{
    DEB_MEMBER_FUNCT();

    // updating the internal copies of trigger mode label, number of cyles, number of frames per cycle, number of frames
    updateTriggerData();

    // converting the detector trigger mode to the sdk trigger mode
    std::string trig_mode;

    switch (in_trigger_mode)
    {       
        case Detector::TriggerMode::TRIGGER_INTERNAL_SINGLE:
            trig_mode = SLS_TRIGGER_MODE_AUTO;
            break;

        case Detector::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
        case Detector::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
            trig_mode = SLS_TRIGGER_MODE_TRIGGER; // same trigger mode, the difference will be in frames and cycles numbers
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "updateTriggerData : Cannot change the Trigger mode of the camera, this mode is not managed: (" << in_trigger_mode << ")";
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
            case Detector::TriggerMode::TRIGGER_INTERNAL_SINGLE:
                nb_frames_per_cycle = m_nb_frames;
                nb_cycles           = 1LL;
                break;

            case Detector::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
                nb_frames_per_cycle = m_nb_frames;
                nb_cycles           = 1LL;
                break;

            case Detector::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
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
            m_trigger_mode = Detector::TriggerMode::TRIGGER_INTERNAL_SINGLE;
        }
        else
        if(m_trigger_mode_label == SLS_TRIGGER_MODE_TRIGGER)
        {
            if(m_nb_cycles == 1LL)
            {
                m_trigger_mode = Detector::TriggerMode::TRIGGER_EXTERNAL_SINGLE;
            }
            else
            {
                m_trigger_mode = Detector::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE;
            }
        }
        else
        {
            THROW_HW_ERROR(ErrorType::Error) << "updateTriggerData : This camera trigger Mode is not managed! (" << m_trigger_mode_label << ")";
        }
    }
}

/*******************************************************************
 * \brief Gets the trigger mode
 * \return trigger mode
 *******************************************************************/
Detector::TriggerMode Detector::getTriggerMode(void)
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
        case Detector::TriggerMode::TRIGGER_INTERNAL_SINGLE:
            nb_frames_per_cycle = in_nb_frames;
            nb_cycles           = 1LL;
            break;

        case Detector::TriggerMode::TRIGGER_EXTERNAL_SINGLE:
            nb_frames_per_cycle = in_nb_frames;
            nb_cycles           = 1LL;
            break;

        case Detector::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE:
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
Detector::ClockDivider Detector::getClockDivider()
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Detector::Status::Idle)
    {
        int clock_divider;

        // protecting the sdk concurrent access
        {
            lima::AutoMutex sdk_mutex = sdkLock(); 
            
            clock_divider = m_detector_control->setClockDivider(SLS_GET_VALUE);
        }

        m_clock_divider = static_cast<enum ClockDivider>(clock_divider);
    }

    return m_clock_divider;
}

/*******************************************************************
 * \brief Sets the clock divider
 * \param in_clock_divider needed clock divider
*******************************************************************/
void Detector::setClockDivider(Detector::ClockDivider in_clock_divider)
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    m_detector_control->setClockDivider(static_cast<int>(in_clock_divider));
}

//------------------------------------------------------------------
// delay after trigger management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the maximum delay after trigger of all modules (in seconds)
 * \return max delay after trigger (in seconds)
 *******************************************************************/
double Detector::getMaxDelayAfterTriggerAllModules(void)
{
    double max_delay = 0.0;

    for(int module_index = 0 ; module_index < m_modules_nb ; module_index++)
    {
    	if(m_delays_after_trigger[module_index] > max_delay)
        {
            max_delay = m_delays_after_trigger[module_index];
        }
    }

    return max_delay;
}

/*******************************************************************
 * \brief Gets the delay after trigger of one module (in seconds)
 * \param in_module_index module index (starts at 0)
 * \return delay after trigger (in seconds)
 *******************************************************************/
double Detector::getDelayAfterTrigger(int in_module_index)
{
    // during acquisition, camera data access is not allowed because it
    // could put the camera into an error state. 
    // So we give the latest read value.
    if(m_status == Detector::Status::Idle)
    {
        // protecting the sdk concurrent access
        lima::AutoMutex sdk_mutex = sdkLock(); 

        m_delays_after_trigger[in_module_index] = m_detector_control->setDelayAfterTrigger(SLS_GET_VALUE, true, in_module_index); // in seconds
    }

    return m_delays_after_trigger[in_module_index];
}

/*******************************************************************
 * \brief Sets the delay after trigger (in seconds)
 * \param in_delay_after_trigger_sec needed delay after trigger (in seconds)
*******************************************************************/
void Detector::setDelayAfterTrigger(const double & in_delay_after_trigger_sec)
{
    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    m_detector_control->setDelayAfterTrigger(in_delay_after_trigger_sec, true); // in seconds for all modules
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
    if(m_status == Detector::Status::Idle)
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

        result = m_detector_control->setThresholdEnergy(in_threshold_energy_eV);
    }

    if(result == slsDetectorDefs::FAIL)
    {
        THROW_HW_ERROR(ErrorType::Error) << "setThresholdEnergy failed!";
    }
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
    if(m_status == Detector::Status::Idle)
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

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current detector status
 * \return current hardware status
 ************************************************************************/
Detector::Status Detector::getStatus()
{
    DEB_MEMBER_FUNCT();

    // protecting the sdk concurrent access
    lima::AutoMutex sdk_mutex = sdkLock(); 

    Detector::Status result;

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
        result = Detector::Status::Idle;
    }
    else
    // waiting for trigger or gate signal
    if(state == slsDetectorDefs::runStatus::WAITING)
    {
        result = Detector::Status::Waiting;
    }
    else
    // waiting for trigger or gate signal or acquisition is running 
    // TRANSMITTING : acquisition running and data in memory 
    if((state == slsDetectorDefs::runStatus::RUNNING) ||
       (state == slsDetectorDefs::runStatus::TRANSMITTING))
    {
        result = Detector::Status::Running;
    }
    else
    // fifo full or unexpected error 
    if(state == slsDetectorDefs::runStatus::ERROR)
    {
        result = Detector::Status::Error;
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
