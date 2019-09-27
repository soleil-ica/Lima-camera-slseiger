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
 *  \file   SlsEigerCamera.cpp
 *  \brief  SlsEiger detector hardware class implementation
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#include <cstdlib>

#include "lima/Exceptions.h"

// LOCAL
#include "SlsEigerCamera.h"
#include "SlsEigerDetector.h"

#include "SlsEigerCameraFrames.h"
#include "SlsEigerCameraThread.h"

using namespace lima;
using namespace lima::SlsEiger;

#include <cmath>

/************************************************************************
 * \brief constructor
 * \param in_config_file_name complete path to the configuration file
 * \param in_readout_time_sec readout time in seconds
 * \param in_receiver_fifo_depth Number of frames in the receiver memory
 * \param in_bit_depth bit depth (8, 16, 32)
 * \param in_frame_packet_number_8 Number of packets we should get in each receiver frame for 8 bits mode
 * \param in_frame_packet_number_16 Number of packets we should get in each receiver frame for 16 bits mode
 * \param in_frame_packet_number_32 Number of packets we should get in each receiver frame for 32 bits mode
 ************************************************************************/
Camera::Camera(const std::string & in_config_file_name      ,
               const double        in_readout_time_sec      ,
               const long          in_receiver_fifo_depth   ,
               const int           in_bit_depth             ,
               const long          in_frame_packet_number_8 ,
               const long          in_frame_packet_number_16,
               const long          in_frame_packet_number_32)
{
    // creating the detector
    m_detector = new Detector(this                     ,
                              in_config_file_name      ,
                              in_readout_time_sec      ,
                              in_receiver_fifo_depth   ,
                              in_bit_depth             ,
                              in_frame_packet_number_8 ,
                              in_frame_packet_number_16,
                              in_frame_packet_number_32);

    // creating the camera frame manager
    m_frames_manager = new CameraFrames(m_detector->getModulesNb() * 2, // 2 parts of images for each module
                                        m_detector->getFramePacketNumber(), 
                                        m_detector->getBitDepth()/8,
                                        m_detector->getWidth (), 
                                        m_detector->getHeight(), 
                                        m_detector->getWidth ()/m_detector->getModulesNb(), 
                                        m_detector->getHeight()/m_detector->getModulesNb());
    // creating the camera thread
    m_thread = new CameraThread(m_frames_manager, this);

    // starting the acquisition thread
    m_thread->start();
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Camera::~Camera()
{
    DEB_DESTRUCTOR();

    // stopping the acquisition
    stopAcq();

    // releasing the camera thread
    delete m_thread;
    
    // releasing the camera frame manager
    delete m_frames_manager;

    // releasing the detector
    delete m_detector;
}

//==================================================================
// Related to HwInterface
//==================================================================
//------------------------------------------------------------------
// acquisition management
//------------------------------------------------------------------
/*******************************************************************
 * \brief prepares the acquisition
 *******************************************************************/
void Camera::prepareAcq()
{
    DEB_MEMBER_FUNCT();

    // Only snap is allowed
    {
        int64_t nb_frames = getNbFrames();

        if(nb_frames == 0LL)
        {
            THROW_HW_ERROR(ErrorType::Error) << "Start mode is not allowed for this device! Please use Snap mode.";
        }
    }

    // clear the frames containers
    m_frames_manager->clear();
}

/*******************************************************************
 * \brief starts the acquisition (start/snap)
 *******************************************************************/
void Camera::startAcq()
{
    m_thread->sendCmd(CameraThread::StartAcq);
    m_thread->waitNotStatus(CameraThread::Idle);
}

/*******************************************************************
 * \brief stops the acquisition
 *******************************************************************/
void Camera::stopAcq()
{
    DEB_MEMBER_FUNCT();

	DEB_TRACE() << "executing StopAcq command...";

    if(m_thread->getStatus() != CameraThread::Error)
    {
        m_thread->execStopAcq();

        // Waiting for thread to finish or to be in error
        m_thread->waitNotStatus(CameraThread::Running);
    }

    // thread in error
    if(m_thread->getStatus() == CameraThread::Error)
    {
        // aborting & restart the thread
        m_thread->abort();
    }
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
void Camera::acquisitionDataReady(const int      in_receiver_index,
                                  uint64_t       in_frame_index   ,
                                  const int      in_pos_x         ,
                                  const int      in_pos_y         ,
                                  const uint32_t in_packet_number ,
                                  const uint64_t in_timestamp     ,
                                  const char *   in_data_pointer  ,
                                  const uint32_t in_data_size     )
{
    yat::SharedPtr<CameraFramePart> frame_part = new CameraFramePart(in_pos_x, in_pos_y,
                                                                     in_packet_number,
                                                                     in_timestamp,
                                                                     (uint8_t*)in_data_pointer, in_data_size);

    m_frames_manager->addFramePart(in_frame_index, frame_part );
}

/************************************************************************
 * \brief start receiver listening mode
 * \return true if ok, else false
 ************************************************************************/
bool Camera::startReceiver()
{
    return m_detector->startReceiver();
}

/************************************************************************
 * \brief stop receiver listening mode
 * \return true if ok, else false
 ************************************************************************/
bool Camera::stopReceiver()
{
    return m_detector->stopReceiver();
}

/************************************************************************
 * \brief start detector real time acquisition in non blocking mode
 * \return true if ok, else false
 ************************************************************************/
bool Camera::startAcquisition()
{
    return m_detector->startAcquisition();
}

/************************************************************************
 * \brief stop detector real time acquisition
 * \return true if ok, else false
 ************************************************************************/
bool Camera::stopAcquisition()
{
    return m_detector->stopAcquisition();
}

//------------------------------------------------------------------
// status management
//------------------------------------------------------------------
/************************************************************************
 * \brief returns the current camera status
 * \return current hardware status
 ************************************************************************/
lima::SlsEiger::Status Camera::getStatus()
{
    DEB_MEMBER_FUNCT();

    SlsEiger::Status result;

    int thread_status = m_thread->getStatus();

    // error during the acquisition management ?
    // the device becomes in error state.
    if(thread_status == CameraThread::Error)
    {
        result = lima::SlsEiger::Error;
    }
    else
    // the device is in acquisition.
    // During an aquisition, this is the acquisition thread which read the hardware camera status.
    // So we use the latest read camera status.
    if(thread_status == CameraThread::Running)
    {
        result = lima::SlsEiger::Running;
    }
    else
    // the device is not in acquisition or in error, so we can read the hardware camera status
    {
        result = getDetectorStatus();
    }

    return result;
}

/************************************************************************
 * \brief returns the current detector status
 * \return current hardware status
 ************************************************************************/
lima::SlsEiger::Status Camera::getDetectorStatus()
{
    return m_detector->getStatus();
}

//------------------------------------------------------------------
// frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the number of acquired frames
 * \return current acquired frames number
 *******************************************************************/
uint64_t Camera::getNbAcquiredFrames() const
{
    // reading in the number of treated frames in the frame manager
    return m_frames_manager->getNbTreatedFrames();
}

/************************************************************************
 * \brief get the number of frames in the containers
 * \return the number of frames by type
 ************************************************************************/
void Camera::getNbFrames(size_t & out_received  , 
                         size_t & out_not_merged,
                         size_t & out_treated   ) const
{
    // reading in the number of frames in the frame manager
    m_frames_manager->getNbFrames(out_received, out_not_merged, out_treated);
}

//==================================================================
// Related to HwDetInfoCtrlObj
//==================================================================
//------------------------------------------------------------------
// image size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the maximum image width
 * \return maximum image width
 *******************************************************************/
unsigned short Camera::getMaxWidth() const
{
    return static_cast<unsigned short>(m_detector->getMaxWidth());
}

/*******************************************************************
 * \brief Gets the maximum image height
 * \return maximum image height
 *******************************************************************/
unsigned short Camera::getMaxHeight() const
{
    return static_cast<unsigned short>(m_detector->getMaxHeight());
}

/*******************************************************************
 * \brief Gets the image width
 * \return image width
 *******************************************************************/
unsigned short Camera::getWidth() const
{
    return static_cast<unsigned short>(m_detector->getWidth());
}

/*******************************************************************
 * \brief Gets the image height
 * \return image height
 *******************************************************************/
unsigned short Camera::getHeight() const
{
    return static_cast<unsigned short>(m_detector->getHeight());
}

//------------------------------------------------------------------
// current image type management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the default image type
 *******************************************************************/
lima::ImageType Camera::getDefImageType() const
{
    DEB_MEMBER_FUNCT();
    return lima::ImageType::Bpp16;
}

/*******************************************************************
 * \brief gets the current image type
 * \return current image type
*******************************************************************/
lima::ImageType Camera::getImageType() const
{
    DEB_MEMBER_FUNCT();

    lima::ImageType type;

    // getting the bit depth of the camera
    int bit_depth = m_detector->getBitDepth();

    switch(bit_depth)
    {
        case 8: 
            type = lima::ImageType::Bpp8;
            break;
    
        case 16: 
            type = lima::ImageType::Bpp16;
            break;

        case 32: 
            type = lima::ImageType::Bpp32;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "This pixel format of the camera is not managed, only 8/16/32 bits cameras are managed!";
            break;
    }

    return type;
}

/*******************************************************************
 * \brief sets the current image type
 * \param in_type new image type
 *******************************************************************/
void Camera::setImageType(lima::ImageType in_type)
{
    DEB_MEMBER_FUNCT();

    int bit_depth;

    switch(in_type)
    {
        case lima::ImageType::Bpp32:
            bit_depth = 32;
            break;

        case lima::ImageType::Bpp16:
            bit_depth = 16;
            break;

        case lima::ImageType::Bpp8:
            bit_depth = 8;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "This pixel format of the camera is not managed, only 8/16/32 bits cameras are managed!";
            break;
    }

    // At the moment, the bit depth is fixed during the init process (in detector class).
    // No need to change it again as we should also perhaps set others detector properties.
    //m_detector->setBitDepth(bit_depth);
}

//------------------------------------------------------------------
// pixel size management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the pixel size
 * \param out_x_size width pixel size
 * \param out_y_size height pixel size
 *******************************************************************/
void Camera::getPixelSize(double & out_x_size, double & out_y_size) const
{
    out_x_size = out_y_size = 75e-6; // to do, put the correct value
}

//------------------------------------------------------------------
// detector info management
//------------------------------------------------------------------
/*******************************************************************
 * \brief gets the detector type
 * \return detector type
 *******************************************************************/
std::string Camera::getDetectorType() const
{
    return m_detector->getDetectorType();
}

/*******************************************************************
 * \brief gets the detector model
 * \return detector model
 *******************************************************************/
std::string Camera::getDetectorModel() const
{
    return m_detector->getDetectorModel();
}

/*******************************************************************
 * \brief gets Detector Firmware Version
 * \return Detector Firmware Version
 *******************************************************************/
std::string Camera::getDetectorFirmwareVersion() const
{
    return m_detector->getDetectorFirmwareVersion();
}

/*******************************************************************
 * \brief gets Detector Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Camera::getDetectorSoftwareVersion() const
{
    return m_detector->getDetectorSoftwareVersion();
}

/*******************************************************************
 * \brief gets Detector This Software Version
 * \return Detector Software Version
 *******************************************************************/
std::string Camera::getDetectorThisSoftwareVersion() const
{
    return m_detector->getDetectorThisSoftwareVersion();
}

//==================================================================
// Related to HwSyncCtrlObj
//==================================================================
//------------------------------------------------------------------
// trigger mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Checks the trigger mode validity
 * \return true if passed trigger mode is supported
 *******************************************************************/
bool Camera::checkTrigMode(lima::TrigMode in_trig_mode) const
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(in_trig_mode);
    bool valid_mode;    

    switch (in_trig_mode)
    {       
        case lima::TrigMode::IntTrig      :
        case lima::TrigMode::ExtTrigSingle:
        case lima::TrigMode::ExtTrigMult  :
        case lima::TrigMode::ExtGate      :
            valid_mode = true;
            break;

        default:
            valid_mode = false;
            break;
    }

    return valid_mode;
}

/*******************************************************************
 * \brief Sets the trigger mode
 * \param in_mode needed trigger mode 
 *                (IntTrig, ExtTrigSingle, ExtTrigMultiple)
 *******************************************************************/
void Camera::setTrigMode(lima::TrigMode in_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(in_mode);

    lima::SlsEiger::TriggerMode detector_trigger_mode;

    switch (in_mode)
    {       
        case lima::TrigMode::IntTrig:
            detector_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE;
            break;

        case lima::TrigMode::ExtTrigSingle:
            detector_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE;
            break;

        case lima::TrigMode::ExtTrigMult:
            detector_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE;
            break;

        case lima::TrigMode::ExtGate:
            detector_trigger_mode = lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE;
            break;

        default:
            THROW_HW_ERROR(ErrorType::Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed:" << in_mode;
            break;
    }

    m_detector->setTriggerMode(detector_trigger_mode);
}

/*******************************************************************
 * \brief Gets the trigger mode
 * \return trigger mode
 *******************************************************************/
lima::TrigMode Camera::getTrigMode()
{
    DEB_MEMBER_FUNCT();

    lima::TrigMode lima_trigger_mode;
    lima::SlsEiger::TriggerMode detector_trigger_mode = m_detector->getTriggerMode();

    if(detector_trigger_mode == lima::SlsEiger::TriggerMode::TRIGGER_INTERNAL_SINGLE  ) lima_trigger_mode = lima::TrigMode::IntTrig      ; else
    if(detector_trigger_mode == lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_SINGLE  ) lima_trigger_mode = lima::TrigMode::ExtTrigSingle; else
    if(detector_trigger_mode == lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_MULTIPLE) lima_trigger_mode = lima::TrigMode::ExtTrigMult  ; else
    if(detector_trigger_mode == lima::SlsEiger::TriggerMode::TRIGGER_EXTERNAL_GATE    ) lima_trigger_mode = lima::TrigMode::ExtGate      ; else
    {
        THROW_HW_ERROR(ErrorType::Error) << "Camera::getTrigMode : This camera trigger Mode is not managed!";
    }

    return lima_trigger_mode;
}

//------------------------------------------------------------------
// times management
//------------------------------------------------------------------
//------------------------------------------------------------------
// exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the exposure time
 * \return exposure time
 *******************************************************************/
double Camera::getExpTime()
{
    return m_detector->getExpTime();
}

/*******************************************************************
 * \brief Sets the exposure time
 * \param in_exp_time needed exposure time
*******************************************************************/
void Camera::setExpTime(double in_exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(in_exp_time) << " (sec)";

    return m_detector->setExpTime(in_exp_time);
}

//------------------------------------------------------------------
// latency time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the latency time
 * \return latency time
 *******************************************************************/
double Camera::getLatencyTime()
{
    return m_detector->getLatencyTime();
}

/*******************************************************************
 * \brief Sets the latency time
 * \param in_latency_time needed latency time
*******************************************************************/
void Camera::setLatencyTime(double in_latency_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setLatencyTime - " << DEB_VAR1(in_latency_time) << " (sec)";

    return m_detector->setLatencyTime(in_latency_time);
}

//------------------------------------------------------------------
// number of frames management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Sets the number of frames
 * \param in_nb_frames number of needed frames
*******************************************************************/
void Camera::setNbFrames(int64_t in_nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(in_nb_frames);
    
    if(in_nb_frames < 0LL)
        throw LIMA_HW_EXC(InvalidValue, "Invalid nb of frames");

    m_detector->setNbFrames(in_nb_frames);
}

/*******************************************************************
 * \brief Gets the number of frames
 * \return number of frames
 *******************************************************************/
int64_t Camera::getNbFrames()
{
    return m_detector->getNbFrames();
}

/*******************************************************************
 * \brief Gets the internal number of frames (for thread access)
 * \return number of frames
 *******************************************************************/
uint64_t Camera::getInternalNbFrames()
{
    return m_detector->getInternalNbFrames();
}

//------------------------------------------------------------------
// valid ranges management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets exposure and latency ranges
 * \return valid ranges structure
 *******************************************************************/
HwSyncCtrlObj::ValidRangesType Camera::getValidRanges() const
{
    DEB_MEMBER_FUNCT();

    double          min_time = 10e-9;
    double          max_time = 10e+9;

    HwSyncCtrlObj::ValidRangesType valid_ranges;

    valid_ranges.min_exp_time = min_time;
    valid_ranges.max_exp_time = max_time;
    valid_ranges.min_lat_time = m_detector->getReadoutTimeSec();
    valid_ranges.max_lat_time = max_time;

    return valid_ranges;
}

//==================================================================
// Related to BufferCtrl object
//==================================================================
/*******************************************************************
 * \brief Gets the internal buffer manager
 * \return the internal buffer manager
 *******************************************************************/
HwBufferCtrlObj * Camera::getBufferCtrlObj()
{
    return &m_buffer_ctrl_obj;
}

/*******************************************************************
 * \brief Gets the standard internal buffer manager
 * \return the internal buffer manager
 *******************************************************************/
StdBufferCbMgr & Camera::getStdBufferCbMgr()
{
    return m_buffer_ctrl_obj.getBuffer();
}

//==================================================================
// Related to commands (put & get)
//==================================================================
/*******************************************************************
 * \brief Executes a set command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Camera::setCmd(const std::string & in_command, int in_module_index)
{
	DEB_MEMBER_FUNCT();
    DEB_PARAM() << "Camera::setCmd - execute set command:\"" << in_command << "\"";

    return m_detector->setCmd(in_command, in_module_index);
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string Camera::getCmd(const std::string & in_command, int in_module_index)
{
	DEB_MEMBER_FUNCT();
    DEB_PARAM() << "Camera::getCmd - execute get command:\"" << in_command << "\"";

    return m_detector->getCmd(in_command, in_module_index);
}

//==================================================================
// Related to specifics attributes
//==================================================================
/*******************************************************************
 * \brief Gets the threshold energy in eV
 * \return threshold energy in eV
 *******************************************************************/
int Camera::getThresholdEnergy()
{
    return m_detector->getThresholdEnergy();
}

/*******************************************************************
 * \brief Sets the threshold energy in eV
 * \param in_threshold_energy_eV needed threshold energy in eV
*******************************************************************/
void Camera::setThresholdEnergy(int in_threshold_energy_eV)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setThresholdEnergy - " << DEB_VAR1(in_threshold_energy_eV) << " (eV)";

    m_detector->setThresholdEnergy(in_threshold_energy_eV);
}

/*******************************************************************
 * \brief Gets the clock divider
 * \return clock divider
 *******************************************************************/
lima::SlsEiger::ClockDivider Camera::getClockDivider()
{
    return m_detector->getClockDivider();
}

/*******************************************************************
 * \brief Sets the clock divider
 * \param in_clock_divider needed clock divider
*******************************************************************/
void Camera::setClockDivider(lima::SlsEiger::ClockDivider in_clock_divider)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setClockDivider - " << DEB_VAR1(in_clock_divider);

    m_detector->setClockDivider(in_clock_divider);
}

/*******************************************************************
 * \brief Gets the parallel mode 
 * \return the parallel mode
 *******************************************************************/
lima::SlsEiger::ParallelMode Camera::getParallelMode()
{
    return m_detector->getParallelMode();
}

/*******************************************************************
 * \brief Sets the parallel mode
 * \param in_parallelMode needed parallel mode
*******************************************************************/
void Camera::setParallelMode(lima::SlsEiger::ParallelMode in_parallel_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setParallelMode - " << DEB_VAR1(in_parallel_mode);

    m_detector->setParallelMode(in_parallel_mode);
}

/*******************************************************************
 * \brief Gets the overflow mode
 * \return the overflow mode
 *******************************************************************/
bool Camera::getOverflowMode()
{
    return m_detector->getOverflowMode();
}

/*******************************************************************
 * \brief Sets the overflow mode
 * \param in_overflow_mode needed overflow mode
*******************************************************************/
void Camera::setOverflowMode(bool in_overflow_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setOverflowMode - " << DEB_VAR1(in_overflow_mode);

    m_detector->setOverflowMode(in_overflow_mode);
}

//------------------------------------------------------------------
// sub frame exposure time management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the sub frame exposure time
 * \return sub frame exposure time
 *******************************************************************/
double Camera::getSubFrameExposureTime()
{
    return m_detector->getSubFrameExposureTime();
}

/*******************************************************************
 * \brief Sets the sub frame exposure time
 * \param in_sub_frame_exposure_time needed sub frame exposure time
*******************************************************************/
void Camera::setSubFrameExposureTime(double in_sub_frame_exposure_time)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setSubFrameExposureTime - " << DEB_VAR1(in_sub_frame_exposure_time);

    m_detector->setSubFrameExposureTime(in_sub_frame_exposure_time);
}

//------------------------------------------------------------------
// gain mode management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the gain mode
 * \return gain mode
 *******************************************************************/
lima::SlsEiger::GainMode Camera::getGainMode()
{
    return m_detector->getGainMode();
}

/*******************************************************************
 * \brief Sets the gain mode
 * \param in_gain_mode needed gain mode 
 *******************************************************************/
void Camera::setGainMode(lima::SlsEiger::GainMode in_gain_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setGainMode - " << DEB_VAR1(in_gain_mode);

    m_detector->setGainMode(in_gain_mode);
}

//------------------------------------------------------------------
// count rate correction management
//------------------------------------------------------------------
/*******************************************************************
 * \brief Gets the activation of count rate correction
 * \return count rate correction activation
 *******************************************************************/
bool Camera::getCountRateCorrectionActivation()
{
    return m_detector->getCountRateCorrectionActivation();
}

/*******************************************************************
 * \brief Sets the activation of count rate correction
 * \param in_count_rate_correction_activated  
*******************************************************************/
void Camera::setCountRateCorrectionActivation(bool in_count_rate_correction_activation)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setCountRateCorrectionActivation - " << DEB_VAR1(in_count_rate_correction_activation);

    m_detector->setCountRateCorrectionActivation(in_count_rate_correction_activation);
}

/*******************************************************************
 * \brief Gets the count rate correction in ns
 * \return count rate correction in eV (0 disabled else default value)
 *******************************************************************/
int Camera::getCountRateCorrection()
{
    return m_detector->getCountRateCorrection();
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
int Camera::getTemperature(lima::SlsEiger::Temperature in_temperature_type, int in_module_index)
{
    return m_detector->getTemperature(in_temperature_type, in_module_index);
}

//==================================================================
// Related to event control object
//==================================================================
/*******************************************************************
 * \brief Gets the Lima event control object
 * \return event control object pointer
*******************************************************************/
HwEventCtrlObj* Camera::getEventCtrlObj()
{
	return &m_event_ctrl_obj;
}

//========================================================================================
