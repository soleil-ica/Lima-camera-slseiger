//------------------------------------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include <string>

#include <cstdlib>
#include <cstring>

#include <stdint.h>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <sstream>
#include <unistd.h>

#include "sls_receiver_defs.h"
#include "slsReceiverUsers.h"

#include "sls_detector_defs.h"
#include "slsDetectorUsers.h"

#include "SlsEigerCameraFrames.h"
#include "SlsEigerCameraThreadTest.h"

#include "StreamNexus.h"

// LIMA
#include "lima/Debug.h"
#include "lima/Constants.h"

using namespace lima::SlsEiger;

//#define GOTTHARD_25_TEST
//#define JUNGFRAU_TEST
//#define GOTTHARD_TEST
#define EIGER_TEST

//======================================================================================================
// test configuration
//======================================================================================================
int      acquisition_nb    = 1; // number of acquisition to make
int      acquisition_nb_ok = 0; // number of correct acquisition
uint64_t last_acquisition_received_frames; // number of received frames during the last acquisition
std::vector <int> acquisition_nb_list;

bool use_trace = false; // activate the acquisition log

//------------------------------------------------------------------------------------------------------
// GOTTHARD 25um
//------------------------------------------------------------------------------------------------------
#ifdef GOTTHARD_25_TEST
    const int receivers_nb = 2; // number of receivers
    const int receivers_rx_tcpport[receivers_nb] = {1954, 1955}; // tcp port for each receiver

    const int detector_id = 0; // detector identifier for slsDetectorUsers constructor
    const std::string detector_config_file_name = "gotthard25.config"; // configuration file name (must be present in the same folder of this application)

    const long        detector_receiver_fifo_depth        = 2500;
    double            detector_exposure_time_sec          = 0.005;
    double            detector_exposure_period_sec        = 0.10;
    const double      detector_delay_after_trigger_sec    = 0.0;
    const std::string detector_trig_mode                  = "auto"; // "auto" or "trigger"
    int64_t           detector_nb_frames_per_cycle        = 10;
    const int64_t     detector_nb_cycles                  = 1;
    int               detector_module_index[receivers_nb] = {0, 1};
#else
//------------------------------------------------------------------------------------------------------
// GOTTHARD
//------------------------------------------------------------------------------------------------------
#ifdef GOTTHARD_TEST
    const int receivers_nb = 1; // number of receivers
    const int receivers_rx_tcpport[receivers_nb] = {1954}; // tcp port for each receiver

    const int detector_id = 0; // detector identifier for slsDetectorUsers constructor
    const std::string detector_config_file_name = "gotthard25.config"; // configuration file name (must be present in the same folder of this application)

    const long        detector_receiver_fifo_depth        = 2500;
    double            detector_exposure_time_sec          = 0.005;
    double            detector_exposure_period_sec        = 0.1;
    const double      detector_delay_after_trigger_sec    = 0.0;
    const std::string detector_trig_mode                  = "auto"; // "auto" or "trigger"
    int64_t           detector_nb_frames_per_cycle        = 10;
    const int64_t     detector_nb_cycles                  = 1;
    int               detector_module_index[receivers_nb] = {0};
#else
//------------------------------------------------------------------------------------------------------
// JUNGFRAU
//------------------------------------------------------------------------------------------------------
#ifdef JUNGFRAU_TEST
    const int receivers_nb = 1; // number of receivers
    const int receivers_rx_tcpport[receivers_nb] = {1954}; // tcp port for each receiver

    const int detector_id = 0; // detector identifier for slsDetectorUsers constructor
    const std::string detector_config_file_name = "jungfrau_nanoscopium_switch.config"; // configuration file name (must be present in the same folder of this application)

    const long        detector_receiver_fifo_depth        = 2500;
    double            detector_exposure_time_sec          = 0.0005;
    double            detector_exposure_period_sec        = 0.001;
    const double      detector_delay_after_trigger_sec    = 0.0;
    const std::string detector_trig_mode                  = "auto"; // "auto" or "trigger"
    int64_t           detector_nb_frames_per_cycle        = 10000;
    const int64_t     detector_nb_cycles                  = 1;
    int               detector_clock_divider              = 1; // 0 FullSpeed, 1 HalfSpeed, 2 QuarterSpeed, 3 SuperSlowSpeed
    int               detector_module_index[receivers_nb] = {0};
#else
//------------------------------------------------------------------------------------------------------
// EIGER
//------------------------------------------------------------------------------------------------------
#ifdef EIGER_TEST
    const int receivers_nb = 2; // number of receivers
    const int receivers_rx_tcpport[receivers_nb] = {1991, 1993}; // tcp port for each receiver

    const int detector_id = 0; // detector identifier for slsDetectorUsers constructor
    const std::string detector_config_file_name = "eiger_10Gb.config"; // configuration file name (must be present in the same folder of this application)

    const long        detector_receiver_fifo_depth        = 8000; //10000;
    double            detector_exposure_time_sec          = 0.0005;
    double            detector_exposure_period_sec        = 0.001;
    const double      detector_delay_after_trigger_sec    = 0.0;
    const std::string detector_trig_mode                  = "auto"; // "auto" or "trigger"
    int64_t           detector_nb_frames_per_cycle        = 10000;
    const int64_t     detector_nb_cycles                  = 1;
    int               detector_clock_divider              = 1; // 0 (FullSpeed), 1 (HalfSpeed), 2 (QuarterSpeed), 3 (SuperSlowSpeed)
    int               detector_parallel_mode              = 0; // 0 (non parallel), 1 (parallel), 2 (safe mode)
    int               detector_flow_control_10G           = 1; // 1 sets, 0 unsets
    int               detector_ten_gigabit_ethernet       = 1; // 1 sets, 0 unsets
    int               detector_enable_gap_pixels          = 0; // enable 1 sets, 0 unsets
    int               detector_enable_overflow_mode       = 0; // enable 1 sets, 0 unsets

    int               detector_module_index[receivers_nb] = {0, 1};
    int               detector_bit_depth                  = 16;

    double            detector_sub_exposure_time_sec      = 0.00262144; // default value

    //const std::string                   target_path     = "./files/";
    const std::string                   target_path     = "/nfs/srv3/spool1/final_storage/kadda/test_eiger_april2019/";
    const std::string                   target_file     = "test_eiger_";
    const uint32_t                      nb_acq_per_file = 100; // number of acquisition per file
    const enum StreamNexus::WriteModes  write_mode      = StreamNexus::WriteModes::ASYNCHRONOUS;
    const std::string                   label_frame     = "Frame";
    const std::size_t                   frame_size_x    = 1024;
    const std::size_t                   frame_size_y    = 512;

    const uint32_t                      frame_part_packet_number_8  = 32 ;
    const uint32_t                      frame_part_packet_number_16 = 64 ;
    const uint32_t                      frame_part_packet_number_32 = 128;
#endif
#endif
#endif
#endif
//------------------------------------------------------------------------------------------------------
// Const strings
//------------------------------------------------------------------------------------------------------
const std::string clock_dividers[] = {"FullSpeed", "HalfSpeed", "QuarterSpeed", "SuperSlowSpeed"};

//------------------------------------------------------------------------------------------------------
// test instances
//------------------------------------------------------------------------------------------------------
std::vector<slsReceiverUsers *> receivers;
slsDetectorUsers * detector = NULL;

yat::SharedPtr<CameraFrames> m_frames_manager = NULL;
yat::SharedPtr<StreamNexus> m_stream_nexus = NULL;

int current_acquisition_index = 0;

//------------------------------------------------------------------------------------------------------
// tools functions
//------------------------------------------------------------------------------------------------------
/** Define Colors to print data call back in different colors for different recievers */
#define PRINT_IN_COLOR(c,f, ...) 	printf ("\033[%dm" f RESET, 30 + c+1, ##__VA_ARGS__)

#define PRINT_SEPARATOR()  	cprintf(MAGENTA, "============================================\n")

/************************************************************************
 * \brief cleans the shared memory used by the camera
 ************************************************************************/
void clean_shared_memory()
{
    std::string cmd = "rm /dev/shm/slsDetectorPackage*;";
    std::system(cmd.c_str());
}

/*******************************************************************
 * \brief converts a version id to a string
 * \return version in string format (uppercase & hexa)
 *******************************************************************/
std::string convertVersionToString(int64_t in_version)
{
    std::stringstream tempStream;
    tempStream << "0x" << std::uppercase << std::hex << in_version;
    return tempStream.str();
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
void convertStringToArgs(const std::string & in_command,
                         char  * *         & out_argv  ,
                         int               & out_argc  )
{
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
 * \param in_out_argv output c-strings c-array*(static_cast<int *>(p))
 * \param in_out_argc output number of arguments of out_argv
 *******************************************************************/
void releaseArgs(char * * & in_out_argv  ,
                 int      & in_out_argc  )
{
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
std::string setCmd(const std::string & in_command, int in_module_index=-1)
{
    std::cout << "setCmd - execute set command:\"" << in_command << "\"" << std::endl;

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        result = detector->putCommand(argc, argv, in_module_index);
    }

    releaseArgs(argv, argc);

	std::cout << "result=\"" << result << "\"" << std::endl;
    return result;
}

/*******************************************************************
 * \brief Executes a get command
 * \param in_command command in command line format
 * \param in_module_index module index
 * \return the command result
 *******************************************************************/
std::string getCmd(const std::string & in_command, int in_module_index=-1)
{
    std::cout << "getCmd - execute get command:\"" << in_command << "\"" << std::endl;

    char  * *   argv  ;
    int         argc  ;
    std::string result;

    convertStringToArgs(in_command, argv, argc);

    if(argc > 0)
    {
        result = detector->getCommand(argc, argv, in_module_index);
    }

    releaseArgs(argv, argc);

	std::cout << "result=\"" << result << "\"" << std::endl;
    return result;
}

//------------------------------------------------------------------------------------------------------
// Receivers callbacks
//------------------------------------------------------------------------------------------------------
/**
 * Start Acquisition Call back
 * slsReceiver writes data if file write enabled.
 * Users get data to write using call back if registerCallBackRawDataReady is registered.
 * @param filepath file path
 * @param filename file name
 * @param fileindex file index
 * @param datasize data size in bytes
 * @param p pointer to object
 * \returns ignored
 */
int StartAcq(char* filepath, char* filename, uint64_t fileindex, uint32_t datasize, void*p){
	cprintf(BLUE, "#### StartAcq:  filepath:%s  filename:%s fileindex:%llu  datasize:%u ####\n",
			filepath, filename, fileindex, datasize);

	cprintf(BLUE, "--StartAcq: returning 0\n");
    last_acquisition_received_frames = 0LL;
	return 0;
}

/**
 * Acquisition Finished Call back
 * @param frames Number of frames caught
 * @param p pointer to object
 */
void AcquisitionFinished(uint64_t frames, void*p){
	cprintf(BLUE, "#### AcquisitionFinished: frames:%llu ####\n",frames);
    last_acquisition_received_frames = frames;
}

/**
 * Get Receiver Data Call back
 * Prints in different colors(for each receiver process) the different headers for each image call back.
 * @param metadata sls_receiver_header metadata
 * @param datapointer pointer to data
 * @param datasize data size in bytes.
 * @param p pointer to object
 */
void GetData(char* metadata, char* datapointer, uint32_t datasize, void* p)
{
    slsReceiverDefs::sls_receiver_header* header = (slsReceiverDefs::sls_receiver_header*)metadata;
    const slsReceiverDefs::sls_detector_header & detectorHeader = header->detHeader;

    if(use_trace)
    {
        PRINT_IN_COLOR (*(static_cast<int *>(p)),
			        "#### %d GetData: ####\n"
			        "frameNumber: %llu\t\texpLength: %u\t\tpacketNumber: %u\t\tbunchId: %llu"
			        "\t\ttimestamp: %llu\t\tmodId: %u\t\t"
			        "row: %u\t\tcolumn: %u\t\treserved: %u\t\tdebug: %u"
			        "\t\troundRNumber: %u\t\tdetType: %u\t\tversion: %u"
			        //"\t\tpacketsMask:%s"
			        "\t\tfirstbytedata: 0x%x\t\tdatsize: %u\n\n",
                    *(static_cast<int *>(p)),
                    (long long unsigned int)detectorHeader.frameNumber,
                    detectorHeader.expLength, 
                    detectorHeader.packetNumber, 
                    (long long unsigned int)detectorHeader.bunchId,
                    (long long unsigned int)detectorHeader.timestamp,
                    detectorHeader.modId,
                    detectorHeader.row,
                    detectorHeader.column,
                    detectorHeader.reserved,
                    detectorHeader.debug,
                    detectorHeader.roundRNumber,
                    detectorHeader.detType,
                    detectorHeader.version,
			        //header->packetsMask.to_string().c_str(),
			        ((uint8_t)(*((uint8_t*)(datapointer)))),
                    datasize);
    }

    if((datapointer != NULL) && (datasize > 0))
    {
        yat::SharedPtr<CameraFramePart> frame_part = new CameraFramePart(detectorHeader.column,
                                                                        detectorHeader.row,
                                                                        detectorHeader.packetNumber,
                                                                        detectorHeader.timestamp,
                                                                        (uint8_t*)datapointer, datasize);

        m_frames_manager->addFramePart(detectorHeader.frameNumber - 1, frame_part ); // frameNumber starts at 1
    }
}

//------------------------------------------------------------------------------------------------------
// CreateReceivers
//------------------------------------------------------------------------------------------------------
void CreateReceivers(void)
{
#ifndef EIGER_DETECTOR_SIMULATION
    // preparing the args for receivers creation
    char        temp_port[10];
    const int   argc       = 3;
    char      * args[argc] = {(char*)"slsReceiver", (char*)"--rx_tcpport", temp_port};

    // creating the receivers instances 
    for(int i = 0 ; i < receivers_nb ; i++)
    {
    	int ret = slsReceiverDefs::OK;

        // changing the udp port in the args
        sprintf(temp_port, "%d", receivers_rx_tcpport[i]);

        // creating the receiver using the args
        slsReceiverUsers * receiver = new slsReceiverUsers(argc, args, ret);

        // managing a failed result
        if(ret==slsReceiverDefs::FAIL)
        {
            delete receiver;
            exit(EXIT_FAILURE);
        }

        // adding the receiver to the receivers container
        receivers.push_back(receiver);

        std::cout << "receiver (" << i << ") created - port (" << receivers_rx_tcpport[i] << ")" << std::endl;

        // registering callbacks
        // Call back for start acquisition
        cprintf(BLUE, "Registering StartAcq()\n");
        receiver->registerCallBackStartAcquisition(StartAcq, NULL);

        // Call back for acquisition finished
        cprintf(BLUE, "Registering AcquisitionFinished()\n");
        receiver->registerCallBackAcquisitionFinished(AcquisitionFinished, NULL);

        // Call back for raw data
        cprintf(BLUE, "Registering GetData() \n");
        receiver->registerCallBackRawDataReady(GetData, &(detector_module_index[i]));

        // starting tcp server thread
        if (receiver->start() == slsReceiverDefs::FAIL)
        {
            delete receiver;
            cprintf(BLUE,"Could not start receiver (%d)\n", i);
            exit(EXIT_FAILURE);
        }
    }
#endif
}

//------------------------------------------------------------------------------------------------------
// ReleaseReceivers
//------------------------------------------------------------------------------------------------------
void ReleaseReceivers(void)
{
#ifndef EIGER_DETECTOR_SIMULATION
    // deleting the receivers instances 
    for(int i = 0 ; i < receivers.size() ; i++)
    {
        slsReceiverUsers * receiver = receivers[i];
        delete receiver;
    }
#endif
}

//------------------------------------------------------------------------------------------------------
// CreateDetector
//------------------------------------------------------------------------------------------------------
void CreateDetector(void)
{
#ifndef EIGER_DETECTOR_SIMULATION
    int result;

    // create the detector instance
    detector = new slsDetectorUsers(result, detector_id);

    if(result == slsDetectorDefs::FAIL)
    {
		std::cout << "slsDetectorUsers constructor failed! Could not initialize the camera!" << std::endl;
		exit(EXIT_FAILURE);
    }

    // configuration file is used to properly configure advanced settings in the shared memory
    result = detector->readConfigurationFile(detector_config_file_name);

    if(result == slsDetectorDefs::FAIL)
    {
		std::cout << "readConfigurationFile failed! Could not initialize the camera!" << std::endl;
		exit(EXIT_FAILURE);
    }

	// set detector in shared memory online (in case no config file was used) */
	detector->setOnline(slsDetectorDefs::ONLINE_FLAG);

	// set receiver in shared memory online (in case no config file was used) */
	detector->setReceiverOnline(slsDetectorDefs::ONLINE_FLAG);

    // disabling the file write by the camera
    detector->enableWriteToFile(slsDetectorDefs::DISABLED);

    // logging some versions informations
    std::cout << "Detector developer        : " << detector->getDetectorDeveloper() << std::endl;
    std::cout << "Detector type             : " << detector->getDetectorType() << std::endl;
    std::cout << "Detector Firmware Version : " << convertVersionToString(detector->getDetectorFirmwareVersion()) << std::endl;
    std::cout << "Detector Software Version : " << convertVersionToString(detector->getDetectorSoftwareVersion()) << std::endl;

	// ensuring detector status is idle
	int status = detector->getDetectorStatus();

    if((status != slsDetectorDefs::IDLE) && (status != slsDetectorDefs::STOPPED))
    {
		std::cout << "Detector not ready: " << slsDetectorUsers::runStatusType(status) << std::endl;
		exit(EXIT_FAILURE);
	}
#endif
}

//------------------------------------------------------------------------------------------------------
// ReleaseDetector
//------------------------------------------------------------------------------------------------------
void ReleaseDetector(void)
{
#ifndef EIGER_DETECTOR_SIMULATION
    if(detector != NULL)
    {
        detector->setReceiverOnline(slsDetectorDefs::OFFLINE_FLAG);
        detector->setOnline(slsDetectorDefs::OFFLINE_FLAG);

        delete detector;
        detector = NULL;
    }
#endif
}

/*******************************************************************
 * \brief init radom number manager
 *******************************************************************/
void init_random_number()
{
    srand (time(NULL));
}

/*******************************************************************
 * \brief generate a random number
 *******************************************************************/
int32_t generate_random_number(const int32_t min, const int32_t max)
{
    int32_t n = (rand() % (max - min + 1)) + min;
    return n;
}

//------------------------------------------------------------------------------------------------------
// RunAcquisition
//------------------------------------------------------------------------------------------------------
int RunAcquisition(void)
{
    std::string trig_mode_label;

    double exposure_time  ;
    double exposure_period;
    double delay_after_trigger;

    int64_t nb_frames_per_cycle;
    int64_t nb_cycles;
    int64_t nb_frames;
#if defined(JUNGFRAU_TEST) || defined(EIGER_TEST)
    int clock_divider;
#endif

#ifdef EIGER_TEST
    int bit_depth            = 0;
    int parallel_mode        = 0;
    int flow_control_10G     = 0;
    int ten_gigabit_ethernet = 0;
    int enable_gap_pixels    = 0;
    double sub_exposure_time = 0;
    int enable_overflow_mode = 0;
#endif

#ifndef EIGER_DETECTOR_SIMULATION
    //----------------------------------------------------------------------------------------------------
    // setting the receiver fifo depth (number of frames in the receiver memory)
    detector->setReceiverFifoDepth(detector_receiver_fifo_depth);

    #ifdef EIGER_TEST
        detector->setBitDepth(detector_bit_depth);
        bit_depth = detector->setBitDepth(-1);

        detector->setParallelMode(detector_parallel_mode);
        parallel_mode = detector->setParallelMode(-1);

        detector->setTenGigabitEthernet(detector_ten_gigabit_ethernet);
        ten_gigabit_ethernet = detector->setTenGigabitEthernet(-1);

        detector->setFlowControl10G(detector_flow_control_10G);
        flow_control_10G = detector->setFlowControl10G(-1);

        detector->enableGapPixels(detector_enable_gap_pixels);
        enable_gap_pixels = detector->enableGapPixels(-1);
    #endif

        //----------------------------------------------------------------------------------------------------
        detector->setExposureTime     (detector_exposure_time_sec  , true); // in seconds
        detector->setExposurePeriod   (detector_exposure_period_sec, true); // in seconds

        exposure_time       = detector->setExposureTime     (-1, true); // in seconds
        exposure_period     = detector->setExposurePeriod   (-1, true); // in seconds

    #ifdef EIGER_TEST
        detector->setSubFrameExposureTime(detector_sub_exposure_time_sec, true); // in seconds
        sub_exposure_time = detector->setSubFrameExposureTime(-1, true); // in seconds

        detector->setOverflowMode(detector_enable_overflow_mode);
        enable_overflow_mode = detector->setOverflowMode(-1);
    #endif

    #ifndef EIGER_TEST
        detector->setDelayAfterTrigger(detector_delay_after_trigger_sec, true); // in seconds
        delay_after_trigger = detector->setDelayAfterTrigger(-1, true, 0); // in seconds
    #endif

        //----------------------------------------------------------------------------------------------------
        // initing the number of frames per cycle and  number of cycles 
        // to avoid problems during the trigger mode change.
        detector->setNumberOfFrames(1);
        detector->setNumberOfCycles(1);

        // conversion of trigger mode label to trigger mode index
        int trigger_mode_index = slsDetectorUsers::getTimingMode(detector_trig_mode);

        // apply the trigger change
        detector->setTimingMode(trigger_mode_index);

        // converting trigger mode index to trigger mode label
        trig_mode_label = slsDetectorUsers::getTimingMode(trigger_mode_index);

        // setting the number of cycles
        nb_cycles = detector->setNumberOfCycles(detector_nb_cycles);

        // setting the number of frames per cycle
        nb_frames_per_cycle = detector->setNumberOfFrames(detector_nb_frames_per_cycle);

        // setting the gain mode
    #ifndef EIGER_TEST
        detector->setSettings(slsDetectorUsers::getDetectorSettings("dynamicgain"));
        
        #ifndef JUNGFRAU_TEST
            detector->setSettings(slsDetectorUsers::getDetectorSettings("mediumgain"));
        #else
            detector->setSettings(slsDetectorUsers::getDetectorSettings("dynamichg0"));
        #endif
    #endif

        // computing the number of frames
        nb_frames = nb_cycles * nb_frames_per_cycle;

    //----------------------------------------------------------------------------------------------------
    #if defined(JUNGFRAU_TEST) || defined(EIGER_TEST)
        // clock divider
        detector->setClockDivider(detector_clock_divider);
        clock_divider = detector->setClockDivider(-1);
    #endif
#else
    bit_depth = detector_bit_depth;
    parallel_mode = detector_parallel_mode;
    ten_gigabit_ethernet = detector_ten_gigabit_ethernet;
    flow_control_10G  = detector_flow_control_10G;
    enable_gap_pixels = detector_enable_gap_pixels;
    exposure_time     = detector_exposure_time_sec; // in seconds
    exposure_period   = detector_exposure_period_sec; // in seconds
    delay_after_trigger = detector_delay_after_trigger_sec; // in seconds
    trig_mode_label = "auto";
    nb_cycles = detector_nb_cycles;
    nb_frames_per_cycle = detector_nb_frames_per_cycle;

    // computing the number of frames
    nb_frames = nb_cycles * nb_frames_per_cycle;

    init_random_number();
#endif

    //----------------------------------------------------------------------------------------------------
    std::cout << "receiver fifo depth : " << detector_receiver_fifo_depth << std::endl;
    std::cout << "Exposure time in seconds : " << exposure_time << std::endl;
    std::cout << "Exposure period in seconds : " << exposure_period << std::endl;
    std::cout << "Delay after trigger in seconds : " << delay_after_trigger << std::endl;
    std::cout << "Trigger mode : " << trig_mode_label << std::endl;
    std::cout << "Nb frames per cycle : " << nb_frames_per_cycle << std::endl;
    std::cout << "Nb cycles : " << nb_cycles << std::endl;
    std::cout << "Nb frames : " << nb_frames << std::endl;

#if defined(JUNGFRAU_TEST) || defined(EIGER_TEST)
    std::cout << "Clock divider : " << clock_divider << std::endl;
#endif

#ifdef EIGER_TEST
    std::cout << "Bit depth : " << bit_depth << std::endl;
    std::cout << "Parallel mode : " << parallel_mode << std::endl;
    std::cout << "Flow control 10G : " << flow_control_10G << std::endl;
    std::cout << "Ten gigabit ethernet : " << ten_gigabit_ethernet << std::endl;
    std::cout << "Enable Gap Pixels : " << enable_gap_pixels << std::endl;
    std::cout << "Sub exposure time : " << sub_exposure_time << std::endl;
    std::cout << "Enable Overflow Mode : " << enable_overflow_mode << std::endl;
#endif

    std::cout << "Estimated frame rate : " << (1.0 / exposure_period) << std::endl;

#ifndef EIGER_DETECTOR_SIMULATION
    //----------------------------------------------------------------------------------------------------
    // reset the number of caught frames in the sdk
    detector->resetFramesCaughtInReceiver();
#endif

    //----------------------------------------------------------------------------------------------------
    // NEXUS management
    //----------------------------------------------------------------------------------------------------
    const unsigned int sleep_time_sec = 1; // sleep the thread in seconds

    std::stringstream tempStream;
    tempStream << target_file << current_acquisition_index << "_";

    m_stream_nexus = new StreamNexus();
    m_stream_nexus->set_parameters(StreamNexus::Parameters(target_path     ,
                                                           tempStream.str(),
                                                           nb_acq_per_file ,
                                                           nb_frames       ,
                                                           write_mode      ,
                                                           label_frame     ,
                                                           frame_size_x    ,
                                                           frame_size_y    ,
                                                           bit_depth       ));

    //----------------------------------------------------------------------------------------------------
    // Frame manager
    //----------------------------------------------------------------------------------------------------
    uint32_t frame_part_packet_number = 0;

    if(bit_depth == 8)
    {
        frame_part_packet_number = frame_part_packet_number_8;
    }
    else
    if(bit_depth == 16)
    {
        frame_part_packet_number = frame_part_packet_number_16;
    }
    else
    if(bit_depth == 32)
    {
        frame_part_packet_number = frame_part_packet_number_32;
    }

    m_frames_manager = new CameraFrames(4, frame_part_packet_number, bit_depth/8, frame_size_x, frame_size_y, frame_size_x/2, frame_size_y/2);
    yat::SharedPtr<CameraThreadTest> acq_thread = new CameraThreadTest(m_frames_manager, m_stream_nexus, detector, nb_frames);

    m_stream_nexus->start_acquisition();

    // starting the acquisition thread
    acq_thread->start();

    acq_thread->sendCmd(CameraThread::StartAcq);
    acq_thread->waitNotStatus(CameraThread::Idle);

#ifdef EIGER_DETECTOR_SIMULATION
    std::size_t datasize = (frame_size_x/2) * (frame_size_y/2) * (bit_depth/8);

    for(int index = 0 ; index < nb_frames; index++)
    {
        for(int x = 0 ; x < 2; x++)
        {
            for(int y = 0 ; y < 2; y++)
            {
                uint32_t current_frame_part_packet_number = frame_part_packet_number;

        #ifdef EIGER_DETECTOR_SIMULATION_LOST_FRAME_PART
                int lost_frame_event = EIGER_DETECTOR_SIMULATION_FRAME_PART_DISTRIBUTION[generate_random_number(0, EIGER_DETECTOR_SIMULATION_FRAME_PART_DISTRIBUTION_NB_ELEMENTS - 1)];

                if(lost_frame_event == EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_LOST)
                    continue;

                if(lost_frame_event == EIGER_DETECTOR_SIMULATION_LOST_FRAMES_PART_UNCOMPLETE)
                    current_frame_part_packet_number /= 2;
        #endif

                uint8_t * datapointer = new uint8_t[datasize];

                if(bit_depth == 8)
                {
                    std::memset(static_cast<void*>(datapointer), ((y<<1) | (x)) + 1, datasize);
                }
                else
                if(bit_depth == 16)
                {
                    std::vector<uint16_t> buffer;
                    buffer.resize(datasize / 2);
                    std::fill(buffer.begin(), buffer.end(), ((y<<1) | (x)) + 1);                    

                    std::memcpy(static_cast<void*>(datapointer), static_cast<const void*>(&buffer[0]), datasize);
                }
                else
                if(bit_depth == 32)
                {
                    std::vector<uint32_t> buffer;
                    buffer.resize(datasize / 4);
                    std::fill(buffer.begin(), buffer.end(), ((y<<1) | (x)) + 1);                    

                    std::memcpy(static_cast<void*>(datapointer), static_cast<const void*>(&buffer[0]), datasize);
                }

                yat::SharedPtr<CameraFramePart> frame_part = new CameraFramePart(x, y, current_frame_part_packet_number, 
                                                                                 0, (uint8_t*)datapointer, datasize);
                m_frames_manager->addFramePart(index, frame_part );

                delete [] datapointer;
            }
        }

        usleep(sleep_time_sec * 1000 * (exposure_period * 1000)); // sleep the thread in seconds
    }

    acq_thread->flagAcquisitionAsEnded();
#endif

    // Waiting for thread to finish or to be in error
    acq_thread->waitNotStatus(CameraThread::Running);

    if(m_frames_manager->getNbTreatedFrames() != nb_frames)
    {
        m_stream_nexus->abort();
    }
    else
    {
        m_stream_nexus->stop_acquisition();
    }

    // Release
    acq_thread       = NULL;
    m_frames_manager = NULL;
    m_stream_nexus   = NULL;

    //----------------------------------------------------------------------------------------------------
    PRINT_SEPARATOR();
    std::cout << "receiver fifo depth : " << detector_receiver_fifo_depth << std::endl;
    std::cout << "Exposure time in seconds : " << exposure_time << std::endl;
    std::cout << "Exposure period in seconds : " << exposure_period << std::endl;
    std::cout << "Delay after trigger in seconds : " << delay_after_trigger << std::endl;
    std::cout << "Trigger mode : " << trig_mode_label << std::endl;
    std::cout << "Nb frames per cycle : " << nb_frames_per_cycle << std::endl;
    std::cout << "Nb cycles : " << nb_cycles << std::endl;
    std::cout << "Nb frames : " << nb_frames << std::endl;

#ifdef JUNGFRAU_TEST
    std::cout << "Clock divider : " << clock_divider << std::endl;
#endif

#ifdef EIGER_TEST
    std::cout << "Bit depth : " << bit_depth << std::endl;
    std::cout << "Parallel mode : " << parallel_mode << std::endl;
    std::cout << "Flow control 10G : " << flow_control_10G << std::endl;
    std::cout << "Ten gigabit ethernet : " << ten_gigabit_ethernet << std::endl;
    std::cout << "Enable Gap Pixels : " << enable_gap_pixels << std::endl;
    std::cout << "Sub exposure time : " << sub_exposure_time << std::endl;
    std::cout << "Enable Overflow Mode : " << enable_overflow_mode << std::endl;
#endif

    std::cout << "Estimated frame rate : " << (1.0 / exposure_period) << std::endl;

#ifdef EIGER_DETECTOR_SIMULATION
    last_acquisition_received_frames = nb_frames;
#endif

    if(last_acquisition_received_frames == nb_frames)
    {
        acquisition_nb_ok++;
        return slsDetectorDefs::OK;
    }

    PRINT_SEPARATOR();
    return slsDetectorDefs::FAIL;
}

//------------------------------------------------------------------------------------------------------
// test
//------------------------------------------------------------------------------------------------------
void Test(void)
{
    try
    {
        lima::DebParams::enableTypeFlags(lima::DebType::DebTypeFatal   | 
                                         lima::DebType::DebTypeError   | 
                                         lima::DebType::DebTypeWarning |
                                         lima::DebType::DebTypeTrace   );
        
        PRINT_SEPARATOR();
        std::cout << "CreateReceivers" << std::endl;
        PRINT_SEPARATOR();

        CreateReceivers();
        
        PRINT_SEPARATOR();
        std::cout << "CreateDetector" << std::endl;
        PRINT_SEPARATOR();

        CreateDetector();

        PRINT_SEPARATOR();
        std::cout << "RunAcquisition" << std::endl;
        PRINT_SEPARATOR();

        for(int acquisition_index = 0 ; acquisition_index < acquisition_nb ; acquisition_index++)
        {
            current_acquisition_index = acquisition_index;
            
            cprintf(MAGENTA, "Acquisition number : %d\n", acquisition_index);
            if (RunAcquisition() == slsDetectorDefs::FAIL) {
                acquisition_nb_list.push_back(acquisition_index);
            }
        }
        
        PRINT_SEPARATOR();
        std::cout << "ReleaseDetector" << std::endl;
        PRINT_SEPARATOR();

        ReleaseDetector();
        
        PRINT_SEPARATOR();
        std::cout << "ReleaseReceivers" << std::endl;
        PRINT_SEPARATOR();

        ReleaseReceivers();

        PRINT_SEPARATOR();
	if (acquisition_nb - acquisition_nb_ok)
		cprintf(BOLD RED, "Correct acquisition(s) %d/%d\n", acquisition_nb_ok, acquisition_nb);
	else
		cprintf(BOLD GREEN, "Correct acquisition(s) %d/%d\n", acquisition_nb_ok, acquisition_nb);
        if (acquisition_nb - acquisition_nb_ok) {
            cprintf(RED, "Acquisition(s) gone wrong :\n");
            for (int list_index = 0; list_index < acquisition_nb_list.size(); ++list_index) {
		cprintf(RED, "%d\n", acquisition_nb_list[list_index]);
            }
        }
        PRINT_SEPARATOR();
    }
    catch (...)
    {
        std::cout << "unknown exception!" << std::endl;
		exit(EXIT_FAILURE);
    }
}

std::string roi_result = 
"detector 0:\n"
"0       255     -1      -1\n"
"detector 1:\n"
"1024    1279    -1      -1\n"
"\n"
"xmin    xmax    ymin    ymax\n"
"0       255     -1      -1\n"
"2304    2559    -1      -1\n"
"roi 2\n";

#include <vector>

// use example :
// std::vector<slsReceiverDefs::ROI> rois;
// get_rois_from_string(roi_result, rois);
/*******************************************************************
 * \brief Cuts the string in pieces
 * \param[in] in_string source string
 * \param[in] in_delimitor line delimitor
 * \param[out] out_lines line container result
 *******************************************************************/
void split_string_line(const std::string & in_string, const char in_delimitor, std::vector<std::string> & out_lines)
{
    std::stringstream ss(in_string);
    std::string sub_string;

    while (getline(ss, sub_string, in_delimitor))
    {
        out_lines.push_back(sub_string);
    }
}

/*******************************************************************
 * \brief retrieve the ROIs from a string
 * \param[in] in_rois_string string from "get roi" command
 * \param[out] out_rois ROI container result (empty if no set ROI)
 *******************************************************************/
void get_rois_from_string(const std::string & in_rois_string, std::vector<slsReceiverDefs::ROI> & out_rois)
{
    out_rois.clear();

    try
    {
        // cuts the string in lines
        std::vector<std::string> lines;
        split_string_line(in_rois_string, '\n', lines);

        if(lines.size() >= 1)
        {
            // checks if no ROI ?
            if(lines[0] != "roi 0")
            {
                for(int roi_index = 0 ; roi_index < 2 ; roi_index++)
                {
                    if(lines.size() >= ((roi_index + 1) * 2)) // two lines per ROI definition
                    {
                        std::stringstream detector_name;
                        detector_name << "detector " << roi_index << ":";
                        
                        // checks the first line
                        if(lines[roi_index * 2] == detector_name.str())
                        {
                            std::stringstream ss(lines[(roi_index * 2) + 1]);

                            slsReceiverDefs::ROI roi;
                            ss >> roi.xmin;
                            ss >> roi.xmax;  
                            ss >> roi.ymin;
                            ss >> roi.ymax;

                            out_rois.push_back(roi);
                        }
                    }
                }
            }
        }
    }
    catch(...)
    {
        out_rois.clear();
    }
}

//------------------------------------------------------------------------------------------------------
// read_simple_option
//------------------------------------------------------------------------------------------------------
bool read_simple_option(int argc, char* argv[], const char * in_option_name)
{
    int option_index = 1;

    while(option_index < argc)
    {
        if (strcmp(argv[option_index], in_option_name) == 0)
        {
            std::cout << "Found option:" << in_option_name << std::endl;
            return true;
        }

        option_index++;
    }

    return false;
}

//------------------------------------------------------------------------------------------------------
// read_option_value
//------------------------------------------------------------------------------------------------------
template <typename T> bool read_option_value(int argc, char* argv[], const char * in_option_name, T & out_option_value)
{
    int option_index = 1;

    while(option_index < argc)
    {
        if (strcmp(argv[option_index], in_option_name) == 0)
        {
            option_index++;

            if(option_index < argc)
            {
                std::stringstream ss(std::string(argv[option_index]));
                ss >> out_option_value;
                std::cout << "Found option: " << in_option_name << " " << out_option_value << std::endl;
                return true;
            }
        }

        option_index++;
    }

    return false;
}

//------------------------------------------------------------------------------------------------------
// main
//------------------------------------------------------------------------------------------------------
int main (int argc, char* argv[])
{
    if(read_simple_option(argc, argv, "-help") || read_simple_option(argc, argv, "--help"))
    {
        PRINT_SEPARATOR();
        std::cout << "Options:" << std::endl;
        std::cout << "-clean -> clean shared memory" << std::endl;
        std::cout << "-trace -> activate acquisition log" << std::endl;
        std::cout << "-exp <value> -> set exposure time value in seconds (for example: -exp 0.0005)" << std::endl;
        std::cout << "-period <value> -> set period time value in seconds (for example: -period 0.001)" << std::endl;
        std::cout << "-frames <value> -> set number of frames (for example: -frames 10000)" << std::endl;
        std::cout << "-acq <value> -> set number of acquisition (for example: -acq 10)" << std::endl;
        std::cout << "-bdepth <value> -> (Eiger only) set bit depth [32/16/8/4] (for example: -bdepth 8)" << std::endl;
        std::cout << "-clockdiv <value> -> (Jungfrau & Eiger only) set clock divider [0 (FullSpeed)/1 (HalfSpeed)/2 (QuarterSpeed)/3 (SuperSlowSpeed)] (for example: -clockdiv 1)" << std::endl;
        std::cout << "-pmode <value> -> (Eiger only) set parallel mode [0 (non parallel)/1 (parallel)/2 (safe mode)] (for example: -pmode 1)" << std::endl;
        std::cout << "-subexp <value> -> (Eiger only) set sub frame exposure time value in seconds (for example: -subexp 0.002)" << std::endl;
        std::cout << "-overflow <value> -> (Eiger only) set overflow mode [0 disabled/1 enabled] (for example: -overflow 1)" << std::endl;
        std::cout << std::endl;
        std::cout << "example: ./manual-acq -clean -trace -acq 1 -exp 0.0005 -period 0.001 -frames 1000" << std::endl;
        PRINT_SEPARATOR();
        return 0;
    }

    if(read_simple_option(argc, argv, "-clean"))
    {
        PRINT_SEPARATOR();
        std::cout << "Cleaning shared memory" << std::endl;
        PRINT_SEPARATOR();

        clean_shared_memory();
    }

    if(read_simple_option(argc, argv, "-trace"))
    {
        PRINT_SEPARATOR();
        std::cout << "Activating acquisition log..." << std::endl;
        PRINT_SEPARATOR();

        use_trace = true;
    }

    int64_t frames_value;

    if(read_option_value(argc, argv, "-frames", frames_value))
    {
        detector_nb_frames_per_cycle = frames_value;
    }

    double exp_value;

    if(read_option_value(argc, argv, "-exp", exp_value))
    {
        detector_exposure_time_sec = exp_value;
    }

    double period_value;

    if(read_option_value(argc, argv, "-period", period_value))
    {
        detector_exposure_period_sec = period_value;
    }

    int acq_nb;

    if(read_option_value(argc, argv, "-acq", acq_nb))
    {
        acquisition_nb = acq_nb;
    }

#if defined(JUNGFRAU_TEST) || defined(EIGER_TEST)
    int clock_divider;

    if(read_option_value(argc, argv, "-clockdiv", clock_divider))
    {
        detector_clock_divider = clock_divider;
    }
#endif

#ifdef EIGER_TEST
    int bit_depth;

    if(read_option_value(argc, argv, "-bdepth", bit_depth))
    {
        detector_bit_depth = bit_depth;
    }

    int parallel_mode;

    if(read_option_value(argc, argv, "-pmode", parallel_mode))
    {
        detector_parallel_mode = parallel_mode;
    }

    double sub_exp;

    if(read_option_value(argc, argv, "-subexp", sub_exp))
    {
        detector_sub_exposure_time_sec = sub_exp;
    }

    int enable_overflow_mode;

    if(read_option_value(argc, argv, "-overflow", enable_overflow_mode))
    {
        detector_enable_overflow_mode = enable_overflow_mode;
    }
#endif

    Test();

    std::cout << "====================== ENDING ======================" << std::endl;

    return 0;
}

//------------------------------------------------------------------------------------------------------
   /**
    * set flow control for 10Gbe (Eiger only)
    * @param i 1 sets, 0 unsets (-1 gets)
    * @return flow control enable for 10 Gbe
    */
//   int setFlowControl10G(int i = -1);

   /**
    * enable/disable 10GbE (Eiger only)
    * @param i 1 sets, 0 unsets (-1 gets)
    * @return 10GbE enable
    */
//   int setTenGigabitEthernet(int i = -1);

/*
• flags continuous/storeinram. Allows to take frame continuously or
storing them on memory. Users should use the continuous flags. Enabling
the stroreinram flag makes the data to be sent out all at the end of the
acquisition. Refer to readout timing specifications in section ?? for how
to set the detector. Examples will be given in section ??.
One should notice that, by default, by choosing the option dr 32, then the
software automatically sets the detector to clkdivider 2. By choosing the
option dr 16, the software automatically sets the detector to clkdivider 1.
One needs to choose clkdivider 0 after setting the dr 16 option to have the
fastest frame rate. We would recommend expert users (beamline people) to
write their parameters file for the users.

*/
