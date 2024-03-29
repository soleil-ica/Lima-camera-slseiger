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
 *  \file   SlsEigerReceivers.h
 *  \brief  SlsEiger detector acquisition receivers controller class implementation 
 *  \author C�dric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include <cstdlib>
#include <fstream> // for std::ifstream
#include <algorithm>

#include "lima/Exceptions.h"
#include "lima/RegExUtils.h"

#include "SlsEigerReceivers.h"
#include "SlsEigerDetector.h"

#include <sls_receiver_defs.h>
#include <slsReceiverUsers.h>

using namespace lima;
using namespace lima::SlsEiger;

// activate this define to trace the sdk callbacks 
//#define CAMERA_RECEIVERS_ACTIVATE_SDK_CALLBACK_TRACE

/************************************************************************
 * \brief constructor
 ************************************************************************/
Receivers::Receivers(Detector & detector) : m_detector(detector)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Receivers::Receivers - BEGIN";

    m_receivers    = NULL;
    m_nb_receivers = 0   ;

    DEB_TRACE() << "Receivers::Receivers - END";
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
Receivers::~Receivers()
{
    DEB_DESTRUCTOR();

    DEB_TRACE() << "Receivers::~Receivers - BEGIN";

    // c-array which contains elements used as user data in sls sdk callbacks
    if(m_receivers != NULL)
    {
        delete [] m_receivers;
        m_receivers = NULL;
    }

    DEB_TRACE() << "Receivers::~Receivers - END";
}

/************************************************************************
 * \brief inits the receivers using the configuration file name
 * \param in_config_file_name complete path to the configuration file
 ************************************************************************/
void Receivers::init(const std::string & in_config_file_name)
{
    DEB_MEMBER_FUNCT();

    std::vector<Receiver> receivers;

    int receiver_index;

    // preparing the args for receivers creation
    char        temp_port[10];
    const int   argc         = 3;
    char      * args[argc]   = {(char*)"slsReceiver", (char*)"--rx_tcpport", temp_port};

    // parsing the config file to build the receivers informations container
    createReceivers(in_config_file_name, receivers);

    //------------------------------------------------------------------------------------
    // creating the receivers instances 
    for(receiver_index = 0 ; receiver_index < receivers.size() ; receiver_index++)
    {
    	int ret = slsReceiverDefs::OK;

        // changing the udp port in the args
        sprintf(temp_port, "%d", receivers[receiver_index].getTcpipPort());

        // creating the receiver using the args
        DEB_TRACE() << "creating the receiver "
                    << "(" << receiver_index << ") "
                    << receivers[receiver_index].getHostName() << " - "
                    << "tcpip port (" << receivers[receiver_index].getTcpipPort() << ")";

        slsReceiverUsers * receiver = new slsReceiverUsers(argc, args, ret);

        // managing a failed result
        if(ret == slsReceiverDefs::FAIL)
        {
            // free the memory
            delete receiver;

            THROW_HW_FATAL(ErrorType::Error) << "Receivers::init failed! " 
                                             << "Could not create the sls receiver (" 
                                             << receiver_index << ")";
        }
        else
        {
            DEB_TRACE() << "receiver created - version (" << receiver->getReceiverVersion() << ")";

            // in case of success, we set the receiver in the receivers informations container
            receivers[receiver_index].setReceiver     (receiver);
            receivers[receiver_index].setReceiverIndex(receiver_index);
            receivers[receiver_index].setReceivers    (this);
        }
    }

    //------------------------------------------------------------------------------------
    // allocating and initing the c-array for using user data in the sls sdk callbacks
    m_receivers    = new Receiver[receivers.size()];
    m_nb_receivers = receivers.size();
    
    // making cases copy
    for(receiver_index = 0 ; receiver_index < m_nb_receivers ; receiver_index++)
    {
        m_receivers[receiver_index] = receivers[receiver_index];
        receivers[receiver_index].setReceiver(NULL);
    }

    receivers.clear(); // no need to keep the container content

    // setting the callbacks for the receivers
    for(receiver_index = 0 ; receiver_index < m_nb_receivers ; receiver_index++)
    {
        slsReceiverUsers * receiver = m_receivers[receiver_index].getReceiver(); // alias

        DEB_TRACE() << "registering the receiver " << "(" << receiver_index << ") callbacks:";

        // Receiver ptr is given to the register methods for access to user data
        Receiver * user_data = m_receivers + receiver_index;

        // callback for start acquisition
        DEB_TRACE() << "registering StartAcq()";
        receiver->registerCallBackStartAcquisition(startedAcquisitionCallBack, user_data); 

        // callback for finished acquisition
        DEB_TRACE() << "registering finishedAcquisition()";
        receiver->registerCallBackAcquisitionFinished(finishedAcquisitionCallBack, user_data);

        // callback for raw data */
        DEB_TRACE() << "registering GetData()";
        receiver->registerCallBackRawDataReady(acquisitionDataReadyCallBack, user_data);
    }

    //------------------------------------------------------------------------------------
    // starting the receivers
    for(receiver_index = 0 ; receiver_index < m_nb_receivers ; receiver_index++)
    {
        slsReceiverUsers * receiver = m_receivers[receiver_index].getReceiver(); // alias

        if (receiver->start() == slsReceiverDefs::FAIL)
        {
            // free the memory
            delete receiver;
            m_receivers[receiver_index].setReceiver(NULL);

            THROW_HW_FATAL(ErrorType::Error) << "Receivers::init failed! " 
                                             << "Could not start the sls receiver (" 
                                             << receiver_index << ")";
        }
    }
}

/************************************************************************
 * \brief get the number of receivers
 * \return number of receivers
 ************************************************************************/
std::size_t Receivers::getNumberOfReceivers(void)
{
    return m_nb_receivers;
}

//==================================================================
// internal callbacks management
//==================================================================
/************************************************************************
 * \brief Started acquisition management
 * \param m_receiver_index receiver index
 ************************************************************************/
void Receivers::startedAcquisition(int in_receiver_index)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Receivers::startedAcquisition";
}

/************************************************************************
 * \brief Finished acquisition management
 * \param m_receiver_index receiver index
 * \param in_frames_nb Number of frames caught
 ************************************************************************/
void Receivers::finishedAcquisition(int      in_receiver_index,
                                    uint64_t in_frames_nb     )
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Receivers::finishedAcquisition";
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
void Receivers::acquisitionDataReady(const int      in_receiver_index,
                                     uint64_t       in_frame_index   ,
                                     const int      in_pos_x         ,
                                     const int      in_pos_y         ,
                                     const uint32_t in_packet_number ,
                                     const uint64_t in_timestamp     ,
                                     const char *   in_data_pointer  ,
                                     const uint32_t in_data_size     )
{
    // the detector will manage the new frame
    m_detector.acquisitionDataReady(in_receiver_index, 
                                    in_frame_index   ,
                                    in_pos_x         ,
                                    in_pos_y         ,
                                    in_packet_number ,
                                    in_timestamp     ,
                                    in_data_pointer  ,
                                    in_data_size     );
}

//==================================================================
// configuration file parsing
//==================================================================
/************************************************************************
 * \brief creates the receivers container using the configuration file
 * \param in_config_file_name complete path to the configuration file
 * \param out_receivers       receivers container result
 ************************************************************************/
void Receivers::createReceivers(const std::string     & in_config_file_name,
                                std::vector<Receiver> & out_receivers      )
{
    DEB_MEMBER_FUNCT();

    //-------------------------------------------------------------------------
    // regex used in this method :
    //
    // we need to duplicate anti-slash chars in a c string: \s -> \\s
    //-------------------------------------------------------------------------
    //
    // regex for hostname label :
    // the hostname label starts at the beginning of a line: ^
    // there is a space char after the hostname string: \s
    // the search result is all hostnames and is under parentheses.
    const std::string regex_hostname_label = "^hostname\\s([A-Za-z0-9\\+]+)$";

    // regex for hostname value :
    // a value can ends with a '+' char or an end of line
    // the search result is each hostname and is under parentheses.
    const std::string regex_hostname_value = "([A-Za-z0-9]+)[\\+|$]*";

    // regex for rx_tcpport index :
    // the search result is the controller index and is under parentheses.
    const std::string regex_tcpport_index = "^([0-9]+):rx_tcpport\\s[0-9]+$";
    
    // regex for rx_tcpport port :
    // the search result is the tecpip port and is under parentheses.
    const std::string regex_tcpport_port = "^[0-9]+:rx_tcpport\\s([0-9]+)$";

    //-------------------------------------------------------------------------
    lima::RegEx re;
    lima::RegEx::MatchListType match_list; // for multiple results
    lima::RegEx::FullMatchType full_match; // for single result

    // opening the config file in read only text mode
    std::ifstream file(in_config_file_name.c_str(), std::ifstream::in);
    std::string   line;

    DEB_TRACE() << "createReceivers - starting to parse config file : " << in_config_file_name;

    // parsing the file
    while (std::getline(file, line))
    {
        // we need to remove leading, trailing and extra spaces to facilitate the next steps
        line = trimString(line);

        // we are searching a line which contains hostname to get the controllers list
        re = regex_hostname_label;

		if (re.match(line, full_match)) 
        {
            DEB_TRACE() << "config matching line found : " << line;

            // we are searching the hostnames values
            std::string values = std::string(full_match[1]);

            re = regex_hostname_value;
            re.multiSearch(values, match_list);

		    if (!match_list.empty())
            {
			    for (std::size_t receiver_index = 0 ; receiver_index < match_list.size(); receiver_index++)
                {
                    // we set the data in the receiver info container
                    manageReceiversResize(receiver_index, out_receivers);
                    out_receivers[receiver_index].setHostName(std::string(match_list[receiver_index][1]));

                    DEB_TRACE() << "hostname (" 
                                << receiver_index << "):" 
                                << out_receivers[receiver_index].getHostName();
                }

                // no need to check other option, we jump to next line
                continue;
            }
            else
            {
                THROW_HW_FATAL(ErrorType::InvalidValue) 
                    << "createReceivers failed! Could not initialize the camera!";
            }
        }

        // communication port between client and receiver and controller index
        re = regex_tcpport_index;

	    if (re.match(line, full_match)) 
        {
            std::string index = std::string(full_match[1]);
            int         receiver_index;
            
            DEB_TRACE() << "config matching line found : " << line;
            DEB_TRACE() << "receiver index : " << index;

            // conversion of string to int to get the controller index
            receiver_index = convertStringToInteger(index, "controller id");

            // getting the tcpip port 
            re = regex_tcpport_port;

	        if (re.match(line, full_match)) 
            {
                std::string tcpip_port = std::string(full_match[1]);
                int         receiver_tcpip_port;

                DEB_TRACE() << "receiver tcpip port : " << tcpip_port;

                // conversion of string to int to get the controller index
                receiver_tcpip_port = convertStringToInteger(tcpip_port, "controller tcpip port");

                // we set the data in the receiver info container
                manageReceiversResize(receiver_index, out_receivers);
                out_receivers[receiver_index].setTcpipPort(receiver_tcpip_port);
            }

            // no need to check other option, we jump to next line
            continue;
        }
    }

    // check if there is at least one set receiver
    if (out_receivers.empty())
    {
        THROW_HW_FATAL(ErrorType::Error) 
            << "readConfigurationFile failed! Please set correctly the hostname.";
    }
    else
    {
        // logging the result
        DEB_TRACE() << "Receivers list";

	    for (std::size_t receiver_index = 0 ; receiver_index < out_receivers.size(); receiver_index++)
        {
            DEB_TRACE() << "hostname ("   << out_receivers[receiver_index].getHostName()  << ")"
                        << " - "
                        << "tcpip port (" << out_receivers[receiver_index].getTcpipPort() << ")"; 
        }
    }

    DEB_TRACE() << "createReceivers - end of parsing";
}

/************************************************************************
 * \brief removes leading, trailing and extra spaces
 * \param in_string string which will be processed
 * \return trimmed string
 ************************************************************************/
std::string Receivers::trimString(const std::string & in_string) const
{
    std::string temp  = in_string;
    std::string result;

    // in case of empty string, returning immediately
    if (temp.empty())
        return temp;

    // replacing tabulations by spaces
    std::replace(temp.begin(), temp.end(), '\t', ' ');

    // removing leading white spaces
    // is the first char a white space ?
    while (!temp.empty() && std::isspace(temp[0])) 
        temp.erase(temp.begin()); // erasing the first char

    // removing trailing white spaces
    // is the last char a white space ?
    while (!temp.empty() && std::isspace(temp[temp.size() - 1]))
        temp.erase(temp.end()-1); // erasing the last char

    // removing extra spaces
    std::size_t char_index          = 0;
    bool        is_previous_a_space = false;

    while(char_index < temp.size())
    {
        // if the current char is not a space or the previous was not a space
        if(!std::isspace(temp[char_index]) || (!is_previous_a_space))
        {
            result += temp[char_index];
        }

        is_previous_a_space = std::isspace(temp[char_index]);
        char_index++;
    }

    return result;
}

/************************************************************************
 * \brief converts a string to an integer and checks the value.
 *        Throws an exception in case of conversion error.
 * \param in_value string value to be converted
 * \param in_label label used for log purpose
 * \return converted integer value
 ************************************************************************/
int Receivers::convertStringToInteger(const std::string & in_value,
                                      const std::string & in_label) const
{
    DEB_MEMBER_FUNCT();

    std::istringstream ss(in_value);
    int                result;
    
    // conversion of string to int
    ss >> result;

    if (result < 0)
    {
        THROW_HW_FATAL(ErrorType::InvalidValue)
            << "readConfigurationFile failed! " 
            << in_label << " (" << in_value << ") is invalid!";
    }

    return result;
}

/************************************************************************
 * \brief automatically resizes the receivers container 
 *        because elements can be inserted in random order 
 * \param in_element_index index of the element we need to set
 * \param in_out_receivers receivers container
 ************************************************************************/
void Receivers::manageReceiversResize(const std::size_t     & in_element_index,
                                      std::vector<Receiver> & in_out_receivers)
{
    if(in_element_index >= in_out_receivers.size())
    {
        in_out_receivers.resize(in_element_index + 1);
    }
}

//==================================================================
// sls sdk callbacks
//==================================================================
// Defines colors to print data call back in different colors for different recievers
#define PRINT_IN_COLOR(c,f, ...) 	printf ("\033[%dm" f RESET, 30 + c+1, ##__VA_ARGS__)

/************************************************************************
 * \brief Start Acquisition Call back
 *        slsReceiver writes data if file write enabled.
 *        Users get data to write using call back if 
 *        registerCallBackRawDataReady is registered.
 * \param in_file_path file path
 * \param in_file_name file name
 * \param in_file_index file index
 * \param in_data_size data size in bytes
 * \param in_user_data pointer to user data object
 * \return ignored
 ************************************************************************/
int Receivers::startedAcquisitionCallBack(char     * in_file_path ,
                                          char     * in_file_name ,
                                          uint64_t   in_file_index,
                                          uint32_t   in_data_size ,
                                          void     * in_user_data )
{
#ifdef CAMERA_RECEIVERS_ACTIVATE_SDK_CALLBACK_TRACE
	cprintf(BLUE, "#### startedAcquisitionCallBack:  filepath:%s  filename:%s fileindex:%llu  datasize:%u ####\n",
			in_file_path, in_file_name, in_file_index, in_data_size);
#endif

    // we call the internal management of the callback using the user data
    // the user data allows to have access to the 
    // Receivers instance smart pointer and to the receiver index.
    Receiver * user_data = static_cast<Receiver *>(in_user_data);
    user_data->getReceivers()->startedAcquisition(user_data->getReceiverIndex());

	return 0;
}

/************************************************************************
 * \brief Acquisition Finished Call back
 * \param in_frames_nb Number of frames caught
 * \param in_user_data pointer to user data object
 ************************************************************************/
void Receivers::finishedAcquisitionCallBack(uint64_t   in_frames_nb,
                                            void     * in_user_data)
{
#ifdef CAMERA_RECEIVERS_ACTIVATE_SDK_CALLBACK_TRACE
	cprintf(BLUE, "#### finishedAcquisition: frames:%llu ####\n",in_frames_nb);
#endif

    // we call the internal management of the callback using the user data
    // the user data allows to have access to the 
    // Receivers instance smart pointer and to the receiver index.
    Receiver * user_data = static_cast<Receiver *>(in_user_data);
    user_data->getReceivers()->finishedAcquisition(user_data->getReceiverIndex(), in_frames_nb);
}

/************************************************************************
 * \brief Get Receiver Data Call back
 * \param in_metadata sls_receiver_header metadata
 * \param in_data_pointer pointer to data
 * \param in_data_size data size in bytes
 * \param in_user_data pointer to user data object
 ************************************************************************/
void Receivers::acquisitionDataReadyCallBack(char     * in_meta_data   ,
                                             char     * in_data_pointer,
                                             uint32_t   in_data_size   ,
                                             void     * in_user_data   )
{
    slsReceiverDefs::sls_receiver_header* header = (slsReceiverDefs::sls_receiver_header*)in_meta_data;
	const slsReceiverDefs::sls_detector_header & detector_header = header->detHeader;

#ifdef CAMERA_RECEIVERS_ACTIVATE_SDK_CALLBACK_TRACE
	PRINT_IN_COLOR (detector_header.modId?detector_header.modId:detector_header.row,
			        "#### %d GetData: ####\n"
			        "frameNumber: %llu\t\texpLength: %u\t\tpacketNumber: %u\t\tbunchId: %llu"
			        "\t\ttimestamp: %llu\t\tmodId: %u\t\t"
			        "row: %u\t\tcolumn: %u\t\treserved: %u\t\tdebug: %u"
			        "\t\troundRNumber: %u\t\tdetType: %u\t\tversion: %u"
			        //"\t\tpacketsMask:%s"
			        "\t\tfirstbytedata: 0x%x\t\tdatsize: %u\n\n",
			        detector_header.row, 
                    (long long unsigned int)detector_header.frameNumber,
			        detector_header.expLength, 
                    detector_header.packetNumber, 
                    (long long unsigned int)detector_header.bunchId,
			        (long long unsigned int)detector_header.timestamp,
                    detector_header.modId,
			        detector_header.row,
                    detector_header.column,
                    detector_header.reserved,
			        detector_header.debug,
                    detector_header.roundRNumber,
			        detector_header.detType,
                    detector_header.version,
			        //header->packetsMask.to_string().c_str(),
                    ((uint8_t)(*((uint8_t*)(in_data_pointer)))),
                    in_data_size);
#endif

    if((in_data_pointer != NULL) && (in_data_size > 0) && (in_user_data != NULL))
    {
        // we call the internal management of the callback using the user data
        // the user data allows to have access to the 
        // Receivers instance smart pointer and to the receiver index.
        Receiver * user_data = static_cast<Receiver *>(in_user_data);
        user_data->getReceivers()->acquisitionDataReady(user_data->getReceiverIndex(),
                                                        detector_header.frameNumber-1, // frameNumber starts at 1
                                                        detector_header.column       ,
                                                        detector_header.row          ,
                                                        detector_header.packetNumber ,
                                                        detector_header.timestamp    ,
                                                        in_data_pointer              ,
                                                        in_data_size                 );
    }
}

//========================================================================================
