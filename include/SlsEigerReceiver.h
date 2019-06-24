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
 *  \file   SlsEigerReceiver.h
 *  \brief  SlsEiger receiver informations class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSEIGERRECEIVER_H
#define SLSEIGERARECEIVER_H

#include "lima/Debug.h"
#include "lima/Constants.h"

/**********************************************************************/
// defines the SLS slsReceiverUsers class
// Class for implementing the SLS data receiver in the users application.
// Callbacks can be defined for processing and/or saving data
class slsReceiverUsers;

namespace lima
{
    namespace SlsEiger
	{
	    /**********************************************************************/
	    // pre-defines the Receivers class for Receiver friend links
	    class Receivers;
	
	    /***********************************************************************
	     * \class Receiver
	     * \brief used to store Receiver informations
	     *
	     * defines an info class used to store Receiver informations 
	     * which were read from the configuration file.
	     *
	     * allows the callbacks :
	     * - to access to the Receivers object
	     * - to know the receiver index which is not always given in
	     *   the callback data
	     ***********************************************************************/
	
        class Receiver
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Receiver", "SlsEiger");

        public:
            //==================================================================
            // constructor
            Receiver();

            // destructor (no need to be virtual)
            ~Receiver();

            //==================================================================
            // gets the receiver
            slsReceiverUsers * getReceiver();

            // sets the receiver
            void setReceiver(slsReceiverUsers * in_receiver);

            // gets the host name
            const std::string & getHostName() const;

            // sets the host name
            void setHostName(const std::string & in_host_name);

            // gets the tcpip port
            int getTcpipPort() const;

            // sets the tcpip port
            void setTcpipPort(const int in_tcpip_port);

            // gets the access to the Receivers object 
            Receivers * getReceivers();

            // sets the access to the Receivers object
            void setReceivers(Receivers * in_receivers);

            // gets the receiver index
            int getReceiverIndex() const;

            // sets the receiver index
            void setReceiverIndex(const int in_receiver_index);
            
        private:
            slsReceiverUsers * m_receiver      ; // sls receiver instance from sls sdk

            std::string        m_host_name     ; // receiver host name  (part of the hostname value in config file)
            int                m_tcpip_port    ; // receiver tcpip port (value of rx_tcpport in config file)

            int                m_receiver_index; // receiver index in m_receivers_info container
            Receivers *        m_receivers     ; // direct access to the Receivers object
        }; 

    }
}
#endif // SLSEIGERRECEIVER_H

/*************************************************************************/