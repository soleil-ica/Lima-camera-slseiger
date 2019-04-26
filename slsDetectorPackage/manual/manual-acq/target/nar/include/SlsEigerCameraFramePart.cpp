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
 *  \file   CameraFramePart.h
 *  \brief  detector CameraFramePart class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include "SlsEigerCameraFramePart.h"

// SYSTEM
#include <cstring>

using namespace lima::SlsEiger;

/************************************************************************
 * \brief constructor with parameters
 * \param in_pos_x horizontal position of the part in the final image (starts at 0)
 * \param in_pos_y vertical position of the part in the final image (starts at 0)
 * \param in_timestamp time stamp
 * \param in_data_pointer frame part image pointer
 * \param in_data_byte_size number of bytes in the frame part
 ************************************************************************/
CameraFramePart::CameraFramePart(const int         in_pos_x         ,
                                 const int         in_pos_y         ,
                                 const uint32_t    in_packet_number ,
                                 const uint64_t    in_timestamp     ,
                                 const uint8_t  *  in_data_pointer  ,
                                 const std::size_t in_data_byte_size)
{
    m_pos_x          = in_pos_x         ;
    m_pos_y          = in_pos_y         ;
    m_packet_number  = in_packet_number ;
    m_timestamp      = in_timestamp     ;
    m_data_pointer   = NULL             ;
    m_data_byte_size = 0                ;

    // make a copy of the frame part image
    if (in_data_byte_size > 0)
    {
        m_data_byte_size = in_data_byte_size;
        m_data_pointer   = new uint8_t[m_data_byte_size];

        if (in_data_pointer != NULL)
        {
            std::memcpy(static_cast<void*>(m_data_pointer), static_cast<const void*>(in_data_pointer), m_data_byte_size);
        }
        else
        {
            std::memset(static_cast<void*>(m_data_pointer), 0, m_data_byte_size);
        }
    }
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraFramePart::~CameraFramePart()
{
    if(m_data_pointer != NULL)
    {
        delete [] m_data_pointer;  
    }
}

/************************************************************************
 * \brief gets the horizontal position of the part in the final image
 * \return posx value
 ************************************************************************/
int CameraFramePart::getPosx() const
{
    return m_pos_x;
}

/************************************************************************
 * \brief gets the vertical position of the part in the final image
 * \return posy value
 ************************************************************************/
int CameraFramePart::getPosy() const
{
    return m_pos_y;
}

/************************************************************************
 * \brief gets the packet number
 * \return packet number value
 ************************************************************************/
uint32_t CameraFramePart::getPacketNumber() const
{
    return m_packet_number;
}

/************************************************************************
 * \brief gets the timestamp
 * \return timestamp value
 ************************************************************************/
uint64_t CameraFramePart::getTimestamp() const
{
    return m_timestamp;
}

/************************************************************************
 * \brief gets the frame part data pointer
 * \return frame part data pointer value
 ************************************************************************/
const uint8_t * CameraFramePart::getDataPointer() const
{
    return m_data_pointer;
}

/************************************************************************
 * \brief gets the frame part data pointer for writing
 * \return frame part data pointer value
 ************************************************************************/
uint8_t * CameraFramePart::getDataWritePointer()
{
    return m_data_pointer;
}

/************************************************************************
 * \brief gets the number of bytes in the frame part
 * \return number of bytes in the frame part
 ************************************************************************/
std::size_t CameraFramePart::getDataByteSize() const
{
    return m_data_byte_size;
}

//========================================================================================
