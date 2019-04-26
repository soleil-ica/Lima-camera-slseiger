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
 *  \file   SlsEigerCameraFramePart.h
 *  \brief  SlsEiger detector frame part class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSEIGERCAMERAFRAMEPART_H
#define SLSEIGERCAMERAFRAMEPART_H

// SYSTEM
#include <cstddef>
#include <stdint.h>
#include <list>

// LIMA
#include "lima/Debug.h"
#include "lima/Constants.h"

// YAT/YAT4TANGO
#include <yat/memory/SharedPtr.h>

namespace lima
{
    namespace SlsEiger
    {
        /***********************************************************************
         * \class CameraFramePart
         * \brief defines an info class used to store frame part informations 
         ***********************************************************************/

        class CameraFramePart
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraFramePart", "SlsEiger");

        public:
            //==================================================================
            // constructor with parameters
            CameraFramePart(const int         in_pos_x         ,  // horizontal position of the part in the final image (starts at 0)
                            const int         in_pos_y         ,  // vertical position of the part in the final image (starts at 0)
                            const uint32_t    in_packet_number ,  // number of packets caught for this frame
                            const uint64_t    in_timestamp     ,  // time stamp 
                            const uint8_t  *  in_data_pointer  ,  // frame part image pointer
                            const std::size_t in_data_byte_size); // number of bytes in the frame part

            // destructor (no need to be virtual)
            ~CameraFramePart();

            //==================================================================
            // gets the horizontal position of the part in the final image
            int getPosx() const;

            // gets the vertical position of the part in the final image
            int getPosy() const;

            // gets the packet number
            uint32_t getPacketNumber() const;

            // gets the timestamp
            uint64_t getTimestamp() const;

            // gets the frame part data pointer
            const uint8_t * getDataPointer() const;

            // gets the frame part data pointer for writing
            uint8_t * getDataWritePointer();

            // gets the size in byte of the frame part
            std::size_t getDataByteSize() const;

        private:
            int         m_pos_x         ; // horizontal position of the part in the final image (starts at 0)
            int         m_pos_y         ; // vertical position of the part in the final image (starts at 0)
            uint32_t    m_packet_number ; // number of packets caught for this frame part
            uint64_t    m_timestamp     ; // time stamp
            uint8_t  *  m_data_pointer  ; // frame part image pointer
            std::size_t m_data_byte_size; // number of bytes in the frame part
        }; 

        /***********************************************************************
         * Frame parts containers types
         ***********************************************************************/
        // type of container which contains a list of frame parts
        typedef std::list<yat::SharedPtr<CameraFramePart> > FramePartsContainer;
    }
}
#endif // SLSEIGERCAMERAFRAMEPART_H

/*************************************************************************/