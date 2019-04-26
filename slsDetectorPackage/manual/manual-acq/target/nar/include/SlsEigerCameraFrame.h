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
 *  \file   SlsEigerCameraFrame.h
 *  \brief  SlsEiger detector frame class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSEIGERCAMERAFRAME_H
#define SLSEIGERCAMERAFRAME_H

// SYSTEM
#include <cstddef>
#include <stdint.h>
#include <vector>
#include <list>
#include <map>

#include "SlsEigerCameraFramePart.h"

namespace lima
{
    namespace SlsEiger
    {
        /***********************************************************************
         * \class CameraFrame
         * \brief defines an info class used to store frame informations 
         ***********************************************************************/

        class CameraFrame
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraFrame", "SlsEiger");

        public:
            //==================================================================
            // constructor without parameter
            CameraFrame(uint64_t in_index);

            // destructor (no need to be virtual)
            ~CameraFrame();

            //==================================================================
            // gets the frame index
            uint64_t getIndex() const;

            // gets the timestamp
            uint64_t getTimestamp() const;

            // sets the timestamp
            void setTimestamp(uint64_t in_timestamp);

            // gets the parts
            FramePartsContainer & getParts();

            // gets the number of frame parts
            int getPartsNumber();

            // clear the parts when they are no more needed
            void clearParts();

            // add a frame part to the frame
            void addPart(yat::SharedPtr<CameraFramePart> in_part);

        private:
            uint64_t            m_index        ; // frame index (starts at 0)
            uint64_t            m_timestamp    ; // time stamp
            FramePartsContainer m_parts        ; // container of frame parts
        }; 

        /***********************************************************************
         * Frames containers types
         ***********************************************************************/
        // type of container which contains a map of frames
	    typedef std::map<uint64_t, yat::SharedPtr<CameraFrame> > FramesMapContainer;

        // type of container which contains a list of frames
	    typedef std::list<yat::SharedPtr<CameraFrame> > FramesListContainer;

        // type of container which contains a list of frame indexes
	    typedef std::list<uint64_t > FrameIndexesListContainer;
    }
}
#endif // SLSEIGERCAMERAFRAME_H

/*************************************************************************/