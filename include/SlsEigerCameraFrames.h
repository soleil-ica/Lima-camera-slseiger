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
 *  \file   SlsEigerCameraFrames.h
 *  \brief  SlsEiger detector frames manager class interface 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/********************************************************************************/

#ifndef SLSEIGERCAMERAFRAMES_H
#define SLSEIGERCAMERAFRAMES_H

#include "lima/Debug.h"
#include "lima/Constants.h"

#include <vector>
#include <list>

#include "SlsEigerCompatibility.h"
#include "SlsEigerCameraFrame.h"

namespace lima
{
    namespace SlsEiger
    {
        /***********************************************************************
         * \class CameraFrames
         * \brief used to manage the sls frames
         ***********************************************************************/

        class CameraFrames
        {
            DEB_CLASS_NAMESPC(DebModCamera, "CameraFrames", "SlsEiger");

        public:
            //==================================================================
            // constructor
            CameraFrames(int         in_complete_frame_parts_number,
                         uint32_t    in_frame_part_packet_number   ,
                         std::size_t in_color_depth_bytes_nb       ,
                         std::size_t in_frame_size_x               ,
                         std::size_t in_frame_size_y               ,
                         std::size_t in_frame_part_size_x          ,
                         std::size_t in_frame_part_size_y          );

            // destructor (no need to be virtual)
            ~CameraFrames();

            //==================================================================
            // get the number of bytes for the current color depth
            std::size_t getColorDepthBytesNb();

            // get the number of horizontal pixels for a complete frame
            std::size_t getFrameSizex();

            // get the number of vertical pixels for a complete frame
            std::size_t getFrameSizey();

            // get the number of horizontal pixels for a frame part
            std::size_t getFramePartSizex();

            // get the number of vertical pixels for a frame part
            std::size_t getFramePartSizey();

            // get the gap pixels management activation state
            bool getEnableGapPixels() const;

            // set the gap pixels management activation state
            void setEnableGapPixels(bool in_enable_gap_pixels);

            // Gets the image width
            unsigned short getWidth() const;

            // Gets the image height
            unsigned short getHeight() const;

            // clear the containers
            void clear();

            // compute the relative timestamp (which starts at 0)
            uint64_t computeRelativeTimestamp(const uint64_t in_absolute_timestamp);

            // add a new received frame part
            void addFramePart(uint64_t                         in_frame_index,
                              yat::SharedPtr<CameraFramePart>  in_frame_part );

            // build the image
            bool buildImage(yat::SharedPtr<CameraFrame> in_frame, char * in_image_buffer, std::size_t in_image_buffer_byte_size);

            // get the first frame waiting to be merged - It does not remove the frame from the container.
            void getFirstNotMerged(yat::SharedPtr<CameraFrame> & out_frame);

            // move the first merged frame to the treated frame container
            void moveFirstNotMergedToTreated();

            // get the number of frames in the containers
            void getNbFrames(std::size_t & out_received  , 
                             std::size_t & out_not_merged,
                             std::size_t & out_treated   ) const;

            // get the number of treated frames in the container
            std::size_t getNbTreatedFrames() const;

            // get the number of not treated frames in the containers
            std::size_t getNbNotTreatedFrames() const;

            // checks if there is no more frame to treat in the containers
            bool NoMoreFrameToTreat() const;

        private:
            // creates an autolock mutex for containers methods access
            lima::AutoMutex containersLock() const;

            // copy a part of the image with gap pixels not filled
            template<typename T> void copyGapFramePart(const uint8_t * in_source_buffer     ,
                                                       uint8_t *       in_destination_buffer,
                                                       int             in_x                 ,
                                                       int             in_y                 );

            // fill a part of the image with gap pixels
            template<typename T> bool fillPart(uint8_t * in_destination_buffer, int x, int y);

            // Fill the gap pixels for the image of a chip
            template<typename T> void FillGapOfChip(uint8_t * in_destination_buffer,
                                                    bool      in_top_border        ,
                                                    bool      in_left_border       ,
                                                    bool      in_right_border      ,
                                                    bool      in_bottom_border     );

        private:
            //==================================================================
            // important : a frame should always be in one of these three containers.
            // container which contains the frames not complete
            FramesMapContainer   m_received_frames;

            // container which contains the frames not merged and not passed to Lima
            FramesListContainer  m_not_merged_frames; 

            // container which contains the frames already passed to Lima newFrameReady method
            FramesListContainer  m_treated_frames;

            // container which contains the received frame indexes
            // used to delete the previous incomplete frames when we can build a complete frame
    	    FrameIndexesListContainer m_received_frame_indexes;

            //==================================================================
            int         m_complete_frame_parts_number; // number of frame parts to build a complete frame
            uint32_t    m_frame_part_packet_number   ; // number of paquets we should receive for each frame part
            std::size_t m_color_depth_bytes_nb       ; // number of bytes for the current color depth
            std::size_t m_frame_size_x               ; // number of horizontal pixels for a complete frame
            std::size_t m_frame_size_y               ; // number of vertical pixels for a complete frame
            std::size_t m_frame_part_size_x          ; // number of horizontal pixels for a frame part
            std::size_t m_frame_part_size_y          ; // number of vertical pixels for a frame part
            std::size_t m_chip_size_x                ; // number of horizontal pixels for a chip
            std::size_t m_chip_size_y                ; // number of vertical pixels for a chip

            bool        m_enable_gap_pixels    ; // is the gap pixels management activated ?
            std::size_t m_gap_frame_size_x     ; // number of horizontal pixels for a complete frame with filled gap pixels
            std::size_t m_gap_frame_size_y     ; // number of vertical pixels   for a complete frame with filled gap pixels
            std::size_t m_gap_frame_part_size_x; // number of horizontal pixels for a frame part with filled gap pixels
            std::size_t m_gap_frame_part_size_y; // number of vertical pixels   for a frame part with filled gap pixels

            //==================================================================
            // used to protect the containers access
            // mutable keyword is used to allow const methods even if they use this class member
            mutable lima::Cond m_containers_cond;

            //==================================================================
            // true if the first frame index has been received, else false.
            bool m_is_first_frame_received;

            // the frame timestamp in the sls data callback is not reset for each new acquisition.
            // so we need to store the first timestamp to compute a relative timestamp (which starts at 0)
            uint64_t m_first_timestamp;
        };
    }
}
#endif // SLSEIGERCAMERAFRAMES_H

/*************************************************************************/