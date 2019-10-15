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
 *  \file   SlsEigerCameraFrames.h
 *  \brief  SlsEiger detector frames manager class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include <cstdlib>
#include <algorithm>

#include "SlsEigerCameraFrames.h"

using namespace lima::SlsEiger;

// define it if you want to check optional frame data during the building of the frame 
#define SLS_EIGER_CAMERA_FRAMES_CHECK_OPTIONAL_FRAME_DATA

// define it if you want to trace more informations about the acquisition
//#define SLS_EIGER_CAMERA_FRAMES_LOG_VERBOSE

/************************************************************************
 * \brief constructor
 * \param in_complete_frame_parts_number number of frame parts to build a complete frame
 * \param in_frame_part_packet_number number of paquets we should receive for each frame part
 * \param in_color_depth_bytes_nb number of bytes for the current color depth
 * \param in_frame_size_x number of horizontal pixels for a complete frame
 * \param in_frame_size_y number of vertical pixels for a complete frame
 * \param in_frame_part_size_x number of horizontal pixels for a frame part
 * \param in_frame_part_size_y number of vertical pixels for a frame part
 ************************************************************************/
CameraFrames::CameraFrames(int         in_complete_frame_parts_number,
                           uint32_t    in_frame_part_packet_number   ,
                           std::size_t in_color_depth_bytes_nb       ,
                           std::size_t in_frame_size_x               ,
                           std::size_t in_frame_size_y               ,
                           std::size_t in_frame_part_size_x          ,
                           std::size_t in_frame_part_size_y          )
{
    DEB_MEMBER_FUNCT();

    m_complete_frame_parts_number = in_complete_frame_parts_number;
    m_frame_part_packet_number    = in_frame_part_packet_number   ;
    m_color_depth_bytes_nb        = in_color_depth_bytes_nb       ;
    m_frame_size_x                = in_frame_size_x               ;
    m_frame_size_y                = in_frame_size_y               ;
    m_frame_part_size_x           = in_frame_part_size_x          ;
    m_frame_part_size_y           = in_frame_part_size_y          ;

    m_first_timestamp         = 0LL  ;
    m_is_first_frame_received = false;

    DEB_TRACE() << "m_complete_frame_parts_number " << m_complete_frame_parts_number;
    DEB_TRACE() << "m_frame_part_packet_number " << m_frame_part_packet_number;
    DEB_TRACE() << "m_color_depth_bytes_nb " << m_color_depth_bytes_nb;
    DEB_TRACE() << "m_frame_size_x " << m_frame_size_x;
    DEB_TRACE() << "m_frame_size_y " << m_frame_size_y;
    DEB_TRACE() << "m_frame_part_size_x " << m_frame_part_size_x;
    DEB_TRACE() << "m_frame_part_size_y " << m_frame_part_size_y;
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraFrames::~CameraFrames()
{
    clear();
}

/************************************************************************
 * \brief get the number of bytes for the current color depth
 * \return number of bytes for the current color depth
 ************************************************************************/
std::size_t CameraFrames::getColorDepthBytesNb()
{
    return m_color_depth_bytes_nb;
}

/************************************************************************
 * \brief get the number of horizontal pixels for a complete frame
 * \return number of horizontal pixels for a complete frame
 ************************************************************************/
std::size_t CameraFrames::getFrameSizex()
{
    return m_frame_size_x;
}

/************************************************************************
 * \brief get the number of vertical pixels for a complete frame
 * \return number of vertical pixels for a complete frame
 ************************************************************************/
std::size_t CameraFrames::getFrameSizey()
{
    return m_frame_size_y;
}

/************************************************************************
 * \brief get the number of horizontal pixels for a frame part
 * \return number of horizontal pixels for a frame part
 ************************************************************************/
std::size_t CameraFrames::getFramePartSizex()
{
    return m_frame_part_size_x;
}

/************************************************************************
 * \brief get the number of vertical pixels for a frame part
 * \return number of vertical pixels for a frame part
 ************************************************************************/
std::size_t CameraFrames::getFramePartSizey()
{
    return m_frame_part_size_y;
}

/************************************************************************
 * \brief clear the containers
 ************************************************************************/
void CameraFrames::clear()
{
    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    m_received_frames.clear  ();
    m_not_merged_frames.clear();
    m_treated_frames.clear   ();

    m_received_frame_indexes.clear();

    m_first_timestamp         = 0LL  ;
    m_is_first_frame_received = false;
}

/************************************************************************
 * \brief compute the relative timestamp (which starts at 0)
 * \param in_absolute_timestamp timestamp which cames with the callback
 * \return relative timestamp
 ************************************************************************/
uint64_t CameraFrames::computeRelativeTimestamp(const uint64_t in_absolute_timestamp)
{
    if(!m_is_first_frame_received)
    {
        m_first_timestamp         = in_absolute_timestamp  ;
        m_is_first_frame_received = true;
    }

    return (in_absolute_timestamp - m_first_timestamp);
}   

/************************************************************************
 * \brief add a new received frame part
 * \param in_frame_index frame index (starts at 0)
 * \param in_frame_part frame part instance to treat
 ************************************************************************/
void CameraFrames::addFramePart(uint64_t                        in_frame_index,
                                yat::SharedPtr<CameraFramePart> in_frame_part )
{
    DEB_MEMBER_FUNCT();

    yat::SharedPtr<CameraFrame> frame = NULL;

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    // checking the number of paquets to reject the frame part if needed
    if(in_frame_part->getPacketNumber() != m_frame_part_packet_number)
    {
#ifdef SLS_EIGER_CAMERA_FRAMES_LOG_VERBOSE
        DEB_TRACE() << "CameraFrames::addFramePart - rejected incomplete frame part (" 
                    << in_frame_index << ", " << in_frame_part->getPacketNumber() << ")";
#endif
        return;
    }

    // searching if the frame is already in the received frames container
    FramesMapContainer::iterator received_search = m_received_frames.find(in_frame_index);
    
    // frame part was already received for this frame
    if(received_search != m_received_frames.end()) 
    {
        frame = received_search->second;

        // adding the frame part to the frame
        frame->addPart(in_frame_part);

        // cheking if the the frame is complete
        if(frame->getPartsNumber() == m_complete_frame_parts_number)
        {
            // moving the frame to not merged frames container
        #ifdef SLS_EIGER_CAMERA_FRAMES_LOG_VERBOSE
            DEB_TRACE() << "New complete frame to merge [ " 
                        << frame->getIndex()     << ", "
                        << frame->getTimestamp() << " ]";
        #endif

            m_not_merged_frames.push_back(frame);

            // removing the frame from the received frame container
            m_received_frames.erase(received_search);

            // purge the previous incomplete frames
            uint64_t frame_to_purge_index;

            frame_to_purge_index = m_received_frame_indexes.front();
            m_received_frame_indexes.pop_front();

            while(frame_to_purge_index != in_frame_index)
            {
                received_search = m_received_frames.find(frame_to_purge_index);

                // this should never happen.
                if(received_search == m_received_frames.end()) 
                {
                    DEB_TRACE() << "CameraFrames::addFramePart - unkown problem - frame (" 
                    << frame_to_purge_index 
                    << ") should be in the received container!";
                    return;
                }

                // removing the frame from the received frame container
                m_received_frames.erase(received_search);

            #ifdef SLS_EIGER_CAMERA_FRAMES_LOG_VERBOSE
                DEB_TRACE() << "Incomplete frame deleted [ " << frame_to_purge_index << " ]"; 
            #endif

                // checking the next frame index
                frame_to_purge_index = m_received_frame_indexes.front();
                m_received_frame_indexes.pop_front();
            }
        }
    }
    // frame part was never received for this frame
    else
    {
        frame = new CameraFrame(in_frame_index);

        // inserting the frame in the received container.
        std::pair<FramesMapContainer::iterator, bool> result = 
        m_received_frames.insert(std::make_pair(frame->getIndex(), frame));

        // this should never happen.
        if(!result.second)
        {
            DEB_TRACE() << "CameraFrames::addFramePart - unkown problem - frame (" 
                        << frame->getIndex() 
                        << ") should not be in the received container!";
            return;
        }

        // inserting the frame in the received frame indexes container.
        m_received_frame_indexes.push_back(in_frame_index);

        // adding the frame part to the frame
        frame->addPart(in_frame_part);
    }

    // managing the timestamp
    // we use only the timestamp of the frame part (0,0) because each port has its own timestamp
    if((in_frame_part->getPosx() == 0) && (in_frame_part->getPosy() == 0))
    {
        // compute the relative timestamp (which starts at 0)
        frame->setTimestamp(computeRelativeTimestamp(in_frame_part->getTimestamp()));
    }
}

/************************************************************************
 * \brief build the image
 * \param in_frame frame to consolidate
 * \param in_image_buffer destination buffer
 * \param in_image_buffer_byte_size destination buffer size (for error check) 
 * \return true if no error occurred, false if error
 ************************************************************************/
bool CameraFrames::buildImage(yat::SharedPtr<CameraFrame>  in_frame                 ,
                              char *                       in_image_buffer          ,
                              std::size_t                  in_image_buffer_byte_size)
{
    DEB_MEMBER_FUNCT();

    // gets the parts
    FramePartsContainer & parts = in_frame->getParts();

#ifdef SLS_EIGER_CAMERA_FRAMES_CHECK_OPTIONAL_FRAME_DATA
    // check the number of parts
    if(parts.size() != m_complete_frame_parts_number)
    {
        DEB_TRACE() << "Incoherent number of frame parts [ " 
                    << parts.size() << " != "
                    << m_complete_frame_parts_number << " ]"; 
        return false;
    }

    // checking the dest buffer size
    std::size_t theorical_size = m_frame_size_x * m_frame_size_y * m_color_depth_bytes_nb;

    if(in_image_buffer_byte_size != theorical_size)
    {
        DEB_TRACE() << "Incoherent size of destination buffer [ " 
                    << in_image_buffer_byte_size << " != "
                    << theorical_size << " ]"; 
        return false;
    }

    // checking the part buffer size
    std::size_t theorical_part_size = m_frame_part_size_x * m_frame_part_size_y * m_color_depth_bytes_nb;

    if(parts.front()->getDataByteSize() != theorical_part_size)
    {
        DEB_TRACE() << "Incoherent part size [ " 
                    << parts.front()->getDataByteSize() << " != "
                    << theorical_part_size << " ]"; 
        return false;
    }

    // checking the frame part size x is a multiple of 8
    if(m_frame_part_size_x % 8)
    {
        DEB_TRACE() << "Incoherent frame part size [ " 
                    << m_frame_part_size_x << "] should be a multiple of 8!"; 
        return false;
    }
#endif

    std::size_t frame_size_x_64      = (m_frame_size_x      * m_color_depth_bytes_nb) / 8; 
    std::size_t frame_part_size_x_64 = (m_frame_part_size_x * m_color_depth_bytes_nb) / 8; 

    // making a copy for each frame part
    for( FramePartsContainer::iterator it = parts.begin(); it != parts.end(); it++ )    
    {
        yat::SharedPtr<CameraFramePart> part = *it;

        bool vertical_flip = (part->getPosy() == 0); // the top part must be vertical flipped

        // compute the top-left corner of the destination buffer 
        uint64_t * destination_buffer = reinterpret_cast<uint64_t *>(in_image_buffer) + 
                                        (part->getPosx() * frame_part_size_x_64) +
                                        (part->getPosy() * frame_size_x_64 * m_frame_part_size_y);

        // if vertical flip, we start writing in the last line of the frame part
        if(vertical_flip)
        {
            destination_buffer += (frame_size_x_64 * (m_frame_part_size_y - 1));
        }

        // We will read the frame part in a sequential way.
        // We will write in the destination buffer making a jump at the end of each line.
        // If there is no vertical flip, we will jump to the start of the next line, adding the size x of a frame part.
        // If there is a vertical flip, we will jump back to the start of the previous line, substracting 3 * the size x of a frame part.
        const uint64_t * source_buffer = reinterpret_cast<const uint64_t *>(part->getDataPointer());
        int              nb_lines      = m_frame_part_size_y;
        int              nb_pixels_by_line;
        int              nb_pixels_to_jump = (vertical_flip) ? -(frame_part_size_x_64 * 3) : frame_part_size_x_64;

        // copy all the lines
        do
        {
            nb_pixels_by_line = frame_part_size_x_64;

            // copy the line
            do
            {
                *destination_buffer++ = *source_buffer++;
            }
            while(--nb_pixels_by_line);

            // jump to the next start of line (next or previous)
            destination_buffer += nb_pixels_to_jump; 
        }
        while(--nb_lines);


/*        {
            const uint16_t * s = reinterpret_cast<const uint16_t *>(in_image_buffer);
            for(int x = 0 ; x < 1024 ; x++)
            {
        std::stringstream tempStream;
        tempStream << "0x" << std::uppercase << std::hex << *(s+x);
    DEB_TRACE() << x << "," << tempStream.str(); 
            }
        }*/
    }


    // clear the parts when they are no more needed to free the memory
    in_frame->clearParts();

    return false;
}

/************************************************************************
 * \brief get the first frame waiting to be merged
 *        It does not remove the frame from the container.
 * \param out_frame NULL or the first not merged frame
 ************************************************************************/
void CameraFrames::getFirstNotMerged(yat::SharedPtr<CameraFrame> & out_frame)
{
    DEB_MEMBER_FUNCT();

    out_frame = NULL;

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    if(!m_not_merged_frames.empty())
    {
        out_frame = m_not_merged_frames.front();
    }
}

/************************************************************************
 * \brief move the first merged frame to the treated frame container
 ************************************************************************/
void CameraFrames::moveFirstNotMergedToTreated()
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    if(!m_not_merged_frames.empty())
    {
        yat::SharedPtr<CameraFrame> frame = m_not_merged_frames.front();
        m_not_merged_frames.pop_front();
        m_treated_frames.push_back(frame);

    #ifdef SLS_EIGER_CAMERA_FRAMES_LOG_VERBOSE
        DEB_TRACE() << "New treated frame [ " << frame->getIndex()        << ", "
                                              << frame->getPacketNumber() << ", " 
                                              << frame->getTimestamp()    << " ]";
    #endif
    }
    else
    // this should never happen.
    {
        DEB_TRACE() << "CameraFrames::moveFirstNotMergedToTreated - unkown problem - container is empty!";
    }
}

/************************************************************************
 * \brief get the number of frames in the containers
 * \return the number of frames by type
 ************************************************************************/
void CameraFrames::getNbFrames(std::size_t & out_received  , 
                               std::size_t & out_not_merged,
                               std::size_t & out_treated   ) const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    out_received   = m_received_frames.size  ();
    out_not_merged = m_not_merged_frames.size();
    out_treated    = m_treated_frames.size   ();
}

/************************************************************************
 * \brief get the number of treated frames in the container
 * \return the number of treated frames
 ************************************************************************/
std::size_t CameraFrames::getNbTreatedFrames() const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    return m_treated_frames.size();
}

/************************************************************************
 * \brief get the number of not treated frames in the containers
 * \return the number of not treated frames
 ************************************************************************/
std::size_t CameraFrames::getNbNotTreatedFrames() const
{
    DEB_MEMBER_FUNCT();

    // protecting the containers access
    lima::AutoMutex container_mutex = containersLock();

    return m_not_merged_frames.size();
}

/************************************************************************
 * \brief checks if there is no more frame to treat in the containers
 * \return true is there is no more frame to treat in the containers
 ************************************************************************/
bool CameraFrames::NoMoreFrameToTreat() const
{
    DEB_MEMBER_FUNCT();

    return (getNbNotTreatedFrames() == 0);
}

/************************************************************************
 * \brief creates an autolock mutex for containers methods access
 ************************************************************************/
lima::AutoMutex CameraFrames::containersLock() const
{
    DEB_MEMBER_FUNCT();
    return lima::AutoMutex(m_containers_cond.mutex());
}

//========================================================================================
