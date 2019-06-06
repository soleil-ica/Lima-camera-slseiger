/*************************************************************************************/
/*! 
 *  \file   CameraFrame.h
 *  \brief  detector CameraFrame class implementation 
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************************/

#include "SlsEigerCameraFrame.h"

using namespace lima::SlsEiger;

/************************************************************************
 * \brief constructor without parameter
 * \param m_index frame index (starts at 0)
 ************************************************************************/
CameraFrame::CameraFrame(uint64_t in_index)
{
    m_index         = in_index;
    m_timestamp     = 0       ;
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
CameraFrame::~CameraFrame()
{
    clearParts();
}

/************************************************************************
 * \brief gets the CameraFrame index
 * \return CameraFrame index value
 ************************************************************************/
uint64_t CameraFrame::getIndex() const
{
    return m_index;
}

/************************************************************************
 * \brief gets the timestamp
 * \return timestamp value
 ************************************************************************/
uint64_t CameraFrame::getTimestamp() const
{
    return m_timestamp;
}

/************************************************************************
 * \brief sets the timestamp
 * \param m_timestamp time stamp
 ************************************************************************/
void CameraFrame::setTimestamp(uint64_t in_timestamp)
{
    m_timestamp = in_timestamp;
}

/************************************************************************
 * \brief gets access to the frame parts container
 * \return frame parts container
 ************************************************************************/
FramePartsContainer & CameraFrame::getParts()
{
    return m_parts;
}

/************************************************************************
 * \brief gets the number of frame parts
 * \return frame parts number
 ************************************************************************/
int CameraFrame::getPartsNumber()
{
    return m_parts.size();
}

/************************************************************************
 * \brief clear the parts when they are no more needed
 ************************************************************************/
void CameraFrame::clearParts()
{
    m_parts.clear();
}

/************************************************************************
 * \brief add a frame part to the frame
 * \param in_part frame part 
 ************************************************************************/
void CameraFrame::addPart(yat::SharedPtr<CameraFramePart> in_part)
{
    m_parts.push_back(in_part);
}

//========================================================================================
