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
 *  \file   SlsEigerTypes.h
 *  \brief  SlsEiger detector hardware common types
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
*/
/*************************************************************************/

#ifndef SLSEIGERTYPES_H
#define SLSEIGERTYPES_H

/**********************************************************************/
namespace lima
{
    namespace SlsEiger
    {
        // status values
        enum Status
        {
            Idle   , // ready to start acquisition
            Waiting, // waiting for trigger signal
            Running, // acquisition is running 
            Error  , // acquisition stopped externally, fifo full or unexpected error 
        };

        // clock divider values
        enum ClockDivider
        {
            FullSpeed     , 
            HalfSpeed     , 
            QuarterSpeed  , 
            SuperSlowSpeed,
        };

        // parallel mode values
        enum ParallelMode
        {
            NonParallel, 
            Parallel   , 
            Safe       , 
        };

        // trigger mode values
        enum TriggerMode 
        { 
            TRIGGER_INTERNAL_SINGLE    = 0, // one trigger for all frames (auto mode)
            TRIGGER_EXTERNAL_SINGLE    ,    // one trigger for all frames (burst_trigger mode)
            TRIGGER_EXTERNAL_MULTIPLE  ,    // one trigger for each frame (trigger)
            TRIGGER_EXTERNAL_GATE      ,    // one trigger for each frame (gating)
        };

        // gain mode values
        enum GainMode 
        { 
            standard = 0,
            low         ,
            medium      ,
            high        ,
            very_high   ,
            undefined   ,
        };

        // temperature types
        enum Temperature
        {
            hw_fpga = 0,
            hw_fpgaext ,
            hw_10ge    , 
            hw_dcdc    , 
            hw_sodl    , 
            hw_sodr    , 
            hw_fpgafl  , 
            hw_fpgafr  , 
            hw_size    , // used to allocate a cache array for the values
        };
    }
}
#endif // SLSEIGERCAMERA_H

/*************************************************************************/
