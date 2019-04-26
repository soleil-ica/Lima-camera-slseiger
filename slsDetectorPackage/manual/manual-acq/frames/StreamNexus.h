/*************************************************************************/
/*! 
 *  \file   StreamNexus.h
 *  \brief  class used to manage the stream of acquisition data
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
 */
/*************************************************************************/

#ifndef EIGER_STREAM_NEXUS_H_
#define EIGER_STREAM_NEXUS_H_

//#define USE_NX_FINALIZER

// SYSTEM
#include <cstddef>
#include <stdint.h>
#include <list>

// LIMA
#include "lima/Debug.h"
#include "lima/Constants.h"

// NEXUS
#include <nexuscpp/nexuscpp.h>

#include "SlsEigerCameraFrames.h"

namespace lima
{
    namespace SlsEiger
    {
   
class StreamNexus : nxcpp::IExceptionHandler
{
    DEB_CLASS_NAMESPC(DebModCamera, "StreamNexus", "SlsEiger");

public :
    /*************************************************************************/
    // WRITE MODES
    enum WriteModes 
    { 
        ASYNCHRONOUS     = 0,
        SYNCHRONOUS         ,
        DELAYED             ,
    };

    /*\brief Class used to pass the needed data for the stream initialization*/
    class Parameters
    {
        friend class StreamNexus;
    
        public:
            // simple constructor
            Parameters();
            
            // constructor
            Parameters(const std::string &                 in_target_path    ,
                       const std::string &                 in_target_file    ,
                       const uint32_t                      in_nb_acq_per_file,
                       const uint32_t                      in_nb_data_per_acq,
                       const enum StreamNexus::WriteModes  in_write_mode     ,
                       const std::string &                 in_label_frame    ,
                       const std::size_t                   in_frame_size_x   ,
                       const std::size_t                   in_frame_size_y   ,
                       const int                           in_color_depth    );

        private :
            std::string                   m_target_path     ; // path for the stream files
            std::string                   m_target_file     ; // name for the stream file
            uint32_t                      m_nb_acq_per_file ; // number of acquisition per file
    	    uint32_t                      m_nb_data_per_acq ; // number of acquired frames
            enum StreamNexus::WriteModes  m_write_mode      ; // stream write mode
            std::string                   m_label_frame     ; // label of frame data in nexus file
            std::size_t                   m_frame_size_x    ; // number of horizontal pixels for a complete frame
            std::size_t                   m_frame_size_y    ; // number of vertical pixels for a complete frame
            int                           m_color_depth     ; // color depth (8, 16, 32)
    };

    public:
        // constructor
        StreamNexus();

        // destructor
        virtual ~StreamNexus();

        // Update the stream with new acquisition data
        void update(FramesListContainer & in_frame_list);

        // Reset the file index
        void reset_index(void);

        // Manage a nexus exception
        void OnNexusException(const nxcpp::NexusException &ex);

        // Abort the stream
        void abort(void);

        //==================================================================
        // acquisition management
        //==================================================================
        // Manage the start of acquisition
        void start_acquisition(void);

        // Manage the acquisition stop
        void stop_acquisition(void);

        //==================================================================
        // parameters management
        //==================================================================
        // Get all the parameters
        const StreamNexus::Parameters & get_parameters(void); 

        // Set the parameters
        void set_parameters(const StreamNexus::Parameters & in_parameters); 

    private :
        Parameters  m_parameters;

        nxcpp::DataStreamer* m_writer   ;
        mutable lima::Cond m_data_lock;

        //- Nexus stuff	
    #if defined(USE_NX_FINALIZER)
        static nxcpp::NexusDataStreamerFinalizer m_data_streamer_finalizer;
        static bool m_is_data_streamer_finalizer_started;
    #endif
};

} 
}
#endif 

//###########################################################################
