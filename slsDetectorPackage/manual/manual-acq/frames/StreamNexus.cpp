/*************************************************************************/
/*! 
 *  \file   StreamNexus.cpp
 *  \brief  class used to manage the stream of acquisition data
 *  \author Cédric Castel - SOLEIL (MEDIANE SYSTEME - IT consultant) 
 */
/*************************************************************************/

#include "StreamNexus.h"

namespace lima
{
    namespace SlsEiger
    {

#if defined(USE_NX_FINALIZER)
    nxcpp::NexusDataStreamerFinalizer StreamNexus::m_data_streamer_finalizer;
    bool StreamNexus::m_is_data_streamer_finalizer_started = false;
#endif

//============================================================================================================
// Stream::Parameters class
//============================================================================================================
/**************************************************************************
* \brief simple constructor
**************************************************************************/
StreamNexus::Parameters::Parameters()
{
}

/**************************************************************************
* \brief constructor
* \param[in] in_target_path      path for the stream files
* \param[in] in_target_file      name for the stream file 
* \param[in] in_nb_acq_per_file  number of files per acquisition
* \param[in] in_nb_data_per_acq  number of data per acquisition
* \param[in] in_write_mode       stream write mode
* \param[in] in_memory_mode      stream memory mode
* \param[in] in_label_frame      label of frame data in nexus file
**************************************************************************/
StreamNexus::Parameters::Parameters(const std::string &                 in_target_path    ,
                                    const std::string &                 in_target_file    ,
                                    const uint32_t                      in_nb_acq_per_file,
                                    const uint32_t                      in_nb_data_per_acq,
                                    const enum StreamNexus::WriteModes  in_write_mode     ,
                                    const std::string &                 in_label_frame    ,
                                    const std::size_t                   in_frame_size_x   ,
                                    const std::size_t                   in_frame_size_y   ,
                                    const int                           in_color_depth    )
{
    m_target_path     = in_target_path    ;
    m_target_file     = in_target_file    ;
    m_nb_acq_per_file = in_nb_acq_per_file;
    m_nb_data_per_acq = in_nb_data_per_acq;
    m_write_mode      = in_write_mode     ;
    m_label_frame     = in_label_frame    ;
    m_frame_size_x    = in_frame_size_x   ;
    m_frame_size_y    = in_frame_size_y   ;
    m_color_depth     = in_color_depth    ;
}

/*******************************************************************
 * \brief constructor
 * \param[in] in_device access to tango device for log management
 *******************************************************************/
StreamNexus::StreamNexus()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::StreamNexus() - [BEGIN]";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

	m_writer = NULL;
	
#if defined(USE_NX_FINALIZER)
    if (!StreamNexus::m_is_data_streamer_finalizer_started )
    {
        DEB_TRACE() << "starting the underlying NexusDataStreamerFinalizer - [BEGIN]";
        StreamNexus::m_data_streamer_finalizer.start();
        StreamNexus::m_is_data_streamer_finalizer_started = true;
        DEB_TRACE() << "starting the underlying NexusDataStreamerFinalizer - [END]";
    }
#endif

    DEB_TRACE() << "StreamNexus::StreamNexus() - [END]";
}

/*******************************************************************
 * \brief destructor
 *******************************************************************/
StreamNexus::~StreamNexus() 
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::~StreamNexus() - [BEGIN]";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    if (m_writer)
    {
        //- this might be required, in case we could not pass the writer to the finalizer	 
        delete m_writer;
        m_writer = NULL;
    }

    DEB_TRACE() << "StreamNexus::~StreamNexus() - [END]";
}

/*******************************************************************
 * \brief Get all the parameters
 * \return current parameters
 *******************************************************************/
const StreamNexus::Parameters & StreamNexus::get_parameters(void) 
{
    return m_parameters;
}

/*******************************************************************
 * \brief Set the parameters
 * \param[in] in_parameters new parameters
 *******************************************************************/
void StreamNexus::set_parameters(const StreamNexus::Parameters & in_parameters) 
{
    // making a bit copy (no pointer in the class) 
    m_parameters = in_parameters;
}

/*******************************************************************
 * \brief Update the stream with new acquisition data
 * \param[in] in_frame_list list of frames to treat
 *******************************************************************/
void StreamNexus::update(FramesListContainer & in_frame_list)
{
    DEB_MEMBER_FUNCT();

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    while(!in_frame_list.empty())
    {
        yat::SharedPtr<CameraFrame> frame = in_frame_list.front();
        in_frame_list.pop_front();

        //DEB_TRACE() << "StreamNexus::update - frame [ " << frame->getIndex() << " ]";

        if (m_writer)
        {
            yat::SharedPtr<CameraFramePart> frame_part = frame->getParts().front();

            // 8 bits color
            if(m_parameters.m_color_depth == 8)
            {
                m_writer->PushData(m_parameters.m_label_frame, (uint8_t *)(frame_part->getDataPointer()));
            }
            else
            // 16 bits color
            if(m_parameters.m_color_depth == 16)
            {
                m_writer->PushData(m_parameters.m_label_frame, (uint16_t *)(frame_part->getDataPointer()));
            }
            else
            // 32 bits color
            if(m_parameters.m_color_depth == 32)
            {
                m_writer->PushData(m_parameters.m_label_frame, (uint32_t *)(frame_part->getDataPointer()));
            }
        }
    }
}

/*******************************************************************
 * \brief Reset the file index
 *******************************************************************/
void StreamNexus::reset_index(void)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::reset_index() - [BEGIN]";
    DEB_TRACE() << "- ResetBufferIndex()";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());
    nxcpp::DataStreamer::ResetBufferIndex();

    DEB_TRACE() << "StreamNexus::reset_index() - [END]";
}

/*******************************************************************
 * \brief Manage a nexus exception
 *******************************************************************/
void StreamNexus::OnNexusException(const nxcpp::NexusException &ex)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::OnNexusException() - [BEGIN]";
    std::ostringstream ossMsgErr;
    ossMsgErr << "===============================================";
    ossMsgErr << "Origin\t: " << ex.errors[0].origin;
    ossMsgErr << "Desc\t: "   << ex.errors[0].desc  ;
    ossMsgErr << "Reason\t: " << ex.errors[0].reason;
    ossMsgErr << "===============================================";
    DEB_TRACE() << ossMsgErr.str();
    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    //1 - finalize the DataStreamer , this will set m_writer = 0 in order to avoid any new push data
    if (m_writer)
    {   
        abort();
    }

    DEB_TRACE() << "StreamNexus::OnNexusException() - [END]";        
};

/***************************************************************************
 * \brief Manage the start of acquisition
 ***************************************************************************/
void StreamNexus::start_acquisition(void)
{
    DEB_MEMBER_FUNCT();

    DEB_TRACE() << "StreamNexus::start_acquisition() - [BEGIN]";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    DEB_TRACE() << "- Create DataStreamer :";
    DEB_TRACE() << "\t- target path     = " << m_parameters.m_target_path     ;
    DEB_TRACE() << "\t- file name       = " << m_parameters.m_target_file     ;
    DEB_TRACE() << "\t- write mode      = " << m_parameters.m_write_mode      ;
    DEB_TRACE() << "\t- nb data per acq = " << m_parameters.m_nb_data_per_acq ;
    DEB_TRACE() << "\t- nb acq per file = " << m_parameters.m_nb_acq_per_file ;
    DEB_TRACE() << "\t- write data for  = " << m_parameters.m_label_frame     ;
    DEB_TRACE() << "\t- frame size x    = " << m_parameters.m_frame_size_x    ;
    DEB_TRACE() << "\t- frame size y    = " << m_parameters.m_frame_size_y    ;

    try
    {
		m_writer = new nxcpp::DataStreamer(m_parameters.m_target_file, 
                                           (std::size_t)m_parameters.m_nb_data_per_acq,
                                           (std::size_t)m_parameters.m_nb_acq_per_file);

        DEB_TRACE() << "- Initialize()";
        m_writer->Initialize(m_parameters.m_target_path);

        DEB_TRACE() << "- SetExceptionHnadler()";
        m_writer->SetExceptionHandler(this);

        DEB_TRACE() << "- Prepare Data Items :";

        // frame
        {
            DEB_TRACE() << "\t- AddDataItem2D(" << m_parameters.m_label_frame << "," << m_parameters.m_frame_size_x << "," << m_parameters.m_frame_size_y << ")";
            m_writer->AddDataItem2D(m_parameters.m_label_frame, m_parameters.m_frame_size_y, m_parameters.m_frame_size_x);
            m_writer->SetDataItemMemoryMode(m_parameters.m_label_frame, nxcpp::DataStreamer::COPY);
        }
        
        // configure the Writer mode 
        DEB_TRACE() << "- Configure the Writer mode :";

        //by default is ASYNCHRONOUS			
        if (m_parameters.m_write_mode == WriteModes::SYNCHRONOUS )
        {
            DEB_TRACE() << "\t- SYNCHRONOUS";
            m_writer->SetWriteMode(nxcpp::NexusFileWriter::SYNCHRONOUS);
        }
        else
        if (m_parameters.m_write_mode == WriteModes::DELAYED )
        {
            DEB_TRACE() << "\t- DELAYED";
            m_writer->SetWriteMode(nxcpp::NexusFileWriter::DELAYED);
        }
        else
        if (m_parameters.m_write_mode == WriteModes::ASYNCHRONOUS )
        {
            DEB_TRACE() << "\t- ASYNCHRONOUS";
            m_writer->SetWriteMode(nxcpp::NexusFileWriter::ASYNCHRONOUS);
        }
    }
    catch(const nxcpp::NexusException & ex)
    {
        DEB_TRACE() << "Nexus error : " << (ex.errors[0].reason).c_str() << " "
                                        << (ex.errors[0].desc  ).c_str() << " "
                                        << (ex.errors[0].origin).c_str();
    }

    DEB_TRACE() << "StreamNexus::start_acquisition() - [END]";
}

/*******************************************************************
 * \brief Manage the acquisition stop
 *******************************************************************/
void StreamNexus::stop_acquisition(void)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::stop_acquisition() - [BEGIN]";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    if (m_writer)
    {        
#if defined (USE_NX_FINALIZER) 
        //- Use NexusFinalizer to optimize the finalize which was extremely long !!!
        DEB_TRACE() << "passing DataStreamer to the NexusDataStreamerFinalizer - [BEGIN]";
        nxcpp::NexusDataStreamerFinalizer::Entry *e = new nxcpp::NexusDataStreamerFinalizer::Entry();
        e->data_streamer = m_writer;
        m_writer = 0;
        StreamNexus::m_data_streamer_finalizer.push(e);
        DEB_TRACE() << "passing DataStreamer to the NexusDataStreamerFinalizer - [END]";
#else  
        DEB_TRACE() << "- Finalize() - [BEGIN]";
        m_writer->Finalize();
        delete m_writer;
        m_writer = NULL;
        DEB_TRACE() << "- Finalize() - [END]" ;
#endif 
    }

    DEB_TRACE() << "StreamNexus::stop_acquisition() - [END]";
}

/*******************************************************************
 * \brief Abort the stream
 *******************************************************************/
void StreamNexus::abort(void)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "StreamNexus::abort() - [BEGIN]";

    lima::AutoMutex container_mutex = lima::AutoMutex(m_data_lock.mutex());

    if (m_writer)
    {        
        DEB_TRACE() << "- Stop() - [BEGIN]";
        m_writer->Stop();
        delete m_writer;
        m_writer = 0;
        DEB_TRACE() << "- Stop() - [END]";
    }

    DEB_TRACE() << "StreamNexus::abort() - [END]";
}

//###########################################################################
}
}