//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2012
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
//############################################################################
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include "Andor3Camera.h"

using namespace lima;
using namespace lima::Andor3;
using namespace std;

//---------------------------
//- utility variables
//---------------------------
bool Andor3::Camera::sAndorSDK3Initted = false;


//---------------------------
//- utility function
//---------------------------

//---------------------------
//- utility thread
//---------------------------

class Camera::_AcqThread : public Thread
{
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "_AcqThread");
public:
    _AcqThread(Camera &aCam);
    virtual ~_AcqThread();
    
protected:
    virtual void threadFunction();
    
private:
    Camera&    m_cam;
};


//---------------------------
// @brief  Constructor // DONE
//---------------------------
Camera::Camera(const std::string& bitflow_path,int camera_number)
    : m_status(Ready),
      m_wait_flag(true),
      m_quit(false),
      m_thread_running(true),
      m_image_number(0),
//      m_latency_time(0.),
      m_bin(1,1),
//      m_shutter_state(false),
      m_bitflow_path(bitflow_path),
      m_camera_number(camera_number),
      m_camera_handle(AT_HANDLE_UNINITIALISED),
/*
      m_adc_speed_number(0),
      m_adc_speed_max(0),
      m_adc(-1),
      m_vss_best(0),
      m_vss(-1),
      m_gain_number(0),
      m_gain_max(0),
      m_gain(-1),
      m_fasttrigger(0),
      m_shutter_level(0),
      m_shutter_close_time(0),
      m_shutter_open_time(0),
 */
      m_adc_gain(Gain1_Gain4),
      m_adc_rate(MHz100),
      m_electronic_shutter_mode(Rolling),
      m_cooler(true),
      m_temperature_sp(5.0),
      m_exp_time(1.0),
      m_frame_rate(1.0),
      m_ring_buffer_size(128)

{
  DEB_CONSTRUCTOR();
  
  _mapAndor3Error();
  
  // Initialisation of the atcore library :
  if ( ! sAndorSDK3Initted ) {
    if ( m_bitflow_path != "" ) {
      setenv("BITFLOW_INSTALL_DIRS", m_bitflow_path.c_str(), true);
    }
    else {
      setenv("BITFLOW_INSTALL_DIRS", "/usr/local/andor/bitflow", false);
    }
    
    if ( AT_SUCCESS != andor3Error(AT_InitialiseLibrary()) ) {
      DEB_ERROR() << "Library initialization failed, check the config. path" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Library initialization failed, check the bitflow path";       

    }
  }
  
  
    // --- Get available cameras and select the choosen one.
  AT_64 numCameras;
  DEB_TRACE() << "Get all attached cameras";
  if ( AT_SUCCESS != getInt(AT_HANDLE_SYSTEM, L"DeviceCount", &numCameras) ) {
    DEB_ERROR() << "No camera present!";
    THROW_HW_ERROR(Error) << "No camera present!";
  }
  DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
  DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
  
  if (m_camera_number < numCameras && m_camera_number >=0) {        
    if(andor3Error(AT_Open(m_camera_number, &m_camera_handle))) {
      DEB_ERROR() << "Cannot get camera handle" << " : error code = " << m_camera_error_str;;
      THROW_HW_ERROR(InvalidValue) << "Cannot get camera handle";
    }
  }
  else {
    DEB_ERROR() << "Invalid camera number " << m_camera_number << ", there is "<< numCameras << " available";
    THROW_HW_ERROR(InvalidValue) << "Invalid Camera number ";
  }
  
  // --- Get Camera model
  AT_WC	model[1024];
  if ( AT_SUCCESS != getString(m_camera_handle, L"CameraModel", model, 1024) ) {
    DEB_ERROR() << "Cannot get camera model: " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get camera model";            
  }
  m_detector_model = WStringToString(std::wstring(model));
  m_detector_type = std::string("sCMOS");
  // --- Initialise deeper parameters of the controller                
  initialiseController();            
    
  //--- Set detector for single image acquisition and get max binning
/*
  m_read_mode = 4;
  if (andor3Error(SetReadMode(m_read_mode))) {
    DEB_ERROR() << "Cannot set camera read mode" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot camera read mode";                
  }
*/ 
/*
  int xbin_max, ybin_max;   
  if (andor3Error(GetMaximumBinning(m_read_mode, 0, &xbin_max)))
  {
    DEB_ERROR() << "Cannot get the horizontal maximum binning" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get the horizontal maximum binning";            
  }
  if (andor3Error(GetMaximumBinning(m_read_mode, 1, &ybin_max)))
  {
    DEB_ERROR() << "Cannot get the vertical maximum binning" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get the vertical maximum binning";            
  }
 */
  // @TODO : m_bin_max should be derived from the parsing of the Implemented values in AOIBinning :
  m_bin_max = Bin(1, 1);
  DEB_TRACE() << "Maximum binning set to : " << m_bin_max.getX() << " x " << m_bin_max.getY() << " (hard-coded so far).";

    // --- set default ROI because there is no way to read bck th image size
    // --- BIN already set to 1,1 above.
    // --- Andor3 sets the ROI by starting coordinates at 1 and not 0 !!!!
  Size sizeMax;
  getDetectorImageSize(sizeMax);
  Roi aRoi = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    
    
  // --- setRoi applies both bin and roi
  DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
  setRoi(aRoi);
    
  // --- Get the maximum exposure time allowed and set default
  if ( AT_SUCCESS != getFloatMax(m_camera_handle, L"ExposureTime", &m_exp_time_max) ) {
    DEB_ERROR() << "Cannot get the maximum exposure time: " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get the maximum exposure time";    
  }    
  DEB_TRACE() << "Maximum exposure time : "<< m_exp_time_max << "sec.";


  setExpTime(m_exp_time);
  
    // --- Set detector for software single image mode    @TODO : Check that the values are the proper one!
  m_trig_mode_maps[IntTrig] = 0;
  m_trig_mode_maps[ExtTrigSingle] = 6;
  m_trig_mode_maps[ExtGate] = 2;
  m_trig_mode_maps[IntTrigMult] = -1;  
  setTrigMode(IntTrig);
    
  // --- Set the Andor3 specific acquistion mode.
  // --- We set acquisition mode to kinetics which is the more useful for us
  // --- This mode allows to manage latency between images and multi-frame acquisition as well
  // Andor3 : Cycle Mode : pre-setting the number of frames to acquire (value = "Fixed").
  m_nb_frames = 1;
  if ( AT_SUCCESS != setEnumString(m_camera_handle, L"CycleMode", L"Fixed") )
  {
    DEB_ERROR() << "Cannot set the camera CycleMode" << " : error code = " << m_camera_error << ", " <<m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set the camera CycleMode";            
  }               

  // --- set shutter mode to FRAME
  //  setShutterMode(FRAME);

  setCooler(m_cooler);
  setTemperatureSP(m_temperature_sp);
  
    // --- finally start the acq thread
  m_acq_thread = new _AcqThread(*this);
  m_acq_thread->start();
}


//---------------------------
// @brief  Destructor // DONE
//---------------------------
Camera::~Camera()
{
    DEB_DESTRUCTOR();
  // Stop Acq thread
  delete m_acq_thread;
  m_acq_thread = NULL;
  
  // Close camera
  if (m_cooler)
    {
      DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"                             
      << "brutale heating could damage the sensor.\n"
      << "And wait until temperature rises above 5 deg, before shuting down.";
      
      THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"                             
      << "brutale heating could damage the sensor.\n"
      << "And wait until temperature rises above 5 deg, before shuting down.";
    }
    
  DEB_TRACE() << "Shutdown camera";
  if ( AT_SUCCESS != andor3Error(AT_Close(m_camera_handle)) ) {
    DEB_ERROR() << "Cannot close the camera" << " : error code = " << m_camera_error << ", " <<m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot close the camera";            
  }
  m_camera_handle = AT_HANDLE_UNINITIALISED;
  // @TODO : shutting down the library ?
  if ( AT_SUCCESS != andor3Error(AT_FinaliseLibrary()) ) {
    DEB_ERROR() << "Cannot finalise Andor SDK 3 library" << " : error code = " << m_camera_error << ", " <<m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot finalise Andor SDK 3 library";            
  }
}



//---------------------------
// @brief  start the acquistion
//---------------------------
void Camera::startAcq()
{
#warning This one should be converted now. Still this is a central part of the acquisition !
  DEB_MEMBER_FUNCT();
  m_image_number=0;
  
  // --- check first the acquisition is idle
  bool acquiring;
  if ( AT_SUCCESS != getBool(m_camera_handle, L"CameraAcquiring", &acquiring)) {
    DEB_ERROR() << "Cannot get status" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get status";            
  }
  if ( acquiring ) {
    _setStatus(Camera::Fault,false);        
    DEB_ERROR() << "Cannot start acquisition, camera is already acquiring" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot start acquisition, camera is already acquiring";            
  }   
  
  /* Irrelevant on the SDK v3.x (circular buffer if any managed by user code) :
   // --- Don't forget to request the maximum number of images the circular buffer can store
   // --- based on the current acquisition settings.
   if (andor3Error(GetSizeOfCircularBuffer(&m_ring_buffer_size)))
   {
   DEB_ERROR() << "Cannot get size of circular buffer" << " : error code = " << m_camera_error_str;
   THROW_HW_ERROR(Error) << "Cannot get size of circular buffer";            
   }        
   DEB_TRACE() << "Andor3 Circular buffer size = " << m_ring_buffer_size << " images";
   */
  
  // Wait running stat of acquisition thread
  AutoMutex aLock(m_cond.mutex());
  m_wait_flag = false;
  m_cond.broadcast();
  while(!m_thread_running)
    m_cond.wait();
        
  StdBufferCbMgr& buffer_mgr = m_buffer_ctrl_obj.getBuffer();
  buffer_mgr.setStartTimestamp(Timestamp::now());
  if ( AT_SUCCESS != sendCommand(m_camera_handle, L"AcquisitionStart") ) {
    DEB_ERROR() << "Cannot start acquisition" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot start acquisition";            
  }
  // in external mode even with FastExtTrigger enabled the camera can not grab the trigger
  // within a too short delay, 100ms is the minimum required, very slow camera !!!
  // and unfortunately the status is not reflecting this lack of synchro.
  //while(1)
  //{
  //    if (andor3Error(GetStatus(&status)))
  //    {
  //        DEB_ERROR() << "Cannot get status" << " : error code = " << m_camera_error_str;
  //        THROW_HW_ERROR(Error) << "Cannot get status";            
  //    }
  //    if (status== DRV_ACQUIRING) break;
  //    usleep(1e3); 
  //}

  if ( 0 == m_nb_frames ) { // Live mode => software trigger
    if ( AT_SUCCESS != sendCommand(m_camera_handle, L"SoftwareTrigger") ) {
      DEB_ERROR() << "Unable to send a SoftwareTrigger in live mode" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Unable to send a SoftwareTrigger in live mode";            
    }
  }
  // To avoid troubles (really ?) in external triggering add a .01 ms delay :
  if (m_trig_mode != IntTrig && m_trig_mode != IntTrigMult) {
    usleep(1e1);
  }
}

//---------------------------
// @brief stop the acquisition
//---------------------------
void Camera::stopAcq()
{
    _stopAcq(false);
}


//---------------------------
// @brief private method // DONE but check required
//---------------------------
void Camera::_stopAcq(bool internalFlag)
{
  DEB_MEMBER_FUNCT();
  
  AutoMutex aLock(m_cond.mutex());
  if(m_status != Camera::Ready) {
    while(!internalFlag && m_thread_running) {
	    // signal the acq. thread to stop acquiring and to return the wait state
      m_wait_flag = true;

	    // Thread is maybe waiting for the Andor3 acq. event
	    if ( false ) {
#warning in Andor SDK 3: not possible to cancel the AT_WaitBuffer (in another thread), so the call has to be using a sensible Timeout parameter.
        DEB_ERROR() << "CancelWait() failed" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "CancelWait() failed";
	    }
      m_cond.wait();
    }
    aLock.unlock();
    
    //Let the acq thread stop the acquisition
    if(!internalFlag) return;
    
    // Stop acquisition
    DEB_TRACE() << "Stop acquisition";
    if ( AT_SUCCESS != andor3Error(AT_Command(m_camera_handle, L"AcquisitionStop")) ) {
      DEB_ERROR() << "Cannot abort acquisition" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot abort acquisition";
    }	
    _setStatus(Camera::Ready,false);    
  }
}


//---------------------------
// @brief the thread function for acquisition
//---------------------------
void Camera::_AcqThread::threadFunction()
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cam.m_cond.mutex());
  StdBufferCbMgr& buffer_mgr = m_cam.m_buffer_ctrl_obj.getBuffer();

  while(!m_cam.m_quit)
  {
    while(m_cam.m_wait_flag && !m_cam.m_quit)
    {
      DEB_TRACE() << "Wait";
      m_cam.m_thread_running = false;
      m_cam.m_cond.broadcast();
      m_cam.m_cond.wait();
    }
    DEB_TRACE() << "Run";
    m_cam.m_thread_running = true;
    if(m_cam.m_quit) return;
    
    m_cam.m_status = Camera::Acquisition;
    m_cam.m_cond.broadcast();
    aLock.unlock();
    
    bool continueAcq = true;
    
#warning This one to be converted.
#warning This one to be converted.
#warning This one to be converted.
    int first = 0, last = 0, prev_last = 0;
    FrameDim frame_dim = buffer_mgr.getFrameDim();
    Size  frame_size = frame_dim.getSize();
    int size = frame_size.getWidth() * frame_size.getHeight();
    int validfirst, validlast;
    
    while(continueAcq && (!m_cam.m_nb_frames || m_cam.m_image_number < m_cam.m_nb_frames)) {
      /*

	    // Check first if acq. has been stopped
	    if (m_cam.m_wait_flag) {
        continueAcq = false;
        continue;
	    }

      // Wait for an "acquisition" event, and use less cpu resources, in kinetic mode (multiframe)
      // an event is generated for each new image
      // /*
      if(m_cam.andor3Error(WaitForAcquisition()))
      {
        // If CancelWait() or acq. not started yet
        if(m_cam.m_camera_error == DRV_NO_NEW_DATA) continue;
        else 
        {
          DEB_ERROR() << "WaitForAcquisition() failed" << " : error code = " << m_cam.m_camera_error_str;
          THROW_HW_ERROR(Error) << "WaitForAcquisition() failed";
        }
      }
      
      // --- Get the available images in cicular buffer            
      prev_last = last;
      if (m_cam.andor3Error(GetNumberNewImages(&first, &last)))
      {
        if (m_cam.m_camera_error == DRV_NO_NEW_DATA) continue;
        else
        {
          DEB_ERROR() << "Cannot get number of new images" << " : error code = " << m_cam.m_camera_error_str;
          THROW_HW_ERROR(Error) << "Cannot get number of new images";
        }
      }        
      DEB_TRACE() << "Available images: first = " << first << " last = " << last;
      // Check if we lose an image
      if(first != prev_last +1 )
      {
        m_cam._setStatus(Camera::Fault,false);
        continueAcq = false;
        DEB_ERROR() << "Lost image(s) from " << prev_last << "to "<< first-1;
        THROW_HW_ERROR(Error) << "Lost image(s) from " << prev_last << "to "<< first-1;	
      }
      // --- Images are available, process images
      m_cam._setStatus(Camera::Readout,false);
      
      for (long im=first; im <= last; im++)
      {
        DEB_TRACE()  << "image #" << m_cam.m_image_number <<" acquired !";
        // ---  must get image one by one to copy to the buffer manager
        void *ptr = buffer_mgr.getFrameBufferPtr(m_cam.m_image_number);
        
        if (m_cam.andor3Error(GetImages16(im, im,(unsigned short*) ptr, (unsigned long)size,&validfirst, &validlast)))
        {
          m_cam._setStatus(Camera::Fault,false);
          continueAcq = false;
          DEB_TRACE() << "size = " << size;
          DEB_ERROR() << "Cannot get image #" << im << " : error code = " << m_cam.m_camera_error_str;
          THROW_HW_ERROR(Error) << "Cannot get last image";                
        }
        HwFrameInfoType frame_info;
        frame_info.acq_frame_nb = m_cam.m_image_number;
        continueAcq = buffer_mgr.newFrameReady(frame_info);
        DEB_TRACE() << DEB_VAR1(continueAcq);
        ++m_cam.m_image_number;
      }
      //       */
    }
    
    m_cam._stopAcq(true);
    
    aLock.lock();
    m_cam.m_wait_flag = true;
  }
}

 
//-----------------------------------------------------
// @brief the acquisition thread Constructor // DONE (?)
//-----------------------------------------------------
Camera::_AcqThread::_AcqThread(Camera &aCam) :
    m_cam(aCam)
{
    pthread_attr_setscope(&m_thread_attr,PTHREAD_SCOPE_PROCESS);
}
//-----------------------------------------------------
// @brief the acquisition thread Destructor // DONE (?)
//-----------------------------------------------------
Camera::_AcqThread::~_AcqThread()
{
    AutoMutex aLock(m_cam.m_cond.mutex());
    m_cam.m_quit = true;
    m_cam.m_cond.broadcast();
    aLock.unlock();
    
    join();
}

//-----------------------------------------------------
// brief return the detector image size // DONE.
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    AT_64 xmax, ymax;
    
    // --- Get the max image size of the detector
    if ( AT_SUCCESS != getInt(m_camera_handle, L"SensorWidth", &xmax)) {
      DEB_ERROR() << "Cannot get detector X size" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot get detector X size";                    
    }
  if ( AT_SUCCESS != getInt(m_camera_handle, L"SensorHeight", &ymax)) {
    DEB_ERROR() << "Cannot get detector Y size" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get detector Y size";                    
  }
    size= Size(static_cast<int>(xmax), static_cast<int>(ymax));
}


//-----------------------------------------------------
// @brief return the image type // DONE
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
  DEB_MEMBER_FUNCT();
    int bits_enum;
  // @TODO Indeed that would be better to parse the enum value STRING !
  //       Should make a functino to convert from the Andor3 "enum" and the ImageType one.
    if ( AT_SUCCESS != getEnumIndex(m_camera_handle, L"BitDepth", &bits_enum) ) {
      DEB_ERROR() << "Cannot get detector bit depth" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot get detector bit depth";                        
    }
  switch (bits_enum) {
    case 0:
      type = Bpp12;
      break;
    case 1:
      type = Bpp16;
  }
}

//-----------------------------------------------------
// @brief set the image type, if supported // DONE
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
  DEB_MEMBER_FUNCT();
  // --- see above for future immprovement
  int bits_enum;
  
  switch (type) {
    case Bpp8:
    case Bpp8S:
    case Bpp10:
    case Bpp10S:
    case Bpp12:
    case Bpp12S:
      bits_enum = 0; // 11 Bits
      break;
    case Bpp14:
    case Bpp14S:
    case Bpp16:
    case Bpp16S:
    default:
      bits_enum = 1; // 16 Bits
      break;
  }
  if ( AT_SUCCESS != setEnumIndex(m_camera_handle, L"BitDepth", bits_enum) ) {
    DEB_ERROR() << "Cannot set detector bit depth" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector bit depth";                        
  }
  return;
}


//-----------------------------------------------------
 // @brief return the detector type // DONE
//-----------------------------------------------------
void Camera::getDetectorType(string& type)
{
  DEB_MEMBER_FUNCT();
    type = m_detector_type;
}


//-----------------------------------------------------
// @brief return the detector model // DONE
//-----------------------------------------------------
void Camera::getDetectorModel(string& model)
{
  DEB_MEMBER_FUNCT();  
  model = m_detector_model;
}

//-----------------------------------------------------
// @brief return the internal buffer manager // DONE (nothing done)
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return &m_buffer_ctrl_obj;
}


//-----------------------------------------------------
// @brief return true if passed trigger mode is supported // DONE (more complete conversion between AT trigger mode and Lima one.
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(trig_mode);
  bool valid_mode;    

  switch (trig_mode) {       
    case IntTrig:
    case ExtTrigSingle:
    case ExtGate:
      m_camera_error = isEnumIndexAvailable(m_camera_handle, L"TriggerMode", m_trig_mode_maps[trig_mode], &valid_mode);
      switch (m_camera_error) {
        case AT_SUCCESS: // The call went ok.
          valid_mode = valid_mode;
          break;
        default: // Some problem occured, make sure we say it's not available :
          valid_mode = false;
          break;
      }                
      break;
      
    default:
      valid_mode = false;
      break;
  }
  return valid_mode;
}

//-----------------------------------------------------
// @brief set the new trigger mode // DONE
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(mode);

  if ( AT_SUCCESS != setEnumIndex(m_camera_handle, L"TriggerMode", m_trig_mode_maps[mode]) ) {
    DEB_ERROR() << "Cannot set trigger mode" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set trigger mode";                    
  }
  // We proof-read the value :
  int		newTM;
  if ( AT_SUCCESS != getEnumIndex(m_camera_handle, L"TriggerMode", &newTM) ) {
    DEB_ERROR() << "Cannot proof-read trigger mode" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read trigger mode";                    
  }
  switch (newTM) {
    case 0:
      m_trig_mode = IntTrig;
      break;
    case 6:
      m_trig_mode = ExtTrigSingle;
      break;
    case 2:
      m_trig_mode = ExtGate;
      break;
    default:
      m_trig_mode = mode;
      DEB_ERROR() << "Proof-reading provided an un-convertible trigger mode !!!";
      break;
  }
  m_trig_mode = mode;    
}

//-----------------------------------------------------
// @brief return the current trigger mode // DONE
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_trig_mode;
    
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
// @brief set the new exposure time // DONE
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(exp_time);
  
  if ( AT_SUCCESS != setFloat(m_camera_handle, L"ExposureTime", exp_time) ) {
    DEB_ERROR() << "Cannot set exposure time" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set exposure time";                        
  }
  // As proposed in Andor SDK 3 documentation, float settings should be folowed by proof-reading :
  // Doing so, rather than take it for granted that it is OK.
  //    m_exp_time = exp_time;
  if ( AT_SUCCESS != getFloat(m_camera_handle, L"ExposureTime", &m_exp_time) ) {
    DEB_ERROR() << "Cannot proof-read exposure time" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read exposure time";                        
  }
}

//-----------------------------------------------------
// @brief return the current exposure time // DONE
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != getFloat(m_camera_handle, L"ExposureTime", &m_exp_time) ) {
    DEB_ERROR() << "Cannot read exposure time" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot read exposure time";                        
  }
  exp_time = m_exp_time;
  DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
// @brief set the new latency time between images // DONE
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(lat_time);
  
  // The Andor SDK 3 does not have a "Latency time" but rather a frame rate.
  // Here we understand the latency time as the "repetition" time of the frames kin_time = (exp+lat_time)
  // Hence we set FrameRate = 1/(lat_time + exp_time)
  double newFrameRate = 1.0 / (lat_time + m_exp_time);

  if ( AT_SUCCESS != setFloat(m_camera_handle, L"FrameRate", newFrameRate) ) {
    DEB_ERROR() << "Cannot set latency time (FrameRate)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set latency time (FrameRate)";                        
  }
  // As proposed in Andor SDK 3 documentation, float settings should be folowed by proof-reading :
  // Doing so, rather than take it for granted that it is OK.
  //    m_exp_time = exp_time;
  if ( AT_SUCCESS != getFloat(m_camera_handle, L"FrameRate", &m_frame_rate) ) {
    DEB_ERROR() << "Cannot proof-read latency time (FrameRate)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read latency time (FrameRate)";                        
  }
}

//-----------------------------------------------------
// @brief return the current latency time // DONE
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  // --- we do calculate the latency by using the frame rate (time between 2 frames)
  // --- minus the exposure time
  
  if ( AT_SUCCESS != getFloat(m_camera_handle, L"FrameRate", &m_frame_rate) ) {
    DEB_ERROR() << "Cannot get latency time (FrameRate)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get latency time (FrameRate)";                        
  }
  getExpTime(m_exp_time);
  
  lat_time = (1.0 / m_frame_rate) - m_exp_time;
  DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
// @brief return the exposure time range // DONE
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != getFloatMax(m_camera_handle, L"ExposureTime", &max_expo) ) {
    DEB_ERROR() << "Cannot get max exposure time" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get max exposure time";                        
  }
  if ( AT_SUCCESS != getFloatMin(m_camera_handle, L"ExposureTime", &min_expo) ) {
    DEB_ERROR() << "Cannot get min exposure time" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get min exposure time";                        
  }
  DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
// @brief return the latency time range // DONE
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{   
  DEB_MEMBER_FUNCT();
  
  // Provide an answer, GIVEN the current value of exposure time (taken as granted to be m_exp_time) :
  double min_frame_rate, max_frame_rate;

  if ( AT_SUCCESS != getFloatMax(m_camera_handle, L"FrameRate", &max_frame_rate) ) {
    DEB_ERROR() << "Cannot get max frame rate" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get max frame rate";                        
  }
  if ( AT_SUCCESS != getFloatMin(m_camera_handle, L"FrameRate", &min_frame_rate) ) {
    DEB_ERROR() << "Cannot get min frame rate" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get min frame rate";                        
  }
  
  // Min latency corresponds to the max frame rate :
  min_lat = (1.0 / max_frame_rate) - m_exp_time;
  
  // Max latency corresponds to the min frame rate :
  max_lat = (1.0 / min_frame_rate) - m_exp_time;

  DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
// @brief set the number of frames to be taken // DONE
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(nb_frames);
  // nb_frames == 0 means continuous mode :    
  if (nb_frames == 0) { // Setting to Continuous mode, and since we are interested in something like live mode, set also the Sowftware trigger.
    if ( AT_SUCCESS != setEnumString(m_camera_handle, L"CycleMode", L"Continuous") ) {
      DEB_ERROR() << "Can not set continuous acquisition (setNbFrames(0))" << " : error code = " << m_camera_error_str ;
      THROW_HW_ERROR(Error) << "Can not set continuous acquisition (setNbFrames(0))";
    }
    if ( AT_SUCCESS != setEnumString(m_camera_handle, L"TriggerMode", L"Software,") ) {
      DEB_ERROR() << "Can not set software trigger (used in continuous mode)" << " : error code = " << m_camera_error_str ;
      THROW_HW_ERROR(Error) << "Can not set software trigger (used in continuous mode)";
    }
    m_nb_frames = 0;
  }
  else { // A given number of frames to be taken :
    if ( AT_SUCCESS != setEnumString(m_camera_handle, L"CycleMode", L"Fixed") ) {
      DEB_ERROR() << "Can not set fixed number of frame acquisition (setNbFrames(>0))" << " : error code = " << m_camera_error_str ;
      THROW_HW_ERROR(Error) << "Can not set fixed number of frame acquisition (setNbFrames(>0)))";      
    }
    AT_64  nb_frames_AT = static_cast<AT_64>(nb_frames);
    if ( AT_SUCCESS != setInt(m_camera_handle, L"FrameCount", nb_frames_AT) ) {
      DEB_ERROR() << "Cannot set number of frames" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot set number of frames";
    }
    // Proof-reading :
    if ( AT_SUCCESS != getInt(m_camera_handle, L"FrameCount", &nb_frames_AT) ) {
      DEB_ERROR() << "Cannot proof-read number of frames" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot proof-read number of frames";
    }
    m_nb_frames = static_cast<int>(nb_frames_AT);
    // Finally making sure that we are no more in Software trigger mode :
    wchar_t		wcs_trigger_mode[1024];
    if ( AT_SUCCESS != getEnumString(m_camera_handle, L"TriggerMode", wcs_trigger_mode, 1023) ) {
      DEB_ERROR() << "Can not check we are not in software trigger (used in continuous mode)" << " : error code = " << m_camera_error_str ;
      THROW_HW_ERROR(Error) << "Can not check we are not in software trigger (used in continuous mode)";
    }
    if ( std::wstring(L"Software") == wcs_trigger_mode ) {
      //      setTrigMode( ( 1 == nb_frames ) ? IntTrig : IntTrigMult );
      setTrigMode(IntTrig); // Currently the IntTrigMult does not corresponds to anything.
    }
  }
}

//-----------------------------------------------------
// @brief return the number of frames to be taken // DONE (trust the cache)
// @note If returning 0, means conitnuous + software trigger (live mode) : software triggering should happen after each frame collected.
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
  nb_frames = m_nb_frames;
  DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
// @brief return the current acquired frames // DONE (nothing to change)
//-----------------------------------------------------
void Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{ 
  DEB_MEMBER_FUNCT();    
  nb_acq_frames = m_image_number;
}
  
//-----------------------------------------------------
// @brief return the camera status // DONE (nothing to change, trust the cache ?)
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
  // @TODO : check that is semantically correct. 
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  status = m_status;
  DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
// @brief set the new camera status // DONE (nothing to change ?)
//-----------------------------------------------------
void Camera::_setStatus(Camera::Status status, bool force)
{
  // @TODO : check that is semantically correct. 
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  //  if(force || m_status != Camera::Fault)
  m_status = status;
  m_cond.broadcast();
}

//-----------------------------------------------------
// @brief do nothing, hw_roi = set_roi. // DONE (nothing to change ?) : this means accept all roi.
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  // @TODO : Should definitely implement correctly this method using FullAOIControl,
  //         and if not implement the table present in the documentation of the SDK.
#warning Currently not possibilities to really handle binned pixels !!!
  // @TODO : Properly handle the pixel binning setup !!!
  // --- Warnign : AOI is in pixel, while Lima passes binned-pixels.
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    hw_roi = set_roi;

    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief set the new roi // DONE (has to implement binning later)
//-----------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
#warning Currently not possibilities to really handle binned pixels !!!
  // @TODO : Properly handle the pixel binning setup !!!
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(set_roi);
  
  Point topleft, size;
  int binx, biny;
  int hleft, hwidth, vtop, vheight;
  Roi hw_roi, roiMax;
  Size sizeMax;
  
  getDetectorImageSize(sizeMax);
  roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    
  
  // --- Warning, SetImage() needs coodinates in full image size not with binning
  // --- but Lima passes image size with binning applied on
  // --- so set a internal binning factor (binx/biny) for size correction.

  if(m_roi == set_roi) return;    
  
  if(set_roi.isActive() && set_roi != roiMax) {
    // --- a real roi available : @TODO should convert to what is feasible !!!
    checkRoi(set_roi, hw_roi);
    //    hw_roi = set_roi;
    binx = m_bin.getX(); biny = m_bin.getY();    	
  }
  else {
    // ---  either No roi or roi fit with max size!!!	
    // --- in that case binning for full size calculation is 1
    hw_roi = roiMax;
    binx=1; biny=1;
  }    
  // --- Andor3 sets the ROI by starting coordinates at 1 and not 0 !!!!
  // --- Warning : documentations says the parameter have to be given in proper order : 
  // --- AOIHBin, AOIVBin or AOIBinning, then 
  // ---  AOIWidth, AOILeft, AOIHeight, AOITop
  // --- Warnign : AOI is in pixel, while Lima passes binned-pixels.
  
  topleft = hw_roi.getTopLeft(); 
  size = hw_roi.getSize();
  
  hleft = topleft.x +1;
  vtop = topleft.y +1;
  hwidth = size.x;
  vheight = size.y;
  
  DEB_TRACE() << "bin =  " << m_bin.getX() <<"x"<< m_bin.getY();
  DEB_TRACE() << "roi = " << hleft << ", width" << hwidth << ", " << vtop << ", height" << vheight;
  //- then fix the new ROI
  if ( AT_SUCCESS != setInt(m_camera_handle, L"AOIWidth", static_cast<AT_64>(hwidth))) {
    DEB_ERROR() << "Cannot set detector ROI (width)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector ROI (width)";
  }
  if ( AT_SUCCESS != setInt(m_camera_handle, L"AOILeft", static_cast<AT_64>(hleft))) {
    DEB_ERROR() << "Cannot set detector ROI (left)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector ROI (left)";
  }
  if ( AT_SUCCESS != setInt(m_camera_handle, L"AOIHeight", static_cast<AT_64>(vheight))) {
    DEB_ERROR() << "Cannot set detector ROI (height)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector ROI (height)";
  }
  if ( AT_SUCCESS != setInt(m_camera_handle, L"AOITop", static_cast<AT_64>(vtop))) {
    DEB_ERROR() << "Cannot set detector ROI (top)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector ROI (top)";
  }
  // Proof-read the roi, and cache the real ROI, used when setting BIN
  AT_64 new_hwidth, new_hleft, new_vheight, new_vtop;
  
  if ( AT_SUCCESS != getInt(m_camera_handle, L"AOIWidth", &new_hwidth) ) {
    DEB_ERROR() << "Cannot proof-read detector ROI (width)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read detector ROI (width)";
  }
  if ( AT_SUCCESS != getInt(m_camera_handle, L"AOILeft", &new_hleft) ) {
    DEB_ERROR() << "Cannot proof-read detector ROI (left)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read detector ROI (left)";
  }
  if ( AT_SUCCESS != getInt(m_camera_handle, L"AOIHeight", &new_vheight) ) {
    DEB_ERROR() << "Cannot proof-read detector ROI (height)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read detector ROI (height)";
  }
  if ( AT_SUCCESS != getInt(m_camera_handle, L"AOITop", &new_vtop) ) {
    DEB_ERROR() << "Cannot proof-read detector ROI (top)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot proof-read detector ROI (top)";
  }
  // Caching the proof-read value.
  m_roi.setTopLeft(lima::Point(static_cast<int>(new_hleft), static_cast<int>(new_vtop)));
  m_roi.setSize(lima::Size(static_cast<int>(new_hwidth), static_cast<int>(new_vheight)));
}

//-----------------------------------------------------
// @brief return the new roi // DONE (trust the cached value).
//-----------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    // ---  no way to read the roi, Andor3 does not provide any function to do that!
    hw_roi = m_roi;
    
    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed // DONE : currently only accept no-binning
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
  // @TODO Currently only accept no-binning
  DEB_MEMBER_FUNCT();

/*
 int x = hw_bin.getX();
 if(x > m_bin_max.getX())
 x = m_bin_max.getX();
 
 int y = hw_bin.getY();
 if(y > m_bin_max.getY())
 y = m_bin_max.getY();
 
 hw_bin = Bin(x,y);
 */
  hw_bin = Bin(1, 1);
  DEB_RETURN() << DEB_VAR1(hw_bin);
}
//-----------------------------------------------------
// @brief set the new binning mode // DONE (Only accept to set to bin 1x1)
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
  // @TODO Only accept to set to bin 1x1
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setEnumString(m_camera_handle, L"AOIBinning", L"1x1") ) {
    DEB_ERROR() << "Cannot set binning to 1x1 through AOIBinning" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set binning to 1x1 through AOIBinning";
  }
  m_bin = Bin(1, 1);
  /*
  Point topleft, size;
  
  int binx, biny;
  int hstart, hend, vstart, vend;
  Roi hw_roi, roiMax;
  Size sizeMax;
  
  getDetectorImageSize(sizeMax);
  roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());
  
  if(m_bin == set_bin) return;
  
  // --- Warning, SetImage() needs coodinates in full image size not with binning
  // --- but Lima passes image size with binning applied on
  // --- so set a internal binning factor (binx/biny) for size correction.
  
  if(m_roi.isActive() && m_roi != roiMax) 
  {
    // --- a real available
    binx = set_bin.getX();  biny = set_bin.getY();
    hw_roi = m_roi;
  }
  else
  {
    // ---  either No roi or roi fit with max size!!!	
    // --- in that case binning for full size calculation is 1
    hw_roi = roiMax;
    binx = 1; biny = 1;
  }
  topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
  hstart = topleft.x*binx +1;          vstart = topleft.y*biny +1;
  hend   = hstart + size.x*binx -1;    vend   = vstart + size.y*biny -1;
  
  DEB_TRACE() << "bin =  " << set_bin.getX() <<"x"<< set_bin.getY();
  DEB_TRACE() << "roi = " << hstart << "-" << hend << ", " << vstart << "-" << vend;
  if (andor3Error(SetImage(set_bin.getX(), set_bin.getY(), hstart, hend, vstart, vend)))
  {
    DEB_ERROR() << "Cannot set detector BIN" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set detector BIN";                                        
  }
  m_bin = set_bin;
  */
  DEB_RETURN() << DEB_VAR1(set_bin);
}

//-----------------------------------------------------
// @brief return the current binning mode // DONE (Trusting the cached value).
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
  DEB_MEMBER_FUNCT();
  // ---  no way to read the bin Andor3 does not provide any function to do that!
  hw_bin = m_bin;
  
  DEB_RETURN() << DEB_VAR1(hw_bin);
}

//-----------------------------------------------------
// @brief return always false, hw binning mode is supported by the camera (up to 8x8) but not the pluggin so far. // DONE (Should support camera binning).
//-----------------------------------------------------
bool Camera::isBinningAvailable()
{
  // @TODO Should support camera binning
  DEB_MEMBER_FUNCT();
  bool isAvailable = false;
  
  // --- ok not realy need this function but could be completed
  // for further camera model which do not support binning
  return isAvailable;
}

//-----------------------------------------------------
// @brief return the detector pixel size in meter // DONE (returning values for Neo only : 6.5µm in both direction)
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
  DEB_MEMBER_FUNCT();
  // @TODO : Currently returning values for Neo only : 6.5µm in both direction
  /*
    float xsize, ysize;
    
    if (andor3Error(GetPixelSize(&xsize, &ysize)))
    {
        DEB_ERROR() << "Cannot pixel sizes" << " : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get pixel size";                                        
    }
    sizex = xsize * 1e-6;
    sizey = ysize * 1e-6;
   */
  sizex = 6.5e-6;
  sizey = 6.5e-6;
  DEB_RETURN() << DEB_VAR2(sizex, sizey); 
}


//-----------------------------------------------------
// @brief reset the camera, no hw reset available on Andor3 camera // DONE (No changes)
//-----------------------------------------------------
void Camera::reset()
{
    DEB_MEMBER_FUNCT();
    return;
}

void
Camera::setAdcGain(A3_Gain iGain)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setEnumIndex(m_camera_handle, L"PreAmpGainControl", static_cast<int>(iGain)) ) {
    DEB_ERROR() << "Cannot set pre-amplifiers gains" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set pre-amplifiers gains";
  }
}

void
Camera::getAdcGain(A3_Gain &oGain) 
{
  DEB_MEMBER_FUNCT();
  int		gain_index;
  if ( AT_SUCCESS != getEnumIndex(m_camera_handle, L"PreAmpGainControl", &gain_index) ) {    
    DEB_ERROR() << "Cannot get pre-amplifiers gains" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get pre-amplifiers gains";
  }
  oGain = static_cast<A3_Gain>(gain_index);
  DEB_RETURN() << DEB_VAR1(gain_index);
}

void
Camera::setAdcRate(A3_ReadOutRate iRate)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setEnumIndex(m_camera_handle, L"PixelReadoutRate", static_cast<int>(iRate)) ) {
    DEB_ERROR() << "Cannot set pixel readout rate" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set pixel readout rate";
  }
}

void
Camera::getAdcRate(A3_ReadOutRate &oRate) 
{
  DEB_MEMBER_FUNCT();
  int		rate_index;
  if ( AT_SUCCESS != getEnumIndex(m_camera_handle, L"PixelReadoutRate", &rate_index) ) {
    DEB_ERROR() << "Cannot get readout rate" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get readout rate";
  }
  oRate = static_cast<A3_ReadOutRate>(rate_index);
  DEB_RETURN() << DEB_VAR1(rate_index);  
}

void
Camera::setElectronicShutterMode(A3_ShutterMode iMode)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setEnumIndex(m_camera_handle, L"ElectronicShutteringMode", static_cast<int>(iMode)) ) {
    DEB_ERROR() << "Cannot set electronic shutter mode" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set electronic shutter mode";
  }
}

void
Camera::getElectronicShutterMode(A3_ShutterMode &oMode)
{
  DEB_MEMBER_FUNCT();
  int		mode_index;
  if ( AT_SUCCESS != getEnumIndex(m_camera_handle, L"ElectronicShutteringMode", &mode_index) ) {
    DEB_ERROR() << "Cannot get electronic shutter mode" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get electronic shutter mode";
  }
  oMode = static_cast<A3_ShutterMode>(mode_index);
  DEB_RETURN() << DEB_VAR1(mode_index);
}



//-----------------------------------------------------
// @brief    initialise controller with speeds and preamp gain
//-----------------------------------------------------
void Camera::initialiseController()
{
  DEB_MEMBER_FUNCT();
  setAdcGain(m_adc_gain);
  setAdcRate(m_adc_rate);
  setElectronicShutterMode(m_electronic_shutter_mode);
  
  // Also making sure that SpuriousNoiseFilter is OFF :
  if ( AT_SUCCESS != setBool(m_camera_handle, L"SpuriousNoiseFilter", false) ) {
    DEB_ERROR() << "Cannot set SpuriousNoiseFilter to false" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set SpuriousNoiseFilter to false";
  }
  /*
    // --- Init adc / speed
    initAdcSpeed();
       
    DEB_TRACE() << "* Horizontal Shift Speed:";
    int is;
    for (is=0; is< m_adc_speed_number; is++)
    {
        DEB_TRACE() << "    (" << is << ") adc #" << m_adc_speeds[is].adc << ", speed = " 
                    << m_adc_speeds[is].speed  << ((is == m_adc_speed_max)? " [max]": "");                        
    }
        
    // --- Set adc / speed
    setAdcSpeed(m_adc_speed_max);
    DEB_TRACE() << "    => Set to " << m_adc_speeds[m_adc_speed_max].speed << "MHz";
       
        
    // --- Init VS Speeds 
    initVSS();
      
    DEB_TRACE() << "* Vertical Shift Speed:";
    for (is=0; is<m_vss_number; is++)
    {
        DEB_TRACE() << "    (" << is << ") speed = " << m_vsspeeds[is] << " us"
                    << ((is == m_vss_best)? " [recommended]": "");
    }
        
    // --- Set VS Speed
    setVSS(m_vss);
    DEB_TRACE() << "    => Set " << m_vsspeeds[m_vss] << "us";
        
        
    // --- Init Preamp Gain
    initPGain();
       
    DEB_TRACE() << "* Preamp Gain:";
        
    for (is=0; is< m_gain_number; is++)
    {
        DEB_TRACE() << "    (" << is << ") gain = x" << m_preamp_gains[is]
                    << ((is == m_gain_max)? " [max]": "");
    }
    
    // --- Set Preamp Gain
    setPGain(m_gain);
  DEB_TRACE() << "    => Set to x" << m_preamp_gains[m_gain];                 
   */
}

/*
//-----------------------------------------------------
// @brief get possible adc/speed for controller
//
// Initialise the list of possible pairs adc/speed
// and find the maximum speed.
//
//-----------------------------------------------------
void Camera::initAdcSpeed()
{
    DEB_MEMBER_FUNCT();
    int		ih, ia, is, nadc, *nSpeed;
    float	speedMax;
    
    
    // --- number of ADC
    if (andor3Error(GetNumberADChannels(&nadc))) 
    {
        DEB_ERROR() << "Cannot get number of ADC" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get number of ADC";            
    }
    // --- Get Horizontal Shift Speed per ADC
    nSpeed = new int[nadc];
    
    m_adc_speed_number= 0;
    for (ia=0; ia<nadc; ia++) 
    {
	if (andor3Error(GetNumberHSSpeeds(ia, 0, &nSpeed[ia]))) {
	    DEB_ERROR() << "Cannot get nb of Horizontal Speed for ADC " <<  ia <<" : error code = " << m_camera_error_str;
	    THROW_HW_ERROR(Error) << "Cannot get nb of Horizontal Speed for an ADC";            
	}
	m_adc_speed_number += nSpeed[ia];
			
    }

    m_adc_speeds = new Adc[m_adc_speed_number];
    speedMax= 0.;
    is= 0;
    for (ia=0; ia<nadc; ia++) {
	for (ih=0; ih<nSpeed[ia]; ih++) {
	    if (andor3Error(GetHSSpeed(ia, 0, ih, &m_adc_speeds[is].speed))) {
                DEB_ERROR() << "Cannot get Horizontal Speed " << ih << " for ADC " << ia <<" : error code = " << m_camera_error_str;
                THROW_HW_ERROR(Error) << "Cannot get Horizontal Speed ";            
	    }
	    m_adc_speeds[is].adc= ia;
	    m_adc_speeds[is].hss= ih;

	    // --- iKon/iXon= speed in MHz ; others in us/pixel shift --> convert in MHz
	    if ((m_camera_capabilities.ulCameraType!=1)&&(m_camera_capabilities.ulCameraType!=13))
		m_adc_speeds[is].speed = (float)(1./ m_adc_speeds[is].speed);

	    if (m_adc_speeds[is].speed > speedMax) {
		speedMax= m_adc_speeds[is].speed;
		m_adc_speed_max= is;
	    }
	    is++;
	}
    }      
}

//-----------------------------------------------------
// @brief	get ADC/Speed settings
// @param	adc pais adc/speed index (if =-1, set to max speed)
//
//-----------------------------------------------------
void Camera::setAdcSpeed(int adc)
{
    DEB_MEMBER_FUNCT();
    int is;
    
    // -- Initialise ad speed
    if ((adc == -1) || (adc > m_adc_speed_number))
    {
        is = m_adc_speed_max;
    }
    else
    {
        is  = adc;
    }
    if (andor3Error(SetADChannel(m_adc_speeds[is].adc)))
    {
        DEB_ERROR() << "Failed to set ADC channel #" << m_adc_speeds[is].adc <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set ADC channel";            
    }
    if (andor3Error(SetHSSpeed(0, m_adc_speeds[is].hss)))
    {
        DEB_ERROR() << "Failed to set HSS #" << m_adc_speeds[is].hss <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set HSS";            
    }    
    m_adc = is;
    
    DEB_TRACE() << "ADC speed set to " << m_adc_speeds[is].speed << " MHz";
}


//-----------------------------------------------------
// @brief get possible VSS (vertical shift speed) for controller
//
// Initialise the list of possible vss index and their value
// Get also the recommended VSS.
//-----------------------------------------------------
void Camera::initVSS()
{
    DEB_MEMBER_FUNCT();
    float speed;
    int ivss;


    // --- number of ADC
    if (andor3Error(GetNumberVSSpeeds(&m_vss_number)))
    {
        DEB_ERROR() << "Cannot get number of possible VSS" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get number of possible VSS";            
    }

    // --- get VSS value for each
    m_vsspeeds = new float[m_vss_number];
    for (ivss=0; ivss<m_vss_number; ivss++)
    {
	if (andor3Error(GetVSSpeed(ivss, &m_vsspeeds[ivss])))
        {
	    DEB_ERROR() << "Cannot get VSS value for #" << ivss <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Cannot get VSS value";
	}
    }

    // --- get recommended VSS value
    if (andor3Error(GetFastestRecommendedVSSpeed(&m_vss_best, &speed)))
    {
	m_vss_best = 0;
        DEB_ERROR() << "Cannot get recommended VSS speed. Set it to 0" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Cannot get recommended VSS speed. Set it to 0";

    }
}


//-----------------------------------------------------
// @brief	get Vertical Shift Speed
// @param	vss index (if =-1, set to recommended)
//
//-----------------------------------------------------
void Camera::setVSS(int vss) 
{
    DEB_MEMBER_FUNCT();
    int is;

    if ((vss == -1)||(vss > m_vss_number))
    {
	is = m_vss_best;
    } 
    else
    {
	is = vss;
    }
    if (andor3Error(SetVSSpeed(is)))
    {
	DEB_ERROR() << "Failed to set VSS #" << is <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set VSS";
    }
    m_vss = is;

    DEB_TRACE() << "VSSpeed Set to " <<m_vsspeeds[is] << "us";
}

//-----------------------------------------------------
// @brief	get possible Preamp Gain values
//
// Initialise the list of possible gain index and their value
//
//-----------------------------------------------------
void Camera::initPGain()
{
    DEB_MEMBER_FUNCT();
    int ig;
    float gmax;

    // --- get number of possible gains
    if (andor3Error(GetNumberPreAmpGains(&m_gain_number)))
    {
	DEB_ERROR() << "Failed to get number of preamp gain" <<" : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Failed to get number of preamp gain";
    }

    // --- get gain value for each
    gmax = 0.;
    m_preamp_gains = new float[m_gain_number];
    for (ig=0; ig<m_gain_number; ig++)
    {
	if (andor3Error(GetPreAmpGain(ig, &m_preamp_gains[ig])))
        {
	    DEB_ERROR() << "Failed to get gain #" << ig <<" : error code = " << m_camera_error_str;
	    THROW_HW_ERROR(Error) << "Failed to get gain";
	}
	if (m_preamp_gains[ig] >= gmax)
        {
	    gmax = m_preamp_gains[ig];
	    m_gain_max = ig;
	}
    }
}

//-----------------------------------------------------
// @brief	set Preamp Gain
// @param	gain premap gain index
//
//-----------------------------------------------------
void Camera::setPGain(int gain) 
{
    DEB_MEMBER_FUNCT();
    int ig;

    if (gain==-1) 
    {
	ig= m_gain_max;
    }
    else
    {
	ig= gain;
    }

    if (andor3Error(SetPreAmpGain(ig)))
    {
	DEB_ERROR() << "Failed to set Preamp Gain #" << ig <<" : error code = " << m_camera_error_str;
	THROW_HW_ERROR(Error) << "Failed to set Preamp Gain";
    }
    m_gain= ig;

    DEB_TRACE() << "Preamp Gain set to x" << m_preamp_gains[ig];
}

//-----------------------------------------------------
// @brief	set external trigger for fast mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Camera::setFastExtTrigger(bool flag)
{
    DEB_MEMBER_FUNCT();
    if (andor3Error(SetFastExtTrigger((flag)?1:0)))
    {
        DEB_ERROR() << "Failed to set ext-trigger fast mode" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set ext-trigger fast mode";      
    }
    m_fasttrigger = flag;

}

//-----------------------------------------------------
// @brief	get external fast trigger mode
// @param	flag fast or not (boolean)
//
//-----------------------------------------------------
void Camera::getFastExtTrigger(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_fasttrigger;
}


//-----------------------------------------------------
// @brief	set the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::setShutterLevel(int level)
{
    DEB_MEMBER_FUNCT();

    if (andor3Error(SetShutter(level, m_shutter_mode, m_shutter_close_time, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed to set shutter level" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter level";          
    }
    m_shutter_level = level;
}

//-----------------------------------------------------
// @brief	get the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::getShutterLevel(int& level)
{
    DEB_MEMBER_FUNCT();
    level = m_shutter_level;    
}

*/

//-----------------------------------------------------
// @brief	set the shutter mode 
// @param	mode FRAME only // DONE (fixed value)
//
//-----------------------------------------------------
void Camera::setShutterMode(ShutterMode mode)
{
  DEB_MEMBER_FUNCT();
  /*
    // --- SetShutter() param mode is both used to set  auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close
    int aMode = (mode == FRAME)? 0:2;
    if (mode == FRAME)
    {    
        if (andor3Error(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time)))
        {
            DEB_ERROR() << "Failed to set the shutter mode" <<" : error code = " << m_camera_error_str;
            THROW_HW_ERROR(Error) << "Failed to set the shutter mode";          
        }
    }
   m_shutter_mode = mode;
   */
  m_shutter_mode = ShutterAutoFrame;
}

//-----------------------------------------------------
// @brief	return the shutter mode
// @param	mode FRAME or MANUAL // DONE (fixed value)
//
//-----------------------------------------------------
void Camera::getShutterMode(ShutterMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_shutter_mode;
}


//-----------------------------------------------------
// @brief	set the shutter open or close    // DONE
// @param	flag True-Open / False-Close
// // DONE : do nothing !!!
//-----------------------------------------------------
void Camera::setShutter(bool flag)
{
  DEB_MEMBER_FUNCT();
  /*
    // --- SetShutter() param mode is both used to set in auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close
    int aMode = (flag)? 1:2; 
    if (andor3Error(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed close/open the shutter" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set close/open the shutter";          
    }
   m_shutter_state = flag;
   */
}


//-----------------------------------------------------
// @brief	return the status of the shutter  // DONE
// @param	flag True-Open / False-Close
// @TODO Curently return the value of CameraAcquiring
//-----------------------------------------------------
void Camera::getShutter(bool& flag)
{
  DEB_MEMBER_FUNCT();
  //  flag = m_shutter_state;
  if ( AT_SUCCESS != getBool(m_camera_handle, L"CameraAcquiring", &flag) ) {
    DEB_ERROR() << "Cannot get shutter state (CameraAcquiring)" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get shutter state (CameraAcquiring)";
  }
}

/*
//-----------------------------------------------------
// @brief	set the shutter opening time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::setShutterOpenTime(double tm)
{
    DEB_MEMBER_FUNCT();
    int aTime = tm *1000;
    
    if (andor3Error(SetShutter(m_shutter_level, m_shutter_mode, m_shutter_close_time, aTime)))
    {
        DEB_ERROR() << "Failed to set shutter openning time" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter opening time";          
    }
    m_shutter_open_time = aTime;
}

//-----------------------------------------------------
// @brief	get the shutter opening time
// @param   tm time in seconds
//
//-----------------------------------------------------
void Camera::getShutterOpenTime(double& tm)
{
    DEB_MEMBER_FUNCT();
    tm = m_shutter_open_time/1000;
}

//-----------------------------------------------------
// @brief	set the shutter closing time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::setShutterCloseTime(double tm)
{
    DEB_MEMBER_FUNCT();
    int aTime = tm *1000;
    
    if (andor3Error(SetShutter(m_shutter_level, m_shutter_mode, aTime, m_shutter_open_time)))
    {
        DEB_ERROR() << "Failed to set shutter closing time" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to set shutter closing time";          
    }
    m_shutter_close_time = aTime;
}


//-----------------------------------------------------
// @brief	get the shutter closing time
// @param	tm time in seconds
//
//-----------------------------------------------------
void Camera::getShutterCloseTime(double& tm)
{
    DEB_MEMBER_FUNCT();
    tm = m_shutter_close_time/1000;
}
*/

//-----------------------------------------------------
// @brief	set the temperature set-point // DONE
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::setTemperatureSP(double temp)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setFloat(m_camera_handle, L"TargetSensorTemperature", temp) ) {
    DEB_ERROR() << "Failed to set temperature set-point" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to set temperature set-point";          
  }
  // As advised by Andor SDK 3 doc : proof-reading after setting :
  if ( AT_SUCCESS != getFloat(m_camera_handle, L"TargetSensorTemperature", &m_temperature_sp) ) {
    DEB_ERROR() << "Failed to proof-read temperature set-point" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to proof-read temperature set-point";          
  }
}

//-----------------------------------------------------
// @brief	return the temperature set-point // DONE (trusting the cached value)
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureSP(double& temp)
{
    DEB_MEMBER_FUNCT();
    temp = m_temperature_sp;
}


//-----------------------------------------------------
// @brief	Gets the real temperature of the detector sensor // DONE
// @param	temp temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperature(double& temp)
{
    DEB_MEMBER_FUNCT();
    if ( AT_SUCCESS != getFloat(m_camera_handle, L"SensorTemperature", &temp) ) {
        DEB_ERROR() << "Failed to read temperature" <<" : error code = " << m_camera_error_str;
        THROW_HW_ERROR(Error) << "Failed to read temperature";          
    }
}

//-----------------------------------------------------
// @brief	start or Stop the cooler    // DONE
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Camera::setCooler(bool flag)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setBool(m_camera_handle, L"SensorCooling", flag) ) {
    DEB_ERROR() << "Failed to set cooler" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to set cooler";          
  }
  // As advised by Andor SDK 3 doc : proof-reading after setting :
  if ( AT_SUCCESS != getBool(m_camera_handle, L"SensorCooling", &m_cooler) ) {
    DEB_ERROR() << "Failed to proof-read cooler" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to proof-read cooler";          
  }
}

//-----------------------------------------------------
// @brief	Get the Cooler status     // DONE (trusting the cached value)
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void Camera::getCooler(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_cooler;
}

//-----------------------------------------------------
// @brief	Gets cooling status
// @param	status status as a string
//
//-----------------------------------------------------
void Camera::getCoolingStatus(std::string& status)   
{
  DEB_MEMBER_FUNCT();
    
  wchar_t		wcs_status_string[1024];
  if ( AT_SUCCESS != getEnumString(m_camera_handle, L"TemperatureStatus", wcs_status_string, 1023) ) {
    DEB_ERROR() << "Failed to read cooling status" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to cooling status";          
  }
  status = WStringToString(wcs_status_string);
}

//-----------------------------------------------------
// @brief handle the andor3 error codes
//-----------------------------------------------------
bool Camera::andor3Error(int code) const
{
  m_camera_error = code;
  //  m_camera_error_str = m_andor3_error_maps[code]; // Not const !
  m_camera_error_str = m_andor3_error_maps.at(code);

  return ( AT_SUCCESS != code );
}

//-----------------------------------------------------
// @brief just build a map of error codes
//-----------------------------------------------------
void Camera::_mapAndor3Error()
{
  m_andor3_error_maps[AT_SUCCESS] = "'AT_SUCCESS' : Function call has been successful";
  m_andor3_error_maps[AT_ERR_NOTINITIALISED] = "'AT_ERR_NOTINITIALISED' : Function called with an uninitialised handle";
  m_andor3_error_maps[AT_ERR_NOTIMPLEMENTED] = "'AT_ERR_NOTIMPLEMENTED' : Feature has not been implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_READONLY] = "'AT_ERR_READONLY' : Feature is read only";
  m_andor3_error_maps[AT_ERR_NOTREADABLE] = "'AT_ERR_NOTREADABLE' : Feature is currently not readable";
  m_andor3_error_maps[AT_ERR_NOTWRITABLE] = "'AT_ERR_NOTWRITABLE' : Feature is currently not writable";
  m_andor3_error_maps[AT_ERR_OUTOFRANGE] = "'AT_ERR_OUTOFRANGE' : Value is outside the maximum and minimum limits";
  m_andor3_error_maps[AT_ERR_INDEXNOTAVAILABLE] = "'AT_ERR_INDEXNOTAVAILABLE' : Index is currently not available";
  m_andor3_error_maps[AT_ERR_INDEXNOTIMPLEMENTED] = "'AT_ERR_INDEXNOTIMPLEMENTED' : Index is not implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_EXCEEDEDMAXSTRINGLENGTH] = "'AT_ERR_EXCEEDEDMAXSTRINGLENGTH' : String value provided exceeds the maximum allowed length";
  m_andor3_error_maps[AT_ERR_CONNECTION] = "'AT_ERR_CONNECTION' : Error connecting to or disconnecting from hardware";
  m_andor3_error_maps[AT_ERR_NODATA] = "AT_ERR_NODATA";
  m_andor3_error_maps[AT_ERR_INVALIDHANDLE] = "AT_ERR_INVALIDHANDLE";
  m_andor3_error_maps[AT_ERR_TIMEDOUT] = "'AT_ERR_TIMEDOUT' : The AT_WaitBuffer function timed out while waiting for data arrive in output queue";
  m_andor3_error_maps[AT_ERR_BUFFERFULL] = "'AT_ERR_BUFFERFULL' : The input queue has reached its capacity";
  m_andor3_error_maps[AT_ERR_INVALIDSIZE] = "'AT_ERR_INVALIDSIZE' : The size of a queued buffer did not match the frame size";
  m_andor3_error_maps[AT_ERR_INVALIDALIGNMENT] = "'AT_ERR_INVALIDALIGNMENT' : A queued buffer was not aligned on an 8-byte boundary";
  m_andor3_error_maps[AT_ERR_COMM] = "'AT_ERR_COMM' : An error has occurred while communicating with hardware";
  m_andor3_error_maps[AT_ERR_STRINGNOTAVAILABLE] = "'AT_ERR_STRINGNOTAVAILABLE' : Index / String is not available";
  m_andor3_error_maps[AT_ERR_STRINGNOTIMPLEMENTED] = "'AT_ERR_STRINGNOTIMPLEMENTED' : Index / String is not implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_NULL_FEATURE] = "AT_ERR_NULL_FEATURE";
  m_andor3_error_maps[AT_ERR_NULL_HANDLE] = "AT_ERR_NULL_HANDLE";
  m_andor3_error_maps[AT_ERR_NULL_IMPLEMENTED_VAR] = "AT_ERR_NULL_IMPLEMENTED_VAR";
  m_andor3_error_maps[AT_ERR_NULL_READABLE_VAR] = "AT_ERR_NULL_READABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_READONLY_VAR] = "AT_ERR_NULL_READONLY_VAR";
  m_andor3_error_maps[AT_ERR_NULL_WRITABLE_VAR] = "AT_ERR_NULL_WRITABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_MINVALUE] = "AT_ERR_NULL_MINVALUE";
  m_andor3_error_maps[AT_ERR_NULL_MAXVALUE] = "AT_ERR_NULL_MAXVALUE";
  m_andor3_error_maps[AT_ERR_NULL_VALUE] = "AT_ERR_NULL_VALUE";
  m_andor3_error_maps[AT_ERR_NULL_STRING] = "AT_ERR_NULL_STRING";
  m_andor3_error_maps[AT_ERR_NULL_COUNT_VAR] = "AT_ERR_NULL_COUNT_VAR";
  m_andor3_error_maps[AT_ERR_NULL_ISAVAILABLE_VAR] = "AT_ERR_NULL_ISAVAILABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_MAXSTRINGLENGTH] = "AT_ERR_NULL_MAXSTRINGLENGTH";
  m_andor3_error_maps[AT_ERR_NULL_EVCALLBACK] = "AT_ERR_NULL_EVCALLBACK";
  m_andor3_error_maps[AT_ERR_NULL_QUEUE_PTR] = "AT_ERR_NULL_QUEUE_PTR";
  m_andor3_error_maps[AT_ERR_NULL_WAIT_PTR] = "AT_ERR_NULL_WAIT_PTR";
  m_andor3_error_maps[AT_ERR_NULL_PTRSIZE] = "AT_ERR_NULL_PTRSIZE";
  m_andor3_error_maps[AT_ERR_NOMEMORY] = "AT_ERR_NOMEMORY";
  m_andor3_error_maps[AT_ERR_DEVICEINUSE] = "AT_ERR_DEVICEINUSE";
  m_andor3_error_maps[AT_ERR_HARDWARE_OVERFLOW] = "AT_ERR_HARDWARE_OVERFLOW";
  
  /*
   m_andor3_error_maps[DRV_P11INVALID] = "DRV_P11INVALID";
   m_andor3_error_maps[DRV_GATESTEPERROR] = "DRV_GATESTEPERROR";
   m_andor3_error_maps[DRV_INVALID_COUNTCONVERT_MODE] = "DRV_INVALID_COUNTCONVERT_MODE";
   m_andor3_error_maps[DRV_OA_NULL_ERROR] = "DRV_OA_NULL_ERROR";
   m_andor3_error_maps[DRV_OA_PARSE_DTD_ERROR] = "DRV_OA_PARSE_DTD_ERROR";
   m_andor3_error_maps[DRV_OA_DTD_VALIDATE_ERROR] = "DRV_OA_DTD_VALIDATE_ERROR";
   m_andor3_error_maps[DRV_OA_FILE_ACCESS_ERROR] = "DRV_OA_FILE_ACCESS_ERROR";
   m_andor3_error_maps[DRV_OA_FILE_DOES_NOT_EXIST] = "DRV_OA_FILE_DOES_NOT_EXIST";
   m_andor3_error_maps[DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR] = "DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR";
   m_andor3_error_maps[DRV_OA_PRESET_FILE_NOT_LOADED] = "DRV_OA_PRESET_FILE_NOT_LOADED";
   m_andor3_error_maps[DRV_OA_USER_FILE_NOT_LOADED] = "DRV_OA_USER_FILE_NOT_LOADED";
   m_andor3_error_maps[DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED] = "DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED";
   m_andor3_error_maps[DRV_OA_INVALID_FILE] = "DRV_OA_INVALID_FILE";
   m_andor3_error_maps[DRV_OA_FILE_HAS_BEEN_MODIFIED] = "DRV_OA_FILE_HAS_BEEN_MODIFIED";
   m_andor3_error_maps[DRV_OA_BUFFER_FULL] = "DRV_OA_BUFFER_FULL";
   m_andor3_error_maps[DRV_OA_INVALID_STRING_LENGTH] = "DRV_OA_INVALID_STRING_LENGTH";
   m_andor3_error_maps[DRV_OA_INVALID_CHARS_IN_NAME] = "DRV_OA_INVALID_CHARS_IN_NAME";
   m_andor3_error_maps[DRV_OA_INVALID_NAMING] = "DRV_OA_INVALID_NAMING";
   m_andor3_error_maps[DRV_OA_GET_CAMERA_ERROR] = "DRV_OA_GET_CAMERA_ERROR";
   m_andor3_error_maps[DRV_OA_MODE_ALREADY_EXISTS] = "DRV_OA_MODE_ALREADY_EXISTS";
   m_andor3_error_maps[DRV_OA_STRINGS_NOT_EQUAL] = "DRV_OA_STRINGS_NOT_EQUAL";
   m_andor3_error_maps[DRV_OA_NO_USER_DATA] = "DRV_OA_NO_USER_DATA";
   m_andor3_error_maps[DRV_OA_VALUE_NOT_SUPPORTED] = "DRV_OA_VALUE_NOT_SUPPORTED";
   m_andor3_error_maps[DRV_OA_MODE_DOES_NOT_EXIST] = "DRV_OA_MODE_DOES_NOT_EXIST";
   m_andor3_error_maps[DRV_OA_CAMERA_NOT_SUPPORTED] = "DRV_OA_CAMERA_NOT_SUPPORTED";
   m_andor3_error_maps[DRV_OA_FAILED_TO_GET_MODE] = "DRV_OA_FAILED_TO_GET_MODE";
   */
}

int
Camera::printInfoForProp(AT_H iCamHandle, const AT_WC * iPropName, A3_TypeInfo iPropType)
{
  DEB_MEMBER_FUNCT();

  int i_err = 0;
  
  AT_BOOL b_exists;
  AT_BOOL b_readonly;
  AT_BOOL b_writable;
  AT_BOOL b_readable;
  
  DEB_TRACE() << "Retrieving information on property named \"" << WStringToString(iPropName) << "\".\n";
  
  // Implemented
  if ( AT_SUCCESS != andor3Error(AT_IsImplemented(iCamHandle, iPropName, &b_exists)) ) {
    DEB_TRACE() << "Error in printInfoForProp : " << m_camera_error_str;
    return i_err;
  }
  DEB_TRACE() << "\tIsImplemented = " << atBoolToString(b_exists);

  if ( ! b_exists ) {
    DEB_TRACE() << "No more information to query, since the feature does not \"exists\" for this camera/driver/SDK.";
    return i_err;
  }
  
  // ReadOnly
  andor3Error(AT_IsReadOnly(iCamHandle, iPropName, &b_readonly));
  DEB_TRACE() << "\tIsReadOnly = " << atBoolToString(b_readonly);
  
  // Writable
  andor3Error(AT_IsWritable(iCamHandle, iPropName, &b_writable));
  DEB_TRACE() << "\tIsWritable = " << atBoolToString(b_writable);
  
  // Readable
  andor3Error(AT_IsReadable(iCamHandle, iPropName, &b_readable));
  DEB_TRACE() << "\tIsReadable = " << atBoolToString(b_readable);
  
  if ( ! b_readable ) {
    DEB_TRACE() << "Since the property is not readable at this time, we will stop here.";
    return i_err;
  }
  
  // Now getting the value itself : we absolutely need now to know the type of the feature :
  if ( Camera::Unknown == iPropType ) {
    DEB_TRACE() << "Could not retrieve information on a property of unknown type !!\n"
    << "Returning error code!!";
    return -1;
  }
  
  AT_64		i_Value;
  double	d_Value;
  AT_BOOL	b_Value;
  int		enum_Value;
  int		enum_Count;
  AT_WC		s_Value[1024];
  int		s_MaxLen;
  
  switch (iPropType) {
    case Camera::Int:
      DEB_TRACE() << "\tFeature of type INTEGER";
      
      if ( AT_SUCCESS == andor3Error(AT_GetInt(iCamHandle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tValue = " << i_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetIntMax(iCamHandle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMax value = " << i_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetIntMin(iCamHandle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMin value = " << i_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Float:
      DEB_TRACE() << "\tFeature of type FLOAT";
      if ( AT_SUCCESS == andor3Error(AT_GetFloat(iCamHandle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tValue = " << d_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_GetFloatMax(iCamHandle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMax value = " << d_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_GetFloatMin(iCamHandle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMin value = " << d_Value;
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Bool:
      DEB_TRACE() << "\tFeature of type BOOLEAN";
      if ( AT_SUCCESS == andor3Error(AT_GetBool(iCamHandle, iPropName, &b_Value)) )
        DEB_TRACE() << "\tValue = " << atBoolToString(b_Value);
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Enum:
      DEB_TRACE() << "\tFeature of type ENUM";
      if ( AT_SUCCESS == andor3Error(AT_GetEnumIndex(iCamHandle, iPropName, &enum_Value)) ) {
        DEB_TRACE() << "\tValue = (" << enum_Value << ")";
      if ( AT_SUCCESS == andor3Error(AT_GetEnumStringByIndex(iCamHandle, iPropName, enum_Value, s_Value, 1024)) )
          DEB_TRACE() << " \"" << WStringToString(s_Value) << "\"";
        else 
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexImplemented(iCamHandle, iPropName, enum_Value, &b_Value)) )
          DEB_TRACE() << "; implemented = " << atBoolToString(b_Value);
        else 
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexAvailable(iCamHandle, iPropName, enum_Value, &b_Value)) )
          DEB_TRACE() << "; available = " << atBoolToString(b_Value);
        else 
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
        DEB_TRACE() << ".";
      }
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;

      if ( AT_SUCCESS == andor3Error(AT_GetEnumCount(iCamHandle, iPropName, &enum_Count)) ) {
        DEB_TRACE() << "\tAvailable choices are (" << enum_Count << ") :";
        for ( int i=0; enum_Count != i; ++i ) {
          DEB_TRACE() << "\t\t(" << i << ")";
          if ( AT_SUCCESS == andor3Error(AT_GetEnumStringByIndex(iCamHandle, iPropName, i, s_Value, 1024)) )
            DEB_TRACE() << " \"" << WStringToString(s_Value) << "\"";
          else 
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexImplemented(iCamHandle, iPropName, i, &b_Value)) )
            DEB_TRACE() << "; implemented = " << atBoolToString(b_Value);
          else 
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexAvailable(iCamHandle, iPropName, i, &b_Value)) )
            DEB_TRACE() << "; available = " << atBoolToString(b_Value);
          else 
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          DEB_TRACE() << ".";
        }
      }
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;

      break;
      
    case Camera::String :
      DEB_TRACE() << "\tFeature of type STRING";
      if ( AT_SUCCESS == andor3Error(AT_GetString(iCamHandle, iPropName, s_Value, 1024)) )
        DEB_TRACE() << "\tValue = \"" <<  WStringToString(s_Value) << "\"";
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetStringMaxLength(iCamHandle, iPropName, &s_MaxLen)) ) {
        DEB_TRACE() << "\tMaximum length of this property's string = " << s_MaxLen << ".";
      }
      else 
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    default:
      DEB_TRACE() << "\tNot TREATED case so far !!!";
      break;
  }
  return i_err;
}

int
Camera::setInt(AT_H Hndl, const AT_WC* Feature, AT_64 Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetInt(Hndl, Feature, Value));
}

int
Camera::getInt(AT_H Hndl, const AT_WC* Feature, AT_64* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetInt(Hndl, Feature, Value));
}

int
Camera::getIntMax(AT_H Hndl, const AT_WC* Feature, AT_64* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetIntMax(Hndl, Feature, MaxValue));
}

int
Camera::getIntMin(AT_H Hndl, const AT_WC* Feature, AT_64* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetIntMin(Hndl, Feature, MinValue));
}

int
Camera::setFloat(AT_H Hndl, const AT_WC* Feature, double Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetFloat(Hndl, Feature, Value));
}

int
Camera::getFloat(AT_H Hndl, const AT_WC* Feature, double* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloat(Hndl, Feature, Value));
}

int
Camera::getFloatMax(AT_H Hndl, const AT_WC* Feature, double* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloatMax(Hndl, Feature, MaxValue));
}

int
Camera::getFloatMin(AT_H Hndl, const AT_WC* Feature, double* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloatMin(Hndl, Feature, MinValue));
}

int
Camera::setBool(AT_H Hndl, const AT_WC* Feature, bool Value)
{
  DEB_MEMBER_FUNCT();
  AT_BOOL newBool = Value;
  return andor3Error(AT_SetBool(Hndl, Feature, newBool));
}

int
Camera::getBool(AT_H Hndl, const AT_WC* Feature, bool* Value) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL	newBool;
  int i_Err = andor3Error(AT_GetBool(Hndl, Feature, &newBool));
  *Value = newBool;
  return i_Err;
}

int
Camera::setEnumIndex(AT_H Hndl, const AT_WC* Feature, int Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetEnumIndex(Hndl, Feature, Value));
}

int
Camera::setEnumString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetEnumString(Hndl, Feature, String));
}

int
Camera::getEnumIndex(AT_H Hndl, const AT_WC* Feature, int* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumIndex(Hndl, Feature, Value));
}

int
Camera::getEnumString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  int Value;
  int i_Err = andor3Error(AT_GetEnumIndex(Hndl, Feature, &Value));
  if ( AT_SUCCESS != i_Err ) 
    return i_Err;
  return andor3Error(AT_GetEnumStringByIndex(Hndl, Feature, Value, String, StringLength));
}

int
Camera::getEnumCount(AT_H Hndl,const  AT_WC* Feature, int* Count) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumCount(Hndl, Feature, Count));
}

int
Camera::isEnumIndexAvailable(AT_H Hndl, const AT_WC* Feature, int Index, bool* Available) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isAvailable;
  int i_Err = andor3Error(AT_IsEnumIndexAvailable(Hndl, Feature, Index, &isAvailable));
  *Available = isAvailable;
  return i_Err;
}
int
Camera::isEnumIndexImplemented(AT_H Hndl, const AT_WC* Feature, int Index, bool* Implemented) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isImplemented;
  int i_Err = andor3Error(AT_IsEnumIndexAvailable(Hndl, Feature, Index, &isImplemented));
  *Implemented = isImplemented;
  return i_Err;
}

int
Camera::getEnumStringByIndex(AT_H Hndl, const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumStringByIndex(Hndl, Feature, Index, String, StringLength));
}

int
Camera::getEnumIndexByString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int *Index) const
{
  DEB_MEMBER_FUNCT();

  int       i_Err;
  int   		i_enumCount;
  int     	i_enumIndex;
  const int i_maxStringLen = 1024;
  AT_WC   	wcs_enumString[i_maxStringLen + 5];

  if ( AT_SUCCESS != (i_Err = getEnumCount(Hndl, Feature, &i_enumCount)) ) {
    DEB_ERROR() << "Failed to get Enum Count" << " : error code = " << m_camera_error_str;
    return i_Err;
  }
  for (i_enumIndex = 0; i_enumCount != i_enumIndex; ++i_enumIndex) {
    if ( AT_SUCCESS != getEnumStringByIndex(Hndl, Feature, i_enumIndex, wcs_enumString, i_maxStringLen) ) {
      DEB_ERROR() << "Failed to get Enum String" << " : error code = " << m_camera_error_str;
      return i_Err;
    }
    if ( ! wcscmp(wcs_enumString, String) ) {
      break;
    }
  }
  if ( i_enumCount == i_enumIndex ) {
    DEB_ERROR() << "Unable to find index of enum string '" << WStringToString(String) << "' in '" << WStringToString(Feature) << "' : no such choice.";
    *Index = -1;
    i_Err = AT_ERR_INDEXNOTAVAILABLE;
  }
  else {
    *Index = i_enumIndex;
    i_Err = AT_SUCCESS;
  }
  return i_Err;
}

int
Camera::setString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String)
{ 
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetString(Hndl, Feature, String));
}

int
Camera::getString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetString(Hndl, Feature, String, StringLength));
}

int 
Camera::sendCommand(AT_H Hndl, const AT_WC* Feature)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_Command(Hndl, Feature));
}
