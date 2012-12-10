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
//###########################################################################
#ifndef ANDOR3CAMERA_H
#define ANDOR3CAMERA_H

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

#include <atcore.h>
// To me the following ones are not required (at least with v3.x of the Andor SDK) ?
/*
 #ifndef DRV_P11INVALID
 
 #define DRV_P11INVALID 20087
 #define DRV_GATESTEPERROR 20092
 #define DRV_INVALID_COUNTCONVERT_MODE 20101
 
 #define DRV_OA_NULL_ERROR 20173
 #define DRV_OA_PARSE_DTD_ERROR 20174
 #define DRV_OA_DTD_VALIDATE_ERROR 20175
 #define DRV_OA_FILE_ACCESS_ERROR 20176
 #define DRV_OA_FILE_DOES_NOT_EXIST 20177
 #define DRV_OA_XML_INVALID_OR_NOT_FOUND_ERROR 20178
 #define DRV_OA_PRESET_FILE_NOT_LOADED 20179
 #define DRV_OA_USER_FILE_NOT_LOADED 20180
 #define DRV_OA_PRESET_AND_USER_FILE_NOT_LOADED 20181
 #define DRV_OA_INVALID_FILE 20182
 #define DRV_OA_FILE_HAS_BEEN_MODIFIED 20183
 #define DRV_OA_BUFFER_FULL 20184
 #define DRV_OA_INVALID_STRING_LENGTH 20185
 #define DRV_OA_INVALID_CHARS_IN_NAME 20186
 #define DRV_OA_INVALID_NAMING 20187
 #define DRV_OA_GET_CAMERA_ERROR 20188
 #define DRV_OA_MODE_ALREADY_EXISTS 20189
 #define DRV_OA_STRINGS_NOT_EQUAL 20190
 #define DRV_OA_NO_USER_DATA 20191
 #define DRV_OA_VALUE_NOT_SUPPORTED 20192
 #define DRV_OA_MODE_DOES_NOT_EXIST 20193
 #define DRV_OA_CAMERA_NOT_SUPPORTED 20194
 #define DRV_OA_FAILED_TO_GET_MODE 20195
 
 #endif
 */

#include <stdlib.h>
#include <limits>
#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

#include <ostream>

using namespace std;


namespace lima
{
  namespace Andor3
  {
    /*******************************************************************
     * \class Camera
     * \brief object controlling the andor3 camera via the Andor SDK 3.x driver
     *******************************************************************/
    class Camera
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Andor3");
	    friend class Interface;
    public:
      
      
      enum Status {
        Ready, 
        Acquisition, 
        Fault
      };

      /*
       enum ShutterMode {
       FRAME,
       MANUAL
       };
       */

      enum A3_TypeInfo { 
        Unknown,
        Int,
        Float,
        Bool,
        Enum,
        String
      };
      
      //! @TODO : later on should do a map (string to int and vice-versa) from parsed enum info
      enum A3_Gain { // In the same order/index as "PreAmpGainControl"
        Gain1 = 0,
        Gain2 = 1,
        Gain3 = 2,
        Gain4 = 3,
        Gain1_Gain3 = 4,
        Gain1_Gain4 = 5,
        Gain2_Gain3 = 6,
        Gain2_Gain4 = 7
      };
      
      //! @TODO : later on should do a map (string to int and vice-versa) from parsed enum info
      enum A3_ShutterMode { // In the same order/index as "ElectronicShutteringMode"
        Rolling = 0,
        Global = 1
      };
      
      //! @TODO : later on should do a map (string to int and vice-versa) from parsed enum info
      enum A3_ReadOutRate { // In the same order/index as "PixelReadoutRate"
        MHz10 = 0,
        MHz100 = 1,
        MHz200 = 2,
        MHz280 = 3
      };
      
      
      Camera(const std::string& bitflow_path,int camera_number=0);
      ~Camera();
      
      void startAcq();
      void stopAcq();
      
      // -- detector info object
      void getImageType(ImageType& type);
      void setImageType(ImageType type);
      
      void getDetectorType(std::string& type);
      void getDetectorModel(std::string& model);
      void getDetectorImageSize(Size& size);
      
      // -- Buffer control object
      HwBufferCtrlObj* getBufferCtrlObj();
      
      //-- Synch control object
      bool checkTrigMode(TrigMode trig_mode);
      void setTrigMode(TrigMode  mode);
      void getTrigMode(TrigMode& mode);
      
      void setExpTime(double  exp_time);
      void getExpTime(double& exp_time);
      
      void setLatTime(double  lat_time);
      void getLatTime(double& lat_time);
      
      void getExposureTimeRange(double& min_expo, double& max_expo) const;
      void getLatTimeRange(double& min_lat, double& max_lat) const;    
      
      void setNbFrames(int  nb_frames);
      void getNbFrames(int& nb_frames);
      void getNbHwAcquiredFrames(int &nb_acq_frames);
      
      void checkRoi(const Roi& set_roi, Roi& hw_roi);
      void setRoi(const Roi& set_roi);
      void getRoi(Roi& hw_roi);    
      
      void checkBin(Bin&);
      void setBin(const Bin&);
      void getBin(Bin&);
      bool isBinningAvailable();
      /*
       void setShutterOpenTime(double tm);
       void getShutterOpenTime(double& tm);
       void setShutterCloseTime(double tm);
       void getShutterCloseTime(double& tm);
       */
      void setShutterMode(ShutterMode mode);
      void getShutterMode(ShutterMode& mode);
      
      void setShutter(bool flag);
      void getShutter(bool& flag);
      
      void getPixelSize(double& sizex, double& sizey);
      
      void getStatus(Camera::Status& status);
      
      void reset();
      
	    // -- andor3 specific, LIMA don't worry about it !
      void setAdcGain(A3_Gain iGain);
      void getAdcGain(A3_Gain &oGain);
      void setAdcRate(A3_ReadOutRate iRate);
      void getAdcRate(A3_ReadOutRate &oRate);
      void setElectronicShutterMode(A3_ShutterMode iMode);
      void getElectronicShutterMode(A3_ShutterMode &oMode);
      
      void initialiseController();
      /*
       void initAdcSpeed();
       void setAdcSpeed(int adc);
       void initVSS();
       void setVSS(int vss);
       void initPGain();
       void setPGain(int gain);
       void setFastExtTrigger(bool flag);
       void getFastExtTrigger(bool& flag);
       void setShutterLevel(int level);
       void getShutterLevel(int& level);
       */
      
      void setTemperatureSP(double temp);
      void getTemperatureSP(double& temp);
      void getTemperature(double& temp);
      void setCooler(bool flag);
      void getCooler(bool& flag);
      void getCoolingStatus(std::string& status);    
      
      // Lower level functions :
      bool andor3Error(int code) const;
      void _mapAndor3Error();
      
      int printInfoForProp(AT_H iCamHandle, const AT_WC * iPropName, A3_TypeInfo iPropType);
      
      int setInt(AT_H Hndl, const AT_WC* Feature, AT_64 Value);
      int getInt(AT_H Hndl, const AT_WC* Feature, AT_64* Value) const;
      int getIntMax(AT_H Hndl, const AT_WC* Feature, AT_64* MaxValue) const;
      int getIntMin(AT_H Hndl, const AT_WC* Feature, AT_64* MinValue) const;
      
      int setFloat(AT_H Hndl, const AT_WC* Feature, double Value);
      int getFloat(AT_H Hndl, const AT_WC* Feature, double* Value) const;
      int getFloatMax(AT_H Hndl, const AT_WC* Feature, double* MaxValue) const;
      int getFloatMin(AT_H Hndl, const AT_WC* Feature, double* MinValue) const;
      
      int setBool(AT_H Hndl, const AT_WC* Feature, bool Value);
      int getBool(AT_H Hndl, const AT_WC* Feature, bool* Value) const;
      
      int setEnumIndex(AT_H Hndl, const AT_WC* Feature, int Value);
      int setEnumString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String);
      int getEnumIndex(AT_H Hndl, const AT_WC* Feature, int* Value) const;
      int getEnumString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const;
      int getEnumCount(AT_H Hndl,const  AT_WC* Feature, int* Count) const;
      int isEnumIndexAvailable(AT_H Hndl, const AT_WC* Feature, int Index, bool* Available) const;
      int isEnumIndexImplemented(AT_H Hndl, const AT_WC* Feature, int Index, bool* Implemented) const;
      int getEnumStringByIndex(AT_H Hndl, const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const;
      int getEnumIndexByString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int *Index) const;
      
      int setString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String);
      int getString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const;
      
      int sendCommand(AT_H Hndl, const AT_WC* Feature);
      //     int AT_GetStringMaxLength(AT_H Hndl, const AT_WC* Feature, int* MaxStringLength);
      
      /*
       int getIntProp(AT_H iCamHandle, const AT_WC * iPropName, int64_t *iValue);
       int getFloatProp(AT_H iCamHandle, const AT_WC * iPropName, double *iValue);
       int getBoolProp(AT_H iCamHandle, const AT_WC * iPropName, bool *iValue);
       int getEnumProp(AT_H iCamHandle, const AT_WC * iPropName, const AT_WC * iEnumString);
       int getStringProp(AT_H iCamHandle, const AT_WC * iPropName, const AT_WC * iValue); 
       
       int setIntProp(AT_H iCamHandle, const AT_WC * iPropName, int64_t iValue);
       int setFloatProp(AT_H iCamHandle, const AT_WC * iPropName, double iValue);
       int setBoolProp(AT_H iCamHandle, const AT_WC * iPropName, bool iValue);
       int setEnumProp(AT_H iCamHandle, const AT_WC * iPropName, const AT_WC * iEnumString);
       int setStringProp(AT_H iCamHandle, const AT_WC * iPropName, const AT_WC * iValue); 
       */
      
    private:
      class _AcqThread;
      friend class _AcqThread;
      
      void _stopAcq(bool);
      void _setStatus(Camera::Status status,bool force);
      
      //- acquisition thread stuff    
      _AcqThread*                 m_acq_thread;
      Cond                        m_cond;
      
      //- lima stuff
      SoftBufferCtrlObj	          m_buffer_ctrl_obj;
      int                         m_nb_frames;    
      Camera::Status              m_status;
      volatile bool               m_wait_flag;
      volatile bool               m_quit;
      volatile bool               m_thread_running;
      int                         m_image_number;
      int                         m_timeout;
      //	    double                      m_latency_time;
      Roi                         m_roi;
      Bin                         m_bin;
      Bin                         m_bin_max;
      TrigMode                    m_trig_mode;
      
      ShutterMode                 m_shutter_mode;
      /*
       bool                        m_shutter_state;
       */
      
      //- camera stuff 
      string                      m_detector_model;
      string                      m_detector_type;
      // @TODO : SerialNumber of camera.
      
      //- andor3 SDK stuff
      string                      m_bitflow_path;
      int                         m_camera_number;
      AT_H                        m_camera_handle;
      //	    Andor3Capabilities           m_camera_capabilities;
      mutable string              m_camera_error_str;
      mutable int                 m_camera_error;
      /*
       struct Adc 
       {
       int		adc;
       int		hss;
       float	        speed;
       };
       
       Adc*                        m_adc_speeds;
       int                         m_adc_speed_number;
       int                         m_adc_speed_max;
       int                         m_adc;
       int                         m_vss_number;
       float*                      m_vsspeeds;
       int                         m_vss_best;
       int                         m_vss;
       int                         m_gain_number;
       int                         m_gain_max;
       float*                      m_preamp_gains;
       int                         m_gain;
       bool                        m_fasttrigger;
       int                         m_shutter_level;
       int                         m_shutter_close_time;
       int                         m_shutter_open_time;
       int                         m_read_mode;
       int                         m_acq_mode;    
       map<int, string>            m_andor3_type_maps;            
       */
      A3_Gain											m_adc_gain;
      A3_ReadOutRate							m_adc_rate;
      A3_ShutterMode							m_electronic_shutter_mode;
      
      bool                        m_cooler;   
      double                      m_temperature_sp;   
      double                      m_exp_time;
      double                      m_exp_time_max;
      double                      m_frame_rate;
      int                         m_ring_buffer_size;                
      map<TrigMode, int>          m_trig_mode_maps;
      map<int, string>            m_andor3_error_maps;
      
      static bool						sAndorSDK3Initted;
      
    };
    
    // Some inline utility functions; used all-over in Andor3 plugin :
    inline std::wstring StringToWString(const std::string & iStr)
    {
      wchar_t		tmpWStringBuf[1024];
      
      mbstowcs(tmpWStringBuf, iStr.c_str(), 1023);
      return std::wstring(tmpWStringBuf);
      /*std::wostringstream		theWSStream;
       theWSStream << iStr.c_str();
       return theWSStream.str();
       */
    }
    
    inline std::string WStringToString(const std::wstring & iStr)
    {
      // Should use wcstombs
      char			tmpStringBuf[1024];
      
      bzero(tmpStringBuf, 1024);
      wcstombs(tmpStringBuf, iStr.c_str(), 1023);
      return std::string(tmpStringBuf);
      /*    std::ostringstream			theSStream;
       theSStream << iStr.c_str();
       return theSStream.str();
       */
    }
    
    inline std::string atBoolToString(AT_BOOL iBool)
    {
      return (iBool) ? std::string("true") : std::string("false");
    }
    
    
  } // namespace Andor3
} // namespace lima


#endif // ANDOR3CAMERA_H
