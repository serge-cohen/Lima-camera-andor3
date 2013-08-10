
namespace Andor3
{
  class Camera
  {
%TypeHeaderCode
#include <Andor3Camera.h>
%End

  public:

    enum Status {
        Ready, Acquisition, Fault,
      };

      /*
       enum ShutterMode {
       FRAME,
       MANUAL
       };
       */

    enum A3_TypeInfo { 
        Unknown, Int, Float, Bool, Enum, String,
      };
      
      enum A3_Gain { // In the same order/index as "PreAmpGainControl"
        Gain1 = 0, Gain2 = 1, Gain3 = 2, Gain4 = 3, Gain1_Gain3 = 4, Gain1_Gain4 = 5, Gain2_Gain3 = 6, Gain2_Gain4 = 7,
      };
      
      enum A3_ShutterMode { // In the same order/index as "ElectronicShutteringMode"
        Rolling = 0, Global = 1,
      };

      enum A3_ReadOutRate { // In the same order/index as "PixelReadoutRate"
        MHz10 = 0, MHz100 = 1, MHz200 = 2, MHz280 = 3,
      };

      Camera(const std::string& bitflow_path,int camera_number=0);
      ~Camera();
      
      void startAcq();
      void stopAcq();
      
      void getImageType(ImageType& type /Out/);
      void setImageType(ImageType type);
      
      void getDetectorModel(std::string& model /Out/);
      void getDetectorType(std::string& model /Out/);
      void getDetectorImageSize(Size& size /Out/);
      
      // -- Buffer control object
      HwBufferCtrlObj* getBufferCtrlObj();
      
      //-- Synch control object
      bool checkTrigMode(TrigMode trig_mode);
      void setTrigMode(TrigMode  mode);
      void getTrigMode(TrigMode& mode /Out/);
      
      void setExpTime(double  exp_time);
      void getExpTime(double& exp_time /Out/);
      
      void setLatTime(double  lat_time);
      void getLatTime(double& lat_time /Out/);
      
      void getExposureTimeRange(double& min_expo /Out/, double& max_expo /Out/) const;
      void getLatTimeRange(double& min_lat /Out/, double& max_lat /Out/) const;    
      
      void setNbFrames(int  nb_frames);
      void getNbFrames(int& nb_frames /Out/);
      void getNbHwAcquiredFrames(int &nb_acq_frames /Out/);
      
      void checkRoi(const Roi& set_roi, Roi& hw_roi /Out/);
      void setRoi(const Roi& set_roi);
      void getRoi(Roi& hw_roi /Out/);    
      
      void checkBin(Bin& /In,Out/);
      void setBin(const Bin&);
      void getBin(Bin& /Out/);
      bool isBinningAvailable();
      void setShutterMode(ShutterMode mode);
      void getShutterMode(ShutterMode& mode /Out/);
      
      void setShutter(bool flag);
      void getShutter(bool& flag /Out/);
      
      void getPixelSize(double& sizex /Out/, double& sizey /Out/);
      
      void getStatus(Status& status /Out/);
      
      void reset();
      
	    // -- andor3 specific, LIMA don't worry about it !
      void setAdcGain(A3_Gain iGain);
      void getAdcGain(A3_Gain &oGain /Out/);
      void setAdcRate(A3_ReadOutRate iRate);
      void getAdcRate(A3_ReadOutRate &oRate /Out/);
      void setElectronicShutterMode(A3_ShutterMode iMode);
      void getElectronicShutterMode(A3_ShutterMode &oMode /Out/);
      
      void initialiseController();
      
      void setTemperatureSP(double temp);
      void getTemperatureSP(double& temp /Out/);
      void getTemperature(double& temp /Out/);
      void setCooler(bool flag);
      void getCooler(bool& flag /Out/);
      void getCoolingStatus(std::string& status /Out/);    
  };

};
