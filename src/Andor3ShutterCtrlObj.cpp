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
#include "Andor3ShutterCtrlObj.h"

using namespace lima;
using namespace lima::Andor3;
using namespace std;

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
ShutterCtrlObj::ShutterCtrlObj(Camera& cam)
: m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
ShutterCtrlObj::~ShutterCtrlObj()
{
  DEB_DESTRUCTOR();
}

//-----------------------------------------------------
// @brief return true if the mode is valid // DONE : Only ShutterAutoFrame
//-----------------------------------------------------
bool ShutterCtrlObj::checkMode(ShutterMode shut_mode) const
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(shut_mode);
  
  bool valid_mode;
  switch (shut_mode) 
  {	
    case ShutterAutoFrame:
      //    case ShutterManual:
      valid_mode = true;
      break;
    default:
      // No AutoSequence mode for Andor3 !
      valid_mode = false;
  }
  
  DEB_RETURN() << DEB_VAR1(valid_mode);
  return valid_mode;
}

//-----------------------------------------------------
// @brief return the shutter valid mode list // DONE : Only ShutterAutoFrame
//-----------------------------------------------------
void ShutterCtrlObj::getModeList(ShutterModeList& mode_list) const
{
  DEB_MEMBER_FUNCT();
  mode_list.push_back(ShutterAutoFrame);
  //    mode_list.push_back(ShutterManual);
}

//-----------------------------------------------------
// @brief set the shutter mode		// DONE : nothing to do, no shutter.
//-----------------------------------------------------
void ShutterCtrlObj::setMode(ShutterMode shut_mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(shut_mode);
  
  if (!checkMode(shut_mode))
    THROW_HW_ERROR(InvalidValue) << "Invalid " 
    << DEB_VAR1(shut_mode);
  
  //    Camera::ShutterMode cam_mode;
  //    cam_mode = (shut_mode == ShutterAutoFrame) ? Camera::FRAME : Camera::MANUAL;
  //    m_cam.setShutterMode(cam_mode);
}

//-----------------------------------------------------
// @brief return the shutter mode // DONE : always returning FRAME !
//-----------------------------------------------------
void ShutterCtrlObj::getMode(ShutterMode& shut_mode) const
{
  DEB_MEMBER_FUNCT();
  
  shut_mode = ShutterAutoFrame;
  //    Camera::ShutterMode cam_mode;
  //    m_cam.getShutterMode(cam_mode);
  //    shut_mode = (cam_mode == Camera::FRAME) ? ShutterAutoFrame : ShutterManual;
  DEB_RETURN() << DEB_VAR1(shut_mode);
}

//-----------------------------------------------------
// @brief open or close manually the shutter // DONE : nothing to do, no shutter.
//-----------------------------------------------------
void ShutterCtrlObj::setState(bool open)
{
  DEB_MEMBER_FUNCT();
  //     m_cam.setShutter(open);
}

//-----------------------------------------------------
// @brief return the shutter state, valid if the shutter // DONE : always true.
// is in manual mode
//-----------------------------------------------------
void ShutterCtrlObj::getState(bool& open) const
{
  DEB_MEMBER_FUNCT();
  //    m_cam.getShutter(open);
  open = true;
}

//-----------------------------------------------------
// @brief set the shutter opening time // DONE : nothing to do, no shutter.
//-----------------------------------------------------
void ShutterCtrlObj::setOpenTime(double shut_open_time)
{
  DEB_MEMBER_FUNCT();			
  //    m_cam.setShutterOpenTime(shut_open_time);
}

//-----------------------------------------------------
// @brief return the shutter opening time // DONE : returns the frame exposure time.
//-----------------------------------------------------
void ShutterCtrlObj::getOpenTime(double& shut_open_time) const
{
  DEB_MEMBER_FUNCT();
  
  //  m_cam.getShutterOpenTime(shut_open_time);
  m_cam.getExpTime(shut_open_time);
  DEB_RETURN() << DEB_VAR1(shut_open_time);
}

//-----------------------------------------------------
// @brief set the shutter closing time // DONE : nothing to do, no shutter.
//-----------------------------------------------------
void ShutterCtrlObj::setCloseTime(double shut_close_time)
{
  DEB_MEMBER_FUNCT();
  //  m_cam.setShutterCloseTime(shut_close_time);
}

//-----------------------------------------------------
// @brief return the shutter closing time // DONE : returning the altency time.
//-----------------------------------------------------
void ShutterCtrlObj::getCloseTime(double& shut_close_time) const
{
  DEB_MEMBER_FUNCT();
  //  m_cam.getShutterCloseTime(shut_close_time);
  m_cam.getLatTime(shut_close_time);
}
