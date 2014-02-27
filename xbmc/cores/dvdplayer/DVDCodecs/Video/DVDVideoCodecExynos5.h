#pragma once

/*
 *      Copyright (C) 2005-2012 Team XBMC
 *      http://www.xbmc.org
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "DVDVideoCodecExynos.h"
#include "DVDResource.h"
#include "DVDStreamInfo.h"
#include "utils/BitstreamConverter.h"
#include <linux/LinuxV4l2.h>

#include <string>
#include "guilib/GraphicContext.h"

#ifndef V4L2_CAP_VIDEO_M2M_MPLANE
  #define V4L2_CAP_VIDEO_M2M_MPLANE       0x00004000
#endif

class CDVDVideoCodecExynos5 : public Exynos::CDVDVideoCodecExynos
{
public:
  CDVDVideoCodecExynos5();
  ~CDVDVideoCodecExynos5();

  virtual bool Open(CDVDStreamInfo &hints, CDVDCodecOptions &options);
  virtual void Dispose();
  virtual int Decode(BYTE* pData, int iSize, double dts, double pts);
  virtual void Reset();

private:

  unsigned int m_iVideoWidth;
  unsigned int m_iVideoHeight;
  unsigned int m_iOutputWidth;
  unsigned int m_iOutputHeight;

  V4L2Buffer m_v4l2OutputBuffer;
  int m_MFCDequeuedBufferNumber;
  
  // Order number of previous frame
  uint32_t m_sequence;
  uint32_t m_inputSequence;
  uint32_t m_missedFrames;
  size_t m_framesToSkip;
  bool m_isInterlaced;

  bool OpenDevices();

  bool SetupCaptureFormat(int& MFCCapturePlane1Size, int& MFCCapturePlane2Size);
  bool GetCaptureCrop();
  void PrepareOutputBuffer(int bufferIndex);
};

#define memzero(x) memset(&(x), 0, sizeof (x))
