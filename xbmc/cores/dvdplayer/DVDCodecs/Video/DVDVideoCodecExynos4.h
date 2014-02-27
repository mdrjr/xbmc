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
#include "utils/BitstreamConverter.h"
#include <string>
#include <queue>
#include <list>
#include <atomic>
#include <mutex>
#include <thread>
#include "guilib/GraphicContext.h"

#ifndef V4L2_CAP_VIDEO_M2M_MPLANE
  #define V4L2_CAP_VIDEO_M2M_MPLANE       0x00004000
#endif

class CDVDVideoCodecExynos4 : public Exynos::CDVDVideoCodecExynos
{
public:
  CDVDVideoCodecExynos4();
  ~CDVDVideoCodecExynos4();

  virtual bool Open(CDVDStreamInfo &hints, CDVDCodecOptions &options);
  virtual void Dispose();
  virtual int Decode(BYTE* pData, int iSize, double dts, double pts);

protected:
  bool OpenDevices();
  bool SetupFIMC();
  bool SetupCaptureFormat(int& MFCCapturePlane1Size, int& MFCCapturePlane2Size);
  bool GetCaptureCrop();
  bool ReturnBuffersToMFC();
  int DequeueBufferFromFIMC();
  void MFCtoFIMCLoop();

  unsigned int m_iVideoWidth;
  unsigned int m_iVideoHeight;
  unsigned int m_iConvertedWidth;
  unsigned int m_iConvertedHeight;

  int m_converterHandle;

  V4l2::Buffers m_v4l2FIMCOutputBuffers;
  V4l2::Buffers m_v4l2FIMCCaptureBuffers;

  int m_iFIMCCapturePlane1Size;
  int m_iFIMCCapturePlane2Size;
  int m_iFIMCCapturePlane3Size;

  int m_FIMCdequeuedBufferNumber;
  std::atomic<bool> m_running;
  std::mutex m_mutex;
  std::thread m_MFCtoFIMCThread;

  // 2 begins to be slow.
  static const size_t FIMC_CAPTURE_BUFFERS_CNT = 3;

  // FIMC does not copy timestamp values between buffers
  double m_pts[FIMC_CAPTURE_BUFFERS_CNT];
  size_t m_ptsWriteIndex;
  size_t m_ptsReadIndex;
};

#define memzero(x) memset(&(x), 0, sizeof (x))
