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

#include "system.h"
#if (defined HAVE_CONFIG_H) && (!defined WIN32)
  #include "config.h"
#endif
#include "DVDVideoCodecExynos4.h"
#include "DVDDemuxers/DVDDemux.h"
#include "DVDStreamInfo.h"
#include "DVDClock.h"
#include "DVDCodecs/DVDCodecs.h"
#include "DVDCodecs/DVDCodecUtils.h"

#define MAJOR_VERSION 12

#include "settings/Settings.h"
#if MAJOR_VERSION < 13
	#include "settings/GUISettings.h"
#else
	#include "settings/DisplaySettings.h"
	#include "settings/AdvancedSettings.h"
#endif
#include "utils/fastmemcpy.h"

#include <linux/LinuxV4l2.h>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CDVDVideoCodecExynos4"

CDVDVideoCodecExynos4::CDVDVideoCodecExynos4() : CDVDVideoCodecExynos() {
  m_iFIMCCapturePlane1Size = -1;
  m_iFIMCCapturePlane2Size = -1;
  m_iFIMCCapturePlane3Size = -1;

  m_iVideoWidth = 0;
  m_iVideoHeight = 0;
  m_iConvertedWidth = 0;
  m_iConvertedHeight = 0;
  m_converterHandle = -1;
}

CDVDVideoCodecExynos4::~CDVDVideoCodecExynos4() {
  Dispose();
}

bool CDVDVideoCodecExynos4::OpenDevices() {
  m_decoderHandle = Exynos::OpenDevice(
    [](const std::string& name) { return name == "s5p-mfc-dec"; });
  if (m_decoderHandle <= 0) {
    return false;
  }

  m_converterHandle = Exynos::OpenDevice(
    [](const std::string& name) {
        return name == "fimc.2.m2m";
        return name.compare(0, 4, "fimc") == 0 && name.compare(name.size() - 4, 3, "m2m");
    });
  if (m_converterHandle <= 0) {
    close(m_decoderHandle);
    m_decoderHandle = -1;
    return false;
  }

  return true;
}

bool CDVDVideoCodecExynos4::SetupCaptureFormat(int& MFCCapturePlane1Size, int& MFCCapturePlane2Size) {
  // Get mfc capture picture format
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl(m_decoderHandle, VIDIOC_G_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_FMT Failed", CLASSNAME, __func__);
    return false;
  }
  MFCCapturePlane1Size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
  MFCCapturePlane2Size = fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
  CLog::Log(LOGNOTICE, "%s::%s - MFC CAPTURE G_FMT: fmt 0x%x (%dx%d), plane[0]=%d plane[1]=%d",
    CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
    MFCCapturePlane1Size, MFCCapturePlane2Size);

  // Setup FIMC OUTPUT fmt with data from MFC CAPTURE received on previous step
  fmt.type                                = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  fmt.fmt.pix_mp.field                    = V4L2_FIELD_ANY;
  fmt.fmt.pix_mp.pixelformat              = V4L2_PIX_FMT_NV12MT;
  fmt.fmt.pix_mp.num_planes               = V4L2_NUM_MAX_PLANES;
  if (ioctl(m_converterHandle, VIDIOC_S_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT S_FMT Failed", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT S_FMT (%dx%d)", CLASSNAME, __func__, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);

  return true;
}

bool CDVDVideoCodecExynos4::GetCaptureCrop() {
  // Get mfc capture crop
  struct v4l2_crop crop = {};
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl(m_decoderHandle, VIDIOC_G_CROP, &crop)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_CROP Failed to get crop information", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGNOTICE, "%s::%s - MFC CAPTURE G_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);
  m_iVideoWidth = crop.c.width;
  m_iVideoHeight = crop.c.height;

  // Setup FIMC OUTPUT crop with data from MFC CAPTURE received on previous step
  crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  if (ioctl(m_converterHandle, VIDIOC_S_CROP, &crop)) {
    CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT S_CROP Failed to set crop information", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT S_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);

  return true;
}

bool CDVDVideoCodecExynos4::SetupFIMC() {
  // Calculate FIMC final picture size be scaled to fit screen
  #if MAJOR_VERSION < 13
    RESOLUTION_INFO& res_info = g_settings.m_ResInfo[g_graphicsContext.GetVideoResolution()];
  #else
    RESOLUTION_INFO res_info =  CDisplaySettings::Get().GetResolutionInfo(g_graphicsContext.GetVideoResolution());
  #endif
  double ratio = std::min((double)res_info.iScreenWidth / (double)m_iVideoWidth, (double)res_info.iScreenHeight / (double)m_iVideoHeight);
  int width = (int)((double)m_iVideoWidth * ratio);
  int height = (int)((double)m_iVideoHeight * ratio);
  if (width % 2)
    width--;
  if (height % 2)
    height--;

  // Setup fimc capture
  struct v4l2_format fmt = {};
  fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix_mp.width = width;
  fmt.fmt.pix_mp.height = height;
  fmt.fmt.pix_mp.num_planes = V4L2_NUM_MAX_PLANES;
  fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
  if (ioctl(m_converterHandle, VIDIOC_S_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE S_FMT Failed", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE S_FMT %dx%d", CLASSNAME, __func__, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);
  m_iConvertedWidth = fmt.fmt.pix_mp.width;
  m_iConvertedHeight = fmt.fmt.pix_mp.height;

  // Setup FIMC CAPTURE crop
  struct v4l2_crop crop = {};
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  crop.c.left = 0;
  crop.c.top = 0;
  crop.c.width = m_iConvertedWidth;
  crop.c.height = m_iConvertedHeight;
  if (ioctl(m_converterHandle, VIDIOC_S_CROP, &crop)) {
    CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE S_CROP Failed", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE S_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);

  // Get FIMC produced picture details to adjust output buffer parameters with these values
  memzero(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl(m_converterHandle, VIDIOC_G_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE G_FMT Failed", CLASSNAME, __func__);
    return false;
  }
  m_iFIMCCapturePlane1Size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
  m_iFIMCCapturePlane2Size = fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
  m_iFIMCCapturePlane3Size = fmt.fmt.pix_mp.plane_fmt[2].sizeimage;

  // Allocate, Memory Map and queue fimc capture buffers
  m_v4l2FIMCCaptureBuffers = V4l2::Buffers(FIMC_CAPTURE_BUFFERS_CNT, m_converterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, true);
  if (!m_v4l2FIMCCaptureBuffers) {
    return false;
  }

  for (int n = 0; n < m_v4l2FIMCCaptureBuffers.size(); n++) {
    m_v4l2FIMCCaptureBuffers[n].iBytesUsed[0] = m_iFIMCCapturePlane1Size;
    m_v4l2FIMCCaptureBuffers[n].iBytesUsed[1] = m_iFIMCCapturePlane2Size;
    m_v4l2FIMCCaptureBuffers[n].iBytesUsed[2] = m_iFIMCCapturePlane3Size;
  }
  CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Succesfully allocated, mmapped and queued %d buffers", CLASSNAME, __func__, m_v4l2FIMCCaptureBuffers.size());

  return true;
}

bool CDVDVideoCodecExynos4::Open(CDVDStreamInfo &hints, CDVDCodecOptions &options) {
  if (hints.software)
    return false;

  Dispose();
  CDVDVideoCodecExynos::Open(hints, options);

  m_FIMCdequeuedBufferNumber = -1;
  m_ptsWriteIndex = 0;
  m_ptsReadIndex = 0;
  m_running = true;

  if (!OpenDevices()) {
    CLog::Log(LOGERROR, "%s::%s - Needed devices not found", CLASSNAME, __func__);
    return false;
  }
 
  // Setup mfc output queue (OUTPUT - name of the queue where TO encoded frames are streamed, CAPTURE - name of the queue where FROM decoded frames are taken)
  if (!SetupOutputFormat(hints))
    return false;
  m_v4l2MFCOutputBuffers = V4l2::Buffers(MFC_OUTPUT_BUFFERS_CNT, m_decoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, false);
  if (!m_v4l2MFCOutputBuffers) {
    return false;
  }
  if (!SendHeader(hints))
    return false;

  // Setup mfc capture queue
  int MFCCapturePlane1Size, MFCCapturePlane2Size;
  if (!SetupCaptureFormat(MFCCapturePlane1Size, MFCCapturePlane2Size))
    return false;
  if (!GetCaptureCrop())
    return false;

  // Request mfc capture buffers
  if (!SetupCaptureBuffers(MFCCapturePlane1Size, MFCCapturePlane2Size))
    return false;

  // Request fimc capture buffers
  m_v4l2FIMCOutputBuffers = V4l2::Buffers(m_converterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, m_v4l2MFCCaptureBuffers);
  if (!m_v4l2FIMCOutputBuffers) {
    return false;
  }

  if (!SetupFIMC()) {
    return false;
  }

  // Dequeue header from MFC, we don't need it anymore
  timeval ptsTime;
  uint32_t sequence;
  int index = m_v4l2MFCOutputBuffers.DequeueBuffer(sequence, ptsTime);

  if (m_v4l2FIMCOutputBuffers.StreamOn())
    CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT Stream ON", CLASSNAME, __func__);
  else
    CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT Failed to Stream ON", CLASSNAME, __func__);
  if (m_v4l2FIMCCaptureBuffers.StreamOn())
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Stream ON", CLASSNAME, __func__);
  else
    CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Failed to Stream ON", CLASSNAME, __func__);

  CLog::Log(LOGNOTICE, "%s::%s - MFC Setup succesfull, start streaming", CLASSNAME, __func__);

  m_MFCtoFIMCThread = std::thread(&CDVDVideoCodecExynos4::MFCtoFIMCLoop, this);

  return true;
}

void CDVDVideoCodecExynos4::Dispose() {
  m_running = false;

  try {
    m_MFCtoFIMCThread.join();
  } catch (std::exception &e){
    // Thread was probably not started
  }

  CLog::Log(LOGDEBUG, "%s::%s - Closing devices", CLASSNAME, __func__);
  if (m_decoderHandle >= 0) {
    if (m_v4l2MFCOutputBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Stream OFF", CLASSNAME, __func__);
    if (m_v4l2MFCCaptureBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Stream OFF", CLASSNAME, __func__);

    m_v4l2MFCOutputBuffers.clear();
    m_v4l2MFCCaptureBuffers.clear();

    close(m_decoderHandle);
  }
  if (m_converterHandle >= 0) {
    if (m_v4l2FIMCOutputBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT Stream OFF", CLASSNAME, __func__);
    if (m_v4l2FIMCCaptureBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Stream OFF", CLASSNAME, __func__);

    m_v4l2FIMCOutputBuffers.clear();
    m_v4l2FIMCCaptureBuffers.clear();

    close(m_converterHandle);
  }

  m_iVideoWidth = 0;
  m_iVideoHeight = 0;
  m_iConvertedWidth = 0;
  m_iConvertedHeight = 0;
  m_decoderHandle = -1;
  m_converterHandle = -1;

  memzero(m_videoBuffer);
}

// Find buffers processed by FIMC and return it to MFC
bool CDVDVideoCodecExynos4::ReturnBuffersToMFC() {
    timeval time;
    uint32_t sequence;
    int index;
    while((index = m_v4l2FIMCOutputBuffers.DequeueBuffer(sequence, time)) >= 0) {
        if (!m_v4l2MFCCaptureBuffers.QueueBuffer(index)) {
          CLog::Log(LOGERROR, "%s::%s - Could not queue MFC capture buffer\n", CLASSNAME, __func__);
          m_videoBuffer.iFlags      |= DVP_FLAG_DROPPED;
          m_videoBuffer.iFlags      &= DVP_FLAG_ALLOCATED;
          return false;
      }
    }
    return true;
}

// Checks if there is ready buffer and converts it to a m_videoBuffer
// Returns VC_PICTURE on success and VC_BUFFER if buffer is not ready. Or VC_ERROR on error.
int CDVDVideoCodecExynos4::DequeueBufferFromFIMC() {
  timeval ptsTime;
  uint32_t sequence;
  m_FIMCdequeuedBufferNumber = m_v4l2FIMCCaptureBuffers.DequeueBuffer(sequence, ptsTime);
  if (m_FIMCdequeuedBufferNumber < 0) {
    if (errno == EAGAIN) { // Dequeue buffer not ready, need more data on input. EAGAIN = 11
      return VC_BUFFER;
    } else {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, m_FIMCdequeuedBufferNumber, errno);
      return VC_ERROR;
    }
  }

  m_videoBuffer.iFlags          = DVP_FLAG_ALLOCATED;
  m_videoBuffer.color_range     = 0;
  m_videoBuffer.color_matrix    = 4;

  m_videoBuffer.iDisplayWidth   = m_iConvertedWidth;
  m_videoBuffer.iDisplayHeight  = m_iConvertedHeight;
  m_videoBuffer.iWidth          = m_iConvertedWidth;
  m_videoBuffer.iHeight         = m_iConvertedHeight;

  m_videoBuffer.data[0]         = 0;
  m_videoBuffer.data[1]         = 0;
  m_videoBuffer.data[2]         = 0;
  m_videoBuffer.data[3]         = 0;
    
  m_videoBuffer.format          = RENDER_FMT_YUV420P;
  m_videoBuffer.iLineSize[0]    = m_iConvertedWidth;
  m_videoBuffer.iLineSize[1]    = m_iConvertedWidth >> 1;
  m_videoBuffer.iLineSize[2]    = m_iConvertedWidth >> 1;
  m_videoBuffer.iLineSize[3]    = 0;
  m_videoBuffer.data[0]         = (BYTE*)m_v4l2FIMCCaptureBuffers[m_FIMCdequeuedBufferNumber].cPlane[0];
  m_videoBuffer.data[1]         = (BYTE*)m_v4l2FIMCCaptureBuffers[m_FIMCdequeuedBufferNumber].cPlane[1];
  m_videoBuffer.data[2]         = (BYTE*)m_v4l2FIMCCaptureBuffers[m_FIMCdequeuedBufferNumber].cPlane[2];

  std::unique_lock<std::mutex> lock(m_mutex);
  m_videoBuffer.pts = m_pts[m_ptsReadIndex];
  m_ptsReadIndex = (m_ptsReadIndex + 1) % FIMC_CAPTURE_BUFFERS_CNT;
  m_videoBuffer.dts = m_videoBuffer.pts;

  return VC_PICTURE;
}

void CDVDVideoCodecExynos4::MFCtoFIMCLoop() {
  while(m_running) {
    struct pollfd pollRequest[] = {
      {
        m_converterHandle,
        POLLOUT | POLLERR
      }, {
        m_decoderHandle,
        POLLIN | POLLERR
      }};
    if (poll(pollRequest, 2, 0) < 0) {
        CLog::Log(LOGERROR, "%s::%s - Polling output", CLASSNAME, __func__);
        return;
    }

    if (pollRequest[0].revents & POLLOUT) {
        ReturnBuffersToMFC();
    }

    if (pollRequest[1].revents & POLLIN) {
      // Transfer decoded frames from MFC to FIMC
      int index = 0;
      timeval ptsTime;
      uint32_t sequence;
      index = m_v4l2MFCCaptureBuffers.DequeueBuffer(sequence, ptsTime);
      if (index >= 0) {
        if (!m_v4l2FIMCOutputBuffers.QueueBuffer(index, ptsTime)) {
          CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, int(index), errno);
        } else {
          std::unique_lock<std::mutex> lock(m_mutex);
          m_pts[m_ptsWriteIndex] = double(ptsTime.tv_sec)*1000000.0 + double(ptsTime.tv_usec);
          m_ptsWriteIndex = (m_ptsWriteIndex + 1) % FIMC_CAPTURE_BUFFERS_CNT;
        }
      }
    }
  }
}

int CDVDVideoCodecExynos4::Decode(BYTE* pData, int iSize, double dts, double pts) {
  if(pData) {
    size_t index;
    for (index = 0; index < m_v4l2MFCOutputBuffers.size(); ++index) {
      if (!m_v4l2MFCOutputBuffers[index].bQueue) {
        break;
      }
    }
    if (index == m_v4l2MFCOutputBuffers.size()) {
      CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT All buffers are queued and busy, no space for new frame to decode. Very broken situation.", CLASSNAME, __func__);
      return VC_ERROR;
    }
    if (!SendBuffer(index, pData, iSize, pts)) {
        return VC_ERROR;
    }
  }

  if (m_FIMCdequeuedBufferNumber >= 0) {
    // XBMC has already displayed our buffer and we can use it again
    if (!m_v4l2FIMCCaptureBuffers.QueueBuffer(m_FIMCdequeuedBufferNumber)) {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Failed to queue buffer with index %d", CLASSNAME, __func__, m_FIMCdequeuedBufferNumber);
      return VC_ERROR;
    }
    m_FIMCdequeuedBufferNumber = -1;
  }

  int ret = 0;

  // Fill up MFC buffer if there is any space.
  for (size_t index = 0; index < m_v4l2MFCOutputBuffers.size(); ++index) {
    if (!m_v4l2MFCOutputBuffers[index].bQueue) {
      // We have free buffer, ask XBMC for new frame
      ret |= VC_BUFFER;
    }
  }

  do {
    struct pollfd pollRequest[] = {
      {
        m_converterHandle,
        POLLIN | POLLERR
      }, {
        m_decoderHandle,
        POLLOUT | POLLERR
      }};
    if (poll(pollRequest, 2, 0) < 0) {
        CLog::Log(LOGERROR, "%s::%s - Polling output", CLASSNAME, __func__);
        return VC_ERROR;
    }

    if (pollRequest[1].revents & POLLOUT) {
      timeval time;
      uint32_t sequence;
      if (m_v4l2MFCOutputBuffers.DequeueBuffer(sequence, time) >= 0) {
        ret |= VC_BUFFER;
      }
    }

    if (pollRequest[0].revents & POLLIN) {
      int readyBuffer = DequeueBufferFromFIMC();
      if (readyBuffer != VC_BUFFER) {
        ret |= readyBuffer;
      }
    }
  } while(!ret);

  return ret;
}
