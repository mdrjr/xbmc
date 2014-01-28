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
#include "DVDVideoCodecExynos5.h"
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
#define CLASSNAME "CDVDVideoCodecExynos5"

#ifndef V4L2_PIX_FMT_NV12MT_16X16
	#define V4L2_PIX_FMT_NV12MT_16X16  v4l2_fourcc('V', 'M', '1', '2') /* 12  Y/CbCr 4:2:0 16x16 macroblocks */
#endif

CDVDVideoCodecExynos5::CDVDVideoCodecExynos5() {
  m_iVideoWidth = 0;
  m_iVideoHeight = 0;
  m_iOutputWidth = 0;
  m_iOutputHeight = 0;
  m_decoderHandle = -1;
  m_dropPictures = false;
  m_framesToSkip = 0;
  
  memzero(m_v4l2OutputBuffer);
  memzero(m_videoBuffer);
}

CDVDVideoCodecExynos5::~CDVDVideoCodecExynos5() {
  Dispose();
}

bool CDVDVideoCodecExynos5::OpenDevices() {
  m_decoderHandle = Exynos::OpenDevice("s5p-mfc-dec", [](int fd) {
      struct v4l2_capability cap = {};
      if (!ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
          return ((cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
            ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) && (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE))) &&
            (cap.capabilities & V4L2_CAP_STREAMING));
      } else {
          return false;
      }
    });
  return m_decoderHandle > 0;
}

bool CDVDVideoCodecExynos5::SetupCaptureFormat(int& MFCCapturePlane1Size, int& MFCCapturePlane2Size) {
  // Set format in which decoded frames are streamed FROM mfc
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
  if (ioctl(m_decoderHandle, VIDIOC_S_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE S_FMT Failed on CAPTURE", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE S_FMT 0x%x",  CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat);

  // Get other picture format properties
  if (ioctl(m_decoderHandle, VIDIOC_G_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_FMT Failed", CLASSNAME, __func__);
    return false;
  }

  MFCCapturePlane1Size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
  MFCCapturePlane2Size = fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
  // Align width by 16, picture returned by MFC is always aligned by 16, but parameters are wrongly returned same as video size.
  m_iOutputWidth = (fmt.fmt.pix_mp.width + 15)&~15;
  m_iOutputHeight = fmt.fmt.pix_mp.height;

  CLog::Log(LOGNOTICE, "%s::%s - MFC CAPTURE G_FMT: fmt 0x%x (%dx%d), plane[0]=%d plane[1]=%d plane[2]=%d",
    CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
    MFCCapturePlane1Size, MFCCapturePlane2Size, fmt.fmt.pix_mp.plane_fmt[2].sizeimage);

  return true;
}

bool CDVDVideoCodecExynos5::GetCaptureCrop() {
  // Get mfc capture crop
  struct v4l2_crop crop = {};
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (ioctl(m_decoderHandle, VIDIOC_G_CROP, &crop)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_CROP Failed to get crop information", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGNOTICE, "%s::%s - MFC CAPTURE G_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);
  m_iVideoWidth = (crop.c.width + 15)&~15; // Align width by 8. Required for NV12 to YUV420 converter
  m_iVideoHeight = crop.c.height;

  m_v4l2OutputBuffer.cPlane[0] = new BYTE[m_iVideoWidth * m_iVideoHeight];
  m_v4l2OutputBuffer.cPlane[1] = new BYTE[m_iVideoWidth * m_iVideoHeight / 4];
  m_v4l2OutputBuffer.cPlane[2] = new BYTE[m_iVideoWidth * m_iVideoHeight / 4];

  return true;
}

bool CDVDVideoCodecExynos5::Open(CDVDStreamInfo &hints, CDVDCodecOptions &/*options*/) {
  if (hints.software)
    return false;

  if (hints.height < 720) {
    // This is SD video. Software decoder can handle it and it is much more reliable.
    return false;
  }

  Dispose();

  if (!OpenDevices()) {
    CLog::Log(LOGERROR, "%s::%s - Needed devices not found", CLASSNAME, __func__);
    return false;
  }

  m_framesToSkip = 0;
  m_missedFrames = 0;
  m_inputSequence = 0;
  m_sequence = -1;

  // Setup mfc output queue (OUTPUT - name of the queue where TO encoded frames are streamed, CAPTURE - name of the queue where FROM decoded frames are taken)
  if (!SetupOutputFormat(hints))
    return false;
  m_v4l2MFCOutputBuffers = V4l2::Buffers(MFC_OUTPUT_BUFFERS_CNT, m_decoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, false);
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
  if (!SetupCaptureBuffers(MFCCapturePlane1Size, MFCCapturePlane2Size))
    return false;
  
  m_hints = hints;

  CLog::Log(LOGNOTICE, "%s::%s - MFC Setup succesfull, start streaming", CLASSNAME, __func__);
  return true;
}

void CDVDVideoCodecExynos5::Dispose() {
  CLog::Log(LOGDEBUG, "%s::%s - Freeing memory allocated for buffers", CLASSNAME, __func__);
  m_v4l2MFCOutputBuffers.clear();
  m_v4l2MFCCaptureBuffers.clear();

  CLog::Log(LOGDEBUG, "%s::%s - Closing devices", CLASSNAME, __func__);
  if (m_decoderHandle >= 0) {
    if (m_v4l2MFCOutputBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Stream OFF", CLASSNAME, __func__);
    if (m_v4l2MFCCaptureBuffers.StreamOff())
      CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Stream OFF", CLASSNAME, __func__);
    close(m_decoderHandle);
  }

  m_iVideoWidth = 0;
  m_iVideoHeight = 0;
  m_iOutputWidth = 0;
  m_iOutputHeight = 0;
  m_decoderHandle = -1;
  m_dropPictures = false;

  memzero(m_v4l2OutputBuffer);
  memzero(m_videoBuffer);
}

void CDVDVideoCodecExynos5::PrepareOutputBuffer(int bufferIndex) {
  	m_videoBuffer.iFlags          = DVP_FLAG_ALLOCATED;

  	m_videoBuffer.color_range     = 0;
  	m_videoBuffer.color_matrix    = 4;
  	m_videoBuffer.format          = RENDER_FMT_YUV420P;
  	m_videoBuffer.iDisplayWidth   = m_iVideoWidth;
  	m_videoBuffer.iDisplayHeight  = m_iVideoHeight;
  	m_videoBuffer.iWidth          = m_iVideoWidth;
  	m_videoBuffer.iHeight         = m_iVideoHeight;
  	m_videoBuffer.iLineSize[0]    = m_iVideoWidth;
  	m_videoBuffer.iLineSize[1]    = m_iVideoWidth >> 1;
  	m_videoBuffer.iLineSize[2]    = m_iVideoWidth >> 1;

  	BYTE *s = (BYTE*)m_v4l2MFCCaptureBuffers[bufferIndex].cPlane[0];
  	BYTE *d = (BYTE*)m_v4l2OutputBuffer.cPlane[0];
  	// Copy Y plane
  	if (m_iOutputWidth == m_iVideoWidth) {
  	  fast_memcpy(d, s, m_iVideoWidth*m_iVideoHeight);
  	} else {
  	  for (int y = 0; y < m_iVideoHeight; y++) {
        fast_memcpy(d, s, m_iVideoWidth);
  		s += m_iOutputWidth;
  	    d += m_iVideoWidth;
  	  }
  	}
  	// Deinteleave NV12 UV plane to U and V planes of YUV420
  	Exynos::deinterleave_chroma_neon(m_v4l2OutputBuffer.cPlane[1], m_v4l2OutputBuffer.cPlane[2], m_iVideoWidth >> 1, m_v4l2MFCCaptureBuffers[bufferIndex].cPlane[1], m_iOutputWidth, m_iVideoHeight >> 1);

  	m_videoBuffer.data[0]         = (BYTE*)m_v4l2OutputBuffer.cPlane[0];
  	m_videoBuffer.data[1]         = (BYTE*)m_v4l2OutputBuffer.cPlane[1];
  	m_videoBuffer.data[2]         = (BYTE*)m_v4l2OutputBuffer.cPlane[2];
}

int CDVDVideoCodecExynos5::Decode(BYTE* pData, int iSize, double dts, double pts) {
  int ret = -1;

  if (m_framesToSkip) {
    if (m_framesToSkip == 1) {
      --m_framesToSkip;
    }
    return VC_ERROR;
  }

//std::cout << "DEC " << dts << ", " << pts << "\n";
//  unsigned int dtime = XbmcThreads::SystemClockMillis();

  if(pData) {
	// Find buffer ready to be filled
    size_t index;
    int ret = m_v4l2MFCOutputBuffers.FindFreeBuffer(index);
    if (ret == V4L2_ERROR) {
      return VC_ERROR;
    } else if (ret == V4L2_BUSY) { // buffer is still busy
      CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT All buffers are queued and busy, no space for new frame to decode. Very broken situation.", CLASSNAME, __func__);
      /* FIXME This should be handled as abnormal situation that should be addressed, otherwise decoding will stuck here forever */
      return VC_FLUSHED;
    }

    if (!SendBuffer(index, pData, iSize, pts)) {
        return VC_ERROR;
    }
    ++m_inputSequence;
  }

  // Dequeue decoded frame
  int index = 0;
  timeval ptsTime;
  uint32_t sequence;
  index = m_v4l2MFCCaptureBuffers.DequeueBuffer(ptsTime, sequence);

  if (index < 0) {
    if (errno == EAGAIN) // Buffer is still busy, queue more
      return VC_BUFFER;
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, ret, errno);
    return VC_ERROR;
  }

  // This is a kludge for broken interlaced video. If field order is not appropriate MFC would enter into broken state and would output garbage.
  // Skipping fields would not help. Full reinitialization is needed.
  m_missedFrames += sequence - m_sequence - 1;
  if (m_missedFrames > 5) {
      CLog::Log(LOGERROR, "%s::%s - MFC fails to decode interlaced video if fields are not in proper order. Reinitializing MFC.", CLASSNAME, __func__);

      Dispose();
      CDVDCodecOptions options;
      Open(m_hints, options);

      m_framesToSkip = m_inputSequence && 1 ? 0 : 1;
      CLog::Log(LOGERROR, "%s::%s - MFC continuing decoding", CLASSNAME, __func__);
      return VC_ERROR;
  }
  m_sequence = sequence;

  if (m_dropPictures) {
    m_videoBuffer.iFlags      |= DVP_FLAG_DROPPED;
    CLog::Log(LOGDEBUG, "%s::%s - Dropping frame with index %d", CLASSNAME, __func__, index);
  } else {
    PrepareOutputBuffer(index);
  }

  // Pop pts/dts only when picture is finally ready to be showed up or skipped
  m_videoBuffer.pts = (ptsTime.tv_sec + double(ptsTime.tv_usec)/1000); 
  m_videoBuffer.dts = m_videoBuffer.pts;
    
  // Queue dequeued from FIMC OUPUT frame back to MFC CAPTURE
  if (&m_v4l2MFCCaptureBuffers[index] && !m_v4l2MFCCaptureBuffers[index].bQueue) {
    if (!m_v4l2MFCCaptureBuffers.QueueBuffer(index)) {
      CLog::Log(LOGERROR, "%s::%s - queue output buffer\n", CLASSNAME, __func__);
      m_videoBuffer.iFlags      |= DVP_FLAG_DROPPED;
      m_videoBuffer.iFlags      &= DVP_FLAG_ALLOCATED;
      return VC_ERROR;
    }
  }

//  msg("Decode time: %d", XbmcThreads::SystemClockMillis() - dtime);
  
  return VC_PICTURE; // Picture is finally ready to be processed further
}
