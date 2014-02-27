#include "DVDVideoCodecExynos.h"

#include "DVDCodecs/DVDCodecs.h"

#include <utils/log.h>
#include <utils/fastmemcpy.h>

#include <dirent.h>
#include <sys/ioctl.h>
#include <fstream>
#include <cmath>

namespace Exynos {

namespace {
const char* CLASSNAME = "CDVDVideoCodecExynos";
} // namsepace

bool isStreamingDevice(int fd) {
  struct v4l2_capability cap = {};
  if (!ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    return ((cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
      ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) && (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE))) &&
      (cap.capabilities & V4L2_CAP_STREAMING));
  } else {
    return false;
  }
}

int OpenDevice(std::function<bool(const std::string&)> driverSelector, std::function<bool(int)> checker) {
  DIR *dir;
  struct dirent *ent;

  if ((dir = opendir("/sys/class/video4linux/")) == NULL)
    return -1;

  while ((ent = readdir (dir)) != NULL) {
    std::string deviceName(ent->d_name);
    if (deviceName.compare(0, 5, "video") == 0) {
      std::fstream file("/sys/class/video4linux/" + deviceName + "/name", std::ios_base::in);
      if (!file.is_open())
          continue;
      std::string currentDriverName;
      file >> currentDriverName;
      file.close();

      if (driverSelector(currentDriverName)) {
        int fd = open(("/dev/" + deviceName).c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd <= 0)
          continue;

        if (checker(fd)) {
          CLog::Log(LOGDEBUG, "%s::%s - Found %s %s", CLASSNAME, __func__, currentDriverName.c_str(), deviceName.c_str());
          closedir (dir);
          return fd;
        }
        close(fd);
      }
    }
  }
  closedir (dir);
  return -1;
}

CDVDVideoCodecExynos::CDVDVideoCodecExynos()
    : m_decoderHandle(-1)
    , m_bVideoConvert(false)
    , m_videoBuffer()
{}

bool CDVDVideoCodecExynos::Open(CDVDStreamInfo &hints, CDVDCodecOptions &options) {
  m_bVideoConvert = false;
  m_converter = CBitstreamConverter();
  m_videoBuffer = DVDVideoPicture();
}

bool CDVDVideoCodecExynos::SetupOutputFormat(CDVDStreamInfo &hints) {
  // Set format in which encoded frames are streamed TO mfc
  struct v4l2_format fmt = {};
  switch(hints.codec)
  {
/*
    case CODEC_TYPE_VC1_RCV:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_VC1_ANNEX_L;
*/
    case CODEC_ID_VC1:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_VC1_ANNEX_G;
      m_name = "mfc-vc1";
      break;
    case CODEC_ID_MPEG1VIDEO:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG1;
      m_name = "mfc-mpeg1";
      break;
    case CODEC_ID_MPEG2VIDEO:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG2;
      m_name = "mfc-mpeg2";
      break;
    case CODEC_ID_MPEG4:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG4;
      m_name = "mfc-mpeg4";
      break;
    case CODEC_ID_H263:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H263;
      m_name = "mfc-h263";
      break;
    case CODEC_ID_H264:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
      m_name = "mfc-h264";
      break;
    default:
      return false;
      break;
  }
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  fmt.fmt.pix_mp.plane_fmt[0].sizeimage = STREAM_BUFFER_SIZE;
  fmt.fmt.pix_mp.num_planes = V4L2_NUM_MAX_PLANES;

  if (ioctl(m_decoderHandle, VIDIOC_S_FMT, &fmt)) {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT S_FMT failed", CLASSNAME, __func__);
    return false;
  } else {
    return true;
  }
}

bool CDVDVideoCodecExynos::SendHeader(CDVDStreamInfo &hints) {
  m_bVideoConvert = m_converter.Open(hints.codec, (uint8_t *)hints.extradata, hints.extrasize, true);

  unsigned int extraSize;
  uint8_t *extraData;
  if (m_bVideoConvert) {
    extraSize = m_converter.GetExtraSize();
    extraData = m_converter.GetExtraData();
  } else {
    extraSize = hints.extrasize;
    extraData = (uint8_t *)hints.extradata;
  }

  fast_memcpy((uint8_t *)m_v4l2MFCOutputBuffers[0].cPlane[0], extraData, extraSize);
  m_v4l2MFCOutputBuffers[0].iBytesUsed[0] = extraSize;

  // Queue header to mfc output queue
  if (!m_v4l2MFCOutputBuffers.QueueBuffer(0, {})) {
    CLog::Log(LOGERROR, "%s::%s - MFC Error queuing header", CLASSNAME, __func__);
    return false;
  }

  // STREAMON on mfc OUTPUT
  if (!m_v4l2MFCOutputBuffers.StreamOn()) {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Failed to Stream ON", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Stream ON", CLASSNAME, __func__);

  m_hints = hints;
  return true;
}

bool CDVDVideoCodecExynos::SendBuffer(int bufferIndex, uint8_t *demuxer_content, int demuxer_bytes, double pts) {
  if(m_bVideoConvert) {
    m_converter.Convert(demuxer_content, demuxer_bytes);
    demuxer_bytes = m_converter.GetConvertSize();
    demuxer_content = m_converter.GetConvertBuffer();
  }

  if(demuxer_bytes >= m_v4l2MFCOutputBuffers[bufferIndex].iSize[0]) {
    CLog::Log(LOGERROR, "%s::%s - Packet to big for streambuffer. Requested: %d Available: %d", CLASSNAME, __func__, demuxer_bytes, m_v4l2MFCOutputBuffers[bufferIndex].iSize[0]);
    return false;
  }

  fast_memcpy((uint8_t *)m_v4l2MFCOutputBuffers[bufferIndex].cPlane[0], demuxer_content, demuxer_bytes);
  m_v4l2MFCOutputBuffers[bufferIndex].iBytesUsed[0] = demuxer_bytes;

  double fractpart, intpart;
  fractpart = modf(pts / 1000000.0, &intpart);
  if (!m_v4l2MFCOutputBuffers.QueueBuffer(bufferIndex, {long(intpart), long(fractpart * 1000000)})) {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, bufferIndex, errno);
    return false;
  }
    
  return true;
}

bool CDVDVideoCodecExynos::SetupCaptureBuffers(int MFCCapturePlane1Size, int MFCCapturePlane2Size) {
  // Get mfc needed number of buffers
  struct v4l2_control ctrl = {};
  ctrl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
  if (ioctl(m_decoderHandle, VIDIOC_G_CTRL, &ctrl)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to get the number of buffers required", CLASSNAME, __func__);
    return false;
  }
  int captureBuffersCount = ctrl.value + MFC_CAPTURE_EXTRA_BUFFER_CNT;

  // Allocate, Memory Map and queue mfc capture buffers
  m_v4l2MFCCaptureBuffers = V4l2::Buffers(captureBuffersCount, m_decoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, true);
  if (!m_v4l2MFCCaptureBuffers) {
    return false;
  }

  // FIXME looks like this initialization is not needed because V4l2::Buffers already sets these fields
  for (int n = 0; n < m_v4l2MFCCaptureBuffers.size(); n++) {
    m_v4l2MFCCaptureBuffers[n].iBytesUsed[0] = MFCCapturePlane1Size;
    m_v4l2MFCCaptureBuffers[n].iBytesUsed[1] = MFCCapturePlane2Size;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Succesfully allocated, mmapped and queued buffers", CLASSNAME, __func__);

  // STREAMON on mfc CAPTURE
  if (!m_v4l2MFCCaptureBuffers.StreamOn()) {
	CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to Stream ON", CLASSNAME, __func__);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Stream ON", CLASSNAME, __func__);

  return true;
}

void CDVDVideoCodecExynos::Reset() {
  CLog::Log(LOGNOTICE, "%s::%s - Resetting codec.", CLASSNAME, __func__);

  Dispose();
  CDVDCodecOptions options;
  if (!Open(m_hints, options)) {
    CLog::Log(LOGERROR, "%s::%s - Could not reset codec", CLASSNAME, __func__);
    Dispose();
  }
}

bool CDVDVideoCodecExynos::GetPicture(DVDVideoPicture* pDvdVideoPicture) {
  *pDvdVideoPicture = m_videoBuffer;
  return true;
}

bool CDVDVideoCodecExynos::ClearPicture(DVDVideoPicture* pDvdVideoPicture)
{
  return CDVDVideoCodec::ClearPicture(pDvdVideoPicture);
}

void CDVDVideoCodecExynos::SetDropState(bool bDrop) {
}

const char* CDVDVideoCodecExynos::GetName() {
  // m_name is never changed after open
  return m_name.c_str();
}

} // namespace Exynos
