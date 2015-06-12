#include "system.h"

#ifndef THIS_IS_NOT_XBMC
  #if (defined HAVE_CONFIG_H) && (!defined WIN32)
    #include "config.h"
  #endif
  #include "DVDDemuxers/DVDDemux.h"
  #include "DVDStreamInfo.h"
  #include "DVDClock.h"
  #include "guilib/GraphicContext.h"
  #include "DVDCodecs/DVDCodecs.h"
  #include "DVDCodecs/DVDCodecUtils.h"
  #include "settings/Settings.h"
  #include "settings/DisplaySettings.h"
  #include "settings/AdvancedSettings.h"
  #include "utils/fastmemcpy.h"
  #include "utils/log.h"
#endif

#include "DVDVideoCodecMFC.h"

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <dirent.h>

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CDVDVideoCodecMFC"

CDVDVideoCodecMFC::CDVDVideoCodecMFC() : CDVDVideoCodec() {

  m_iDecoderHandle = NULL;
  m_iConverterHandle = NULL;
  m_MFCOutput = NULL;
  m_MFCCapture = NULL;
  m_FIMCOutput = NULL;
  m_FIMCCapture = NULL;

  m_Buffer = NULL;
  m_BufferNowOnScreen = NULL;

  memzero(m_videoBuffer);

}

CDVDVideoCodecMFC::~CDVDVideoCodecMFC() {

  Dispose();

}

bool CDVDVideoCodecMFC::OpenDevices() {
  DIR *dir;

  if ((dir = opendir ("/sys/class/video4linux/")) != NULL) {
    struct dirent *ent;
    while ((ent = readdir (dir)) != NULL) {
      if (strncmp(ent->d_name, "video", 5) == 0) {
        char *p;
        char name[64];
        char devname[64];
        char sysname[64];
        char drivername[32];
        char target[1024];
        int ret;

        snprintf(sysname, 64, "/sys/class/video4linux/%s", ent->d_name);
        snprintf(name, 64, "/sys/class/video4linux/%s/name", ent->d_name);

        FILE* fp = fopen(name, "r");
        if (fgets(drivername, 32, fp) != NULL) {
          p = strchr(drivername, '\n');
          if (p != NULL)
            *p = '\0';
        } else {
          fclose(fp);
          continue;
        }
        fclose(fp);

        ret = readlink(sysname, target, sizeof(target));
        if (ret < 0)
          continue;
        target[ret] = '\0';
        p = strrchr(target, '/');
        if (p == NULL)
          continue;

        sprintf(devname, "/dev/%s", ++p);

        if (!m_iDecoderHandle && strstr(drivername, "mfc") != NULL && strstr(drivername, "dec") != NULL) {
          int fd = open(devname, O_RDWR | O_NONBLOCK, 0);
          if (fd > -1) {
            struct v4l2_capability cap;
            memzero(cap);
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
              if (cap.capabilities & V4L2_CAP_STREAMING &&
                (cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
                (cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE)))) {
                m_iDecoderHandle = new V4l2Device;
                m_iDecoderHandle->device = fd;
                strcpy(m_iDecoderHandle->name, drivername);
                CLog::Log(LOGDEBUG, "%s::%s - MFC Found %s %s", CLASSNAME, __func__, drivername, devname);
                struct v4l2_format fmt;
                memzero(fmt);
                fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
                if (ioctl(fd, VIDIOC_TRY_FMT, &fmt) == 0) {
                  CLog::Log(LOGDEBUG, "%s::%s - Direct decoding to untiled picture on device %s is supported, no conversion needed", CLASSNAME, __func__, m_iDecoderHandle->name);
                  delete m_iConverterHandle;
                  m_iConverterHandle = NULL;
                  return true;
                }
              }
          }
          if (!m_iDecoderHandle)
            close(fd);
        }
        if (!m_iConverterHandle && strstr(drivername, "fimc") != NULL && strstr(drivername, "m2m") != NULL) {
          int fd = open(devname, O_RDWR | O_NONBLOCK, 0);
          if (fd > -1) {
            struct v4l2_capability cap;
            memzero(cap);
            if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
              if (cap.capabilities & V4L2_CAP_STREAMING &&
                (cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
                (cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE)))) {
                m_iConverterHandle = new V4l2Device;
                m_iConverterHandle->device = fd;
                strcpy(m_iConverterHandle->name, drivername);
                CLog::Log(LOGDEBUG, "%s::%s - FIMC Found %s %s", CLASSNAME, __func__, drivername, devname);
              }
          }
          if (!m_iConverterHandle)
            close(fd);
        }
        if (m_iDecoderHandle && m_iConverterHandle) {
          closedir (dir);
          return true;
        }
      }
    }
    closedir (dir);
  }

  return false;

}

void CDVDVideoCodecMFC::Dispose() {

  CLog::Log(LOGDEBUG, "%s::%s - Starting cleanup", CLASSNAME, __func__);

  delete m_BufferNowOnScreen;
  delete m_Buffer;

  m_Buffer = NULL;
  m_BufferNowOnScreen = NULL;

  delete m_FIMCCapture;
  delete m_FIMCOutput;
  delete m_MFCCapture;
  delete m_MFCOutput;

  m_MFCOutput = NULL;
  m_MFCCapture = NULL;
  m_FIMCOutput = NULL;
  m_FIMCCapture = NULL;

  if (m_iConverterHandle) {
    close(m_iConverterHandle->device);
    delete m_iConverterHandle;
    m_iConverterHandle = NULL;
  }

  if (m_iDecoderHandle) {
    close(m_iDecoderHandle->device);
    delete m_iDecoderHandle;
    m_iDecoderHandle = NULL;
  }

}

bool CDVDVideoCodecMFC::Open(CDVDStreamInfo &hints, CDVDCodecOptions &options) {
  struct v4l2_format fmt;
  struct v4l2_crop crop;
  struct V4l2SinkBuffer sinkBuffer;
  V4l2Device *finalSink = NULL;
  int finalFormat = -1;
  int resultVideoWidth;
  int resultVideoHeight;
  int resultLineSize;
  unsigned int extraSize = 0;
  uint8_t *extraData = NULL;

  m_hints = hints;
  if (m_hints.software)
    return false;

  Dispose();

  m_Buffer = new V4l2SinkBuffer();
  m_BufferNowOnScreen = new V4l2SinkBuffer();
  m_BufferNowOnScreen->iIndex = -1;
  m_bVideoConvert = false;
  m_bDropPictures = false;
  memzero(m_videoBuffer);

  if (!OpenDevices()) {
    CLog::Log(LOGERROR, "%s::%s - No Exynos MFC Decoder/Converter found", CLASSNAME, __func__);
    return false;
  }

  m_bVideoConvert = m_converter.Open(m_hints.codec, (uint8_t *)m_hints.extradata, m_hints.extrasize, true);

  if(m_bVideoConvert) {
    if(m_converter.GetExtraData() != NULL && m_converter.GetExtraSize() > 0) {
      extraSize = m_converter.GetExtraSize();
      extraData = m_converter.GetExtraData();
    }
  } else {
    if(m_hints.extrasize > 0 && m_hints.extradata != NULL) {
      extraSize = m_hints.extrasize;
      extraData = (uint8_t*)m_hints.extradata;
    }
  }

  // Test what formats we can get finally
  // If converter is present, it is our final sink
  (m_iConverterHandle) ? finalSink = m_iConverterHandle : finalSink = m_iDecoderHandle;
  // Test NV12 2 Planes Y/CbCr
  memzero(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
  if (ioctl(finalSink->device, VIDIOC_TRY_FMT, &fmt) == 0)
    finalFormat = V4L2_PIX_FMT_NV12M;
  memzero(fmt);
  // Test YUV420 3 Planes Y/Cb/Cr
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
  if (ioctl(finalSink->device, VIDIOC_TRY_FMT, &fmt) == 0)
    finalFormat = V4L2_PIX_FMT_YUV420M;

  // No suitable output formats available
  if (finalFormat < 0) {
    CLog::Log(LOGERROR, "%s::%s - No suitable format on %s to convert to found", CLASSNAME, __func__, finalSink->name);
    return false;
  }

  // Create MFC Output sink (the one where encoded frames are feed)
  m_MFCOutput = new CLinuxV4l2Sink(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
  memzero(fmt);
  switch(m_hints.codec)
  {
    case AV_CODEC_ID_VC1:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_VC1_ANNEX_G;
      m_name = "mfc-vc1";
      break;
    case AV_CODEC_ID_MPEG1VIDEO:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG1;
      m_name = "mfc-mpeg1";
      break;
    case AV_CODEC_ID_MPEG2VIDEO:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG2;
      m_name = "mfc-mpeg2";
      break;
    case AV_CODEC_ID_MPEG4:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MPEG4;
      m_name = "mfc-mpeg4";
      break;
    case AV_CODEC_ID_H263:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H263;
      m_name = "mfc-h263";
      break;
    case AV_CODEC_ID_H264:
      fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
      m_name = "mfc-h264";
      break;
    default:
      return false;
      break;
  }
  fmt.fmt.pix_mp.plane_fmt[0].sizeimage = BUFFER_SIZE;
  // Set encoded format
  if (!m_MFCOutput->SetFormat(&fmt))
    return false;
  // Init with number of input buffers predefined
  if (!m_MFCOutput->Init(INPUT_BUFFERS))
    return false;

  // Get empty buffer to fill
  if (!m_MFCOutput->GetBuffer(&sinkBuffer))
    return false;
  // Fill it with the header
  sinkBuffer.iBytesUsed[0] = extraSize;
  memcpy(sinkBuffer.cPlane[0], extraData, extraSize);
  // Enqueue buffer
  if (!m_MFCOutput->PushBuffer(&sinkBuffer))
    return false;

  // Create MFC Capture sink (the one from which decoded frames are read)
  m_MFCCapture = new CLinuxV4l2Sink(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
  memzero(fmt);
  // If there is no converter set output format on the MFC Capture sink
  if (!m_iConverterHandle) {
    fmt.fmt.pix_mp.pixelformat = finalFormat;
    if (!m_MFCCapture->SetFormat(&fmt))
        return false;
  }

  // Turn on MFC Output with header in it to initialize MFC with all we just setup
  m_MFCOutput->StreamOn(VIDIOC_STREAMON);

  // Initialize MFC Capture
  if (!m_MFCCapture->Init(0))
    return false;
  // Queue all buffers (empty) to MFC Capture
  m_MFCCapture->QueueAll();

  // Read the format of MFC Capture
  if (!m_MFCCapture->GetFormat(&fmt))
    return false;
  // Size of resulting picture coming out of MFC
  // It will be aligned by 16 since the picture is tiled
  // We need this to know where to split buffer line by line
  resultLineSize = fmt.fmt.pix_mp.width;
  // Get MFC capture crop settings
  if (!m_MFCCapture->GetCrop(&crop))
    return false;
  // This is the picture boundaries we are interested in, everything outside is alignement because of tiled MFC output
  resultVideoWidth = crop.c.width;
  resultVideoHeight = crop.c.height;

  // Turn on MFC Capture
  m_MFCCapture->StreamOn(VIDIOC_STREAMON);

  // If converter is needed (we need to untile the picture from format MFC produces it)
  if (m_iConverterHandle) {
    // Create FIMC Output sink
    m_FIMCOutput = new CLinuxV4l2Sink(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    // Set the FIMC Output format to the one read from MFC
    if (!m_FIMCOutput->SetFormat(&fmt))
      return false;
    // Set the FIMC Output crop to the one read from MFC
    if (!m_FIMCOutput->SetCrop(&crop))
      return false;
    // Init FIMC Output and link it to buffers of MFC Capture
    if (!m_FIMCOutput->Init(m_MFCCapture))
      return false;
    // Get FIMC Output crop settings
    if (!m_FIMCOutput->GetCrop(&crop))
      return false;

    // Create FIMC Capture sink
    m_FIMCCapture = new CLinuxV4l2Sink(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
    // Set the final picture format and the same picture dimension settings to FIMC Capture
    // as picture crop coming from MFC (original picture dimensions)
    memzero(fmt);
    fmt.fmt.pix_mp.pixelformat = finalFormat;
    fmt.fmt.pix_mp.width = crop.c.width;
    fmt.fmt.pix_mp.height = crop.c.height;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    if (!m_FIMCCapture->SetFormat(&fmt))
      return false;
    // Init FIMC capture with number of buffers predefined
    if (!m_FIMCCapture->Init(OUTPUT_BUFFERS))
      return false;

    // Queue all buffers (empty) to FIMC Capture
    m_FIMCCapture->QueueAll();

    // Read FIMC capture format settings
    if (!m_FIMCCapture->GetFormat(&fmt))
      return false;
    resultLineSize = fmt.fmt.pix_mp.width;
    // Read FIMC capture crop settings
    if (!m_FIMCCapture->GetCrop(&crop))
      return false;
    resultVideoWidth = crop.c.width;
    resultVideoHeight = crop.c.height;

    // Turn on FIMC Output and Capture enabling the converter
    m_FIMCOutput->StreamOn(VIDIOC_STREAMON);
    m_FIMCCapture->StreamOn(VIDIOC_STREAMON);
  }

  m_videoBuffer.iFlags          = DVP_FLAG_ALLOCATED;

  m_videoBuffer.color_range     = 0;
  m_videoBuffer.color_matrix    = 4;

  m_videoBuffer.iDisplayWidth   = resultVideoWidth;
  m_videoBuffer.iDisplayHeight  = resultVideoHeight;
  m_videoBuffer.iWidth          = resultVideoWidth;
  m_videoBuffer.iHeight         = resultVideoHeight;

  if (m_hints.aspect > 0.0 && !m_hints.forced_aspect)
  {
    m_videoBuffer.iDisplayWidth  = ((int)lrint(m_videoBuffer.iHeight * m_hints.aspect)) & -3;
    if (m_videoBuffer.iDisplayWidth > m_videoBuffer.iWidth)
    {
      m_videoBuffer.iDisplayWidth  = m_videoBuffer.iWidth;
      m_videoBuffer.iDisplayHeight = ((int)lrint(m_videoBuffer.iWidth / m_hints.aspect)) & -3;
    }
  }

  m_videoBuffer.data[0]         = NULL;
  m_videoBuffer.data[1]         = NULL;
  m_videoBuffer.data[2]         = NULL;
  m_videoBuffer.data[3]         = NULL;

  m_videoBuffer.pts             = DVD_NOPTS_VALUE;
  m_videoBuffer.dts             = DVD_NOPTS_VALUE;

  m_videoBuffer.iLineSize[0]    = resultLineSize;
  m_videoBuffer.iLineSize[3]    = 0;

  if (finalFormat == V4L2_PIX_FMT_NV12M) {
    m_videoBuffer.format          = RENDER_FMT_NV12;
    m_videoBuffer.iLineSize[1]    = resultLineSize;
    m_videoBuffer.iLineSize[2]    = 0;
  } else if (finalFormat == V4L2_PIX_FMT_YUV420M) {
    /*
    Due to BUG in MFC v8 (-XU3) firmware the Y plane of the picture has the right line size,
     but the U and V planes line sizes are actually halves of Y plane line size padded to 32
     This is pure workaround for -XU3 MFCv8 firmware "MFC v8.0, F/W: 14yy, 01mm, 13dd (D):
    */
    resultLineSize = (resultLineSize + 31) &~31;
    m_videoBuffer.format          = RENDER_FMT_YUV420P;
    m_videoBuffer.iLineSize[1]    = resultLineSize >> 1;
    m_videoBuffer.iLineSize[2]    = resultLineSize >> 1;
  }

  m_BufferNowOnScreen->iIndex = -1;
  m_bCodecHealthy = true;

  CLog::Log(LOGNOTICE, "%s::%s - MFC Setup succesfull (%dx%d, linesize %d, format 0x%x), start streaming", CLASSNAME, __func__, resultVideoWidth, resultVideoHeight, resultLineSize, finalFormat);

  return true;

}

void CDVDVideoCodecMFC::SetDropState(bool bDrop) {

  m_bDropPictures = bDrop;
  if (m_bDropPictures)
    m_videoBuffer.iFlags |=  DVP_FLAG_DROPPED;
  else
    m_videoBuffer.iFlags &= ~DVP_FLAG_DROPPED;

}

int CDVDVideoCodecMFC::Decode(BYTE* pData, int iSize, double dts, double pts) {

  if (m_hints.ptsinvalid)
    pts = DVD_NOPTS_VALUE;

  //unsigned int dtime = XbmcThreads::SystemClockMillis();
  debug_log(LOGDEBUG, "%s::%s - input frame iSize %d, pts %lf, dts %lf", CLASSNAME, __func__, iSize, pts, dts);

  if(pData) {
    int demuxer_bytes = iSize;
    uint8_t *demuxer_content = pData;

    if(m_bVideoConvert) {
      m_converter.Convert(demuxer_content, demuxer_bytes);
      demuxer_bytes = m_converter.GetConvertSize();
      demuxer_content = m_converter.GetConvertBuffer();
    }

    m_MFCOutput->Poll(1000/3); // Wait up to 0.3 of a second for buffer availability
    if (m_MFCOutput->GetBuffer(m_Buffer)) {
      debug_log(LOGDEBUG, "%s::%s - Got empty buffer %d from MFC Output, filling", CLASSNAME, __func__, m_Buffer->iIndex);
      m_Buffer->iBytesUsed[0] = demuxer_bytes;
      fast_memcpy((uint8_t *)m_Buffer->cPlane[0], demuxer_content, m_Buffer->iBytesUsed[0]);
      long* longPts = (long*)&pts;
      m_Buffer->timeStamp.tv_sec = longPts[0];
      m_Buffer->timeStamp.tv_usec = longPts[1];

      if (!m_MFCOutput->PushBuffer(m_Buffer)) {
        m_bCodecHealthy = false;
        return VC_FLUSHED; // MFC unrecoverable error, reset needed
      }
    } else {
      if (errno == EAGAIN)
        CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT All buffers are queued and busy, no space for new frame to decode. Very broken situation. Current encoded frame will be lost", CLASSNAME, __func__);
      else {
        m_bCodecHealthy = false;
        return VC_FLUSHED; // MFC unrecoverable error, reset needed
      }
    }
  }

  // Get a buffer from MFC Capture
  if (!m_MFCCapture->DequeueBuffer(m_Buffer))
    if (errno == EAGAIN)
      return VC_BUFFER;
    else
      return VC_ERROR;

  if (m_bDropPictures) {

    CLog::Log(LOGWARNING, "%s::%s - Dropping frame with index %d", CLASSNAME, __func__, m_Buffer->iIndex);
    // Queue it back to MFC CAPTURE since the picture is dropped anyway
    m_MFCCapture->PushBuffer(m_Buffer);
    return VC_DROPPED | VC_BUFFER;

  }

  if (m_iConverterHandle) {
    // Push the buffer got from MFC Capture to FIMC Output (decoded from decoder to converter)
    if (!m_FIMCOutput->PushBuffer(m_Buffer)) {
      m_bCodecHealthy = false;
      return VC_FLUSHED; // FIMC unrecoverable error, reset needed
    }
    // Get a buffer from FIMC Capture
    if (!m_FIMCCapture->DequeueBuffer(m_Buffer))
      if (errno == EAGAIN)
        return VC_BUFFER;
      else
        return VC_ERROR;
  }

  // We got a new buffer to show, so we can enqeue back the buffer wich was on screen
  if (m_BufferNowOnScreen->iIndex > -1) {
    if (m_iConverterHandle)
      m_FIMCCapture->PushBuffer(m_BufferNowOnScreen);
    else
      m_MFCCapture->PushBuffer(m_BufferNowOnScreen);
    m_BufferNowOnScreen->iIndex = -1;
  }

  long longPts[2] = { m_Buffer->timeStamp.tv_sec, m_Buffer->timeStamp.tv_usec };
  m_videoBuffer.data[0]         = (BYTE*)m_Buffer->cPlane[0];
  m_videoBuffer.data[1]         = (BYTE*)m_Buffer->cPlane[1];
  m_videoBuffer.data[2]         = (BYTE*)m_Buffer->cPlane[2];
  m_videoBuffer.pts             = *((double*)&longPts[0]);

  std::swap(m_Buffer, m_BufferNowOnScreen);

  if (m_iConverterHandle && m_FIMCOutput->DequeueBuffer(m_Buffer))
    m_MFCCapture->PushBuffer(m_Buffer);

  //debug_log("Decode time: %d", XbmcThreads::SystemClockMillis() - dtime);
  // Picture is finally ready to be processed further and more info can be enqueued
  return VC_PICTURE | VC_BUFFER;

}

void CDVDVideoCodecMFC::Reset() {

  if (m_bCodecHealthy) // We need real reset only when codec went unhealthy
    return;
  CLog::Log(LOGERROR, "%s::%s - Codec Reset. Reinitializing", CLASSNAME, __func__);
  CDVDCodecOptions options;
  // We need full MFC/FIMC reset with device reopening.
  // I wasn't able to reinitialize both IP's without fully closing and reopening them.
  // There are always some clips that cause MFC or FIMC go into state which cannot be reset without close/open
  Open(m_hints, options);
}

bool CDVDVideoCodecMFC::GetPicture(DVDVideoPicture* pDvdVideoPicture) {

  *pDvdVideoPicture = m_videoBuffer;
  debug_log(LOGDEBUG, "%s::%s - output frame pts %lf", CLASSNAME, __func__, m_videoBuffer.pts);
  return true;

}

bool CDVDVideoCodecMFC::ClearPicture(DVDVideoPicture* pDvdVideoPicture) {

  return CDVDVideoCodec::ClearPicture(pDvdVideoPicture);

}
