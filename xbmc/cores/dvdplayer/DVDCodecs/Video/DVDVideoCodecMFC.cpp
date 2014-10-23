/*
 *      Copyright (C) 2005-2014 Team XBMC
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

#include "DVDCodecs/DVDCodecs.h"
#include "DVDVideoCodecMFC.h"
#include "DVDClock.h"

#include "utils/fastmemcpy.h"
#include <sys/ioctl.h>
#include <dirent.h>

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CDVDVideoCodecMFC"

CDVDVideoCodecMFC::CDVDVideoCodecMFC() : CDVDVideoCodec()
{
  m_iDecoderHandle = -1;
  m_iConverterHandle = -1;

  memzero(m_videoBuffer);

  m_bVideoConvert = false;
  m_bDropPictures = false;
  m_iDequeuedToPresentBufferNumber = -1;

  m_MFCOutputBuffersCount = 0;
  m_MFCCaptureBuffersCount = 0;
  m_FIMCCaptureBuffersCount = 0;

  m_v4l2MFCOutputBuffers = NULL;
  m_v4l2MFCCaptureBuffers = NULL;
  m_v4l2FIMCCaptureBuffers = NULL;
}

CDVDVideoCodecMFC::~CDVDVideoCodecMFC()
{
  Dispose();
}

void CDVDVideoCodecMFC::Dispose()
{

  if (m_iConverterHandle >= 0)
  {
    m_v4l2FIMCCaptureBuffers = CLinuxV4L2::FreeBuffers(m_FIMCCaptureBuffersCount, m_v4l2FIMCCaptureBuffers);
    if (CLinuxV4L2::StreamOn(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, VIDIOC_STREAMOFF))
      CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT Stream OFF", CLASSNAME, __func__);
    if (CLinuxV4L2::StreamOn(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, VIDIOC_STREAMOFF))
      CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Stream OFF", CLASSNAME, __func__);

    m_FIMCCaptureBuffersCount = 0;
    CLog::Log(LOGDEBUG, "%s::%s - FIMC Closing", CLASSNAME, __func__);
    close(m_iConverterHandle);
    m_iConverterHandle = -1;
  }

  if (m_iDecoderHandle >= 0)
  {
    m_v4l2MFCOutputBuffers = CLinuxV4L2::FreeBuffers(m_MFCOutputBuffersCount, m_v4l2MFCOutputBuffers);
    m_v4l2MFCCaptureBuffers = CLinuxV4L2::FreeBuffers(m_MFCCaptureBuffersCount, m_v4l2MFCCaptureBuffers);
    if (CLinuxV4L2::StreamOn(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, VIDIOC_STREAMOFF))
      CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Stream OFF", CLASSNAME, __func__);
    if (CLinuxV4L2::StreamOn(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, VIDIOC_STREAMOFF))
      CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Stream OFF", CLASSNAME, __func__);

    m_MFCOutputBuffersCount = 0;
    m_MFCCaptureBuffersCount = 0;
    CLog::Log(LOGDEBUG, "%s::%s - MFC Closing", CLASSNAME, __func__);
    close(m_iDecoderHandle);
    m_iDecoderHandle = -1;
  }

  memzero(m_videoBuffer);

  m_iDequeuedToPresentBufferNumber = -1;
}

bool CDVDVideoCodecMFC::OpenDecoder()
{
  DIR *dir;
  struct dirent *ent;

  if ((dir = opendir ("/sys/class/video4linux/")) != NULL)
  {
    while ((ent = readdir (dir)) != NULL)
    {
      if (strncmp(ent->d_name, "video", 5) == 0)
      {
        char *p;
        char name[64];
        char devname[64];
        char sysname[64];
        char drivername[32];
        char target[1024];
        int ret;

        snprintf(sysname, sizeof(sysname), "/sys/class/video4linux/%s", ent->d_name);
        snprintf(name, sizeof(name), "/sys/class/video4linux/%s/name", ent->d_name);

        FILE* fp = fopen(name, "r");
        if (fp == NULL)
          continue;

        if (fgets(drivername, 32, fp) != NULL)
        {
          p = strchr(drivername, '\n');
          if (p != NULL)
            *p = '\0';
        }
        else
        {
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

        snprintf(devname, sizeof(devname), "/dev/%s", ++p);

        if (m_iDecoderHandle < 0 && strncmp(drivername, "s5p-mfc-dec", 11) == 0)
        {
          struct v4l2_capability cap;
          int fd = open(devname, O_RDWR | O_NONBLOCK, 0);
          if (fd > 0)
          {
            memset(&(cap), 0, sizeof (cap));
            ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
            if (ret == 0)
              if ((cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
                ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) && (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE))) &&
                (cap.capabilities & V4L2_CAP_STREAMING))
              {
                m_iDecoderHandle = fd;
                CLog::Log(LOGDEBUG, "%s::%s - Found %s %s", CLASSNAME, __func__, drivername, devname);
              }
          }
          if (m_iDecoderHandle < 0)
            close(fd);
        }
        if (m_iDecoderHandle >= 0)
        {
          // MFC should at least support NV12MT format
          return true;
        }
      }
    }
    closedir (dir);
  }
  return false;
}

bool CDVDVideoCodecMFC::CheckDecoderFormats()
{
  // we enumerate all the supported formats looking for NV12MT and NV12
  int index = 0;
  int ret   = -1;
  bool hasNV12MTSupport = false;
  m_hasNV12Support      = false;
  while (true)
  {
    struct v4l2_fmtdesc vid_fmtdesc = {};
    vid_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vid_fmtdesc.index = index++;

    ret = ioctl(m_iDecoderHandle, VIDIOC_ENUM_FMT, &vid_fmtdesc);
    if (ret != 0)
      break;
    CLog::Log(LOGDEBUG, "%s::%s - Decoder format %d: %c%c%c%c (%s)", CLASSNAME, __func__, vid_fmtdesc.index,
        vid_fmtdesc.pixelformat & 0xFF, (vid_fmtdesc.pixelformat >> 8) & 0xFF,
        (vid_fmtdesc.pixelformat >> 16) & 0xFF, (vid_fmtdesc.pixelformat >> 24) & 0xFF,
        vid_fmtdesc.description);
    if (vid_fmtdesc.pixelformat == V4L2_PIX_FMT_NV12MT)
      hasNV12MTSupport = true;
    if (vid_fmtdesc.pixelformat == V4L2_PIX_FMT_NV12)
      m_hasNV12Support = true;
  }
  return true;
}

bool CDVDVideoCodecMFC::OpenConverter()
{
  DIR *dir;
  struct dirent *ent;

  if ((dir = opendir ("/sys/class/video4linux/")) != NULL)
  {
    while ((ent = readdir (dir)) != NULL)
    {
      if (strncmp(ent->d_name, "video", 5) == 0)
      {
        char *p;
        char name[64];
        char devname[64];
        char sysname[64];
        char drivername[32];
        char target[1024];
        int ret;

        snprintf(sysname, sizeof(sysname), "/sys/class/video4linux/%s", ent->d_name);
        snprintf(name, sizeof(name), "/sys/class/video4linux/%s/name", ent->d_name);

        FILE* fp = fopen(name, "r");
        if (fp == NULL)
          continue;

        if (fgets(drivername, 32, fp) != NULL)
        {
          p = strchr(drivername, '\n');
          if (p != NULL)
            *p = '\0';
        }
        else
        {
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

        snprintf(devname, sizeof(devname), "/dev/%s", ++p);

        if (m_iConverterHandle < 0 && strstr(drivername, "fimc") != NULL && strstr(drivername, "m2m") != NULL)
        {
          struct v4l2_capability cap;
          int fd = open(devname, O_RDWR, 0);
          if (fd > 0) {
            memset(&(cap), 0, sizeof (cap));
            ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
            if (ret == 0)
              if ((cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE ||
                ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) && (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE))) &&
                (cap.capabilities & V4L2_CAP_STREAMING))
              {
                m_iConverterHandle = fd;
                CLog::Log(LOGDEBUG, "%s::%s - Found %s %s", CLASSNAME, __func__, drivername, devname);
              }
          }
          if (m_iConverterHandle < 0)
            close(fd);
        }
        if (m_iConverterHandle >= 0)
          return true;
      }
    }
    closedir (dir);
  }
  return false;
}

bool CDVDVideoCodecMFC::Open(CDVDStreamInfo &hints, CDVDCodecOptions &options)
{
  struct v4l2_format fmt;
  struct v4l2_control ctrl;
  struct v4l2_crop crop;
  int iResultVideoWidth;
  int iResultLineSize;
  int iResultVideoHeight;
  int ret = 0;
  unsigned int extraSize = 0;
  uint8_t *extraData = NULL;

  Dispose();
  m_hints = hints;

  if (!OpenDecoder())
  {
    CLog::Log(LOGERROR, "%s::%s - MFC device not found", CLASSNAME, __func__);
    return false;
  }

  if (!m_hasNV12Support) //MFC5
  {
    // FIMC color convertor required
    if (!OpenConverter())
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC device not found", CLASSNAME, __func__);
      return false;
    }
  }

  m_bVideoConvert = m_converter.Open(m_hints.codec, (uint8_t *)m_hints.extradata, m_hints.extrasize, true);

  if (m_bVideoConvert)
  {
    if (m_converter.GetExtraData() != NULL && m_converter.GetExtraSize() > 0)
    {
      extraSize = m_converter.GetExtraSize();
      extraData = m_converter.GetExtraData();
    }
  }
  else
  {
    if (m_hints.extrasize > 0 && m_hints.extradata != NULL)
    {
      extraSize = m_hints.extrasize;
      extraData = (uint8_t*)m_hints.extradata;
    }
  }

  // Setup MFC OUTPUT queue (OUTPUT - name of the queue where TO encoded frames are streamed, CAPTURE - name of the queue where FROM decoded frames are taken)
  // Set MFC OUTPUT format
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
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  fmt.fmt.pix_mp.plane_fmt[0].sizeimage = STREAM_BUFFER_SIZE;
  ret = ioctl(m_iDecoderHandle, VIDIOC_S_FMT, &fmt);
  if (ret != 0)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT S_FMT Failed, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  // Request MFC OUTPUT buffers
  m_MFCOutputBuffersCount = CLinuxV4L2::RequestBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, MFC_OUTPUT_BUFFERS_CNT);
  if (m_MFCOutputBuffersCount == V4L2_ERROR)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT REQBUFS Failed, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT REQBUFS Number of MFC buffers is %d (requested %d)", CLASSNAME, __func__, m_MFCOutputBuffersCount, MFC_OUTPUT_BUFFERS_CNT);

  // Memory Map MFC OUTPUT buffers
  m_v4l2MFCOutputBuffers = (V4L2Buffer *)calloc(m_MFCOutputBuffersCount, sizeof(V4L2Buffer));
  if (!m_v4l2MFCOutputBuffers)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Cannot allocate buffers in memory", CLASSNAME, __func__);
    return false;
  }
  if (!CLinuxV4L2::MmapBuffers(m_iDecoderHandle, m_MFCOutputBuffersCount, m_v4l2MFCOutputBuffers, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, false))
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Cannot mmap buffers, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Succesfully mmapped %d buffers", CLASSNAME, __func__, m_MFCOutputBuffersCount);

  // Prepare header
  m_v4l2MFCOutputBuffers[0].iBytesUsed[0] = extraSize;
  fast_memcpy((uint8_t *)m_v4l2MFCOutputBuffers[0].cPlane[0], extraData, extraSize);

  // Queue header to MFC OUTPUT queue
  ret = CLinuxV4L2::QueueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2MFCOutputBuffers[0]);
  if (ret == V4L2_ERROR)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Error queuing header, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT <- %d header of size %d", CLASSNAME, __func__, ret, extraSize);

  // Setup MFC CAPTURE format
  if (m_hasNV12Support) //MFC>5
  {
    memzero(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
    ret = ioctl(m_iDecoderHandle, VIDIOC_S_FMT, &fmt);
    if (ret != 0)
    {
      CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE S_FMT Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE S_FMT: fmt 0x%x", CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat);
  }

  // STREAMON on MFC OUTPUT
  if (!CLinuxV4L2::StreamOn(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, VIDIOC_STREAMON))
  {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Failed to Stream ON, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT Stream ON", CLASSNAME, __func__);


  // Get MFC CAPTURE picture format to check, and to setup FIMC converter if needed
  memzero(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  ret = ioctl(m_iDecoderHandle, VIDIOC_G_FMT, &fmt);
  if (ret)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_FMT Failed, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE G_FMT: fmt 0x%x, (%dx%d), plane[0]=%d plane[1]=%d", CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.plane_fmt[0].sizeimage, fmt.fmt.pix_mp.plane_fmt[1].sizeimage);
  // Size of resulting picture coming out of MFC
  // It will be aligned by 16 since the picture is tiled
  // We need this to know where to split buffer line by line
  iResultLineSize = fmt.fmt.pix_mp.width;

  // Get MFC CAPTURE crop to check and setup Line Size as well as FIMC converter if needed
  memzero(crop);
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  ret = ioctl(m_iDecoderHandle, VIDIOC_G_CROP, &crop);
  if (ret)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE G_CROP Failed to get crop information, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE G_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);
  // This is the picture boundaries we are interested in, everything outside is alignement because of tiled MFC output
  iResultVideoWidth = crop.c.width;
  iResultVideoHeight = crop.c.height;

  // Get MFC needed number of buffers on CAPTURE
  memzero(ctrl);
  ctrl.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
  ret = ioctl(m_iDecoderHandle, VIDIOC_G_CTRL, &ctrl);
  if (ret)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to get the number of buffers required, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE want %d buffers", CLASSNAME, __func__, ctrl.value);
  m_MFCCaptureBuffersCount = (ctrl.value + MFC_CAPTURE_EXTRA_BUFFER_CNT) % 32; // We need more extra capture buffers for cozy decoding
                                                                               // We use the defined extra buffers, but we cap to 32 buffers (max MFC supports)
  // Request MFC CAPTURE buffers
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Going to ask for %d buffers", CLASSNAME, __func__, m_MFCCaptureBuffersCount);
  m_MFCCaptureBuffersCount = CLinuxV4L2::RequestBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, m_MFCCaptureBuffersCount);
  if (m_MFCCaptureBuffersCount == V4L2_ERROR)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE REQBUFS Failed, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE REQBUFS Number of buffers allowed by MFC %d", CLASSNAME, __func__, m_MFCCaptureBuffersCount);

  // Allocate, Memory Map and queue MFC CAPTURE buffers
  m_v4l2MFCCaptureBuffers = (V4L2Buffer *)calloc(m_MFCCaptureBuffersCount, sizeof(V4L2Buffer));
  if (!m_v4l2MFCCaptureBuffers)
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Cannot allocate memory for buffers", CLASSNAME, __func__);
    return false;
  }
  if (!CLinuxV4L2::MmapBuffers(m_iDecoderHandle, m_MFCCaptureBuffersCount, m_v4l2MFCCaptureBuffers, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, true))
  {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Cannot mmap memory for buffers, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Succesfully allocated, mmapped and queued %d buffers", CLASSNAME, __func__, m_MFCCaptureBuffersCount);

  // STREAMON on mfc CAPTURE
  if (!CLinuxV4L2::StreamOn(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, VIDIOC_STREAMON))
  {
	CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to Stream ON, errno %d", CLASSNAME, __func__, errno);
    return false;
  }
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE Stream ON", CLASSNAME, __func__);

  if (!m_hasNV12Support) //MFC5
  {
    // Setup FIMC OUTPUT fmt with data from MFC CAPTURE
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ret = ioctl(m_iConverterHandle, VIDIOC_S_FMT, &fmt);
    if (ret != 0)
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT S_FMT Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT S_FMT: fmt 0x%x, (%dx%d), plane[0]=%d plane[1]=%d", CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.plane_fmt[0].sizeimage, fmt.fmt.pix_mp.plane_fmt[1].sizeimage);

    // Setup FIMC OUTPUT crop with data from MFC CAPTURE
    crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (ioctl(m_iConverterHandle, VIDIOC_S_CROP, &crop))
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT S_CROP Failed to set crop information, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT S_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);

    // Request FIMC OUTPUT buffers
    ret = CLinuxV4L2::RequestBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_USERPTR, m_MFCCaptureBuffersCount);
    if (ret == V4L2_ERROR)
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT REQBUFS Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT REQBUFS Number of buffers is %d", CLASSNAME, __func__, ret);

    // Setup FIMC CAPTURE
    memzero(fmt);
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = iResultVideoWidth;
    fmt.fmt.pix_mp.height = iResultVideoHeight;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    ret = ioctl(m_iConverterHandle, VIDIOC_S_FMT, &fmt);
    if (ret != 0)
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE S_FMT Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE S_FMT: fmt 0x%x, (%dx%d)", CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);

    // Get FIMC produced picture details to adjust output buffer parameters with these values
    memzero(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    ret = ioctl(m_iConverterHandle, VIDIOC_G_FMT, &fmt);
    if (ret)
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE G_FMT Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE G_FMT: fmt 0x%x, (%dx%d), plane[0]=%d plane[1]=%d", CLASSNAME, __func__, fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.plane_fmt[0].sizeimage, fmt.fmt.pix_mp.plane_fmt[1].sizeimage);
    // The length of the line on the result buffer
    iResultLineSize = fmt.fmt.pix_mp.width;

    memzero(crop);
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(m_iConverterHandle, VIDIOC_G_CROP, &crop))
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE G_CROP Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE G_CROP (%dx%d)", CLASSNAME, __func__, crop.c.width, crop.c.height);
    // Width and Height returned after this call is the real resulting picture size produced by FIMC
    iResultVideoWidth = crop.c.width;
    iResultVideoHeight = crop.c.height;

    // Request FIMC CAPTURE buffers
    m_FIMCCaptureBuffersCount = CLinuxV4L2::RequestBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, FIMC_CAPTURE_BUFFERS_CNT);
    if (m_FIMCCaptureBuffersCount == V4L2_ERROR)
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE REQBUFS Failed, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE REQBUFS Number of buffers is %d", CLASSNAME, __func__, m_FIMCCaptureBuffersCount);

    // Allocate, Memory Map and queue FIMC CAPTURE buffers
    m_v4l2FIMCCaptureBuffers = (V4L2Buffer *)calloc(m_FIMCCaptureBuffersCount, sizeof(V4L2Buffer));
    if (!m_v4l2FIMCCaptureBuffers)
    {
     CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Cannot allocate memory for buffers", CLASSNAME, __func__);
     return false;
    }
    if (!CLinuxV4L2::MmapBuffers(m_iConverterHandle, m_FIMCCaptureBuffersCount, m_v4l2FIMCCaptureBuffers, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, true))
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Cannot mmap for capture buffers, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Succesfully allocated, mmapped and queued %d buffers", CLASSNAME, __func__, m_FIMCCaptureBuffersCount);

    if (!CLinuxV4L2::StreamOn(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, VIDIOC_STREAMON))
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT Failed to Stream ON, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT Stream ON", CLASSNAME, __func__);
    if (!CLinuxV4L2::StreamOn(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, VIDIOC_STREAMON))
    {
      CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Failed to Stream ON, errno %d", CLASSNAME, __func__, errno);
      return false;
    }
    CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE Stream ON", CLASSNAME, __func__);
  }

  m_videoBuffer.iFlags          = DVP_FLAG_ALLOCATED;

  m_videoBuffer.color_range     = 0;
  m_videoBuffer.color_matrix    = 4;

  m_videoBuffer.iDisplayWidth   = iResultVideoWidth;
  m_videoBuffer.iDisplayHeight  = iResultVideoHeight;
  m_videoBuffer.iWidth          = iResultVideoWidth;
  m_videoBuffer.iHeight         = iResultVideoHeight;

  m_videoBuffer.data[0]         = NULL;
  m_videoBuffer.data[1]         = NULL;
  m_videoBuffer.data[2]         = NULL;
  m_videoBuffer.data[3]         = NULL;

  m_videoBuffer.format          = RENDER_FMT_NV12;
  m_videoBuffer.iLineSize[0]    = iResultLineSize;
  m_videoBuffer.iLineSize[1]    = iResultLineSize;
  m_videoBuffer.iLineSize[2]    = 0;
  m_videoBuffer.iLineSize[3]    = 0;
  m_videoBuffer.pts             = DVD_NOPTS_VALUE;
  m_videoBuffer.dts             = DVD_NOPTS_VALUE;

  CLog::Log(LOGNOTICE, "%s::%s - MFC Setup succesfull, start streaming", CLASSNAME, __func__);
  return true;
}

void CDVDVideoCodecMFC::SetDropState(bool bDrop)
{

  m_bDropPictures = bDrop;
  if (m_bDropPictures)
    m_videoBuffer.iFlags |=  DVP_FLAG_DROPPED;
  else
    m_videoBuffer.iFlags &= ~DVP_FLAG_DROPPED;
}

int CDVDVideoCodecMFC::Decode(BYTE* pData, int iSize, double dts, double pts)
{
  int ret = -1;
  int index = 0;
  double dequeuedTimestamp;

  if (m_hints.ptsinvalid)
    pts = DVD_NOPTS_VALUE;

  CLog::Log(LOGDEBUG, "%s::%s - input frame iSize %d, pts %lf, dts %lf", CLASSNAME, __func__, iSize, pts, dts);

  if (pData)
  {
    int demuxer_bytes = iSize;
    uint8_t *demuxer_content = pData;

    // Find buffer ready to be filled
    while (index < m_MFCOutputBuffersCount && m_v4l2MFCOutputBuffers[index].bQueue)
      index++;

    if (index >= m_MFCOutputBuffersCount)
    {
      // All input buffers are busy, dequeue needed
      ret = CLinuxV4L2::PollOutput(m_iDecoderHandle, 1000/3); // Wait up to 1/3 (3 fps) sec for buffer to become available to recieve new encoded frame.
                                                              // POLLIN - Capture, POLLOUT - Output
      if (ret == V4L2_ERROR)
      {
        CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT PollOutput Error", CLASSNAME, __func__);
        return VC_ERROR;
      }
      else if (ret == V4L2_READY)
      {
        index = CLinuxV4L2::DequeueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, &dequeuedTimestamp);
        if (index < 0)
        {
          CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, index, errno);
          return VC_FLUSHED;
        }
        CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT -> %d", CLASSNAME, __func__, index);
        m_v4l2MFCOutputBuffers[index].bQueue = false;
      }
      else if (ret == V4L2_BUSY)
      {
        // Buffer is still busy
        CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT All buffers are queued and busy, no space for new frame to decode. Very broken situation.", CLASSNAME, __func__);
        return VC_PICTURE; // MFC is so busy it cannot accept more input frames, call ::Decode with pData = NULL to request a picture dequeue
                           // FIXME
                           // This will actually cause the current encoded frame to be lost in void, so this has to be fully reworked to queues storing all frames coming in
                           // In current realization the picture will distort in this case scenarios
      }
      else
      {
        CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT PollOutput error %d, errno %d", CLASSNAME, __func__, ret, errno);
        return VC_ERROR;
      }
    }

    if (m_bVideoConvert)
    {
      m_converter.Convert(demuxer_content, demuxer_bytes);
      demuxer_bytes = m_converter.GetConvertSize();
      demuxer_content = m_converter.GetConvertBuffer();
    }

    m_v4l2MFCOutputBuffers[index].iBytesUsed[0] = (demuxer_bytes < m_v4l2MFCOutputBuffers[index].iSize[0]) ? demuxer_bytes : m_v4l2MFCOutputBuffers[index].iSize[0];
    fast_memcpy((uint8_t *)m_v4l2MFCOutputBuffers[index].cPlane[0], demuxer_content, m_v4l2MFCOutputBuffers[index].iBytesUsed[0]);
    m_v4l2MFCOutputBuffers[index].timestamp = pts;
    ret = CLinuxV4L2::QueueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2MFCOutputBuffers[index]);
    if (ret == V4L2_ERROR)
    {
      CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, index, errno);
      return VC_FLUSHED;
    }
    CLog::Log(LOGDEBUG, "%s::%s - MFC OUTPUT <- %d", CLASSNAME, __func__, index);
  }

  if (m_iDequeuedToPresentBufferNumber >= 0)
  {
    if (!m_hasNV12Support) //MFC5
    {
     if (!m_v4l2FIMCCaptureBuffers[m_iDequeuedToPresentBufferNumber].bQueue)
     {
        ret = CLinuxV4L2::QueueBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2FIMCCaptureBuffers[m_iDequeuedToPresentBufferNumber]);
        if (ret == V4L2_ERROR)
        {
          CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, m_iDequeuedToPresentBufferNumber, errno);
          return VC_FLUSHED;
        }
        CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE <- %d", CLASSNAME, __func__, m_iDequeuedToPresentBufferNumber);
        m_iDequeuedToPresentBufferNumber = -1;
      }
    }
    else //MFC>5
    {
     if (!m_v4l2MFCCaptureBuffers[m_iDequeuedToPresentBufferNumber].bQueue)
     {
        ret = CLinuxV4L2::QueueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2MFCCaptureBuffers[m_iDequeuedToPresentBufferNumber]);
        if (ret == V4L2_ERROR)
        {
          CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, m_iDequeuedToPresentBufferNumber, errno);
          return VC_FLUSHED;
        }
        CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE <- %d", CLASSNAME, __func__, m_iDequeuedToPresentBufferNumber);
        m_iDequeuedToPresentBufferNumber = -1;
      }
    }
  }

  // Dequeue decoded frame
  index = CLinuxV4L2::DequeueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &dequeuedTimestamp);
  if (index < 0)
  {
    if (errno == EAGAIN) // Buffer is still busy, queue more
      return VC_BUFFER;
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, index, errno);
    return VC_FLUSHED;
  }
  m_v4l2MFCCaptureBuffers[index].bQueue = false;
  m_v4l2MFCCaptureBuffers[index].timestamp = dequeuedTimestamp;
  CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE -> %d", CLASSNAME, __func__, index);

  if (m_bDropPictures)
  {
    CLog::Log(LOGWARNING, "%s::%s - Dropping frame with index %d", CLASSNAME, __func__, index);
    // Queue it back to MFC CAPTURE since the picture is dropped anyway
    ret = CLinuxV4L2::QueueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2MFCCaptureBuffers[index]);
    if (ret < 0)
    {
      CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, index, errno);
      return VC_FLUSHED;
    }
    CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE <- %d", CLASSNAME, __func__, index);
    return VC_BUFFER; // Continue, we have no picture to show
  }
  else
  {
    if (!m_hasNV12Support) //MFC5
    {
      ret = CLinuxV4L2::QueueBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_USERPTR, &m_v4l2MFCCaptureBuffers[index]);
      if (ret == V4L2_ERROR)
      {
        CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, index, errno);
        return VC_FLUSHED;
      }
      CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT <- %d", CLASSNAME, __func__, index);

      index = CLinuxV4L2::DequeueBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &dequeuedTimestamp);
      if (index < 0)
      {
        if (errno == EAGAIN) // Dequeue buffer not ready, need more data on input. EAGAIN = 11
          return VC_BUFFER;
        CLog::Log(LOGERROR, "%s::%s - FIMC CAPTURE error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, index, errno);
        return VC_FLUSHED;
      }
      CLog::Log(LOGDEBUG, "%s::%s - FIMC CAPTURE -> %d", CLASSNAME, __func__, index);

      ret = CLinuxV4L2::DequeueBuffer(m_iConverterHandle, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_USERPTR, &dequeuedTimestamp);
      if (ret < 0)
      {
        if (errno == EAGAIN) // Dequeue buffer not ready, need more data on input. EAGAIN = 11
          return VC_BUFFER;
        CLog::Log(LOGERROR, "%s::%s - FIMC OUTPUT error dequeue output buffer, got number %d, errno %d", CLASSNAME, __func__, ret, errno);
        return VC_FLUSHED;
      }
      CLog::Log(LOGDEBUG, "%s::%s - FIMC OUTPUT -> %d", CLASSNAME, __func__, ret);
      // Queue it back to MFC CAPTURE
      if (CLinuxV4L2::QueueBuffer(m_iDecoderHandle, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_MMAP, &m_v4l2MFCCaptureBuffers[ret]) < 0)
      {
        CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Failed to queue buffer with index %d, errno %d", CLASSNAME, __func__, ret, errno);
        return VC_FLUSHED;
      }
      CLog::Log(LOGDEBUG, "%s::%s - MFC CAPTURE <- %d", CLASSNAME, __func__, ret);

      m_v4l2FIMCCaptureBuffers[index].bQueue = false;
      m_v4l2FIMCCaptureBuffers[index].timestamp = dequeuedTimestamp;
      m_videoBuffer.data[0]         = (BYTE*)m_v4l2FIMCCaptureBuffers[index].cPlane[0];
      m_videoBuffer.data[1]         = (BYTE*)m_v4l2FIMCCaptureBuffers[index].cPlane[1];
      m_videoBuffer.pts             = m_v4l2FIMCCaptureBuffers[index].timestamp;
    }
    else //MFC>5
    {
      m_videoBuffer.data[0]         = (BYTE*)m_v4l2MFCCaptureBuffers[index].cPlane[0];
      m_videoBuffer.data[1]         = (BYTE*)m_v4l2MFCCaptureBuffers[index].cPlane[1];
      m_videoBuffer.pts             = m_v4l2MFCCaptureBuffers[index].timestamp;
    }
    m_iDequeuedToPresentBufferNumber = index;
  }

  return VC_PICTURE | VC_BUFFER; // Picture is finally ready to be processed further and more info can be enqueued
}

void CDVDVideoCodecMFC::Reset()
{

  CLog::Log(LOGERROR, "%s::%s - Codec Reset. Reinitializing", CLASSNAME, __func__);
  CDVDCodecOptions options;
  // We need full MFC/FIMC reset with device reopening.
  // I wasn't able to reinitialize both IP's without fully closing and reopening them.
  // There are always some clips that cause MFC or FIMC go into state which cannot be reset without close/open
  Open(m_hints, options);
}

bool CDVDVideoCodecMFC::GetPicture(DVDVideoPicture* pDvdVideoPicture)
{

  CLog::Log(LOGDEBUG, "%s::%s - GetPicture", CLASSNAME, __func__);
  *pDvdVideoPicture = m_videoBuffer;
  CLog::Log(LOGDEBUG, "%s::%s - output frame pts %lf", CLASSNAME, __func__, m_videoBuffer.pts);
  return true;
}

bool CDVDVideoCodecMFC::ClearPicture(DVDVideoPicture* pDvdVideoPicture)
{
  return CDVDVideoCodec::ClearPicture(pDvdVideoPicture);
}
