#pragma once

#include "DVDVideoCodec.h"
#include "DVDStreamInfo.h"
#include "utils/BitstreamConverter.h"

#include <linux/LinuxV4l2.h>

#include <functional>
#include <string>

namespace Exynos {

bool isStreamingDevice(int fd);

/// Searches for video devices with name accepted by driverSelector. Found ones are checked with checker.
/// Returns file descriptor or -1 if nothing found
int OpenDevice(std::function<bool(const std::string&)> driverSelector, std::function<bool(int)> checker = isStreamingDevice);

void deinterleave_chroma_neon(void *u_out, void *v_out, int width_out, void *uv_in, int width_in, int height_in) asm("deinterleave_chroma_neon");

/// Base class for exynos decoders
class CDVDVideoCodecExynos : public CDVDVideoCodec
{
public:
  CDVDVideoCodecExynos();

  virtual bool Open(CDVDStreamInfo &hints, CDVDCodecOptions &options);
  virtual void Reset();
  virtual void SetDropState(bool bDrop);

  virtual bool GetPicture(DVDVideoPicture* pDvdVideoPicture);
  virtual bool ClearPicture(DVDVideoPicture* pDvdVideoPicture);
  virtual const char* GetName();

protected:
  bool SendHeader(CDVDStreamInfo &hints);
  bool SetupOutputFormat(CDVDStreamInfo &hints);
  bool SendBuffer(int bufferIndex, uint8_t *demuxer_content, int demuxer_bytes, double pts);
  bool SetupCaptureBuffers(int MFCCapturePlane1Size, int MFCCapturePlane2Size);

  V4l2::Buffers m_v4l2MFCOutputBuffers;
  V4l2::Buffers m_v4l2MFCCaptureBuffers;

  // compressed frame size. 1080p mpeg4 10Mb/s can be un to 786k in size, so this is to make sure frame fits into buffer
  static const size_t STREAM_BUFFER_SIZE = 786432;
  // 1 doesn't work at all
  static const size_t MFC_OUTPUT_BUFFERS_CNT = 2;
  // these are extra buffers, better keep their count as big as going to be simultaneous dequeued buffers number
  static const size_t MFC_CAPTURE_EXTRA_BUFFER_CNT = 3;

  int m_decoderHandle;

  DVDVideoPicture m_videoBuffer;

  CDVDStreamInfo m_hints;

private:
  std::string m_name;

  bool m_bVideoConvert;
  CBitstreamConverter m_converter;
};

} // namespace Exynos
