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

#include <linux/videodev2.h>
#include <vector>

#define V4L2_ERROR -1
#define V4L2_BUSY  1
#define V4L2_READY 2
#define V4L2_OK    3

extern "C" {

#define V4L2_NUM_MAX_PLANES 3

typedef struct V4L2Buffer
{
  int   iSize[V4L2_NUM_MAX_PLANES];
  int   iOffset[V4L2_NUM_MAX_PLANES];
  int   iBytesUsed[V4L2_NUM_MAX_PLANES];
  void  *cPlane[V4L2_NUM_MAX_PLANES];
  int   iNumPlanes;
  int   iIndex;
  bool  bQueue;
} V4L2Buffer;

}

class CLinuxV4l2
{
public:
// TODO this block is not needed outside
  static int RequestBuffer(int device, enum v4l2_buf_type type, enum v4l2_memory memory, int numBuffers);
  static bool StreamOn(int device, enum v4l2_buf_type type, int onoff);
  static bool MmapBuffers(int device, int count, V4L2Buffer *v4l2Buffers, enum v4l2_buf_type type, enum v4l2_memory memory, bool queue = true);
  static V4L2Buffer *FreeBuffers(int count, V4L2Buffer *v4l2Buffers);
  static int DequeueBuffer(int device, enum v4l2_buf_type type, enum v4l2_memory memory, int planes);
  static int QueueBuffer(int device, enum v4l2_buf_type type, enum v4l2_memory memory, 
      int planes, int index, V4L2Buffer *buffer);

// TODO move to namespace
  static int PollInput(int device, int timeout);
  static int PollOutput(int device, int timeout);
  static int SetControllValue(int device, int id, int value);
};

namespace V4l2 {

/// Convinient class for managing V4L2Buffer buffer array
class Buffers {
public:
  // Creates empty buffers
  Buffers() {}

  Buffers(size_t size, int device, enum v4l2_buf_type type, enum v4l2_memory memory, bool queue = true);

  Buffers(Buffers&&) = default;
  Buffers& operator=(Buffers&&) = default;

  Buffers(const Buffers&) = delete;
  Buffers& operator=(const Buffers&) = delete;

  ~Buffers();

  void clear();

  bool QueueBuffer(size_t index, const timeval& pts = {});
  // Returns dequeued buffer index or V4L2_ERROR in case of error
  int DequeueBuffer(timeval& time, uint32_t& sequence);
  // Searches for unused buffers or dequeues processed ones
  // Returns V4L2_ERROR, V4L2_BUSY or V4L2_OK
  int FindFreeBuffer(size_t& index);

  bool StreamOn();
  bool StreamOff();

  // Returns true if buffers are initialized
  explicit operator bool() {return !buffers_.empty();}
  size_t size() const {return buffers_.size();}

  V4L2Buffer& operator[](size_t index) {return buffers_[index];}
  const V4L2Buffer& operator[](size_t index) const {return buffers_[index];}

private:
  std::vector<V4L2Buffer> buffers_;
  int device_;
  enum v4l2_buf_type type_;
  enum v4l2_memory memory_;
};
} // namespace V4l2
