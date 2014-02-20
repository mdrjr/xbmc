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
#include "LinuxV4l2.h"

#include "xbmc/utils/log.h"

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <linux/media.h>
#include <cstddef>

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CLinuxV4l2"

namespace {

int RequestBuffer(int device, enum v4l2_buf_type type, enum v4l2_memory memory, int numBuffers)
{
  struct v4l2_requestbuffers reqbuf;
  int ret = 0;

  if(device < 0)
    return V4L2_ERROR;

  memset(&reqbuf, 0, sizeof(struct v4l2_requestbuffers));

  reqbuf.type     = type;
  reqbuf.memory   = memory;
  reqbuf.count    = numBuffers;

  ret = ioctl(device, VIDIOC_REQBUFS, &reqbuf);
  if (ret)
  {
    CLog::Log(LOGERROR, "%s::%s - Request buffers", CLASSNAME, __func__);
    return V4L2_ERROR;
  }

  return reqbuf.count;
}

bool MmapBuffers(int device, int count, V4L2Buffer *v4l2Buffers, enum v4l2_buf_type type, enum v4l2_memory memory, bool queue)
{
  struct v4l2_buffer buf;
  struct v4l2_plane planes[V4L2_NUM_MAX_PLANES];
  int ret;
  int i, j;

  if(device < 0 || !v4l2Buffers || count == 0)
    return false;

  for(i = 0; i < count; i++)
  {
    memset(&buf, 0, sizeof(struct v4l2_buffer));
    memset(&planes, 0, sizeof(struct v4l2_plane) * V4L2_NUM_MAX_PLANES);
    buf.type      = type;
    buf.memory    = memory;
    buf.index     = i;
    buf.m.planes  = planes;
    buf.length    = V4L2_NUM_MAX_PLANES;

    ret = ioctl(device, VIDIOC_QUERYBUF, &buf);
    if (ret)
    {
      CLog::Log(LOGERROR, "%s::%s - Query buffer", CLASSNAME, __func__);
      return false;
    }

    V4L2Buffer *buffer = &v4l2Buffers[i];

    buffer->iNumPlanes = 0;
    for (j = 0; j < V4L2_NUM_MAX_PLANES; j++) 
    {
      buffer->iSize[j]       = buf.m.planes[j].length;
      buffer->iBytesUsed[j]  = buf.m.planes[j].bytesused;
      if(buffer->iSize[j])
      {
        buffer->cPlane[j] = mmap(NULL, buf.m.planes[j].length, PROT_READ | PROT_WRITE,
                       MAP_SHARED, device, buf.m.planes[j].m.mem_offset);
        if(buffer->cPlane[j] == MAP_FAILED)
        {
          CLog::Log(LOGERROR, "%s::%s - Mmapping buffer", CLASSNAME, __func__);
          return false;
        }
        memset(buffer->cPlane[j], 0, buf.m.planes[j].length);
        buffer->iNumPlanes++;
      }
    }
    buffer->iIndex = i;

    if(queue)
    {
      ret = ioctl(device, VIDIOC_QBUF, &buf);
      if (ret)
      {
        CLog::Log(LOGERROR, "%s::%s - Queue buffer", CLASSNAME, __func__);
        return false;
      }
      buffer->bQueue = true;
    }
  }

  return true;
}

} // namespace

namespace V4l2 {

int PollInput(int device, int timeout)
{
  int ret = 0;
  struct pollfd p;
  p.fd = device;
  p.events = POLLIN | POLLERR;

  ret = poll(&p, 1, timeout);
  if (ret < 0)
  {
    CLog::Log(LOGERROR, "%s::%s - Polling input", CLASSNAME, __func__);
    return V4L2_ERROR;
  }
  else if (ret == 0)
  {
    return V4L2_BUSY;
  }

  return V4L2_READY;
}

int PollOutput(int device, int timeout)
{
  int ret = 0;
  struct pollfd p;
  p.fd = device;
  p.events = POLLOUT | POLLERR;

  ret = poll(&p, 1, timeout);
  if (ret < 0)
  {
    CLog::Log(LOGERROR, "%s::%s - Polling output", CLASSNAME, __func__);
    return V4L2_ERROR;
  }
  else if (ret == 0)
  {
    return V4L2_BUSY;
  }

  return V4L2_READY;
}

int SetControllValue(int device, int id, int value)
{
  struct v4l2_control control;
  int ret;

  control.id    = id;
  control.value = value;

  ret = ioctl(device, VIDIOC_S_CTRL, &control);

  if(ret < 0) 
  {
    CLog::Log(LOGERROR, "%s::%s - Set controll if %d value %d\n", CLASSNAME, __func__, id, value);
    return V4L2_ERROR;
  }

  return V4L2_OK;
}


Buffers::Buffers(size_t size, int device, enum v4l2_buf_type type, bool queue) 
  : buffers_(&ownedBuffers_)
  , device_(device)
  , type_(type)
  , memory_(V4L2_MEMORY_MMAP)
{
  // Request capture buffers
  size = RequestBuffer(device_, type_, memory_, size);
  if (size == V4L2_ERROR) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE REQBUFS failed", CLASSNAME, __func__);
    return;
  }

  ownedBuffers_.resize(size, V4L2Buffer());

  if(!MmapBuffers(device_, ownedBuffers_.size(), &ownedBuffers_[0], type_, memory_, queue)) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE Cannot mmap memory for buffers", CLASSNAME, __func__);
    clear();
  }
}

Buffers::Buffers(int device, enum v4l2_buf_type type, Buffers& buffers)
  : buffers_(buffers.buffers_)
  , device_(device)
  , type_(type)
  , memory_(V4L2_MEMORY_USERPTR)
{
  // Request capture buffers
  if (RequestBuffer(device_, type_, memory_, buffers.size()) == V4L2_ERROR) {
    CLog::Log(LOGERROR, "%s::%s - MFC CAPTURE REQBUFS failed", CLASSNAME, __func__);
    return;
  }
}

Buffers::Buffers(Buffers&& buffers)
    : ownedBuffers_(std::move(buffers.ownedBuffers_))
    , buffers_(buffers.buffers_ == &buffers.ownedBuffers_ ? &ownedBuffers_ : buffers.buffers_)
    , device_(buffers.device_)
    , type_(buffers.type_)
    , memory_(buffers.memory_)
{}

Buffers& Buffers::operator=(Buffers&& buffers) {
    ownedBuffers_ = std::move(buffers.ownedBuffers_);
    buffers_ = buffers.buffers_ == &buffers.ownedBuffers_ ? &ownedBuffers_ : buffers.buffers_;
    device_ = buffers.device_;
    type_ = buffers.type_;
    memory_ = buffers.memory_;
}

Buffers::~Buffers() {
  clear();
}

bool Buffers::QueueBuffer(size_t index, const timeval& pts) {
  if (index >= size()) {
    return false;
  }
  auto& buffer = (*buffers_)[index];

  struct v4l2_plane vplanes[V4L2_NUM_MAX_PLANES] = {};
  for (int planeIndex = 0; planeIndex < buffer.iNumPlanes; planeIndex++) 
  {
    vplanes[planeIndex].m.userptr   = (unsigned long)buffer.cPlane[planeIndex];
    vplanes[planeIndex].length      = buffer.iSize[planeIndex];
    vplanes[planeIndex].bytesused   = buffer.iBytesUsed[planeIndex];
  }

  struct v4l2_buffer vbuf = {};
  vbuf.type     = type_;
  vbuf.memory   = memory_;
  vbuf.index    = index;
  vbuf.m.planes = vplanes;
  vbuf.length   = buffer.iNumPlanes;
  vbuf.timestamp= pts;

  if (ioctl(device_, VIDIOC_QBUF, &vbuf))
  {
    CLog::Log(LOGERROR, "%s::%s - Queue buffer", CLASSNAME, __func__);
    return false;
  }
  buffer.bQueue = true;

  return true;
}

int Buffers::DequeueBuffer(uint32_t& sequence, timeval& time) {
  struct v4l2_plane  vplanes[V4L2_NUM_MAX_PLANES] = {};

  struct v4l2_buffer vbuf = {};
  vbuf.type     = type_;
  vbuf.memory   = memory_;
  vbuf.m.planes = vplanes;
  vbuf.length   = V4L2_NUM_MAX_PLANES;

  if (ioctl(device_, VIDIOC_DQBUF, &vbuf)) {
    if (errno != EAGAIN)
      CLog::Log(LOGERROR, "%s::%s - Dequeue buffer", CLASSNAME, __func__);
    return V4L2_ERROR;
  }

  (*buffers_)[vbuf.index].bQueue = false;
  time = vbuf.timestamp;
  sequence = vbuf.sequence;

  return vbuf.index;
}

int Buffers::FindFreeBuffer(size_t& index) {
  for (index = 0; index < buffers_->size(); ++index) {
    if (!(*buffers_)[index].bQueue) {
      return V4L2_OK;
    }
  }

  int ret = PollOutput(device_, 1000); // POLLIN - Capture, POLLOUT - Output
  if (ret == V4L2_READY) {
    timeval time;
    uint32_t sequence;
    index = DequeueBuffer(sequence, time);
    return V4L2_OK;
  } else if (ret != V4L2_BUSY) {
    CLog::Log(LOGERROR, "%s::%s - MFC OUTPUT\e[0m PollOutput error %d, errno %d", CLASSNAME, __func__, ret, errno);
    return V4L2_ERROR;
  }

  return ret;
}

bool Buffers::StreamOn() {
  enum v4l2_buf_type setType = type_;
  return !ioctl(device_, VIDIOC_STREAMON, &setType);
}

bool Buffers::StreamOff() {
  enum v4l2_buf_type setType = type_;
  return !ioctl(device_, VIDIOC_STREAMOFF, &setType);
}

void Buffers::clear() {
  for(auto& buffer : ownedBuffers_)
  {
    for (size_t i = 0; i < buffer.iNumPlanes; i++)
    {
      if(buffer.cPlane[i] && buffer.cPlane[i] != MAP_FAILED)
      {
        munmap(buffer.cPlane[i], buffer.iSize[i]);
        CLog::Log(LOGDEBUG, "%s::%s - unmap convert buffer", CLASSNAME, __func__);
      }
    }
  }
  ownedBuffers_.clear();
}

} // namespace V4l2
