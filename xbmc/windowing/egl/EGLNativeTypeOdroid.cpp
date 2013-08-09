/*
 *      Copyright (C) 2011-2012 Team XBMC
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
#include <EGL/egl.h>
#include "EGLNativeTypeOdroid.h"
#include "utils/log.h"
#include <stdlib.h>
#include "guilib/gui3d.h"
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <unistd.h>

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CEGLNativeTypeOdroid"

CEGLNativeTypeOdroid::CEGLNativeTypeOdroid()
{
}

CEGLNativeTypeOdroid::~CEGLNativeTypeOdroid()
{
}

bool CEGLNativeTypeOdroid::CheckCompatibility()
{
    FILE* m_fCPUInfo = fopen("/proc/cpuinfo", "r");
    if (m_fCPUInfo)
    {
      char buffer[512];

      while (fgets(buffer, sizeof(buffer), m_fCPUInfo))
      {
        if (strncmp(buffer, "Hardware", strlen("Hardware"))==0)
        {
          char *needle = strstr(buffer, ":");
          if (needle && strlen(needle)>3)
          {
            needle+=2;
            if (strncmp(needle, "ODROID", strlen("ODROID"))==0) {
                printf("%s::%s using odroid EGL\n", CLASSNAME, __func__);
                return true;
            }
          }
        }
      }
    }

    return false;
}

void CEGLNativeTypeOdroid::Initialize()
{
  return;
}

void CEGLNativeTypeOdroid::Destroy()
{
  return;
}

bool CEGLNativeTypeOdroid::CreateNativeDisplay()
{
  m_nativeDisplay = XOpenDisplay(NULL);
  if (m_nativeDisplay)
    return true;
  return false;
}

bool CEGLNativeTypeOdroid::CreateNativeWindow()
{
    if (!m_nativeDisplay) 
      return false;

    Display *xDisplay;
    Window nativeWindow;
    Pixmap bitmapNoData;
    XColor black;
    static char noData[] = { 0,0,0,0,0,0,0,0 };
    XWindowAttributes window_attr;
    XSetWindowAttributes attr;
    XSetWindowAttributes xattr;
    XWMHints hints;
    
    xDisplay = (Display*) m_nativeDisplay;
    Window rootWindow = DefaultRootWindow(xDisplay);
    XGetWindowAttributes(xDisplay, rootWindow, &window_attr);
    attr.background_pixel = 0;
    attr.border_pixel = 0;
    attr.event_mask = ExposureMask | PointerMotionMask | KeyPressMask;
    nativeWindow = XCreateWindow(xDisplay, rootWindow, 0, 0, window_attr.width, window_attr.height, 0, CopyFromParent, InputOutput, CopyFromParent, CWBackPixel | CWBorderPixel | CWEventMask, &attr);
    xattr.override_redirect = 0;
    XChangeWindowAttributes ( xDisplay, nativeWindow, CWOverrideRedirect, &xattr );
    hints.input = 1;
    hints.flags = InputHint;
    XSetWMHints(xDisplay, nativeWindow, &hints);

    black.red = black.green = black.blue = 0;
    bitmapNoData = XCreateBitmapFromData(xDisplay, nativeWindow, noData, 8, 8);
    Cursor invisibleCursor = XCreatePixmapCursor(xDisplay, bitmapNoData, bitmapNoData, &black, &black, 0, 0);
    XDefineCursor(xDisplay, nativeWindow, invisibleCursor);
    XFreeCursor(xDisplay, invisibleCursor);
/*
   {
      XEvent	x11_event;
      Atom	x11_state_atom;
      Atom	x11_fs_atom;

      x11_state_atom	= XInternAtom(xDisplay, "_NET_WM_STATE", True);
      x11_fs_atom		= XInternAtom(xDisplay, "_NET_WM_STATE_FULLSCREEN", True);

      x11_event.xclient.type			= ClientMessage;
      x11_event.xclient.serial		= 0;
      x11_event.xclient.send_event	= True;
      x11_event.xclient.window		= nativeWindow;
      x11_event.xclient.message_type	= x11_state_atom;
      x11_event.xclient.format		= 32;
      x11_event.xclient.data.l[0]	= 1;
      x11_event.xclient.data.l[1]	= x11_fs_atom;
      x11_event.xclient.data.l[2]	= 0;

      XSendEvent(xDisplay, rootWindow, False, SubstructureRedirectMask | SubstructureNotifyMask, &x11_event);
    }
*/

    Atom atom[2] = { XInternAtom(xDisplay, "_NET_WM_STATE_FULLSCREEN", False), None };
    XChangeProperty(xDisplay, nativeWindow, XInternAtom(xDisplay, "_NET_WM_STATE", False), XA_ATOM, 32, PropModeReplace, (const unsigned char *)atom,  1);

    XStoreName(xDisplay , nativeWindow,  "XBMC");

    XFlush(xDisplay);

    m_nativeWindow = (EGLNativeWindowType) nativeWindow;

    if (!m_nativeWindow)
      return false;

    return true;
}

bool CEGLNativeTypeOdroid::GetNativeDisplay(XBNativeDisplayType **nativeDisplay) const
{
  if (!nativeDisplay)
    return false;
  *nativeDisplay = (XBNativeDisplayType*) &m_nativeDisplay;
  return true;
}

bool CEGLNativeTypeOdroid::GetNativeWindow(XBNativeWindowType **nativeWindow) const
{
  if (!nativeWindow)
    return false;
  *nativeWindow = (XBNativeWindowType*) &m_nativeWindow;
  return true;
}

bool CEGLNativeTypeOdroid::DestroyNativeDisplay()
{
  XCloseDisplay((Display *) m_nativeDisplay);
  free(m_nativeDisplay);
  return true;
}

bool CEGLNativeTypeOdroid::DestroyNativeWindow()
{
  free(m_nativeWindow);
  return true;
}

bool CEGLNativeTypeOdroid::GetNativeResolution(RESOLUTION_INFO *res) const
{
    XWindowAttributes window_attr;
    XGetWindowAttributes((Display *) m_nativeDisplay, (Window) m_nativeWindow, &window_attr);
    res->iWidth         = window_attr.width;
    res->iHeight        = window_attr.height;
    res->iScreenWidth   = res->iWidth;
    res->iScreenHeight  = res->iHeight;
    res->fRefreshRate   = 60;
    res->dwFlags        = D3DPRESENTFLAG_PROGRESSIVE;
    res->iScreen        = 0;
    res->bFullScreen    = true;
    res->iSubtitles     = (int)(0.965 * res->iHeight);
    res->fPixelRatio    = 1.0f;
    res->strMode.Format("%dx%d @ %.2f%s - Full Screen", res->iScreenWidth, res->iScreenHeight, res->fRefreshRate, res->dwFlags & D3DPRESENTFLAG_INTERLACED ? "i" : "");

    CLog::Log(LOGNOTICE,"Current resolution: %dx%d\n", window_attr.width, window_attr.height);
    return true;
}

bool CEGLNativeTypeOdroid::SetNativeResolution(const RESOLUTION_INFO &res)
{
    return false;
}

bool CEGLNativeTypeOdroid::ProbeResolutions(std::vector<RESOLUTION_INFO> &resolutions)
{
    RESOLUTION_INFO res;
    int iRet = GetNativeResolution(&res);
    if (iRet && res.iWidth > 1 && res.iHeight > 1)
    {
        resolutions.push_back(res);
        return true;
    }
    return false;
}

bool CEGLNativeTypeOdroid::GetPreferredResolution(RESOLUTION_INFO *res) const
{
    GetNativeResolution(res);
    return true;
}

bool CEGLNativeTypeOdroid::ShowWindow(bool show)
{
    Display *xDisplay;
    Window nativeWindow;
    xDisplay = (Display *) m_nativeDisplay;
    nativeWindow = (Window) m_nativeWindow;
    if (show) {
      XMapWindow(xDisplay, nativeWindow);
    } else {
      XUnmapWindow(xDisplay, nativeWindow);
    }
    XSync(xDisplay, False);
    return false;
}
