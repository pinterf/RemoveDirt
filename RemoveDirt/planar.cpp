#define VC_EXTRALEAN 
#include <Windows.h>
#include "avisynth.h"
#include "planar.h"

const	int		plane[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };

int __stdcall PlanarAccess::YV12_GetPitch(VideoFrame *frame, int i)
{
  return	frame->GetPitch(plane[i]);
}

int __stdcall PlanarAccess::YUY2_GetPitch(VideoFrame *frame, int i)
{
  return	frame->GetPitch();
}

const BYTE *__stdcall PlanarAccess::YV12_GetReadPtr(VideoFrame *frame, int i)
{
  return	frame->GetReadPtr(plane[i]);
}

const BYTE *__stdcall PlanarAccess::YUY2_GetReadPtr(VideoFrame *frame, int i)
{
  return	frame->GetReadPtr() + planeoffset[i];
}

BYTE *__stdcall PlanarAccess::YV12_GetWritePtr(VideoFrame *frame, int i)
{
  return	frame->GetWritePtr(plane[i]);
}

BYTE *__stdcall PlanarAccess::YUY2_GetWritePtr(VideoFrame *frame, int i)
{
  return	frame->GetWritePtr() + planeoffset[i];
}


PlanarAccess::PlanarAccess(const VideoInfo &vi, bool planar)
{
  width[0] = vi.width;
  height[0] = vi.height;

  if (vi.IsYUV() && !vi.IsY())
  {
    if (!vi.IsYUY2()) {
      width[1] = vi.width / 2;
      height[1] = vi.height;
      _GetPitch = &PlanarAccess::YUY2_GetPitch;
      _GetReadPtr = &PlanarAccess::YUY2_GetReadPtr;
      _GetWritePtr = &PlanarAccess::YUY2_GetWritePtr;
      planeoffset[0] = 0;
      planeoffset[1] = width[0];
      planeoffset[2] = planeoffset[1] + width[1];
    }
    else {
      width[1] = vi.width >> vi.GetPlaneWidthSubsampling(PLANAR_U);
      height[1] = vi.height >> vi.GetPlaneHeightSubsampling(PLANAR_U);
      _GetPitch = &PlanarAccess::YV12_GetPitch;
      _GetReadPtr = &PlanarAccess::YV12_GetReadPtr;
      _GetWritePtr = &PlanarAccess::YV12_GetWritePtr;
    }
    width[2] = width[1];
    height[2] = height[1];
    planes = 2; // max plane index
  }
  else
  {
    // Y? packed RGB?
    planes = 0; // max plane index
    width[0] = vi.RowSize();
  }
}

HomogeneousChild::HomogeneousChild(PClip _child, bool grey, IScriptEnvironment* env) : child(_child)
{
  vi = _child->GetVideoInfo();
  width[2] = width[1] = (width[0] = vi.width) / 2;
  height[2] = height[1] = (height[0] = vi.height) / 2;

  if (vi.IsYUY2()) // was: !IsYV12
  {
    if (!grey) width[0] = vi.RowSize(); // single plane
    grey = true;
  }

  PVideoFrame nf = env->NewVideoFrame(vi);
  pitch[0] = nf->GetPitch();

  if (vi.IsYV12())
  {
    pitch[1] = nf->GetPitch(PLANAR_U);
  }
  else
  {
    if (!grey) width[0] = vi.RowSize();
    pitch[1] = pitch[0];
    grey = true;
  }
  planes = grey ? 0 : 2; // max plane index
  pitch[2] = pitch[1];
}

PVideoFrame __stdcall HomogeneousChild::GetFrame(int n, IScriptEnvironment* env)
{
  PVideoFrame rf = child->GetFrame(n, env);
  if (rf->GetPitch() == pitch[0]) // pitch is OK
    return rf;
  PVideoFrame nf = env->NewVideoFrame(vi);
  int i = planes;
  do
  {
    int j = plane[i];
    env->BitBlt(nf->GetWritePtr(j), pitch[i], rf->GetReadPtr(j), rf->GetPitch(j), width[i], height[i]);
  } while (--i >= 0);
  return nf;
}