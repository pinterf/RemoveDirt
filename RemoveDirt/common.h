#ifndef __COMMON___H__
#define __COMMON___H__

#include "avisynth.h"

void debug_printf(const char *format, ...);
void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname, IScriptEnvironment* env);

// helper for uniformly access planar-hacked YUY2 and standard YV12 planes
class   AccessFrame
{
  int   uoffset, voffset;

  int(__stdcall AccessFrame::*_GetPitchUV)(VideoFrame *frame);
  const BYTE* (__stdcall AccessFrame::*_GetReadPtrU)(VideoFrame *frame);
  const BYTE* (__stdcall AccessFrame::*_GetReadPtrV)(VideoFrame *frame);
  BYTE* (__stdcall AccessFrame::*_GetWritePtrU)(VideoFrame *frame);
  BYTE* (__stdcall AccessFrame::*_GetWritePtrV)(VideoFrame *frame);


  int __stdcall YV12_GetPitchUV(VideoFrame *frame)
  {
    return  frame->GetPitch(PLANAR_U);
  }

  int __stdcall YUY2_GetPitchUV(VideoFrame *frame)
  {
    return  frame->GetPitch();
  }

  const BYTE *__stdcall YV12_GetReadPtrU(VideoFrame *frame)
  {
    return  frame->GetReadPtr(PLANAR_U);
  }

  const BYTE *__stdcall YUY2_GetReadPtrU(VideoFrame *frame)
  {
    return  frame->GetReadPtr() + uoffset;
  }

  const BYTE *__stdcall YV12_GetReadPtrV(VideoFrame *frame)
  {
    return  frame->GetReadPtr(PLANAR_V);
  }

  const BYTE *__stdcall YUY2_GetReadPtrV(VideoFrame *frame)
  {
    return  frame->GetReadPtr() + voffset;
  }

  BYTE *__stdcall YV12_GetWritePtrU(VideoFrame *frame)
  {
    return  frame->GetWritePtr(PLANAR_U);
  }

  BYTE *__stdcall YUY2_GetWritePtrU(VideoFrame *frame)
  {
    return  frame->GetWritePtr() + uoffset;
  }

  BYTE *__stdcall YV12_GetWritePtrV(VideoFrame *frame)
  {
    return  frame->GetWritePtr(PLANAR_V);
  }

  BYTE *__stdcall YUY2_GetWritePtrV(VideoFrame *frame)
  {
    return  frame->GetWritePtr() + voffset;
  }

public:
  inline int GetPitchY(PVideoFrame &frame)
  {
    return  frame->GetPitch();
  }

  inline int GetPitchUV(PVideoFrame &frame)
  {
    return  (this->*_GetPitchUV)(frame.operator ->());
  }

  inline const BYTE * GetReadPtrY(PVideoFrame &frame)
  {
    return  frame->GetReadPtr();
  }

  inline const BYTE * GetReadPtrU(PVideoFrame &frame)
  {
    return  (this->*_GetReadPtrU)(frame.operator ->());
  }

  inline const BYTE * GetReadPtrV(PVideoFrame &frame)
  {
    return  (this->*_GetReadPtrV)(frame.operator ->());
  }

  inline BYTE * GetWritePtrY(PVideoFrame &frame)
  {
    return  frame->GetWritePtr();
  }

  inline BYTE * GetWritePtrU(PVideoFrame &frame)
  {
    return  (this->*_GetWritePtrU)(frame.operator ->());
  }

  inline BYTE * GetWritePtrV(PVideoFrame &frame)
  {
    return  (this->*_GetWritePtrV)(frame.operator ->());
  }

  AccessFrame(int   width, bool yuy2)
  {
    _GetPitchUV = &AccessFrame::YV12_GetPitchUV;
    _GetReadPtrU = &AccessFrame::YV12_GetReadPtrU;
    _GetReadPtrV = &AccessFrame::YV12_GetReadPtrV;
    _GetWritePtrU = &AccessFrame::YV12_GetWritePtrU;
    _GetWritePtrV = &AccessFrame::YV12_GetWritePtrV;
    uoffset = 0;
    voffset = 0;
    if (yuy2)
    {
      voffset = (uoffset = width) + width / 2;

      _GetPitchUV = &AccessFrame::YUY2_GetPitchUV;
      _GetReadPtrU = &AccessFrame::YUY2_GetReadPtrU;
      _GetReadPtrV = &AccessFrame::YUY2_GetReadPtrV;
      _GetWritePtrU = &AccessFrame::YUY2_GetWritePtrU;
      _GetWritePtrV = &AccessFrame::YUY2_GetWritePtrV;
    }
  }
};


#endif	// __COMMON___H__
