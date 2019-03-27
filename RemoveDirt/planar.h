#ifndef	PLANAR_H
#define	PLANAR_H

#include "avisynth.h"

extern const int plane[];

class	PlanarAccess
{
private:
  int	planeoffset[3];
  int(__stdcall PlanarAccess::*_GetPitch)(VideoFrame *frame, int i);
  const BYTE* (__stdcall PlanarAccess::*_GetReadPtr)(VideoFrame *frame, int i);
  BYTE* (__stdcall PlanarAccess::*_GetWritePtr)(VideoFrame *frame, int i);

  int __stdcall YV12_GetPitch(VideoFrame *frame, int i);

  int __stdcall YUY2_GetPitch(VideoFrame *frame, int i);

  const BYTE *__stdcall YV12_GetReadPtr(VideoFrame *frame, int i);

  const BYTE *__stdcall YUY2_GetReadPtr(VideoFrame *frame, int i);

  BYTE *__stdcall YV12_GetWritePtr(VideoFrame *frame, int i);

  BYTE *__stdcall YUY2_GetWritePtr(VideoFrame *frame, int i);

public:
  int	width[3];
  int	height[3];
  int	planes;

  inline int GetPitch(PVideoFrame &frame, int i)
  {
    return	(this->*_GetPitch)(frame.operator ->(), i);
  }

  inline const BYTE * GetReadPtr(PVideoFrame &frame, int i)
  {
    return	(this->*_GetReadPtr)(frame.operator ->(), i);
  }

  inline BYTE * GetWritePtr(PVideoFrame &frame, int i)
  {
    return	(this->*_GetWritePtr)(frame.operator ->(), i);
  }

  PlanarAccess(const VideoInfo &vi, bool planar = true);
};

#if 0
// The two subsequent classes have nothing to do with PlanarAccess, but they provide an analogous
// service, namely to assure that the frames of the child have always the same pitch.
// Unfortunately only the original, not neccessary aligned frames are cached by Avisynth

class HomogeneousChild
{
  PClip	child;
  int		planes;
  int		width[3], height[3];
  VideoInfo vi;

public:
  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
  void __stdcall GetAudio(void* buf, __int64 start, __int64 count, IScriptEnvironment* env) { child->GetAudio(buf, start, count, env); }
  const VideoInfo& __stdcall GetVideoInfo() { return vi; }
  bool __stdcall GetParity(int n) { return child->GetParity(n); }
  void __stdcall SetCacheHints(int cachehints, int frame_range) { child->SetCacheHints(cachehints, frame_range); }
  int		pitch[3];
  HomogeneousChild* operator->() { return this; }
  HomogeneousChild(PClip _child, bool grey, IScriptEnvironment* env);
};

class HomogeneousVideoFilter : public IClip // Replacement for GenericVideoFilter
{
  HomogeneousChild child;
public:
  VideoInfo vi;
  HomogeneousVideoFilter(PClip _child, bool grey, IScriptEnvironment* env) : child(_child, grey, env) { vi = _child->GetVideoInfo(); }
  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) { return child->GetFrame(n, env); }
  void __stdcall GetAudio(void* buf, __int64 start, __int64 count, IScriptEnvironment* env) { child->GetAudio(buf, start, count, env); }
  const VideoInfo& __stdcall GetVideoInfo() { return vi; }
  bool __stdcall GetParity(int n) { return child->GetParity(n); }
  int __stdcall SetCacheHints(int cachehints, int frame_range) { return 0;  };
};
#endif
#endif // PLANAR_H