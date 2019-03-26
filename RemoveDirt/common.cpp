#include "common.h"

#include "avisynth.h"
#include <Windows.h>
#include <stdint.h>
#include <stdio.h>

void debug_printf(const char *format, ...)
{
  char buffer[200];
  va_list   args;
  va_start(args, format);
  vsprintf_s(buffer, format, args);
  va_end(args);
  OutputDebugString(buffer);
}


#define COMPARE_MASK    (~24)

void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname, IScriptEnvironment* env)
{
  if ((vi1.width != vi2.width) || (vi1.height != vi2.height) || ((vi1.pixel_type & COMPARE_MASK) != (vi2.pixel_type & COMPARE_MASK)))
  {
#if 1
    debug_printf("widths = %u, %u, heights = %u, %u, color spaces = %X, %X\n"
      , vi1.width, vi2.width, vi1.height, vi2.height, vi1.pixel_type, vi2.pixel_type);
#endif
    env->ThrowError("%s: clips must be of equal type", progname);
  }
  if (vi1.num_frames > vi2.num_frames) vi1.num_frames = vi2.num_frames;
}
