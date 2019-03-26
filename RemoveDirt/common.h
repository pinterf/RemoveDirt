#ifndef __COMMON___H__
#define __COMMON___H__

#include "avisynth.h"

void debug_printf(const char *format, ...);
void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname, IScriptEnvironment* env);


#endif	// __COMMON___H__
