#define LOGO "RemoveDirt 0.9\n"
// Avisynth filter for removing dirt from film clips
//
// By Rainer Wittmann <gorw@gmx.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// To get a copy of the GNU General Public License write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .


//
// Part 1: options at compile time
//

//#define ISSE 2
//#define DEBUG_NAME
//#define SSE2TEST
//#define RANGEFILES 

#ifndef ISSE
#define ISSE 0
#endif

#define FHANDLERS 9
//#define STATISTICS // only for internal use
//#define TEST_BLOCKCOMPARE
//#define TEST_VERTICAL_DIFF
//#define TEST_VERTICAL_DIFF_CHROMA


#define MOTIONBLOCKWIDTH  8
#define MOTIONBLOCKHEIGHT 8

#define MOTION_FLAG     1
#define MOTION_FLAGN    2
#define MOTION_FLAGP    4
#define TO_CLEAN        8
#define BMARGIN         16
#define MOTION_FLAG1    (MOTION_FLAG | TO_CLEAN)
#define MOTION_FLAG2    (MOTION_FLAGN | TO_CLEAN)
#define MOTION_FLAG3    (MOTION_FLAGP | TO_CLEAN)
#define MOTION_FLAGS    (MOTION_FLAG | MOTION_FLAGN | MOTION_FLAGP)

#define U_N                 54u     // green
#define V_N                 34
#define U_M                 90      // red
#define V_M                 240
#define U_P                 240     // blue
#define V_P                 110
#define u_ncolor            (U_N + (U_N << 8) + (U_N << 16) + (U_N << 24))
#define v_ncolor            (V_N + (V_N << 8) + (V_N << 16) + (V_N << 24))
#define u_mcolor            (U_M + (U_M << 8) + (U_M << 16) + (U_M << 24))
#define v_mcolor            (V_M + (V_M << 8) + (V_M << 16) + (V_M << 24))
#define u_pcolor            (U_P + (U_P << 8) + (U_P << 16) + (U_P << 24))
#define v_pcolor            (V_P + (V_P << 8) + (V_P << 16) + (V_P << 24))

#define DEFAULT_GMTHRESHOLD 80
#define DEFAULT_DIST        1
#define DEFAULT_MTHRESHOLD  160
#define DEFAULT_PTHRESHOLD  10
#define DEFAULT_FTHRESHOLD  (-1)
#define DEFAULT_TOLERANCE   12
#define DEFAULT_NOISE       0

#if ISSE > 1
#define SSESIZE     16
#else
#define SSESIZE     8
#endif

//
// Part 2: include files and basic definitions
//

#define VC_EXTRALEAN
#include <Windows.h>
#include <stdlib.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <stdio.h>
#include "avisynth.h"

#ifdef  STATISTICS
unsigned    compare8;
unsigned    compare16;
unsigned    clean8x8;
unsigned    clean16x8;
unsigned    clean4x4;
unsigned    clean4x8;
unsigned    clean8x4;
unsigned    copy8x8;
unsigned    copy16x8;
unsigned    copy4x4;
unsigned    copy4x8;
unsigned    copy8x4;
#endif

static  IScriptEnvironment  *AVSenvironment;

//
// Part 3: auxiliary functions
//

void    debug_printf(const char *format, ...)
{
  char buffer[200];
  va_list   args;
  va_start(args, format);
  vsprintf_s(buffer, format, args);
  va_end(args);
  OutputDebugString(buffer);
}

//
// Part 4: block comparison functions
//

#if ISSE > 1
__declspec(align(16))
unsigned    blockcompare_result[4]; // must be global otherwise compiler may generate incorrect alignment
#endif

unsigned __stdcall SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
{
#ifdef  STATISTICS
  ++compare8;
#endif
  __asm mov         edx, pitch1
  __asm mov         esi, pitch2
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   lea         edi, [esi + 2 * esi]
    __asm   mov         ebx, p2
  __asm movq        mm0, [eax]
    __asm   movq        mm1, [eax + edx]
    __asm   psadbw      mm0, [ebx]
    __asm   psadbw      mm1, [ebx + esi]
    __asm   movq        mm2, [eax + 2 * edx]
    __asm   movq        mm3, [eax + ecx]
    __asm   psadbw      mm2, [ebx + 2 * esi]
    __asm   lea         eax, [eax + 4 * edx]
    __asm   psadbw      mm3, [ebx + edi]
    __asm   paddd       mm0, mm2
  __asm paddd       mm1, mm3
  __asm lea         ebx, [ebx + 4 * esi]
    __asm   movq        mm2, [eax]
    __asm   movq        mm3, [eax + edx]
    __asm   psadbw      mm2, [ebx]
    __asm   psadbw      mm3, [ebx + esi]
    __asm   paddd       mm0, mm2
  __asm paddd       mm1, mm3
  __asm movq        mm2, [eax + 2 * edx]
    __asm   movq        mm3, [eax + ecx]
    __asm   psadbw      mm2, [ebx + 2 * esi]
    __asm   psadbw      mm3, [ebx + edi]
    __asm   paddd       mm0, mm2
  __asm paddd       mm1, mm3
  __asm paddd       mm0, mm1
  __asm movd        eax, mm0
}

#ifdef  SSE2TEST
unsigned __stdcall SSE2test(const BYTE *p1, int pitch1)
{
  __asm mov         edx, pitch1
  __asm mov         eax, p1
  __asm mov         ecx, 8
  __asm pxor        xmm4, xmm4
  __asm _loop:
  __asm movhps      xmm0, QWORD PTR[eax]
    __asm   movq        xmm1, QWORD PTR[eax]
    __asm   movhlps     xmm2, xmm0
  __asm psadbw      xmm1, xmm2
  __asm paddd       xmm4, xmm1
  __asm add         eax, edx
  __asm loop        _loop
  __asm movd        eax, xmm4
}
#endif

// mm7 contains already the noise level!
unsigned __stdcall NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
{
#ifdef  STATISTICS
  ++compare8;
#endif
  __asm mov         edx, pitch1
  __asm mov         esi, pitch2
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   lea         edi, [esi + 2 * esi]
    __asm   mov         ebx, p2
#if ISSE > 1
  __asm movq        xmm0, QWORD PTR[eax]
    __asm   movq        xmm2, QWORD PTR[eax + edx]
    __asm   movhps      xmm0, [eax + 2 * edx]
    __asm   movhps      xmm2, [eax + ecx]
    __asm   movdqa      xmm3, xmm0
  __asm movdqa      xmm4, xmm2
  __asm movq        xmm5, QWORD PTR[ebx]
    __asm   movq        xmm6, QWORD PTR[ebx + esi]
    __asm   lea         eax, [eax + 4 * edx]
    __asm   movhps      xmm5, [ebx + 2 * esi]
    __asm   movhps      xmm6, [ebx + edi]
    __asm   psubusb     xmm0, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm0, xmm7
  __asm psubusb     xmm2, xmm7
  __asm lea         ebx, [ebx + 4 * esi]
    __asm   psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm0, xmm5
  __asm psadbw      xmm6, xmm2
  __asm movq        xmm1, QWORD PTR[eax]
    __asm   paddd       xmm0, xmm6
  __asm movq        xmm2, QWORD PTR[eax + edx]
    __asm   movhps      xmm1, [eax + 2 * edx]
    __asm   movhps      xmm2, [eax + ecx]
    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movq        xmm5, QWORD PTR[ebx]
    __asm   movq        xmm6, QWORD PTR[ebx + esi]
    __asm   movhps      xmm5, [ebx + 2 * esi]
    __asm   movhps      xmm6, [ebx + edi]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm1, xmm5
  __asm psadbw      xmm6, xmm2
  __asm paddd       xmm0, xmm1
  __asm paddd       xmm0, xmm6
  __asm movhlps     xmm1, xmm0
  __asm paddd       xmm0, xmm1
  __asm movd        eax, xmm0
#else   // ISSE > 1
  __asm movq        mm0, [eax]
    __asm   movq        mm2, [eax + edx]
    __asm   movq        mm3, mm0
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx]
    __asm   movq        mm6, [ebx + esi]
    __asm   psubusb     mm0, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm0, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm psadbw      mm0, mm5
  __asm psadbw      mm6, mm2
  __asm movq        mm1, [eax + 2 * edx]
    __asm   paddd       mm0, mm6
  __asm movq        mm2, [eax + ecx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx + 2 * esi]
    __asm   movq        mm6, [ebx + edi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm lea         eax, [eax + 4 * edx]
    __asm   psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm lea         ebx, [ebx + 4 * esi]
    __asm   psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm psadbw      mm5, mm1
  __asm psadbw      mm6, mm2
  __asm paddd       mm0, mm5
  __asm movq        mm1, [eax]
    __asm   paddd       mm0, mm6
  __asm movq        mm2, [eax + edx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx]
    __asm   movq        mm6, [ebx + esi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm psadbw      mm5, mm1
  __asm psadbw      mm6, mm2
  __asm paddd       mm0, mm5
  __asm movq        mm1, [eax + 2 * edx]
    __asm   paddd       mm0, mm6
  __asm movq        mm2, [eax + ecx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx + 2 * esi]
    __asm   movq        mm6, [ebx + edi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm psadbw      mm5, mm1
  __asm psadbw      mm6, mm2
  __asm paddd       mm0, mm5
  __asm paddd       mm0, mm6

  __asm movd        eax, mm0
#endif
}

static const __declspec(align(SSESIZE)) BYTE excessadd[SSESIZE]
#if ISSE > 1
= { 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4 };
#else
= { 8,8,8,8,8,8,8,8 };
#endif


unsigned __stdcall ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
{
#ifdef  STATISTICS
  ++compare16;
#endif
  __asm mov         edx, pitch1
  __asm mov         esi, pitch2
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   lea         edi, [esi + 2 * esi]
    __asm   mov         ebx, p2
#if ISSE > 1
  __asm movq        xmm0, QWORD PTR[eax]
    __asm   movq        xmm2, QWORD PTR[eax + edx]
    __asm   movhps      xmm0, [eax + 2 * edx]
    __asm   movhps      xmm2, [eax + ecx]
    __asm   movdqa      xmm3, xmm0
  __asm movdqa      xmm4, xmm2
  __asm movq        xmm5, QWORD PTR[ebx]
    __asm   movq        xmm6, QWORD PTR[ebx + esi]
    __asm   lea         eax, [eax + 4 * edx]
    __asm   movhps      xmm5, [ebx + 2 * esi]
    __asm   movhps      xmm6, [ebx + edi]
    __asm   psubusb     xmm0, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm0, xmm7
  __asm psubusb     xmm2, xmm7
  __asm lea         ebx, [ebx + 4 * esi]
    __asm   psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm0, xmm5
  __asm pcmpeqb     xmm6, xmm2
  __asm movq        xmm1, QWORD PTR[eax]
    __asm   paddb       xmm0, xmm6
  __asm movq        xmm2, QWORD PTR[eax + edx]
    __asm   movhps      xmm1, [eax + 2 * edx]
    __asm   movhps      xmm2, [eax + ecx]
    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movq        xmm5, QWORD PTR[ebx]
    __asm   movq        xmm6, QWORD PTR[ebx + esi]
    __asm   movhps      xmm5, [ebx + 2 * esi]
    __asm   movhps      xmm6, [ebx + edi]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm1, xmm5
  __asm pcmpeqb     xmm6, xmm2
  __asm pxor        xmm5, xmm5
  __asm paddb       xmm0, xmm1
  __asm paddb       xmm6, excessadd
  __asm paddb       xmm0, xmm6
  __asm psadbw      xmm0, xmm5
  __asm movhlps     xmm1, xmm0
  __asm paddd       xmm0, xmm1
  __asm movd        eax, xmm0
#else   // ISSE > 1
  __asm movq        mm0, [eax]
    __asm   movq        mm2, [eax + edx]
    __asm   movq        mm3, mm0
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx]
    __asm   movq        mm6, [ebx + esi]
    __asm   psubusb     mm0, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm0, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm pcmpeqb     mm0, mm5
  __asm pcmpeqb     mm6, mm2
  __asm movq        mm1, [eax + 2 * edx]
    __asm   paddb       mm0, mm6
  __asm movq        mm2, [eax + ecx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx + 2 * esi]
    __asm   movq        mm6, [ebx + edi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm lea         eax, [eax + 4 * edx]
    __asm   psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm lea         ebx, [ebx + 4 * esi]
    __asm   psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm pcmpeqb     mm5, mm1
  __asm pcmpeqb     mm6, mm2
  __asm paddb       mm0, mm5
  __asm movq        mm1, [eax]
    __asm   paddb       mm0, mm6
  __asm movq        mm2, [eax + edx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx]
    __asm   movq        mm6, [ebx + esi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm pcmpeqb     mm5, mm1
  __asm pcmpeqb     mm6, mm2
  __asm paddb       mm0, mm5
  __asm movq        mm1, [eax + 2 * edx]
    __asm   paddb       mm0, mm6
  __asm movq        mm2, [eax + ecx]

    __asm   movq        mm3, mm1
  __asm movq        mm4, mm2
  __asm movq        mm5, [ebx + 2 * esi]
    __asm   movq        mm6, [ebx + edi]
    __asm   psubusb     mm1, mm5
  __asm psubusb     mm2, mm6
  __asm psubusb     mm5, mm3
  __asm psubusb     mm6, mm4
  __asm psubusb     mm1, mm7
  __asm psubusb     mm2, mm7
  __asm psubusb     mm5, mm7
  __asm psubusb     mm6, mm7
  __asm pcmpeqb     mm5, mm1
  __asm pcmpeqb     mm6, mm2
  __asm paddb       mm0, mm5
  __asm pxor        mm1, mm1
  __asm paddb       mm0, mm6
  __asm paddb       mm0, excessadd
  __asm psadbw      mm0, mm1
  __asm movd        eax, mm0
#endif  // ISSE > 1
}


#define mminit()    \
__asm   movq        mm7,                [ecx].noiselevel    

#ifdef  TEST_BLOCKCOMPARE
unsigned __stdcall test_SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  pitch1 -= 8;
  pitch2 -= 8;
  int   res = 0;
  int   i = 8;
  do
  {
    int j = 8;
    do
    {
      int diff = p1[0] - p2[0];
      if (diff < 0) diff = -diff;
      res += diff;
      ++p1;
      ++p2;
    } while (--j);
    p1 += pitch1;
    p2 += pitch2;
  } while (--i);
  return res;
}

unsigned __stdcall test_NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  pitch1 -= 8;
  pitch2 -= 8;
  int   res = 0;
  int   i = 8;
  do
  {
    int j = 8;
    do
    {
      int diff = p1[0] - p2[0];
      if (diff < 0) diff = -diff;
      diff -= noise;
      if (diff < 0) diff = 0;
      res += diff;
      ++p1;
      ++p2;
    } while (--j);
    p1 += pitch1;
    p2 += pitch2;
  } while (--i);
  return res;
}

unsigned __stdcall test_ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  pitch1 -= 8;
  pitch2 -= 8;
  int   count = 0;
  int   i = 8;
  do
  {
    int j = 8;
    do
    {
      int diff = p1[0] - p2[0];
      if ((diff > noise) || (diff < -noise)) ++count;
      ++p1;
      ++p2;
    } while (--j);
    p1 += pitch1;
    p2 += pitch2;
  } while (--i);
  return count;
}
#endif  // TEST_BLOCKCOMPARE



#if ISSE > 1
void __stdcall SADcompareSSE2(const BYTE *p1, const BYTE *p2, int pitch)
{
#ifdef  STATISTICS
  ++compare16;
#endif
  __asm mov         edx, pitch
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   mov         ebx, p2
  __asm movdqa      xmm0, [eax]
    __asm   movdqa      xmm1, [eax + edx]
    __asm   psadbw      xmm0, [ebx]
    __asm   psadbw      xmm1, [ebx + edx]
    __asm   movdqa      xmm2, [eax + 2 * edx]
    __asm   movdqa      xmm3, [eax + ecx]
    __asm   psadbw      xmm2, [ebx + 2 * edx]
    __asm   lea         eax, [eax + 4 * edx]
    __asm   psadbw      xmm3, [ebx + ecx]
    __asm   paddd       xmm0, xmm2
  __asm paddd       xmm1, xmm3
  __asm lea         ebx, [ebx + 4 * edx]
    __asm   movdqa      xmm2, [eax]
    __asm   movdqa      xmm3, [eax + edx]
    __asm   psadbw      xmm2, [ebx]
    __asm   psadbw      xmm3, [ebx + edx]
    __asm   paddd       xmm0, xmm2
  __asm paddd       xmm1, xmm3
  __asm movdqa      xmm2, [eax + 2 * edx]
    __asm   movdqa      xmm3, [eax + ecx]
    __asm   psadbw      xmm2, [ebx + 2 * edx]
    __asm   psadbw      xmm3, [ebx + ecx]
    __asm   paddd       xmm0, xmm2
  __asm paddd       xmm1, xmm3
  __asm paddd       xmm0, xmm1
  __asm movdqa      blockcompare_result, xmm0
}

// xmm7 contains already the noise level!
void __stdcall NSADcompareSSE2(const BYTE *p1, const BYTE *p2, int pitch)
{
#ifdef  STATISTICS
  ++compare16;
#endif
  __asm mov         edx, pitch
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   mov         ebx, p2
  __asm movdqa      xmm0, [eax]
    __asm   movdqa      xmm2, [eax + edx]
    __asm   movdqa      xmm3, xmm0
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx]
    __asm   movdqa      xmm6, [ebx + edx]
    __asm   psubusb     xmm0, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm0, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm0, xmm5
  __asm psadbw      xmm6, xmm2
  __asm movdqa      xmm1, [eax + 2 * edx]
    __asm   paddd       xmm0, xmm6
  __asm movdqa      xmm2, [eax + ecx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx + 2 * edx]
    __asm   movdqa      xmm6, [ebx + ecx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm lea         eax, [eax + 4 * edx]
    __asm   psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm lea         ebx, [ebx + 4 * edx]
    __asm   psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm5, xmm1
  __asm psadbw      xmm6, xmm2
  __asm paddd       xmm0, xmm5
  __asm movdqa      xmm1, [eax]
    __asm   paddd       xmm0, xmm6
  __asm movdqa      xmm2, [eax + edx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx]
    __asm   movdqa      xmm6, [ebx + edx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm5, xmm1
  __asm psadbw      xmm6, xmm2
  __asm paddd       xmm0, xmm5
  __asm movdqa      xmm1, [eax + 2 * edx]
    __asm   paddd       xmm0, xmm6
  __asm movdqa      xmm2, [eax + ecx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx + 2 * edx]
    __asm   movdqa      xmm6, [ebx + ecx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm psadbw      xmm5, xmm1
  __asm psadbw      xmm6, xmm2
  __asm paddd       xmm0, xmm5
  __asm paddd       xmm0, xmm6

  __asm movdqa      blockcompare_result, xmm0
}

static const __declspec(align(SSESIZE)) BYTE excessaddSSE2[SSESIZE] = { 8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8 };

void __stdcall ExcessPixelsSSE2(const BYTE *p1, const BYTE *p2, int pitch)
{
#ifdef  STATISTICS
  ++compare16;
#endif
  __asm mov         edx, pitch
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   mov         ebx, p2
  __asm movdqa      xmm0, [eax]
    __asm   movdqa      xmm2, [eax + edx]
    __asm   movdqa      xmm3, xmm0
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx]
    __asm   movdqa      xmm6, [ebx + edx]
    __asm   psubusb     xmm0, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm0, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm0, xmm5
  __asm pcmpeqb     xmm6, xmm2
  __asm movdqa      xmm1, [eax + 2 * edx]
    __asm   paddb       xmm0, xmm6
  __asm movdqa      xmm2, [eax + ecx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx + 2 * edx]
    __asm   movdqa      xmm6, [ebx + ecx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm lea         eax, [eax + 4 * edx]
    __asm   psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm lea         ebx, [ebx + 4 * edx]
    __asm   psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm5, xmm1
  __asm pcmpeqb     xmm6, xmm2
  __asm paddb       xmm0, xmm5
  __asm movdqa      xmm1, [eax]
    __asm   paddb       xmm0, xmm6
  __asm movdqa      xmm2, [eax + edx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx]
    __asm   movdqa      xmm6, [ebx + edx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm5, xmm1
  __asm pcmpeqb     xmm6, xmm2
  __asm paddb       xmm0, xmm5
  __asm movdqa      xmm1, [eax + 2 * edx]
    __asm   paddb       xmm0, xmm6
  __asm movdqa      xmm2, [eax + ecx]

    __asm   movdqa      xmm3, xmm1
  __asm movdqa      xmm4, xmm2
  __asm movdqa      xmm5, [ebx + 2 * edx]
    __asm   movdqa      xmm6, [ebx + ecx]
    __asm   psubusb     xmm1, xmm5
  __asm psubusb     xmm2, xmm6
  __asm psubusb     xmm5, xmm3
  __asm psubusb     xmm6, xmm4
  __asm psubusb     xmm1, xmm7
  __asm psubusb     xmm2, xmm7
  __asm psubusb     xmm5, xmm7
  __asm psubusb     xmm6, xmm7
  __asm pcmpeqb     xmm5, xmm1
  __asm pcmpeqb     xmm6, xmm2
  __asm paddb       xmm0, xmm5
  __asm pxor        xmm1, xmm1
  __asm paddb       xmm0, xmm6
  __asm paddb       xmm0, excessaddSSE2
  __asm psadbw      xmm0, xmm1
  __asm movdqa      blockcompare_result, xmm0
}

#define SSE2init()  \
__asm   movdqu      xmm7,               [ecx].noiselevel

#endif  // ISSE > 1

//
// Part 5: Motion Detection
//

class   MotionDetection
{
  __declspec(align(SSESIZE)) BYTE noiselevel[SSESIZE];
  unsigned char *blockproperties_addr;
protected:
  int   pline, nline;
public:
  int   motionblocks;
  unsigned char *blockproperties;
  int   linewidth;
  unsigned(__stdcall *blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2);
  int   hblocks, vblocks;
  unsigned threshold;

#if ISSE > 1
  void(__stdcall *blockcompareSSE2)(const BYTE *p1, const BYTE *p2, int pitch);
  int   hblocksSSE2;        // = hblocks / 2
  bool remainderSSE2;   // = hblocks & 1
  int   linewidthSSE2;
#endif

#ifdef  TEST_BLOCKCOMPARE
  unsigned(__stdcall *test_blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);
#endif

#if ISSE > 1
  void  markblocks1(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    SSE2init()
#else
  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
#endif
    mminit()

#if ISSE <= 1
      motionblocks = 0;
#endif
    int inc1 = MOTIONBLOCKHEIGHT * pitch1 - linewidth;
    int inc2 = MOTIONBLOCKHEIGHT * pitch2 - linewidth;
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        properties[0] = 0;
#ifdef  TEST_BLOCKCOMPARE
        int d;
        if ((d = blockcompare(p1, pitch1, p2, pitch2) - test_blockcompare(p1, pitch1, p2, pitch2, noiselevel[0])) != 0)
          debug_printf("blockcompare test fails with difference = %i\n", d);
#endif
#ifdef  SSE2TEST
        if (SSE2test(p1, pitch1) + SSE2test(p2, pitch2) != 0) debug_printf("SSE2 failure\n");
#endif
        if (blockcompare(p1, pitch1, p2, pitch2) >= threshold)
        {
          properties[0] = MOTION_FLAG1;
          ++motionblocks;
        }

        p1 += MOTIONBLOCKWIDTH;
        p2 += MOTIONBLOCKWIDTH;
        ++properties;
      } while (--i);
      p1 += inc1;
      p2 += inc2;
      ++properties;
    } while (--j);
  }

#if ISSE > 1
  void  markblocks2(const BYTE *p1, const BYTE *p2, int pitch)
  {
    SSE2init();

    int inc = MOTIONBLOCKHEIGHT * pitch - linewidthSSE2;
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocksSSE2;
      do
      {
        blockcompareSSE2(p1, p2, pitch);
        properties[0] = properties[1] = 0;
        if (blockcompare_result[0] >= threshold)
        {
          properties[0] = MOTION_FLAG1;
          ++motionblocks;
        }
        if (blockcompare_result[2] >= threshold)
        {
          properties[1] = MOTION_FLAG1;
          ++motionblocks;
        }
        p1 += 2 * MOTIONBLOCKWIDTH;
        p2 += 2 * MOTIONBLOCKWIDTH;
        properties += 2;
      } while (--i);
      if (remainderSSE2)
      {
        properties[0] = 0;
        if (blockcompare(p1, pitch, p2, pitch) >= threshold)
        {
          properties[0] = MOTION_FLAG1;
          ++motionblocks;
        }
        ++properties;
      }
      p1 += inc;
      p2 += inc;
      ++properties;
    } while (--j);
  }

  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    motionblocks = 0;
    if (((pitch1 - pitch2) | (((unsigned)p1) & 15) | (((unsigned)p2) & 15)) == 0)
      markblocks2(p1, p2, pitch1);
    else    markblocks1(p1, pitch1, p2, pitch2);
  }
#endif  // ISSE > 1

  MotionDetection(int   width, int height, unsigned _threshold, int noise, int noisy) : threshold(_threshold)
  {
    hblocks = (linewidth = width) / MOTIONBLOCKWIDTH;
    vblocks = height / MOTIONBLOCKHEIGHT;

#if ISSE > 1
    linewidthSSE2 = linewidth;
    hblocksSSE2 = hblocks / 2;
    if ((remainderSSE2 = (hblocks & 1)) != 0) linewidthSSE2 -= MOTIONBLOCKWIDTH;
    if ((hblocksSSE2 == 0) || (vblocks == 0)) AVSenvironment->ThrowError("RemoveDirt: width or height of the clip too small");
    blockcompareSSE2 = SADcompareSSE2;
#else
    if ((hblocks == 0) || (vblocks == 0)) AVSenvironment->ThrowError("RemoveDirt: width or height of the clip too small");
#endif
    blockcompare = SADcompare;
#ifdef  TEST_BLOCKCOMPARE
    test_blockcompare = test_SADcompare;
#endif
    if (noise > 0)
    {
#if ISSE > 1
      blockcompareSSE2 = NSADcompareSSE2;
#endif
#ifdef  TEST_BLOCKCOMPARE
      test_blockcompare = test_NSADcompare;
#endif
      blockcompare = NSADcompare;
      memset(noiselevel, noise, SSESIZE);
      if (noisy >= 0)
      {
#if ISSE > 1
        blockcompareSSE2 = ExcessPixelsSSE2;
#endif
#ifdef  TEST_BLOCKCOMPARE
        test_blockcompare = test_ExcessPixels;
#endif
        blockcompare = ExcessPixels;
        threshold = noisy;
      }
    }
    int size;
    blockproperties_addr = new unsigned char[size = (nline = hblocks + 1)*(vblocks + 2)];
    blockproperties = blockproperties_addr + nline;
    pline = -nline;
    memset(blockproperties_addr, BMARGIN, size);
  }


  ~MotionDetection()
  {
    delete[] blockproperties_addr;
  }
};

class   MotionDetectionDist : public MotionDetection
{
public:
  int           distblocks;
  unsigned  blocks;
private:
  unsigned  *isum;
  unsigned  tolerance;
  int           dist, dist1, dist2, hinterior, vinterior, colinc, isumline, isuminc1, isuminc2;
  void  (MotionDetectionDist::*processneighbours)(void);

  using fn_processneighbours_t = void(MotionDetectionDist::*)(void);

  void  markneighbours();

  void  processneighbours1()
  {
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        if (properties[0] == MOTION_FLAG2) ++distblocks;
        ++properties;
      } while (--i);
      properties++;
    } while (--j);
  }

  void  processneighbours2()
  {
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        if (properties[0] == MOTION_FLAG1)
        {
          --distblocks;
          properties[0] = 0;
        }
        else if (properties[0] == MOTION_FLAG2) ++distblocks;
        ++properties;
      } while (--i);
      properties++;
    } while (--j);
  }

  void  processneighbours3()
  {
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        if (properties[0] != (MOTION_FLAG1 | MOTION_FLAG2))
        {
          if (properties[0] == MOTION_FLAG1) --distblocks;
          properties[0] = 0;
        }
        ++properties;
      } while (--i);
      properties++;
    } while (--j);
  }

public:


  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    MotionDetection::markblocks(p1, pitch1, p2, pitch2);
    distblocks = 0;
    if (dist)
    {
      markneighbours();
      (this->*processneighbours)();
    }
  }

  MotionDetectionDist(int   width, int height, int _dist, int _tolerance, int dmode, unsigned threshold, int noise, int noisy)
    : MotionDetection(width, height, threshold, noise, noisy)
  {
    fn_processneighbours_t neighbourproc[] = { &MotionDetectionDist::processneighbours1, &MotionDetectionDist::processneighbours2, &MotionDetectionDist::processneighbours3 };
    blocks = hblocks * vblocks;
    isum = new unsigned[blocks];
    if ((unsigned)dmode >= 3) dmode = 0;
    if ((unsigned)_tolerance > 100) _tolerance = 100;
    if (_tolerance == 0)
    {
      if (dmode == 2) _dist = 0;
      dmode = 0;
    }
    processneighbours = neighbourproc[dmode];
    dist2 = (dist1 = (dist = _dist) + 1) + 1;
    isumline = hblocks * sizeof(unsigned);
    int d = dist1 + dist;
    tolerance = ((unsigned)d * (unsigned)d * (unsigned)_tolerance * MOTION_FLAG1) / 100;
    hinterior = hblocks - d;
    vinterior = vblocks - d;
    colinc = 1 - (vblocks * nline);
    isuminc1 = (1 - (vinterior + dist)*hblocks) * sizeof(unsigned);
    isuminc2 = (1 - vblocks * hblocks) * sizeof(unsigned);
  }

  ~MotionDetectionDist()
  {
    delete[] isum;
  }
};

void    MotionDetectionDist::markneighbours()
{
  unsigned char *begin = blockproperties;
  unsigned char *end = begin;
  unsigned  *isum2 = isum;

  int   j = vblocks;
  do
  {
    unsigned sum = 0;
    int i = dist;
    do
    {
      sum += *end++;
    } while (--i);

    i = dist1;
    do
    {
      *isum2++ = (sum += *end++);
    } while (--i);

    i = hinterior;
    do
    {
      *isum2++ = (sum += *end++ - *begin++);
    } while (--i);

    i = dist;
    do
    {
      *isum2++ = (sum -= *begin++);
    } while (--i);

    begin += dist2;
    end++;
  } while (--j);

  unsigned * isum1 = isum2 = isum;
  begin = blockproperties;
  j = hblocks;
  do
  {
    unsigned sum = 0;
    int i = dist;
    do
    {
      sum += *isum2;
      isum2 = (unsigned*)((char*)isum2 + isumline);
    } while (--i);

    i = dist1;
    do
    {
      sum += *isum2;
      isum2 = (unsigned*)((char*)isum2 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    i = vinterior;
    do
    {
      sum += *isum2 - *isum1;
      isum2 = (unsigned*)((char*)isum2 + isumline);
      isum1 = (unsigned*)((char*)isum1 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    i = dist;
    do
    {
      sum -= *isum1;
      isum1 = (unsigned*)((char*)isum1 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    begin += colinc;
    isum1 = (unsigned*)((char*)isum1 + isuminc1);
    isum2 = (unsigned*)((char*)isum2 + isuminc2);
  } while (--j);

}

int __stdcall horizontal_diff(const BYTE *p, int pitch)
{
#ifdef STATISTICS
  st_horizontal_diff_luma++;
#endif
  __asm mov         edx, p
  __asm mov         eax, pitch
  __asm movq        mm0, [edx]
    __asm   psadbw      mm0, [edx + eax]
    __asm   movd        eax, mm0
}

int __stdcall horizontal_diff_chroma(const BYTE *u, const BYTE *v, int pitch)
{
#ifdef STATISTICS
  st_horizontal_diff_chroma++;
#endif
  __asm mov         edx, u
  __asm mov         eax, pitch
  __asm movd        mm0, [edx]
    __asm   mov         ecx, v
  __asm movd        mm1, [edx + eax]
    __asm   punpckldq   mm0, [ecx]
    __asm   punpckldq   mm1, [ecx + eax]
    __asm   psadbw      mm0, mm1
  __asm movd        eax, mm0
}

int __stdcall vertical_diff(const BYTE *p, int pitch)
{
#ifdef STATISTICS
  st_vertical_diff++;
#endif
  __asm mov         eax, p
  __asm mov         edx, pitch
  __asm pinsrw      mm0, [eax], 0
  __asm lea         ecx, [2 * edx + edx]
    __asm   pinsrw      mm1, [eax + edx], 0
  __asm pinsrw      mm0, [eax + 2 * edx], 1
  __asm pinsrw      mm1, [eax + ecx], 1
  __asm lea         eax, [eax + 4 * edx]
    __asm   pcmpeqb     mm7, mm7
  __asm pinsrw      mm0, [eax], 2
  __asm pinsrw      mm1, [eax + edx], 2
  __asm psrlw       mm7, 8
  __asm pinsrw      mm0, [eax + 2 * edx], 3
  __asm pinsrw      mm1, [eax + ecx], 3
  __asm movq        mm2, mm0
  __asm movq        mm3, mm1
  __asm pand        mm0, mm7
  __asm psllw       mm1, 8
  __asm psrlw       mm2, 8
  __asm psubusb     mm3, mm7
  __asm por         mm0, mm1
  __asm por         mm2, mm3
  __asm psadbw      mm0, mm2
  __asm movd        eax, mm0
}

#ifdef  TEST_VERTICAL_DIFF
int __stdcall test_vertical_diff(const BYTE *p, int pitch)
{
  int   res = 0;
  int   i = 8;
  do
  {
    int diff = p[0] - p[1];
    if (diff < 0) res -= diff;
    else res += diff;
    p += pitch;
  } while (--i);
  return    res;
}
#endif

int __stdcall vertical_diff_yv12_chroma(const BYTE *u, const BYTE *v, int pitch)
{
#ifdef STATISTICS
  st_vertical_diff_chroma++;
#endif
  __asm mov         eax, u
  __asm mov         edx, pitch
  __asm pinsrw      mm0, [eax], 0
  __asm lea         ecx, [2 * edx + edx]
    __asm   pinsrw      mm1, [eax + edx], 0
  __asm pinsrw      mm0, [eax + 2 * edx], 1
  __asm pinsrw      mm1, [eax + ecx], 1
  __asm mov         eax, v
  __asm pcmpeqb     mm7, mm7
  __asm pinsrw      mm0, [eax], 2
  __asm pinsrw      mm1, [eax + edx], 2
  __asm psrlw       mm7, 8
  __asm pinsrw      mm0, [eax + 2 * edx], 3
  __asm pinsrw      mm1, [eax + ecx], 3
  __asm movq        mm2, mm0
  __asm movq        mm3, mm1
  __asm pand        mm0, mm7
  __asm psllw       mm1, 8
  __asm psrlw       mm2, 8
  __asm psubusb     mm3, mm7
  __asm por         mm0, mm1
  __asm por         mm2, mm3
  __asm psadbw      mm0, mm2
  __asm movd        eax, mm0
}

#ifdef  TEST_VERTICAL_DIFF_CHROMA
int __stdcall test_vertical_diff_yv12_chroma(const BYTE *u, const BYTE *v, int pitch)
{
  int res = 0;
  int   i = 4;
  do
  {
    int diff = u[0] - u[1];
    if (diff < 0) res -= diff;
    else res += diff;
    diff = v[0] - v[1];
    if (diff < 0) res -= diff;
    else res += diff;
    u += pitch;
    v += pitch;
  } while (--i);
  return    res;
}

int __stdcall test_vertical_diff_yuy2_chroma(const BYTE *u, const BYTE *v, int pitch)
{
  int   res1 = 0;
  int   i = 4;
  do
  {
    int diff = u[0] - u[1];
    if (diff < 0) res1 -= diff;
    else res1 += diff;
    diff = v[0] - v[1];
    if (diff < 0) res1 -= diff;
    else res1 += diff;
    u += pitch;
    v += pitch;
  } while (--i);

  int   res2 = 0;
  i = 4;
  do
  {
    int diff = u[0] - u[1];
    if (diff < 0) res2 -= diff;
    else res2 += diff;
    diff = v[0] - v[1];
    if (diff < 0) res2 -= diff;
    else res2 += diff;
    u += pitch;
    v += pitch;
  } while (--i);

  return    (res1 + res2 + 1) / 2;
}
#endif  // TEST_VERTICAL_DIFF_CHROMA

int __stdcall vertical_diff_yuy2_chroma(const BYTE *u, const BYTE *v, int pitch)
{
#ifdef STATISTICS
  st_vertical_diff_chroma++;
#endif
  __asm mov         eax, u
  __asm mov         edx, pitch
  __asm pinsrw      mm0, [eax], 0
  //__asm   pcmpeqb     mm7,            mm7
  __asm lea         ecx, [2 * edx + edx]
    __asm   pinsrw      mm1, [eax + edx], 0
  //__asm   psrlw       mm7,            8
  __asm pinsrw      mm0, [eax + 2 * edx], 1
  __asm pinsrw      mm1, [eax + ecx], 1
  __asm lea         eax, [eax + 4 * edx]
    __asm   pinsrw      mm0, [eax], 2
  __asm pinsrw      mm1, [eax + edx], 2
  __asm pinsrw      mm0, [eax + 2 * edx], 3
  __asm pinsrw      mm1, [eax + ecx], 3
  __asm movq        mm2, mm0
  __asm movq        mm3, mm1
  __asm pand        mm0, mm7
  __asm psllw       mm1, 8
  __asm psrlw       mm2, 8
  __asm psubusb     mm3, mm7
  __asm mov         eax, v
  __asm por         mm0, mm1
  __asm por         mm2, mm3
  __asm pinsrw      mm4, [eax], 0
  __asm pinsrw      mm1, [eax + edx], 0
  __asm pinsrw      mm4, [eax + 2 * edx], 1
  __asm pinsrw      mm1, [eax + ecx], 1
  __asm lea         eax, [eax + 4 * edx]
    __asm   pinsrw      mm4, [eax], 2
  __asm pinsrw      mm1, [eax + edx], 2
  __asm pinsrw      mm4, [eax + 2 * edx], 3
  __asm pinsrw      mm1, [eax + ecx], 3
  __asm movq        mm6, mm4
  __asm movq        mm3, mm1
  __asm pand        mm4, mm7
  __asm psllw       mm1, 8
  __asm psrlw       mm6, 8
  __asm psubusb     mm3, mm7
  __asm por         mm4, mm1
  __asm por         mm6, mm3
  __asm psadbw      mm0, mm2
  __asm psadbw      mm4, mm6
  __asm pavgw       mm0, mm4
  __asm movd        eax, mm0
}

void __stdcall copy8x8(BYTE *dest, int dpitch, const BYTE *src, int spitch)
{
  __asm mov         esi, src
  __asm mov         eax, spitch
  __asm mov         edi, dest
  __asm mov         ebx, dpitch
  __asm movq        mm0, [esi]
    __asm   lea         ecx, [eax + 2 * eax]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm lea         edx, [ebx + 2 * ebx]
    __asm   movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm lea         esi, [esi + 4 * eax]
    __asm   movq[edi + edx], mm1

  __asm movq        mm0, [esi]
    __asm   lea         edi, [edi + 4 * ebx]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm movq[edi + edx], mm1
}

void __stdcall copy_yv12_chroma(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
  __asm mov         esi, srcu
  __asm mov         eax, spitch
  __asm mov         edi, destu
  __asm mov         ebx, dpitch
  __asm movq        mm0, [esi]
    __asm   lea         ecx, [eax + 2 * eax]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm lea         edx, [ebx + 2 * ebx]
    __asm   movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm movq[edi + edx], mm1

  __asm mov         esi, srcv
  __asm mov         edi, destv
  __asm movq        mm0, [esi]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm movq[edi + edx], mm1
}

void __stdcall copy_yuy2_chroma(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
  __asm mov         esi, srcu
  __asm mov         eax, spitch
  __asm mov         edi, destu
  __asm mov         ebx, dpitch
  __asm movq        mm0, [esi]
    __asm   lea         ecx, [eax + 2 * eax]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm lea         edx, [ebx + 2 * ebx]
    __asm   movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm lea         esi, [esi + 4 * eax]
    __asm   movq[edi + edx], mm1

  __asm movq        mm0, [esi]
    __asm   lea         edi, [edi + 4 * ebx]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm movq[edi + edx], mm1

  __asm mov         esi, srcv
  __asm mov         edi, destv
  __asm movq        mm0, [esi]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm lea         esi, [esi + 4 * eax]
    __asm   movq[edi + edx], mm1

  __asm movq        mm0, [esi]
    __asm   lea         edi, [edi + 4 * ebx]
    __asm   movq        mm1, [esi + eax]
    __asm   movq[edi], mm0
  __asm movq[edi + ebx], mm1

  __asm movq        mm0, [esi + 2 * eax]
    __asm   movq        mm1, [esi + ecx]
    __asm   movq[edi + 2 * ebx], mm0
  __asm movq[edi + edx], mm1
}

void    inline colorise(BYTE *u, BYTE *v, int pitch, int height, unsigned ucolor, unsigned vcolor)
{
  int i = height;
  do
  {
    *(unsigned*)u = ucolor;
    *(unsigned*)v = vcolor;
    u += pitch;
    v += pitch;
  } while (--i);
}

class   Postprocessing
  : public MotionDetectionDist
{
  int   linewidthUV;
  int   chromaheight;
  int   chromaheightm;  // = chromaheight - 1
  int   pthreshold;
  int   cthreshold;
  int(__stdcall *vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);
  void(__stdcall *copy_chroma)(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch);
#ifdef  TEST_VERTICAL_DIFF_CHROMA
  int(__stdcall *test_vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);
#endif

public:
  int       loops;
  int       restored_blocks;

#define leftdp      (-1)
#define rightdp     7
#define leftsp      leftdp
#define rightsp     rightdp
#define topdp       (-dpitch)
#define topsp       (-spitch)
#define rightbldp   8
#define rightblsp   rightbldp
#define leftbldp    (-8)
#define leftblsp    leftbldp

  void  postprocessing_grey(BYTE *dp, int dpitch, const BYTE *sp, int spitch)
  {
    int bottomdp = 7 * dpitch;
    int bottomsp = 7 * spitch;
    int dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth;
    int sinc = MOTIONBLOCKHEIGHT * spitch - linewidth;

    loops = restored_blocks = 0;

    //debug_printf("%u\n", vertical_diff(dp + 5000, dpitch)); // to see some values

    int to_restore;

    do
    {
      BYTE      *dp2 = dp;
      const BYTE    *sp2 = sp;
      unsigned char *cl = blockproperties;
      ++loops;
      to_restore = 0;

      int i = vblocks;
      do
      {
        int j = hblocks;
        do
        {
          if ((cl[0] & TO_CLEAN) != 0)
          {
            copy8x8(dp2, dpitch, sp2, spitch);
            cl[0] &= ~TO_CLEAN;

            if (cl[-1] == 0)
            {
#ifdef  TEST_VERTICAL_DIFF
              if (vertical_diff(dp2 + leftdp, dpitch) != test_vertical_diff(dp2 + leftdp, dpitch)) debug_printf("vertical_diff incorrect\n");
#endif
              if (vertical_diff(dp2 + leftdp, dpitch) > vertical_diff(sp2 + leftsp, spitch) + pthreshold)
              {
                ++to_restore;
                cl[-1] = MOTION_FLAG3;
              }
            }
            if (cl[1] == 0)
            {
              if (vertical_diff(dp2 + rightdp, dpitch) > vertical_diff(sp2 + rightsp, spitch) + pthreshold)
              {
                ++to_restore;
                cl[1] = MOTION_FLAG3;
              }
            }
            if (cl[pline] == 0)
            {
              if (horizontal_diff(dp2 + topdp, dpitch) > horizontal_diff(sp2 + topsp, spitch) + pthreshold)
              {
                ++to_restore;
                cl[pline] = MOTION_FLAG3;
              }
            }
            if (cl[nline] == 0)
            {
              if (horizontal_diff(dp2 + bottomdp, dpitch) > horizontal_diff(sp2 + bottomsp, spitch) + pthreshold)
              {
                ++to_restore;
                cl[nline] = MOTION_FLAG3;
              }
            }
          }
          ++cl;
          dp2 += rightbldp;
          sp2 += rightblsp;
        } while (--j);
        cl++;
        dp2 += dinc;
        sp2 += sinc;
      } while (--i);
      restored_blocks += to_restore;
    } while (to_restore != 0);
  }

#define Cleftdp     (-1)
#define Crightdp    3
#define Cleftsp     Cleftdp
#define Crightsp    Crightdp
#define Crightbldp  4
#define Crightblsp  Crightbldp
#define Ctopdp      (-dpitchUV)
#define Ctopsp      (-spitchUV)


  void  postprocessing(BYTE *dp, int dpitch, BYTE *dpU, BYTE *dpV, int dpitchUV, const BYTE *sp, int spitch, const BYTE *spU, const BYTE *spV, int spitchUV)
  {
    int bottomdp = 7 * dpitch;
    int bottomsp = 7 * spitch;
    int Cbottomdp = chromaheightm * dpitchUV;
    int Cbottomsp = chromaheightm * spitchUV;
    int dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth;
    int sinc = MOTIONBLOCKHEIGHT * spitch - linewidth;
    int dincUV = chromaheight * dpitchUV - linewidthUV;
    int sincUV = chromaheight * spitchUV - linewidthUV;

    loops = restored_blocks = 0;

    //debug_printf("%u\n", vertical_diff(dp + 5000, dpitch)); // to see some values

    int to_restore;

    do
    {
      BYTE      *dp2 = dp;
      BYTE      *dpU2 = dpU;
      BYTE      *dpV2 = dpV;
      const BYTE    *sp2 = sp;
      const BYTE    *spU2 = spU;
      const BYTE    *spV2 = spV;
      unsigned char *cl = blockproperties;
      ++loops;
      to_restore = 0;

      int i = vblocks;
      do
      {
        int j = hblocks;
        do
        {
          if ((cl[0] & TO_CLEAN) != 0)
          {
            copy8x8(dp2, dpitch, sp2, spitch);
            copy_chroma(dpU2, dpV2, dpitchUV, spU2, spV2, spitchUV);
            cl[0] &= ~TO_CLEAN;

            if (cl[-1] == 0)
            {
#ifdef  TEST_VERTICAL_DIFF_CHROMA
              if (vertical_diff_chroma(dpU2 + Cleftdp, dpV2 + Cleftdp, dpitchUV) != test_vertical_diff_chroma(dpU2 + Cleftdp, dpV2 + Cleftdp, dpitchUV)) debug_printf("vertical_diff_chroma incorrect\n");
#endif
              if ((vertical_diff(dp2 + leftdp, dpitch) > vertical_diff(sp2 + leftsp, spitch) + pthreshold)
                || (vertical_diff_chroma(dpU2 + Cleftdp, dpV2 + Cleftdp, dpitchUV) > vertical_diff_chroma(spU2 + Cleftsp, spV2 + Cleftsp, spitchUV) + cthreshold))
              {
                ++to_restore;
                cl[-1] = MOTION_FLAG3;
              }
            }
            if (cl[1] == 0)
            {
              if ((vertical_diff(dp2 + rightdp, dpitch) > vertical_diff(sp2 + rightsp, spitch) + pthreshold)
                || (vertical_diff_chroma(dpU2 + Crightdp, dpV2 + Crightdp, dpitchUV) > vertical_diff_chroma(spU2 + Crightsp, spV2 + Crightsp, spitchUV) + cthreshold))
              {
                ++to_restore;
                cl[1] = MOTION_FLAG3;
              }
            }
            if (cl[pline] == 0)
            {
              if ((horizontal_diff(dp2 + topdp, dpitch) > horizontal_diff(sp2 + topsp, spitch) + pthreshold)
                || (horizontal_diff_chroma(dpU2 + Ctopdp, dpV2 + Ctopdp, dpitchUV) > horizontal_diff_chroma(spU2 + Ctopsp, spV2 + Ctopsp, spitchUV) + cthreshold))
              {
                ++to_restore;
                cl[pline] = MOTION_FLAG3;
              }
            }
            if (cl[nline] == 0)
            {
              if ((horizontal_diff(dp2 + bottomdp, dpitch) > horizontal_diff(sp2 + bottomsp, spitch) + pthreshold)
                || (horizontal_diff_chroma(dpU2 + Cbottomdp, dpV2 + Cbottomdp, dpitchUV) > horizontal_diff_chroma(spU2 + Cbottomsp, spV2 + Cbottomsp, spitchUV) + cthreshold))
              {
                ++to_restore;
                cl[nline] = MOTION_FLAG3;
              }
            }
          }
          ++cl;
          dp2 += rightbldp;
          sp2 += rightblsp;
          dpU2 += Crightbldp;
          spU2 += Crightbldp;
          dpV2 += Crightbldp;
          spV2 += Crightbldp;
        } while (--j);
        cl++;
        dp2 += dinc;
        sp2 += sinc;
        dpU2 += dincUV;
        spU2 += sincUV;
        dpV2 += dincUV;
        spV2 += sincUV;
      } while (--i);
      restored_blocks += to_restore;
    } while (to_restore != 0);
  }

  void  show_motion(BYTE *u, BYTE *v, int pitchUV)
  {
    int inc = chromaheight * pitchUV - linewidthUV;

    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        if (properties[0])
        {
          unsigned u_color = u_ncolor;
          unsigned v_color = v_ncolor;
          if ((properties[0] & MOTION_FLAG) != 0)
          {
            u_color = u_mcolor; v_color = v_mcolor;
          }
          if ((properties[0] & MOTION_FLAGP) != 0)
          {
            u_color = u_pcolor; v_color = v_pcolor;
          }
          colorise(u, v, pitchUV, chromaheight, u_color, v_color);
        }
        u += MOTIONBLOCKWIDTH / 2;
        v += MOTIONBLOCKWIDTH / 2;
        ++properties;
      } while (--i);
      u += inc;
      v += inc;
      ++properties;
    } while (--j);
  }

  Postprocessing(int width, int height, int dist, int tolerance, int dmode, unsigned threshold, int noise, int noisy, bool yuy2, int _pthreshold, int _cthreshold)
    : MotionDetectionDist(width, height, dist, tolerance, dmode, threshold, noise, noisy)
    , pthreshold(_pthreshold), cthreshold(_cthreshold)
  {
#ifdef  TEST_VERTICAL_DIFF_CHROMA
    test_vertical_diff_chroma = test_vertical_diff_yv12_chroma;
#endif
    vertical_diff_chroma = vertical_diff_yv12_chroma;
    copy_chroma = copy_yv12_chroma;
    linewidthUV = linewidth / 2;
    chromaheight = MOTIONBLOCKHEIGHT / 2;
    if (yuy2)
    {
      chromaheight *= 2;
      vertical_diff_chroma = vertical_diff_yuy2_chroma;
      copy_chroma = copy_yuy2_chroma;
#ifdef  TEST_VERTICAL_DIFF_CHROMA
      test_vertical_diff_chroma = test_vertical_diff_yuy2_chroma;
#endif
    }
    chromaheightm = chromaheight - 1;
  }
};

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

class RemoveDirt : public Postprocessing, public AccessFrame
{
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args);
  bool  show;
  int       blocks;
  bool grey;
public:

  int   ProcessFrame(PVideoFrame &dest, PVideoFrame &src, PVideoFrame &previous, PVideoFrame &next, int frame);


  RemoveDirt(int _width, int _height, int dist, int tolerance, int dmode, unsigned threshold, int noise, int noisy, bool yuy2, int pthreshold, int cthreshold, bool _grey, bool _show, bool debug)
    : Postprocessing(_width, _height, dist, tolerance, dmode, threshold, noise, noisy, yuy2, pthreshold, cthreshold)
    , AccessFrame(_width, yuy2), grey(_grey), show(_show)
  {
    blocks = debug ? (hblocks * vblocks) : 0;
  }
};

int RemoveDirt::ProcessFrame(PVideoFrame &dest, PVideoFrame &src, PVideoFrame &previous, PVideoFrame &next, int frame)
{
  const BYTE *nextY = GetReadPtrY(next);
  int   nextPitchY = GetPitchY(next);
  markblocks(GetReadPtrY(previous), GetPitchY(previous), nextY, nextPitchY);

  BYTE *destY = GetWritePtrY(dest);
  BYTE *destU = GetWritePtrU(dest);
  BYTE *destV = GetWritePtrV(dest);
  int   destPitchY = GetPitchY(dest);
  int   destPitchUV = GetPitchUV(dest);
  const BYTE *srcY = GetReadPtrY(src);
  const BYTE *srcU = GetReadPtrU(src);
  const BYTE *srcV = GetReadPtrV(src);
  int   srcPitchY = GetPitchY(src);
  int   srcPitchUV = GetPitchUV(src);

  if (grey) postprocessing_grey(destY, destPitchY, srcY, srcPitchY);
  else postprocessing(destY, destPitchY, destU, destV, destPitchUV, srcY, srcPitchY, srcU, srcV, srcPitchUV);

  if (show) show_motion(destU, destV, destPitchUV);

  __asm emms

  if (blocks) debug_printf("[%u] RemoveDirt: motion blocks = %4u(%2u%%), %4i(%2i%%), %4u(%2u%%), loops = %u\n", frame, motionblocks, (motionblocks * 100) / blocks
    , distblocks, (distblocks * 100) / (int)blocks, restored_blocks, (restored_blocks * 100) / blocks, loops);

  return restored_blocks + distblocks + motionblocks;
}

#define COMPARE_MASK    (~24)

static  void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname)
{
  if ((vi1.width != vi2.width) || (vi1.height != vi2.height) || ((vi1.pixel_type & COMPARE_MASK) != (vi2.pixel_type & COMPARE_MASK)))
  {
#if 1
    debug_printf("widths = %u, %u, heights = %u, %u, color spaces = %X, %X\n"
      , vi1.width, vi2.width, vi1.height, vi2.height, vi1.pixel_type, vi2.pixel_type);
#endif
    AVSenvironment->ThrowError("%s: clips must be of equal type", progname);
  }
  if (vi1.num_frames > vi2.num_frames) vi1.num_frames = vi2.num_frames;
}

class   RestoreMotionBlocks : public GenericVideoFilter
{
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args);
protected:
#ifdef  RANGEFILES
  RemoveDirt *rd[FHANDLERS + 1];
  unsigned char *select;
  int       mthreshold[FHANDLERS + 1];
#define SELECT(n)   rd[select[n]]
#define MTHRESHOLD(n)   mthreshold[select[n]]
#else
  RemoveDirt *rd;
  int   mthreshold;
#define MTHRESHOLD(n)   mthreshold
#define SELECT(n)   rd  

#endif
  PClip restore;
  PClip before;
  PClip after;
  PClip alternative;
  int       lastframe;
  int       before_offset, after_offset;

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if ((n + before_offset < 0) || (n + after_offset > lastframe)) return alternative->GetFrame(n, env);
    PVideoFrame pf = before->GetFrame(n + before_offset, env);
    PVideoFrame df = child->GetFrame(n, env);
    PVideoFrame rf = restore->GetFrame(n, env);
    PVideoFrame nf = after->GetFrame(n + after_offset, env);
    env->MakeWritable(&df);
    if (SELECT(n)->ProcessFrame(df, rf, pf, nf, n) > MTHRESHOLD(n))
      return alternative->GetFrame(n, env);
    else return df;
  }

public:
  RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative);

  ~RestoreMotionBlocks()
  {
#ifdef  RANGEFILES
    int i = FHANDLERS;
    do
    {
      if (rd[i] != NULL) delete rd[i];
    } while (--i >= 0);

    delete[] select;
#else
    delete rd;
#endif
  }
};

RestoreMotionBlocks::RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative)
  : GenericVideoFilter(filtered), restore(_restore), after(neighbour), before(neighbour2), alternative(_alternative)
{
#ifdef  RANGEFILES
  select = new unsigned char[vi.num_frames];
#endif
  child->SetCacheHints(CACHE_NOTHING, 0);
  restore->SetCacheHints(CACHE_RANGE, 0);
  lastframe = vi.num_frames - 1;
  before_offset = after_offset = 0;
  if (after == NULL)
  {
    after = restore;
    goto set_before;
  }
  if (before != NULL)
  {
    after->SetCacheHints(CACHE_RANGE, 0);
    before->SetCacheHints(CACHE_RANGE, 0);
  }
  else
  {
  set_before:
    before_offset = -1;
    after_offset = 1;
    before = after;
    after->SetCacheHints(CACHE_RANGE, 2);
  }
  if (alternative == NULL) alternative = restore;
  else alternative->SetCacheHints(CACHE_RANGE, 0);
  CompareVideoInfo(vi, restore->GetVideoInfo(), "RemoveDirt");
  CompareVideoInfo(vi, before->GetVideoInfo(), "RemoveDirt");
  CompareVideoInfo(vi, after->GetVideoInfo(), "RemoveDirt");
}


#ifdef  RANGEFILES

#else   // RANGEFILES


const   char    *creatstr = "cc[neighbour]c[neighbour2]c[alternative]c[planar]b[show]b[debug]b[gmthreshold]i[mthreshold]i[noise]i[noisy]i[dist]i[tolerance]i[dmode]i[pthreshold]i[cthreshold]i[grey]b";

enum    creatargs { SRC, RESTORE, AFTER, BEFORE, ALTERNATIVE, PLANAR, SHOW, DEBUG, GMTHRES, MTHRES, NOISE, NOISY, DIST, TOLERANCE, DMODE, PTHRES, CTHRES, GREY, REDUCEF };

AVSValue    InitRemoveDirt(RestoreMotionBlocks *filter, AVSValue args)
{
  VideoInfo &vi = filter->vi;

  if (vi.IsRGB() || (vi.IsYV12() + args[PLANAR].AsBool(false) == 0))
    AVSenvironment->ThrowError("RemoveDirt: only YV12 and planar YUY2 clips are supported");

  int   pthreshold = args[PTHRES].AsInt(DEFAULT_PTHRESHOLD);


  filter->rd = new RemoveDirt(vi.width, vi.height, args[DIST].AsInt(DEFAULT_DIST), args[TOLERANCE].AsInt(DEFAULT_TOLERANCE), args[DMODE].AsInt(0)
    , args[MTHRES].AsInt(DEFAULT_MTHRESHOLD), args[NOISE].AsInt(0), args[NOISY].AsInt(-1), vi.IsYUY2()
    , pthreshold, args[CTHRES].AsInt(pthreshold)
    , args[GREY].AsBool(false), args[SHOW].AsBool(false), args[DEBUG].AsBool(false));


  filter->mthreshold = (args[GMTHRES].AsInt(DEFAULT_GMTHRESHOLD) * filter->rd->hblocks * filter->rd->vblocks) / 100;
  return filter;
}

AVSValue __cdecl CreateRestoreMotionBlocks(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  if (!args[RESTORE].Defined()) env->ThrowError("RestoreMotionBlocks: a restore clip must be specified");
  return InitRemoveDirt(new RestoreMotionBlocks(args[SRC].AsClip(), args[RESTORE].AsClip()
    , args[AFTER].Defined() ? args[AFTER].AsClip() : NULL, args[BEFORE].Defined() ? args[BEFORE].AsClip() : NULL
    , args[ALTERNATIVE].Defined() ? args[ALTERNATIVE].AsClip() : NULL), args);

}

#if     ISSE > 1
#define SSE_INCREMENT   16
#define SSE_MOVE        movdqu
#if     ISSE > 2
#define SSE3_MOVE       lddqu
#else
#define SSE3_MOVE       movdqu
#endif
#define SSE_RMOVE       movdqa
#define SSE0            xmm0
#define SSE1            xmm1
#define SSE2            xmm2
#define SSE3            xmm3
#define SSE4            xmm4
#define SSE5            xmm5
#define SSE6            xmm6
#define SSE7            xmm7
#define SSE_EMMS    
#else
#define SSE_INCREMENT   8
#define SSE_MOVE        movq
#define SSE3_MOVE       movq
#define SSE_RMOVE       movq
#define SSE0            mm0
#define SSE1            mm1
#define SSE2            mm2
#define SSE3            mm3
#define SSE4            mm4
#define SSE5            mm5
#define SSE6            mm6
#define SSE7            mm7
#define SSE_EMMS        __asm   emms
#endif  // ISSE

#if     ISSE > 1
static  inline unsigned aligned_diff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
#else
static  unsigned gdiff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
#endif
{
  __asm pxor        SSE0, SSE0
  __asm mov         eax, incpitch
  __asm mov         ebx, spitch2
  __asm mov         esi, sp1
  __asm add         ebx, eax
  __asm mov         edi, sp2
  __asm add         eax, spitch1
  __asm pxor        SSE1, SSE1
  __asm mov         edx, height
  __asm mov         ecx, hblocks
  __asm align       16
  __asm _loop:
  __asm SSE_RMOVE   SSE2, [esi]
    __asm   SSE_RMOVE   SSE3, [esi + SSE_INCREMENT]
    __asm   psadbw      SSE2, [edi]
    __asm   add         esi, 2 * SSE_INCREMENT
  __asm psadbw      SSE3, [edi + SSE_INCREMENT]
    __asm   paddd       SSE0, SSE2
  __asm add         edi, 2 * SSE_INCREMENT
  __asm paddd       SSE1, SSE3
  __asm loop        _loop
  __asm add         esi, eax
  __asm add         edi, ebx
  __asm dec         edx
  __asm mov         ecx, hblocks
  __asm jnz         _loop
  __asm paddd       SSE0, SSE1
  __asm movd        eax, SSE0
}

#if     ISSE > 1
static  inline unsigned unaligned_diff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
{
  __asm pxor        SSE0, SSE0
  __asm mov         eax, incpitch
  __asm mov         ebx, spitch2
  __asm mov         esi, sp1
  __asm add         ebx, eax
  __asm mov         edi, sp2
  __asm add         eax, spitch1
  __asm pxor        SSE1, SSE1
  __asm mov         edx, height
  __asm mov         ecx, hblocks
  __asm align       16
  __asm _loop:
  __asm SSE3_MOVE   SSE2, [esi]
    __asm   SSE3_MOVE   SSE3, [esi + SSE_INCREMENT]
    __asm   add         esi, 2 * SSE_INCREMENT
  __asm SSE3_MOVE   SSE4, [edi]
    __asm   SSE3_MOVE   SSE5, [edi + SSE_INCREMENT]
    __asm   psadbw      SSE2, SSE4
  __asm add         edi, 2 * SSE_INCREMENT
  __asm psadbw      SSE3, SSE5
  __asm paddd       SSE0, SSE2
  __asm paddd       SSE1, SSE3
  __asm loop        _loop
  __asm add         esi, eax
  __asm add         edi, ebx
  __asm dec         edx
  __asm mov         ecx, hblocks
  __asm jnz         _loop
  __asm paddd       SSE0, SSE1
  __asm movd        eax, SSE0
}

static unsigned gdiff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
{
  if ((((unsigned)sp1 & (SSE_INCREMENT - 1)) + ((unsigned)sp2 & (SSE_INCREMENT - 1))) == 0)
    aligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch, height);
  else unaligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch, height);

}
#endif // ISSE > 1

#define SPOINTER(p) p.operator->()

class   SCSelect : public GenericVideoFilter, public AccessFrame
{
  PClip scene_begin;
  PClip scene_end;
  PClip global_motion;
  int hblocks, incpitch;
  unsigned lastdiff;
  unsigned lnr;
  bool debug;
  double dirmult;

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    IClip *selected;
    const char *debugmsg;
    if (n == 0)
    {
    set_begin:
      debugmsg = "[%u] SCSelect: scene begin\n";
      selected = SPOINTER(scene_begin);
    }
    else if (n >= vi.num_frames)
    {
    set_end:
      debugmsg = "[%u] SCSelect: scene end\n";
      selected = SPOINTER(scene_end);
    }
    else
    {
      PVideoFrame sf = child->GetFrame(n, env);
      if (lnr != n - 1)
      {
        PVideoFrame pf = child->GetFrame(n - 1, env);
        lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(pf), GetPitchY(pf), hblocks, incpitch, vi.height);
      }
      int olddiff = lastdiff;
      {
        PVideoFrame nf = child->GetFrame(n + 1, env);
        lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(nf), GetPitchY(nf), hblocks, incpitch, vi.height);
        lnr = n;
      }
      SSE_EMMS
        if (dirmult * olddiff < lastdiff) goto set_end;
      if (dirmult * lastdiff < olddiff) goto set_begin;
      debugmsg = "[%u] SCSelect: global motion\n";
      selected = SPOINTER(global_motion);
    }
    if (debug) debug_printf(debugmsg, n);
    return selected->GetFrame(n, env);
  }
public:
  SCSelect(PClip clip, PClip _scene_begin, PClip _scene_end, PClip _global_motion, double dfactor, bool _debug, bool planar, int cache, int gcache)
    : GenericVideoFilter(clip), AccessFrame(vi.width, vi.IsYUY2()), scene_begin(_scene_begin), scene_end(_scene_end), global_motion(_global_motion), dirmult(dfactor), debug(_debug), lnr(-2)
  {
    if (vi.IsYV12() + planar == 0) AVSenvironment->ThrowError("SCSelect: only YV12 and planar YUY2 clips are supported");
    CompareVideoInfo(vi, scene_begin->GetVideoInfo(), "SCSelect");
    CompareVideoInfo(vi, scene_end->GetVideoInfo(), "SCSelect");
    CompareVideoInfo(vi, global_motion->GetVideoInfo(), "SCSelect");
    hblocks = vi.width / (2 * SSE_INCREMENT);
    incpitch = hblocks * (-2 * SSE_INCREMENT);
    scene_begin->SetCacheHints(CACHE_RANGE, 0);
    scene_end->SetCacheHints(CACHE_RANGE, 0);
    if (gcache >= 0) global_motion->SetCacheHints(CACHE_RANGE, 0);
    if (cache >= 0) child->SetCacheHints(CACHE_RANGE, cache);
  }
};

AVSValue __cdecl CreateSCSelect(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  enum ARGS { CLIP, SBEGIN, SEND, GMOTION, DFACTOR, DEBUG, PLANAR, CACHE, GCACHE };
  return new SCSelect(args[CLIP].AsClip(), args[SBEGIN].AsClip(), args[SEND].AsClip(), args[GMOTION].AsClip(), args[DFACTOR].AsFloat(4.0)
    , args[DEBUG].AsBool(false), args[PLANAR].AsBool(false), args[CACHE].AsInt(2), args[GCACHE].AsInt(0));
};

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit2(IScriptEnvironment* env)
{
  AVSenvironment = env;
#ifdef  DEBUG_NAME
  env->AddFunction("DSCSelect", "cccc[dfactor]f[debug]b[planar]b[cache]i[gcache]i", CreateSCSelect, 0);
  env->AddFunction("DRestoreMotionBlocks", creatstr, CreateRestoreMotionBlocks, 0);
#else
  env->AddFunction("SCSelect", "cccc[dfactor]f[debug]b[planar]b[cache]i[gcache]i", CreateSCSelect, 0);
  env->AddFunction("RestoreMotionBlocks", creatstr, CreateRestoreMotionBlocks, 0);
#endif
  debug_printf(LOGO);
  return NULL;
}
#endif  // RANGEFILES