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

#include "emmintrin.h"
#include <cstdint>

//
// Part 1: options at compile time
//

//#define DEBUG_NAME
//#define RANGEFILES 

#define FHANDLERS 9
//#define STATISTICS // only for internal use
//#define TEST_BLOCKCOMPARE
//#define TEST_VERTICAL_DIFF
//#define TEST_VERTICAL_DIFF_CHROMA

// PF: This USE_DOUBLE_H_SIZED_BLOCKS define triggers old *"SSE2" mode with 2x8 pixel wide code paths instead of 8x8 pixels
// not visibly faster but makes the code difficult.
// *Code is using SSE2 for 8 pixel modes as well, no MMX any more
// #define USE_DOUBLE_H_SIZED_BLOCKS

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

#define SSESIZE     16

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

/****************************************************
* C functions
****************************************************/

__forceinline unsigned int SADABS(int x) { return (x < 0) ? -x : x; }

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
static unsigned int Sad_C(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch)
{
  unsigned int sum = 0;
  for (int y = 0; y < nBlkHeight; y++)
  {
    for (int x = 0; x < nBlkWidth; x++)
      sum += SADABS(reinterpret_cast<const pixel_t *>(pSrc)[x] - reinterpret_cast<const pixel_t *>(pRef)[x]);
    pSrc += nSrcPitch;
    pRef += nRefPitch;
  }
  return sum;
}

// FIXME: to be removed - mimics a hand-optimized MMX code but present compiler likes basic siple SAD algorithm better - we'll use Sad_C instead
template<int blksizeX, int blksizeY>
unsigned __stdcall test_SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/)
{
  pitch1 -= blksizeX;
  pitch2 -= blksizeX;
  int   res = 0;
  int   i = blksizeY;
  do
  {
    int j = blksizeX;
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

template<int blksizeX, int blksizeY>
unsigned __stdcall test_NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  pitch1 -= blksizeX;
  pitch2 -= blksizeX;
  int   res = 0;
  int   i = blksizeY;
  do
  {
    int j = blksizeX;
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

template<int blksizeX, int blksizeY>
unsigned __stdcall test_ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  pitch1 -= blksizeX;
  pitch2 -= blksizeX;
  int   count = 0;
  int   i = blksizeY;
  do
  {
    int j = blksizeX;
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

/****************************************************
* End of C functions
****************************************************/

/****************************************************
* SIMD functions
****************************************************/

unsigned __stdcall SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/) {
  // optimizer makes it fast for SIMD
  return Sad_C<8, 8, uint8_t>(p1, pitch1, p2, pitch2);
}

#if 0
// old v0.9
unsigned __stdcall SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/)
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
#ifndef _M_X64 
  _mm_empty();
#endif 
}
#endif

static __forceinline __m128i _mm_loadh_epi64(__m128i x, __m128i *p)
{
  return _mm_castpd_si128(_mm_loadh_pd(_mm_castsi128_pd(x), (double *)p));
}

unsigned int __stdcall NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  __m128i noise_vector = _mm_set1_epi8(noise);
  auto zero = _mm_setzero_si128();

  // first 4 lines
  __m128i src1a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 0 * pitch1)), (__m128i *)(p1 + 2 * pitch1));
  __m128i src1b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 1 * pitch1)), (__m128i *)(p1 + 3 * pitch1));

  __m128i src2a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 0 * pitch2)), (__m128i *)(p2 + 2 * pitch2));
  __m128i src2b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 1 * pitch2)), (__m128i *)(p2 + 3 * pitch2));

  __m128i diff_12a = _mm_subs_epu8(src1a, src2a);
  __m128i diff_12b = _mm_subs_epu8(src1b, src2b);
  __m128i diff_21a = _mm_subs_epu8(src2a, src1a);
  __m128i diff_21b = _mm_subs_epu8(src2b, src1b);

  diff_12a = _mm_subs_epu8(diff_12a, noise_vector);
  diff_12b = _mm_subs_epu8(diff_12b, noise_vector);
  diff_21a = _mm_subs_epu8(diff_21a, noise_vector);
  diff_21b = _mm_subs_epu8(diff_21b, noise_vector);

  auto absdiff_a = _mm_sad_epu8(diff_12a, diff_21a);
  auto absdiff_b = _mm_sad_epu8(diff_12b, diff_21b);
  auto absdiff_first4 = _mm_add_epi32(absdiff_a, absdiff_b);

  // next 4 lines
  p1 += pitch1 * 4;
  p2 += pitch2 * 4;

  src1a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 0 * pitch1)), (__m128i *)(p1 + 2 * pitch1));
  src1b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 1 * pitch1)), (__m128i *)(p1 + 3 * pitch1));

  src2a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 0 * pitch2)), (__m128i *)(p2 + 2 * pitch2));
  src2b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 1 * pitch2)), (__m128i *)(p2 + 3 * pitch2));

  diff_12a = _mm_subs_epu8(src1a, src2a);
  diff_12b = _mm_subs_epu8(src1b, src2b);
  diff_21a = _mm_subs_epu8(src2a, src1a);
  diff_21b = _mm_subs_epu8(src2b, src1b);

  diff_12a = _mm_subs_epu8(diff_12a, noise_vector);
  diff_12b = _mm_subs_epu8(diff_12b, noise_vector);
  diff_21a = _mm_subs_epu8(diff_21a, noise_vector);
  diff_21b = _mm_subs_epu8(diff_21b, noise_vector);

  absdiff_a = _mm_sad_epu8(diff_12a, diff_21a);
  absdiff_b = _mm_sad_epu8(diff_12b, diff_21b);
  auto absdiff_second4 = _mm_add_epi32(absdiff_a, absdiff_b);

  // sum up counters
  auto counts = _mm_add_epi32(absdiff_first4, absdiff_second4); // int32 sums result lo 64, hi 64
  __m128i counts_hi = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(counts), _mm_castsi128_ps(counts)));
  auto count_final = _mm_add_epi32(counts, counts_hi);
  return (uint32_t)_mm_cvtsi128_si32(count_final);
}

#if 0
// VS version, have some bugs, and finally not used
// static __forceinline uint32_t NSADcompare(const uint8_t *p1, int32_t pitch1, const uint8_t *p2, int32_t pitch2, const uint8_t *noiselevel)
unsigned int __stdcall NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  __m128i xmm7 = _mm_set1_epi8(noise);

  int32_t pitch1x2 = pitch1 + pitch1;
  int32_t pitch1x3 = pitch1x2 + pitch1;
  int32_t pitch1x4 = pitch1x3 + pitch1;
  int32_t pitch2x2 = pitch2 + pitch2;
  int32_t pitch2x3 = pitch2x2 + pitch2;
  int32_t pitch2x4 = pitch2x3 + pitch2;

  __m128i xmm0;
  xmm0 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm0), (__m64 *)(p1)), (__m64 *)(p1 + pitch1x2)));

  __m128i xmm2;
  xmm2 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm2), (__m64 *)(p1 + pitch1)), (__m64 *)(p1 + pitch1x3)));

  __m128i xmm3 = xmm0;
  __m128i xmm4 = xmm2;

  __m128i xmm5;
  xmm5 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm5), (__m64 *)(p2)), (__m64 *)(p2 + pitch2x2)));

  __m128i xmm6;
  xmm6 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm6), (__m64 *)(p2 + pitch2)), (__m64 *)(p2 + pitch2x3)));

  xmm0 = _mm_subs_epu8(xmm0, xmm5);
  xmm2 = _mm_subs_epu8(xmm2, xmm6);
  xmm5 = _mm_subs_epu8(xmm5, xmm3);
  xmm6 = _mm_subs_epu8(xmm6, xmm4);
  xmm0 = _mm_subs_epu8(xmm0, xmm7);
  xmm2 = _mm_subs_epu8(xmm2, xmm7);
  xmm5 = _mm_subs_epu8(xmm5, xmm7);
  xmm6 = _mm_subs_epu8(xmm6, xmm7);

  xmm0 = _mm_sad_epu8(xmm0, xmm5);
  xmm6 = _mm_sad_epu8(xmm6, xmm2); // FIXME by PF: bug in VS version was: xmm0 = _mm_sad_epu8(xmm6, xmm2); 

  p1 += pitch1x4;
  p2 += pitch2x4;

  __m128i xmm1;
  xmm1 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm1), (__m64 *)(p1)), (__m64 *)(p1 + pitch1x2)));

  xmm0 = _mm_add_epi32(xmm0, xmm6);

  xmm2 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm2), (__m64 *)(p1 + pitch1)), (__m64 *)(p1 + pitch1x3)));

  xmm3 = xmm1;
  xmm4 = xmm2;

  xmm5 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm5), (__m64 *)(p2)), (__m64 *)(p2 + pitch2x2)));
  xmm6 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm6), (__m64 *)(p2 + pitch2)), (__m64 *)(p2 + pitch2x3)));

  xmm1 = _mm_subs_epu8(xmm1, xmm5);
  xmm2 = _mm_subs_epu8(xmm2, xmm6);
  xmm5 = _mm_subs_epu8(xmm5, xmm3);
  xmm6 = _mm_subs_epu8(xmm6, xmm4);
  xmm1 = _mm_subs_epu8(xmm1, xmm7);
  xmm2 = _mm_subs_epu8(xmm2, xmm7);
  xmm5 = _mm_subs_epu8(xmm5, xmm7);
  xmm6 = _mm_subs_epu8(xmm6, xmm7);

  xmm1 = _mm_sad_epu8(xmm1, xmm5);  // FIXME by PF: bug in VS version. xmm0 holds previous sum and should be reserved. Was: xmm0 = _mm_sad_epu8(xmm1, xmm5);
  xmm6 = _mm_sad_epu8(xmm6, xmm2);  // FIXME by PF: bug in VS version was: xmm0 = _mm_sad_epu8(xmm6, xmm2); 

  xmm0 = _mm_add_epi32(xmm0, xmm1);
  xmm0 = _mm_add_epi32(xmm0, xmm6);

  xmm1 = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(xmm1), _mm_castsi128_ps(xmm0)));

  xmm0 = _mm_add_epi32(xmm0, xmm1);

  return (uint32_t)_mm_cvtsi128_si32(xmm0);
}
#endif

#if 0
// old v0.9
unsigned __stdcall NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  // __m128i xmm7 = _mm_set1_epi8(noise);
  __asm pxor xmm1, xmm1
  __asm movd xmm7, noise
  __asm pshufb xmm7, xmm1

#ifdef  STATISTICS
  ++compare8;
#endif
  __asm mov         edx, pitch1
  __asm mov         esi, pitch2
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   lea         edi, [esi + 2 * esi]
    __asm   mov         ebx, p2

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
}
#endif

unsigned int __stdcall ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  __m128i noise_vector = _mm_set1_epi8(noise);
  auto zero = _mm_setzero_si128();

  // first 4 lines
  __m128i src1a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 0 * pitch1)), (__m128i *)(p1 + 2 * pitch1));
  __m128i src1b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 1 * pitch1)), (__m128i *)(p1 + 3 * pitch1));

  __m128i src2a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 0 * pitch2)), (__m128i *)(p2 + 2 * pitch2));
  __m128i src2b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 1 * pitch2)), (__m128i *)(p2 + 3 * pitch2));

  __m128i diff_12a = _mm_subs_epu8(src1a, src2a);
  __m128i diff_12b = _mm_subs_epu8(src1b, src2b);
  __m128i diff_21a = _mm_subs_epu8(src2a, src1a);
  __m128i diff_21b = _mm_subs_epu8(src2b, src1b);

  diff_12a = _mm_subs_epu8(diff_12a, noise_vector);
  diff_12b = _mm_subs_epu8(diff_12b, noise_vector);
  diff_21a = _mm_subs_epu8(diff_21a, noise_vector);
  diff_21b = _mm_subs_epu8(diff_21b, noise_vector);

  auto in_range_a = _mm_cmpeq_epi8(diff_12a, diff_21a); // sets FF (-1) where they are equal (e.g. zero)
  auto in_range_b = _mm_cmpeq_epi8(diff_12b, diff_21b);
  auto in_range_first4 = _mm_add_epi8(in_range_a, in_range_b);

  // Trick for counting the noise-corrected-pixel-sads within the given range:
  // Each byte holds (0 * -1 or 1 * -1 or 2 * -1, that is count of the pixels (0,1,2) in range at the given position
  // This calculation will be repeated on the next 4 lines -> each position will hold -4,-3,-2,-1 or 0 as a counter
  // Trick2: finally we'll add 4 (correction makes -4 to 0, -3 to 1 ... 0 to 4)
  // Trick3: then apply SAD for horizontal summing of these partial sums

  // next 4 lines
  p1 += pitch1 * 4;
  p2 += pitch2 * 4;

  src1a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 0 * pitch1)), (__m128i *)(p1 + 2 * pitch1));
  src1b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p1 + 1 * pitch1)), (__m128i *)(p1 + 3 * pitch1));

  src2a = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 0 * pitch2)), (__m128i *)(p2 + 2 * pitch2));
  src2b = _mm_loadh_epi64(_mm_loadl_epi64((__m128i *)(p2 + 1 * pitch2)), (__m128i *)(p2 + 3 * pitch2));

  diff_12a = _mm_subs_epu8(src1a, src2a);
  diff_12b = _mm_subs_epu8(src1b, src2b);
  diff_21a = _mm_subs_epu8(src2a, src1a);
  diff_21b = _mm_subs_epu8(src2b, src1b);

  diff_12a = _mm_subs_epu8(diff_12a, noise_vector);
  diff_12b = _mm_subs_epu8(diff_12b, noise_vector);
  diff_21a = _mm_subs_epu8(diff_21a, noise_vector);
  diff_21b = _mm_subs_epu8(diff_21b, noise_vector);

  in_range_a = _mm_cmpeq_epi8(diff_12a, diff_21a); // sets FF (-1) where they are equal (e.g. zero)
  in_range_b = _mm_cmpeq_epi8(diff_12b, diff_21b);
  auto in_range_second4 = _mm_add_epi8(in_range_a, in_range_b);

  auto in_range = _mm_add_epi8(in_range_first4, in_range_second4);
  // applying trick 2 and 3:
  // correction to shift -4..0 to 0..4 at each position
  const __m128i excessadd = _mm_set1_epi8(4);
  in_range = _mm_add_epi8(in_range, excessadd);
  // sum up counters
  auto counts = _mm_sad_epu8(in_range, zero); // int32 sums result lo 64, hi 64
  __m128i counts_hi = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(counts), _mm_castsi128_ps(counts)));
  auto count_final =  _mm_add_epi32(counts, counts_hi);
  return (uint32_t)_mm_cvtsi128_si32(count_final);

}

#if 0
// VS version, have some bugs, finally not used
static const __declspec(align(SSESIZE)) BYTE excessadd[SSESIZE]
= { 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4 };

//uint8_t ALIGNED_ARRAY(excessadd, 16)[16] = { 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4 };

unsigned int __stdcall ExcessPixels_vs(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  __m128i xmm7 = _mm_set1_epi8(noise);

  int32_t pitch1x2 = pitch1 + pitch1;
  int32_t pitch1x3 = pitch1x2 + pitch1;
  int32_t pitch1x4 = pitch1x3 + pitch1;
  int32_t pitch2x2 = pitch2 + pitch2;
  int32_t pitch2x3 = pitch2x2 + pitch2;
  int32_t pitch2x4 = pitch2x3 + pitch2;

  __m128i xmm0;
  xmm0 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm0), (__m64 *)(p1)), (__m64 *)(p1 + pitch1x2)));

  __m128i xmm2;
  xmm2 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm2), (__m64 *)(p1 + pitch1)), (__m64 *)(p1 + pitch1x3)));

  __m128i xmm3 = xmm0;
  __m128i xmm4 = xmm2;

  __m128i xmm5;
  xmm5 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm5), (__m64 *)(p2)), (__m64 *)(p2 + pitch2x2)));

  __m128i xmm6;
  xmm6 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm6), (__m64 *)(p2 + pitch2)), (__m64 *)(p2 + pitch2x3)));

  xmm0 = _mm_subs_epu8(xmm0, xmm5);
  xmm2 = _mm_subs_epu8(xmm2, xmm6);
  xmm5 = _mm_subs_epu8(xmm5, xmm3);
  xmm6 = _mm_subs_epu8(xmm6, xmm4);
  xmm0 = _mm_subs_epu8(xmm0, xmm7);
  xmm2 = _mm_subs_epu8(xmm2, xmm7);
  xmm5 = _mm_subs_epu8(xmm5, xmm7);
  xmm6 = _mm_subs_epu8(xmm6, xmm7);

  xmm0 = _mm_cmpeq_epi8(xmm0, xmm5);
  xmm6 = _mm_cmpeq_epi8(xmm6, xmm2);

  p1 += pitch1x4;
  p2 += pitch2x4;

  __m128i xmm1;
  xmm1 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm1), (__m64 *)(p1)), (__m64 *)(p1 + pitch1x2)));

  xmm0 = _mm_add_epi8(xmm0, xmm6);

  xmm2 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm2), (__m64 *)(p1 + pitch1)), (__m64 *)(p1 + pitch1x3)));

  xmm3 = xmm1;
  xmm4 = xmm2;

  xmm5 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm5), (__m64 *)(p2)), (__m64 *)(p2 + pitch2x2)));

  xmm6 = _mm_castps_si128(_mm_loadh_pi(_mm_loadl_pi(_mm_castsi128_ps(xmm6), (__m64 *)(p2 + pitch2)), (__m64 *)(p2 + pitch2x3)));

  xmm1 = _mm_subs_epu8(xmm1, xmm5);
  xmm2 = _mm_subs_epu8(xmm2, xmm6);
  xmm5 = _mm_subs_epu8(xmm5, xmm3);
  xmm6 = _mm_subs_epu8(xmm6, xmm4);
  xmm1 = _mm_subs_epu8(xmm1, xmm7);
  xmm2 = _mm_subs_epu8(xmm2, xmm7);
  xmm5 = _mm_subs_epu8(xmm5, xmm7);
  xmm6 = _mm_subs_epu8(xmm6, xmm7);

  xmm1 = _mm_cmpeq_epi8(xmm1, xmm5);
  xmm6 = _mm_cmpeq_epi8(xmm6, xmm2);

  //_mm_xor_si128(xmm5, xmm5);  // wrong in VS version, there was no target variable. Fix by PF
  xmm5 = _mm_setzero_si128(); // clearer (PF)

  xmm0 = _mm_add_epi8(xmm0, xmm1);
  xmm6 = _mm_add_epi8(xmm6, *((__m128i*)excessadd));
  xmm0 = _mm_add_epi8(xmm0, xmm6);

  xmm0 = _mm_sad_epu8(xmm0, xmm5);

  xmm1 = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(xmm1), _mm_castsi128_ps(xmm0)));

  xmm0 = _mm_add_epi32(xmm0, xmm1); // wrong in VS version. Fix by PF! was: epi8

  return (uint32_t)_mm_cvtsi128_si32(xmm0);
}
#endif

#if 0
// old from v0.9
unsigned __stdcall ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  // __m128i xmm7 = _mm_set1_epi8(noise);
  __asm pxor xmm1, xmm1
  __asm movd xmm7, noise
  __asm pshufb xmm7, xmm1

#ifdef  STATISTICS
  ++compare16;
#endif
  __asm mov         edx, pitch1
  __asm mov         esi, pitch2
  __asm mov         eax, p1
  __asm lea         ecx, [edx + 2 * edx]
    __asm   lea         edi, [esi + 2 * esi]
    __asm   mov         ebx, p2

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
}
#endif

/****************************************************
* SIMD functions end
****************************************************/

/****************************************************
* Double width (v0.9 terminology: "SSE2") functions
* We'll omit them finally, no visible speed gain, but make life more difficult
****************************************************/
#ifdef USE_DOUBLE_H_SIZED_BLOCKS
__declspec(align(16))
unsigned    blockcompare_result[4]; // must be global otherwise compiler may generate incorrect alignment

void __stdcall SADcompareSSE2(const BYTE *p1, const BYTE *p2, int pitch, int /*noise*/)
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
void __stdcall NSADcompareSSE2(const BYTE *p1, const BYTE *p2, int pitch, int noise)
{
  // __m128i xmm7 = _mm_set1_epi8(noise);
  __asm pxor xmm1, xmm1
  __asm movd xmm7, noise
  __asm pshufb xmm7, xmm1

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

void __stdcall ExcessPixelsSSE2(const BYTE *p1, const BYTE *p2, int pitch, int noise)
{
  // __m128i xmm7 = _mm_set1_epi8(noise);
  __asm pxor xmm1, xmm1
  __asm movd xmm7, noise
  __asm pshufb xmm7, xmm1

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
#endif

/***********************************************************
* End of Double width (v0.9 terminology: "SSE2") functions
***********************************************************/

//
// Part 5: Motion Detection
//

class   MotionDetection
{
  unsigned char *blockproperties_addr;
protected:
  int   pline, nline;
public:
  int noise;
  int   motionblocks;
  unsigned char *blockproperties;
  int   linewidth;
  unsigned(__stdcall *blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);
  int   hblocks, vblocks;
  unsigned threshold;

#ifdef USE_DOUBLE_H_SIZED_BLOCKS
  void(__stdcall *blockcompareSSE2)(const BYTE *p1, const BYTE *p2, int pitch, int noise);
  int   hblocksSSE2;        // = hblocks / 2
  bool remainderSSE2;   // = hblocks & 1
  int   linewidthSSE2;
#endif

  unsigned(__stdcall *test_blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);

  void  markblocks1(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
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
        if ((d = blockcompare(p1, pitch1, p2, pitch2, noise) - test_blockcompare(p1, pitch1, p2, pitch2, noise)) != 0)
          debug_printf("blockcompare test fails with difference = %i\n", d);
#endif
        if (blockcompare(p1, pitch1, p2, pitch2, noise) >= threshold)
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

#ifdef USE_DOUBLE_H_SIZED_BLOCKS
  void  markblocks2(const BYTE *p1, const BYTE *p2, int pitch)
  {
    int inc = MOTIONBLOCKHEIGHT * pitch - linewidthSSE2;
    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocksSSE2;
      do
      {
        blockcompareSSE2(p1, p2, pitch, noise);
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
        if (blockcompare(p1, pitch, p2, pitch, noise) >= threshold)
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
#endif

  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    motionblocks = 0;
#ifdef USE_DOUBLE_H_SIZED_BLOCKS
    if (((pitch1 - pitch2) | (((unsigned)p1) & 15) | (((unsigned)p2) & 15)) == 0)
      markblocks2(p1, p2, pitch1); // pitches are the same and both are mod16
    else
#endif
      markblocks1(p1, pitch1, p2, pitch2); // generic way. Also for different pitches or not mod16
  }

  MotionDetection(int   width, int height, unsigned _threshold, int _noise, int _noisy, IScriptEnvironment* env) : threshold(_threshold), noise(_noise)
  {

    hblocks = (linewidth = width) / MOTIONBLOCKWIDTH;
    vblocks = height / MOTIONBLOCKHEIGHT;

    bool use_SSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;

#ifdef USE_DOUBLE_H_SIZED_BLOCKS
    linewidthSSE2 = linewidth;
    hblocksSSE2 = hblocks / 2;
    if ((remainderSSE2 = (hblocks & 1)) != 0) linewidthSSE2 -= MOTIONBLOCKWIDTH;

    if (use_SSE2 && ((hblocksSSE2 == 0) || (vblocks == 0))) 
      use_SSE2 = false;
#endif

#ifdef USE_DOUBLE_H_SIZED_BLOCKS
    blockcompareSSE2 = SADcompareSSE2;
#endif

    // old non-SSE2 check
    if (!use_SSE2 & ((hblocks == 0) || (vblocks == 0)))
        env->ThrowError("RemoveDirt: width or height of the clip too small");

    blockcompare = use_SSE2 ? SADcompare : test_SADcompare<8, 8>;
    test_blockcompare = test_SADcompare<8, 8>;
    if (noise > 0)
    {
#ifdef USE_DOUBLE_H_SIZED_BLOCKS
      blockcompareSSE2 = NSADcompareSSE2;
#endif
      test_blockcompare = test_NSADcompare<8,8>;
      blockcompare = use_SSE2 ? NSADcompare : test_NSADcompare<8,8>;

      if (_noisy >= 0)
      {
#ifdef USE_DOUBLE_H_SIZED_BLOCKS
        blockcompareSSE2 = ExcessPixelsSSE2;
#endif
        test_blockcompare = test_ExcessPixels<8,8>;
        blockcompare = use_SSE2 ? ExcessPixels : test_ExcessPixels<8,8>;
        threshold = _noisy;
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

  MotionDetectionDist(int   width, int height, int _dist, int _tolerance, int dmode, unsigned _threshold, int _noise, int _noisy, IScriptEnvironment* env)
    : MotionDetection(width, height, _threshold, _noise, _noisy, env)
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
#if 1
  // 8 pixels
  auto src1 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p));
  auto src2 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p + pitch));
  return _mm_cvtsi128_si32(_mm_sad_epu8(src1, src2));
#else
  __asm mov         edx, p
  __asm mov         eax, pitch
  __asm movq        mm0, [edx]
    __asm   psadbw      mm0, [edx + eax]
    __asm   movd        eax, mm0
#ifndef _M_X64 
  _mm_empty();
#endif 
#endif
}

// 4
int __stdcall horizontal_diff_chroma(const BYTE *u, const BYTE *v, int pitch)
{
#ifdef STATISTICS
  st_horizontal_diff_chroma++;
#endif
#if 1
  // interleave u and v as 2x4 pixels then do sad
  auto src1_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u)));
  auto src1_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v)));
  auto src2_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u + pitch)));
  auto src2_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v + pitch)));
  return _mm_cvtsi128_si32(_mm_sad_epu8(_mm_unpacklo_epi32(src1_u, src1_v), _mm_unpacklo_epi32(src2_u, src2_v)));
#else
  __asm mov         edx, u
  __asm mov         eax, pitch
  __asm movd        mm0, [edx]
    __asm   mov         ecx, v
  __asm movd        mm1, [edx + eax]
    __asm   punpckldq   mm0, [ecx]
    __asm   punpckldq   mm1, [ecx + eax]
    __asm   psadbw      mm0, mm1
  __asm movd        eax, mm0
#ifndef _M_X64 
  _mm_empty();
#endif 
#endif
}

static __forceinline int32_t vertical_diff(const uint8_t *p, int32_t pitch/*, const uint8_t *noiselevel*/)
// diff from VS: noiselevel is not needed here!
{
  int32_t pitchx2 = pitch + pitch;
  int32_t pitchx3 = pitchx2 + pitch;
  int32_t pitchx4 = pitchx3 + pitch;
  __m128i xmm7 = _mm_setzero_si128(); // _mm_loadu_si128((__m128i*)noiselevel);

  __m128i xmm0 = _mm_setzero_si128(); // diff from VS: fixme: set it but leave it 'unused'
  __m128i xmm1 = _mm_setzero_si128();
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)p), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + pitchx2)), 1);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + pitchx3)), 1);

  xmm7 = _mm_cmpeq_epi8(xmm7, xmm7); // set it all FF

  p += pitchx4;

  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)p), 2);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + pitchx2)), 3);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + pitch)), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + pitchx3)), 3);

  __m128i xmm2 = xmm0;
  __m128i xmm3 = xmm1;

  xmm7 = _mm_srli_epi16(xmm7, 8);

  xmm0 = _mm_and_si128(xmm0, xmm7);

  xmm1 = _mm_slli_epi16(xmm1, 8);
  xmm2 = _mm_srli_epi16(xmm2, 8);

  xmm3 = _mm_subs_epu8(xmm3, xmm7);

  xmm0 = _mm_or_si128(xmm0, xmm1);
  xmm2 = _mm_or_si128(xmm2, xmm3);

  xmm0 = _mm_sad_epu8(xmm0, xmm2);

  return _mm_cvtsi128_si32(xmm0);
}

#if 0
int __stdcall vertical_diff_old_mmx(const BYTE *p, int pitch)
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
#ifndef _M_X64 
  _mm_empty();
#endif 
}
#endif

template<int blocksizeY>
int __stdcall test_vertical_diff(const BYTE *p, int pitch)
{
  int   res = 0;
  int   i = blocksizeY;
  do
  {
    int diff = p[0] - p[1];
    if (diff < 0) res -= diff;
    else res += diff;
    p += pitch;
  } while (--i);
  return    res;
}

// from VS, minor mod, bugfix
int __stdcall vertical_diff_yv12_chroma(const BYTE *u, const BYTE *v, int pitch)
{
//  __m128i xmm7 = _mm_set1_epi8(noise); // FIXME by PF: no need, not used here!
  int32_t pitchx2 = pitch + pitch;
  int32_t pitchx3 = pitchx2 + pitch;

  __m128i xmm0, xmm1;
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)u), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + pitchx2)), 1);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitchx3)), 1);

  __m128i xmm7 = _mm_cmpeq_epi8(xmm7, xmm7);

  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)v), 2);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + pitchx2)), 3);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)v + pitch), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + pitchx3)), 3);
  __m128i xmm2 = xmm0;
  __m128i xmm3 = xmm1;

  xmm7 = _mm_srli_epi16(xmm7, 8);

  xmm0 = _mm_and_si128(xmm0, xmm7);

  xmm1 = _mm_slli_epi16(xmm1, 8);
  xmm2 = _mm_srli_epi16(xmm2, 8);

  xmm3 = _mm_subs_epu8(xmm3, xmm7);

  xmm0 = _mm_or_si128(xmm0, xmm1);
  xmm2 = _mm_or_si128(xmm2, xmm3);

  xmm0 = _mm_sad_epu8(xmm0, xmm2);

  return (uint32_t)_mm_cvtsi128_si32(xmm0);
}

// from VS, minor mod, bugfix
static int __stdcall vertical_diff_yuy2_chroma(const BYTE *u, const BYTE *v, int pitch)
{
  //  __m128i xmm7 = _mm_set1_epi8(noise); // FIXME by PF: no need, not used here!

  int32_t pitchx2 = pitch + pitch;
  int32_t pitchx3 = pitchx2 + pitch;
  int32_t pitchx4 = pitchx3 + pitch;

  __m128i xmm0, xmm1;
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)u), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + pitchx2)), 1);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitchx3)), 1);

  u += pitchx4;

  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)u), 2);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + pitchx2)), 3);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitch)), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + pitchx3)), 3);
  __m128i xmm2 = xmm0;
  __m128i xmm3 = xmm1;

  __m128i xmm7 = _mm_cmpeq_epi8(xmm7, xmm7); // FIXME by PF: this line was missing, VS is buggy

  xmm0 = _mm_and_si128(xmm0, xmm7);

  xmm1 = _mm_slli_epi16(xmm1, 8);
  xmm2 = _mm_srli_epi16(xmm2, 8);

  xmm3 = _mm_subs_epu8(xmm3, xmm7);

  xmm0 = _mm_or_si128(xmm0, xmm1);
  xmm2 = _mm_or_si128(xmm2, xmm3);

  __m128i xmm4;
  xmm4 = _mm_insert_epi16(xmm4, *((int32_t*)v), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + pitchx2)), 1);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + pitchx3)), 1);

  v += pitchx4;

  xmm4 = _mm_insert_epi16(xmm4, *((int32_t*)v), 2);
  xmm4 = _mm_insert_epi16(xmm4, *((int32_t*)(v + pitchx2)), 3);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + pitch)), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + pitchx3)), 3);

  __m128i xmm6 = xmm4;
  xmm3 = xmm1;

  xmm4 = _mm_and_si128(xmm4, xmm7);

  xmm1 = _mm_slli_epi16(xmm1, 8);
  xmm6 = _mm_srli_epi16(xmm6, 8);

  xmm3 = _mm_subs_epu8(xmm3, xmm7);

  xmm4 = _mm_or_si128(xmm4, xmm1);
  xmm6 = _mm_or_si128(xmm6, xmm3);

  xmm0 = _mm_sad_epu8(xmm0, xmm2);
  xmm4 = _mm_sad_epu8(xmm4, xmm6);

  xmm0 = _mm_avg_epu16(xmm0, xmm4);

  return _mm_cvtsi128_si32(xmm0);
}

#if 0
//old v0.9
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
#ifndef _M_X64 
  _mm_empty();
#endif 
}
#endif

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

#if 0
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
#ifndef _M_X64 
  _mm_empty();
#endif 
}
#endif

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
void Copy_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch)
{
  for (int j = 0; j < nBlkHeight; j++)
  {
    memcpy(pDst, pSrc, nBlkWidth * sizeof(pixel_t));
    pDst += nDstPitch;
    pSrc += nSrcPitch;
  }
}

void __stdcall copy8x8(BYTE *dest, int dpitch, const BYTE *src, int spitch)
{
#if 1
  Copy_C<8, 8, uint8_t>(dest, dpitch, src, spitch);
#else
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
#ifndef _M_X64 
  _mm_empty();
#endif 

#endif
}

void __stdcall copy_yv12_chroma(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
#if 1
  Copy_C<4, 4, uint8_t>(destu, dpitch, srcu, spitch); // FIXME by PF: Don't think, it was intentionally 8x4 instead of 4x4 ???
  Copy_C<4, 4, uint8_t>(destv, dpitch, srcv, spitch);
#else
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
#ifndef _M_X64 
  _mm_empty();
#endif 

#endif
}

void __stdcall copy_yuy2_chroma(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
#if 1
  Copy_C<4, 8, uint8_t>(destu, dpitch, srcu, spitch); // FIXME: Don't think, it was intentionally 8x8 instead of 4x8?
  Copy_C<4, 8, uint8_t>(destv, dpitch, srcv, spitch);
#else
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

#ifndef _M_X64 
  _mm_empty();
#endif 
#endif
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
  int(__stdcall *test_vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);

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
              if (vertical_diff(dp2 + leftdp, dpitch) != test_vertical_diff<8>(dp2 + leftdp, dpitch))
                debug_printf("vertical_diff incorrect\n");
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
              if (vertical_diff_chroma(dpU2 + Cleftdp, dpV2 + Cleftdp, dpitchUV) != test_vertical_diff_chroma(dpU2 + Cleftdp, dpV2 + Cleftdp, dpitchUV))
                debug_printf("vertical_diff_chroma incorrect\n");
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

  Postprocessing(int width, int height, int dist, int tolerance, int dmode, unsigned threshold, int _noise, int _noisy, bool yuy2, int _pthreshold, int _cthreshold, IScriptEnvironment* env)
    : MotionDetectionDist(width, height, dist, tolerance, dmode, threshold, _noise, _noisy, env)
    , pthreshold(_pthreshold), cthreshold(_cthreshold)
  {
    test_vertical_diff_chroma = test_vertical_diff_yv12_chroma;
    vertical_diff_chroma = (env->GetCPUFlags() & CPUF_SSE2) ? vertical_diff_yv12_chroma : test_vertical_diff_yv12_chroma;
    copy_chroma = copy_yv12_chroma;
    linewidthUV = linewidth / 2;
    chromaheight = MOTIONBLOCKHEIGHT / 2;
    if (yuy2)
    {
      chromaheight *= 2;
      vertical_diff_chroma = (env->GetCPUFlags() & CPUF_SSE2) ? vertical_diff_yuy2_chroma : test_vertical_diff_yuy2_chroma;
      copy_chroma = copy_yuy2_chroma;
      test_vertical_diff_chroma = test_vertical_diff_yuy2_chroma;
    }
    chromaheightm = chromaheight - 1;
  }
};

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
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env);
  bool  show;
  int       blocks;
  bool grey;
public:

  int   ProcessFrame(PVideoFrame &dest, PVideoFrame &src, PVideoFrame &previous, PVideoFrame &next, int frame);


  RemoveDirt(int _width, int _height, int dist, int tolerance, int dmode, unsigned threshold, int noise, int noisy, bool yuy2, int pthreshold, int cthreshold, bool _grey, bool _show, bool debug, IScriptEnvironment* env)
    : Postprocessing(_width, _height, dist, tolerance, dmode, threshold, noise, noisy, yuy2, pthreshold, cthreshold, env)
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

  if (blocks) debug_printf("[%u] RemoveDirt: motion blocks = %4u(%2u%%), %4i(%2i%%), %4u(%2u%%), loops = %u\n", frame, motionblocks, (motionblocks * 100) / blocks
    , distblocks, (distblocks * 100) / (int)blocks, restored_blocks, (restored_blocks * 100) / blocks, loops);

  return restored_blocks + distblocks + motionblocks;
}

#define COMPARE_MASK    (~24)

static  void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname, IScriptEnvironment* env)
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

class   RestoreMotionBlocks : public GenericVideoFilter
{
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env);
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
  RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative, IScriptEnvironment* env);

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

RestoreMotionBlocks::RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative, IScriptEnvironment* env)
  : GenericVideoFilter(filtered), restore(_restore), after(neighbour), before(neighbour2), alternative(_alternative)
{
#ifdef  RANGEFILES
  select = new unsigned char[vi.num_frames];
#endif
  child->SetCacheHints(CACHE_NOTHING, 0);
  restore->SetCacheHints(CACHE_GENERIC, 0);
  lastframe = vi.num_frames - 1;
  before_offset = after_offset = 0;
  if (after == NULL)
  {
    after = restore;
    goto set_before;
  }
  if (before != NULL)
  {
    after->SetCacheHints(CACHE_GENERIC, 0);
    before->SetCacheHints(CACHE_GENERIC, 0);
  }
  else
  {
  set_before:
    before_offset = -1;
    after_offset = 1;
    before = after;
    after->SetCacheHints(CACHE_GENERIC, 2);
  }
  if (alternative == NULL) alternative = restore;
  else alternative->SetCacheHints(CACHE_GENERIC, 0);
  CompareVideoInfo(vi, restore->GetVideoInfo(), "RemoveDirt", env);
  CompareVideoInfo(vi, before->GetVideoInfo(), "RemoveDirt", env);
  CompareVideoInfo(vi, after->GetVideoInfo(), "RemoveDirt", env);
}


#ifdef  RANGEFILES

#else   // RANGEFILES


const   char    *creatstr = "cc[neighbour]c[neighbour2]c[alternative]c[planar]b[show]b[debug]b[gmthreshold]i[mthreshold]i[noise]i[noisy]i[dist]i[tolerance]i[dmode]i[pthreshold]i[cthreshold]i[grey]b";

enum    creatargs { SRC, RESTORE, AFTER, BEFORE, ALTERNATIVE, PLANAR, SHOW, DEBUG, GMTHRES, MTHRES, NOISE, NOISY, DIST, TOLERANCE, DMODE, PTHRES, CTHRES, GREY, REDUCEF };

AVSValue    InitRemoveDirt(RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env)
{
  VideoInfo &vi = filter->vi;

  if (vi.IsRGB() || (vi.IsYV12() + args[PLANAR].AsBool(false) == 0))
    env->ThrowError("RemoveDirt: only YV12 and planar YUY2 clips are supported");

  int   pthreshold = args[PTHRES].AsInt(DEFAULT_PTHRESHOLD);


  filter->rd = new RemoveDirt(vi.width, vi.height, args[DIST].AsInt(DEFAULT_DIST), args[TOLERANCE].AsInt(DEFAULT_TOLERANCE), args[DMODE].AsInt(0)
    , args[MTHRES].AsInt(DEFAULT_MTHRESHOLD), args[NOISE].AsInt(0), args[NOISY].AsInt(-1), vi.IsYUY2()
    , pthreshold, args[CTHRES].AsInt(pthreshold)
    , args[GREY].AsBool(false), args[SHOW].AsBool(false), args[DEBUG].AsBool(false), env);


  filter->mthreshold = (args[GMTHRES].AsInt(DEFAULT_GMTHRESHOLD) * filter->rd->hblocks * filter->rd->vblocks) / 100;
  return filter;
}

AVSValue __cdecl CreateRestoreMotionBlocks(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  if (!args[RESTORE].Defined()) env->ThrowError("RestoreMotionBlocks: a restore clip must be specified");
  return InitRemoveDirt(new RestoreMotionBlocks(args[SRC].AsClip(), args[RESTORE].AsClip()
    , args[AFTER].Defined() ? args[AFTER].AsClip() : NULL, args[BEFORE].Defined() ? args[BEFORE].AsClip() : NULL
    , args[ALTERNATIVE].Defined() ? args[ALTERNATIVE].AsClip() : NULL, env), args, env);

}

// from VS
static uint32_t aligned_diff(const uint8_t *sp1, int32_t spitch1, const uint8_t *sp2, int32_t spitch2, int32_t hblocks, int32_t incpitch, int32_t height)
{
  __m128i xmm0 = _mm_setzero_si128();
  __m128i xmm1 = _mm_setzero_si128();

  spitch2 += incpitch;
  incpitch += spitch1;

  int32_t counter = hblocks;
  do {
    __m128i xmm2 = _mm_load_si128((__m128i*)sp1);
    __m128i xmm3 = _mm_load_si128((__m128i*)(sp1 + 16));
    __m128i xmm4 = _mm_load_si128((__m128i*)sp2);
    __m128i xmm5 = _mm_load_si128((__m128i*)(sp2 + 16));

    xmm2 = _mm_sad_epu8(xmm2, xmm4);
    xmm3 = _mm_sad_epu8(xmm3, xmm5);
    xmm0 = _mm_add_epi32(xmm0, xmm2);
    xmm1 = _mm_add_epi32(xmm1, xmm3);

    sp1 += 32;
    sp2 += 32;
    if (--counter > 0) {
      continue;
    }
    sp1 += incpitch;
    sp2 += spitch2;
    counter = hblocks;
  } while (--height > 0);

  xmm0 = _mm_add_epi32(xmm0, xmm1);
  return (uint32_t)_mm_cvtsi128_si32(xmm0);
}

// from VS
static uint32_t unaligned_diff(const uint8_t *sp1, int32_t spitch1, const uint8_t *sp2, int32_t spitch2, int32_t hblocks, int32_t incpitch, int32_t height)
{
  __m128i xmm0 = _mm_setzero_si128();
  __m128i xmm1 = _mm_setzero_si128();

  spitch2 += incpitch;
  incpitch += spitch1;

  int32_t counter = hblocks;
  do {
    __m128i xmm2 = _mm_loadu_si128((__m128i*)sp1);
    __m128i xmm3 = _mm_loadu_si128((__m128i*)(sp1 + 16));
    __m128i xmm4 = _mm_loadu_si128((__m128i*)sp2);
    __m128i xmm5 = _mm_loadu_si128((__m128i*)(sp2 + 16));

    xmm2 = _mm_sad_epu8(xmm2, xmm4);
    xmm3 = _mm_sad_epu8(xmm3, xmm5);
    xmm0 = _mm_add_epi32(xmm0, xmm2);
    xmm1 = _mm_add_epi32(xmm1, xmm3);

    sp1 += 32;
    sp2 += 32;
    if (--counter > 0) {
      continue;
    }
  } while (--height > 0);

  xmm0 = _mm_add_epi32(xmm0, xmm1);
  return (uint32_t)_mm_cvtsi128_si32(xmm0);
}

#if 0
#define SSE_INCREMENT   16
#define SSE_MOVE        movdqu
#define SSE3_MOVE       lddqu
#define SSE_RMOVE       movdqa
#define SSE0            xmm0
#define SSE1            xmm1
#define SSE2            xmm2
#define SSE3            xmm3
#define SSE4            xmm4
#define SSE5            xmm5
#define SSE6            xmm6
#define SSE7            xmm7

static  inline unsigned aligned_diff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
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
  __asm SSE_RMOVE   SSE3, [esi + SSE_INCREMENT]
  __asm psadbw      SSE2, [edi]
  __asm add         esi, 2 * SSE_INCREMENT
  __asm psadbw      SSE3, [edi + SSE_INCREMENT]
  __asm paddd       SSE0, SSE2
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
  __asm SSE3_MOVE   SSE3, [esi + SSE_INCREMENT]
  __asm add         esi, 2 * SSE_INCREMENT
  __asm SSE3_MOVE   SSE4, [edi]
  __asm SSE3_MOVE   SSE5, [edi + SSE_INCREMENT]
  __asm psadbw      SSE2, SSE4
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

#undef	SSE_INCREMENT
#undef	SSE_MOVE
#undef	SSE3_MOVE
#undef	SSE_RMOVE
#undef	SSE0
#undef	SSE1
#undef	SSE2
#undef	SSE3
#undef	SSE4
#undef	SSE5
#undef	SSE6
#undef	SSE7
#undef	SSE_EMMS
#endif

#if 0
#define	SSE_INCREMENT	8
#define	SSE_MOVE		movq
#define	SSE3_MOVE		movq
#define	SSE_RMOVE		movq
#define	SSE0			mm0
#define	SSE1			mm1
#define	SSE2			mm2
#define	SSE3			mm3
#define	SSE4			mm4
#define	SSE5			mm5
#define	SSE6			mm6
#define	SSE7			mm7
#define	SSE_EMMS		__asm	emms

static	unsigned gdiff_mmx(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
{
  __asm	pxor		SSE0, SSE0
  __asm	mov			eax, incpitch
  __asm	mov			ebx, spitch2
  __asm	mov			esi, sp1
  __asm	add			ebx, eax
  __asm	mov			edi, sp2
  __asm	add			eax, spitch1
  __asm	pxor		SSE1, SSE1
  __asm	mov			edx, height
  __asm	mov			ecx, hblocks
  __asm	align		16
  __asm	_loop:
  __asm	SSE_RMOVE	SSE2, [esi]
    __asm	SSE_RMOVE	SSE3, [esi + SSE_INCREMENT]
    __asm	psadbw		SSE2, [edi]
    __asm	add			esi, 2 * SSE_INCREMENT
  __asm	psadbw		SSE3, [edi + SSE_INCREMENT]
    __asm	paddd		SSE0, SSE2
  __asm	add			edi, 2 * SSE_INCREMENT
  __asm	paddd		SSE1, SSE3
  __asm	loop		_loop
  __asm	add			esi, eax
  __asm	add			edi, ebx
  __asm	dec			edx
  __asm	mov			ecx, hblocks
  __asm	jnz			_loop
  __asm	paddd		SSE0, SSE1
  __asm	movd		eax, SSE0
  SSE_EMMS
}
#endif

static unsigned gdiff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int hblocks, int incpitch, int height)
{
  if (((intptr_t)sp1 & 15) == 0 && ((intptr_t)sp2 & 15) == 0)
    return aligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch, height);
  else
    return unaligned_diff(sp1, spitch1, sp2, spitch2, hblocks, incpitch, height);
}

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
      if (dirmult * olddiff < lastdiff) goto set_end;
      if (dirmult * lastdiff < olddiff) goto set_begin;
      debugmsg = "[%u] SCSelect: global motion\n";
      selected = SPOINTER(global_motion);
    }
    if (debug) debug_printf(debugmsg, n);
    return selected->GetFrame(n, env);
  }
public:
  SCSelect(PClip clip, PClip _scene_begin, PClip _scene_end, PClip _global_motion, double dfactor, bool _debug, bool planar, int cache, int gcache, IScriptEnvironment* env)
    : GenericVideoFilter(clip), AccessFrame(vi.width, vi.IsYUY2()), scene_begin(_scene_begin), scene_end(_scene_end), global_motion(_global_motion), dirmult(dfactor), debug(_debug), lnr(-2)
  {
    if (vi.IsYV12() + planar == 0) env->ThrowError("SCSelect: only YV12 and planar YUY2 clips are supported");
    CompareVideoInfo(vi, scene_begin->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, scene_end->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, global_motion->GetVideoInfo(), "SCSelect", env);
    
    const int SSE_INCREMENT = 16;

    hblocks = vi.width / (2 * SSE_INCREMENT);
    incpitch = hblocks * (-2 * SSE_INCREMENT);
    scene_begin->SetCacheHints(CACHE_GENERIC, 0);
    scene_end->SetCacheHints(CACHE_GENERIC, 0);
    if (gcache >= 0) global_motion->SetCacheHints(CACHE_GENERIC, 0);
    if (cache >= 0) child->SetCacheHints(CACHE_GENERIC, cache);
  }
};

AVSValue __cdecl CreateSCSelect(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  enum ARGS { CLIP, SBEGIN, SEND, GMOTION, DFACTOR, DEBUG, PLANAR, CACHE, GCACHE };
  return new SCSelect(args[CLIP].AsClip(), args[SBEGIN].AsClip(), args[SEND].AsClip(), args[GMOTION].AsClip(), args[DFACTOR].AsFloat(4.0)
    , args[DEBUG].AsBool(false), args[PLANAR].AsBool(false), args[CACHE].AsInt(2), args[GCACHE].AsInt(0), env);
};

/* New 2.6 requirement!!! */
// Declare and initialise server pointers static storage.
const AVS_Linkage *AVS_linkage = 0;

/* New 2.6 requirement!!! */
// DLL entry point called from LoadPlugin() to setup a user plugin.
extern "C" __declspec(dllexport) const char* __stdcall
AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* const vectors) {
  /* New 2.6 requirment!!! */
  // Save the server pointers.
  AVS_linkage = vectors;
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