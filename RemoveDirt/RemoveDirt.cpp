#define LOGO "RemoveDirt 0.9.1\n"
// Avisynth filter for removing dirt from film clips
//
// By Rainer Wittmann <gorw@gmx.de>
// Additional work by Ferenc Pintér
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
#include "immintrin.h"
#include <cstdint>
#include <cassert>

//
// Part 1: options at compile time
//

//#define DEBUG_NAME
//#define RANGEFILES 

#define FHANDLERS 9
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
static uint32_t Sad_C(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch)
{
  uint32_t sum = 0;
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
uint32_t test_SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/)
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
uint32_t test_NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
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
uint32_t test_ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
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

uint32_t SADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/) {
  // optimizer makes it fast for SIMD
  return Sad_C<8, 8, uint8_t>(p1, pitch1, p2, pitch2);
}

static __forceinline __m128i _mm_loadh_epi64(__m128i x, __m128i *p)
{
  return _mm_castpd_si128(_mm_loadh_pd(_mm_castsi128_pd(x), (double *)p));
}

uint32_t NSADcompare(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
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

uint32_t ExcessPixels(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
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

/****************************************************
* SIMD functions end
****************************************************/

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
  uint32_t (*blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);
  int   hblocks, vblocks;
  uint32_t threshold;

  uint32_t (*test_blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);

  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    motionblocks = 0;

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

  MotionDetection(int   width, int height, uint32_t _threshold, int _noise, int _noisy, IScriptEnvironment* env) : threshold(_threshold), noise(_noise)
  {
    linewidth = width;
    hblocks = width / MOTIONBLOCKWIDTH;
    vblocks = height / MOTIONBLOCKHEIGHT;

    bool use_SSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;

    // old non-SSE2 check
    if (!use_SSE2 & ((hblocks == 0) || (vblocks == 0)))
        env->ThrowError("RemoveDirt: width or height of the clip too small");

    blockcompare = use_SSE2 ? SADcompare : test_SADcompare<8, 8>;
    test_blockcompare = test_SADcompare<8, 8>;
    if (noise > 0)
    {
      test_blockcompare = test_NSADcompare<8,8>;
      blockcompare = use_SSE2 ? NSADcompare : test_NSADcompare<8,8>;

      if (_noisy >= 0)
      {
        test_blockcompare = test_ExcessPixels<8,8>;
        blockcompare = use_SSE2 ? ExcessPixels : test_ExcessPixels<8,8>;
        threshold = _noisy;
      }
    }
    nline = hblocks + 1;
    int size = nline*(vblocks + 2);
    blockproperties_addr = new unsigned char[size];
    blockproperties = blockproperties_addr + nline;
    pline = -nline;
    memset(blockproperties_addr, BMARGIN, size);
  }

  ~MotionDetection()
  {
    delete[] blockproperties_addr;
  }
};

class MotionDetectionDist : public MotionDetection
{
public:
  int distblocks;
  uint32_t blocks;
private:
  uint32_t *isum;
  uint32_t tolerance;
  int dist, dist1, dist2, hinterior, vinterior, colinc, isumline, isuminc1, isuminc2;
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

  MotionDetectionDist(int   width, int height, int _dist, int _tolerance, int dmode, uint32_t _threshold, int _noise, int _noisy, IScriptEnvironment* env)
    : MotionDetection(width, height, _threshold, _noise, _noisy, env)
  {
    fn_processneighbours_t neighbourproc[] = { &MotionDetectionDist::processneighbours1, &MotionDetectionDist::processneighbours2, &MotionDetectionDist::processneighbours3 };
    blocks = hblocks * vblocks;
    isum = new uint32_t[blocks];
    if ((uint32_t)dmode >= 3) dmode = 0;
    if ((uint32_t)_tolerance > 100) _tolerance = 100;
    if (_tolerance == 0)
    {
      if (dmode == 2) _dist = 0;
      dmode = 0;
    }
    processneighbours = neighbourproc[dmode];
    dist2 = (dist1 = (dist = _dist) + 1) + 1;
    isumline = hblocks * sizeof(uint32_t);
    int d = dist1 + dist;
    tolerance = ((uint32_t)d * (uint32_t)d * (uint32_t)_tolerance * MOTION_FLAG1) / 100;
    hinterior = hblocks - d;
    vinterior = vblocks - d;
    colinc = 1 - (vblocks * nline);
    isuminc1 = (1 - (vinterior + dist)*hblocks) * sizeof(uint32_t);
    isuminc2 = (1 - vblocks * hblocks) * sizeof(uint32_t);
  }

  ~MotionDetectionDist()
  {
    delete[] isum;
  }
};

void MotionDetectionDist::markneighbours()
{
  unsigned char *begin = blockproperties;
  unsigned char *end = begin;
  uint32_t  *isum2 = isum;

  int   j = vblocks;
  do
  {
    uint32_t sum = 0;
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

  uint32_t * isum1 = isum2 = isum;
  begin = blockproperties;
  j = hblocks;
  do
  {
    uint32_t sum = 0;
    int i = dist;
    do
    {
      sum += *isum2;
      isum2 = (uint32_t*)((char*)isum2 + isumline);
    } while (--i);

    i = dist1;
    do
    {
      sum += *isum2;
      isum2 = (uint32_t*)((char*)isum2 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    i = vinterior;
    do
    {
      sum += *isum2 - *isum1;
      isum2 = (uint32_t*)((char*)isum2 + isumline);
      isum1 = (uint32_t*)((char*)isum1 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    i = dist;
    do
    {
      sum -= *isum1;
      isum1 = (uint32_t*)((char*)isum1 + isumline);
      if (sum > tolerance) *begin |= MOTION_FLAG2;
      begin += nline;
    } while (--i);

    begin += colinc;
    isum1 = (uint32_t*)((char*)isum1 + isuminc1);
    isum2 = (uint32_t*)((char*)isum2 + isuminc2);
  } while (--j);

}

uint32_t horizontal_diff(const BYTE *p, int pitch)
{
  // 8 pixels
  auto src1 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p));
  auto src2 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p + pitch));
  return _mm_cvtsi128_si32(_mm_sad_epu8(src1, src2)); // 8 pixels, lower sad result
}

uint32_t horizontal_diff_chroma(const BYTE *u, const BYTE *v, int pitch)
{
  // interleave u and v as 2x4 pixels then do sad
  auto src1_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u)));
  auto src1_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v)));
  auto src2_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u + pitch)));
  auto src2_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v + pitch)));
  return _mm_cvtsi128_si32(_mm_sad_epu8(_mm_unpacklo_epi32(src1_u, src1_v), _mm_unpacklo_epi32(src2_u, src2_v)));
}

static __forceinline int32_t vertical_diff(const uint8_t *p, int32_t pitch)
{
  __m128i xmm0 = _mm_undefined_si128();
  __m128i xmm1 = _mm_undefined_si128();
  // using lower 8 bytes
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + 0 * pitch)), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + 2 * pitch)), 1); // L2A1 L2A0 L0A1 L0A0
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + 1 * pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + 3 * pitch)), 1); // L3A1 L3A0 L1A1 L1A0
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + 4 * pitch)), 2);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(p + 6 * pitch)), 3); // L6A1 L6A0 L4A1 L4A0 L2A1 L2A0 L0A1 L0A0
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + 5 * pitch)), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(p + 7 * pitch)), 3); // L7A1 L7A0 L5A1 L5A0 L3A1 L3A0 L1A1 L1A0

  __m128i mask = _mm_undefined_si128();
  mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
  mask = _mm_srli_epi16(mask, 8);    // 00FF 00FF 00FF...

  auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 L6A0 0000 L4A0 0000 L2A0 0000 L0A0
  auto tmp0_odd = _mm_slli_epi16(xmm1, 8);    // L7A0 0000 L5A0 0000 L3A0 0000 L1A0 0000
  auto tmp1_even = _mm_srli_epi16(xmm0, 8);   // 0000 L6A1 0000 L4A1 0000 L2A1 0000 L0A1
  auto tmp1_odd = _mm_subs_epu8(xmm1, mask);  // L7A1 0000 L5A1 0000 L3A1 0000 L1A1 0000  tricky nullify lower 8 bits of words

  auto tmp0 = _mm_or_si128(tmp0_even, tmp0_odd);  // L7A0 L6A0 L5A0 L4A0 L3A0 L2A0 L1A0 L0A0
  auto tmp1 = _mm_or_si128(tmp1_even, tmp1_odd);  // L7A1 L6A1 L5A1 L4A1 L3A1 L2A1 L1A1 L0A1

  auto absdiff = _mm_sad_epu8(tmp0, tmp1);

  return _mm_cvtsi128_si32(absdiff);
}

// vertical_diff_yv12_chroma: vertical_diff_chroma<4>
// vertical_diff_yuy2_chroma: vertical_diff_chroma<8>
template<int blksizeY>
uint32_t vertical_diff_chroma_core(const BYTE *u, const BYTE *v, int pitch)
{
  assert(blksizeY == 4 || blksizeY == 8);

  __m128i xmm0 = _mm_undefined_si128();
  __m128i xmm1 = _mm_undefined_si128();
  // using lower 8 bytes
  // u
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + 0 * pitch)), 0);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + 2 * pitch)), 1); // U2A1 U2A0 U0A1 U0A0
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + 1 * pitch)), 0);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + 3 * pitch)), 1); // U3A1 U3A0 U1A1 U1A0
  // v
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + 0 * pitch)), 2);
  xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + 2 * pitch)), 3); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + 1 * pitch)), 2);
  xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + 3 * pitch)), 3); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

  __m128i mask = _mm_undefined_si128();
  mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
  mask = _mm_srli_epi16(mask, 8);    // 00FF 00FF 00FF...

  auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 V2A0 0000 V0A0 0000 U2A0 0000 U0A0
  auto tmp0_odd = _mm_slli_epi16(xmm1, 8);    // V3A0 0000 V1A0 0000 U3A0 0000 U1A0 0000
  auto tmp1_even = _mm_srli_epi16(xmm0, 8);   // 0000 V2A1 0000 V0A1 0000 U2A1 0000 U0A1
  auto tmp1_odd = _mm_subs_epu8(xmm1, mask);  // V3A1 0000 V1A1 0000 U3A1 0000 U1A1 0000  tricky nullify lower 8 bits of words

  auto tmp0 = _mm_or_si128(tmp0_even, tmp0_odd);  // V3A0 V2A0 V1A0 V0A0 U3A0 U2A0 U1A0 U0A0
  auto tmp1 = _mm_or_si128(tmp1_even, tmp1_odd);  // V3A1 V2A1 V1A1 V0A1 U3A1 U2A1 U1A1 U0A1

  auto absdiff = _mm_sad_epu8(tmp0, tmp1);

  if constexpr (blksizeY == 8) {
    // next 4 lines
    u += 4 * pitch;
    v += 4 * pitch;
    // no vertical subsampling: planar YUY2, YU16, YV24
    // u
    xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + 0 * pitch)), 0);
    xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(u + 2 * pitch)), 1); // U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + 1 * pitch)), 0);
    xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(u + 3 * pitch)), 1); // U3A1 U3A0 U1A1 U1A0
    // v
    xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + 0 * pitch)), 2);
    xmm0 = _mm_insert_epi16(xmm0, *((int32_t*)(v + 2 * pitch)), 3); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + 1 * pitch)), 2);
    xmm1 = _mm_insert_epi16(xmm1, *((int32_t*)(v + 3 * pitch)), 3); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

    __m128i mask = _mm_undefined_si128();
    mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
    mask = _mm_srli_epi16(mask, 8);    // 00FF 00FF 00FF...

    auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 V2A0 0000 V0A0 0000 U2A0 0000 U0A0
    auto tmp0_odd = _mm_slli_epi16(xmm1, 8);   // V3A0 0000 V1A0 0000 U3A0 0000 U1A0 0000
    auto tmp1_even = _mm_srli_epi16(xmm0, 8);   // 0000 V2A1 0000 V0A1 0000 U2A1 0000 U0A1
    auto tmp1_odd = _mm_subs_epu8(xmm1, mask); // V3A1 0000 V1A1 0000 U3A1 0000 U1A1 0000  tricky nullify lower 8 bits of words

    auto tmp0 = _mm_or_si128(tmp0_even, tmp0_odd);  // V3A0 V2A0 V1A0 V0A0 U3A0 U2A0 U1A0 U0A0
    auto tmp1 = _mm_or_si128(tmp1_even, tmp1_odd);  // V3A1 V2A1 V1A1 V0A1 U3A1 U2A1 U1A1 U0A1

    auto absdiff2 = _mm_sad_epu8(tmp0, tmp1);
    absdiff = _mm_avg_epu16(absdiff, absdiff2); // 16 bit enough, normalize result to YV12 case
  }

  return _mm_cvtsi128_si32(absdiff);
}

// usually test_vertical_diff<8>
template<int blocksizeY>
uint32_t test_vertical_diff(const BYTE *p, int pitch)
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

// test_vertical_diff_yv12_chroma: test_vertical_diff_chroma<4>
// test_vertical_diff_yuy2_chroma: test_vertical_diff_chroma<8>
template<int blksizeY>
uint32_t test_vertical_diff_chroma_core(const BYTE *u, const BYTE *v, int pitch)
{
  assert(blksizeY == 4 || blksizeY == 8);

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

  if constexpr (blksizeY == 8) {
    // no vertical subsampling e.g. planar YUY2, YV16, YV24
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
    res = (res + res2 + 1) / 2; // normalize result
  }
  return    res;
}

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
  Copy_C<8, 8, uint8_t>(dest, dpitch, src, spitch);
}

// copy_yv12_chroma: copy_chroma_core<4,4>
// copy_yuy2_chroma: copy_chroma_core<4,8>
template<int blksizeX, int blksizeY>
void copy_chroma_core(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
  Copy_C<blksizeX, blksizeY, uint8_t>(destu, dpitch, srcu, spitch);
  Copy_C<blksizeX, blksizeY, uint8_t>(destv, dpitch, srcv, spitch);
}

void inline colorise(BYTE *u, BYTE *v, int pitch, int height, uint32_t ucolor, uint32_t vcolor)
{
  int i = height;
  do
  {
    *(uint32_t*)u = ucolor; // 4 bytes
    *(uint32_t*)v = vcolor;
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
  uint32_t (*vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);
  void (*copy_chroma)(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch);
  uint32_t (*test_vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);

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
          uint32_t u_color = u_ncolor;
          uint32_t v_color = v_ncolor;
          if ((properties[0] & MOTION_FLAG) != 0)
          {
            u_color = u_mcolor; v_color = v_mcolor;
          }
          if ((properties[0] & MOTION_FLAGP) != 0)
          {
            u_color = u_pcolor; v_color = v_pcolor;
          }
          colorise(u, v, pitchUV, chromaheight, u_color, v_color); // FIXME: h subsampling
        }
        u += MOTIONBLOCKWIDTH / 2; // FIXME: h subsampling
        v += MOTIONBLOCKWIDTH / 2; // FIXME: h subsampling
        ++properties;
      } while (--i);
      u += inc;
      v += inc;
      ++properties;
    } while (--j);
  }

  Postprocessing(int width, int height, int dist, int tolerance, int dmode, uint32_t threshold, int _noise, int _noisy, bool yuy2, int _pthreshold, int _cthreshold, IScriptEnvironment* env)
    : MotionDetectionDist(width, height, dist, tolerance, dmode, threshold, _noise, _noisy, env)
    , pthreshold(_pthreshold), cthreshold(_cthreshold)
  {
    test_vertical_diff_chroma = test_vertical_diff_chroma_core<4>;
    vertical_diff_chroma = (env->GetCPUFlags() & CPUF_SSE2) ? vertical_diff_chroma_core<4> : test_vertical_diff_chroma_core<4>;
    copy_chroma = copy_chroma_core<4,4>; // v0.9 was copying 8x4 even for YV12, possible bug??? -> visible differences compared to v0.9
    linewidthUV = linewidth / 2;
    chromaheight = MOTIONBLOCKHEIGHT / 2;
    if (yuy2)
    {
      test_vertical_diff_chroma = test_vertical_diff_chroma_core<8>;
      chromaheight *= 2;
      vertical_diff_chroma = (env->GetCPUFlags() & CPUF_SSE2) ? vertical_diff_chroma_core<8> : test_vertical_diff_chroma_core<8>;
      copy_chroma = copy_chroma_core<4,8>; // v0.9 was copying 8x8 for YUY2 (YV16), possible bug??? -> visible differences compared to v0.9
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


  RemoveDirt(int _width, int _height, int dist, int tolerance, int dmode, uint32_t threshold, int noise, int noisy, bool yuy2, int pthreshold, int cthreshold, bool _grey, bool _show, bool debug, IScriptEnvironment* env)
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

  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE /*MT_SERIALIZED*/ : 0;
  }

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
  lastframe = vi.num_frames - 1;
  before_offset = after_offset = 0;
  if (after == NULL)
  {
    after = restore;
    goto set_before;
  }
  if (before != NULL)
  {
  }
  else
  {
  set_before:
    before_offset = -1;
    after_offset = 1;
    before = after;
  }
  if (alternative == NULL)
    alternative = restore;

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

  if (!vi.IsYV12() && !(vi.IsYUY2() && args[PLANAR].AsBool(false)))
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

// SCSelect helper
static uint32_t aligned_diff(const uint8_t *sp1, int32_t spitch1, const uint8_t *sp2, int32_t spitch2, int32_t width, int32_t height)
{
  __m128i sum = _mm_setzero_si128();

  const int wmod32 = width / 32;

  for (int h = 0; h < height; h++) {
    for (int x = 0; x < wmod32; x += 32) {
      __m128i src1_lo = _mm_load_si128((__m128i*)sp1 + x);
      __m128i src2_lo = _mm_load_si128((__m128i*)sp2 + x);
      auto absdiff = _mm_sad_epu8(src1_lo, src2_lo);

      sum = _mm_add_epi32(sum, absdiff);

      __m128i src1_hi = _mm_load_si128((__m128i*)(sp1 + x + 16));
      __m128i src2_hi= _mm_load_si128((__m128i*)(sp2 + x + 16));
      absdiff = _mm_sad_epu8(src1_hi, src2_hi);

      sum = _mm_add_epi32(sum, absdiff);
    }
    sp1 += spitch1;
    sp2 += spitch2;
  }

  // don't care the rest non wmod32

  // sums result lo 64, hi 64
  // Note: this part was not present in v0.9 nor in VS
  __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(sum), _mm_castsi128_ps(sum)));
  auto sum_final = _mm_add_epi32(sum, sum_hi);
  return (uint32_t)_mm_cvtsi128_si32(sum_final);
}

// SCSelect helper
static uint32_t unaligned_diff(const uint8_t *sp1, int32_t spitch1, const uint8_t *sp2, int32_t spitch2, int32_t width, int32_t height)
{
  __m128i sum = _mm_setzero_si128();

  const int wmod32 = width / 32;

  for (int h = 0; h < height; h++) {
    for (int x = 0; x < wmod32; x += 32) {
      __m128i src1_lo = _mm_loadu_si128((__m128i*)sp1 + x);
      __m128i src2_lo = _mm_loadu_si128((__m128i*)sp2 + x);
      auto absdiff = _mm_sad_epu8(src1_lo, src2_lo);

      sum = _mm_add_epi32(sum, absdiff);

      __m128i src1_hi = _mm_loadu_si128((__m128i*)(sp1 + x + 16));
      __m128i src2_hi = _mm_loadu_si128((__m128i*)(sp2 + x + 16));
      absdiff = _mm_sad_epu8(src1_hi, src2_hi);

      sum = _mm_add_epi32(sum, absdiff);
    }
    sp1 += spitch1;
    sp2 += spitch2;
  }

  // don't care the rest non wmod32

  // sums result lo 64, hi 64
  // Note: this horizontal summing part was not present in v0.9 nor in VS
  __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_castsi128_ps(sum), _mm_castsi128_ps(sum)));
  auto sum_final = _mm_add_epi32(sum, sum_hi);
  return (uint32_t)_mm_cvtsi128_si32(sum_final);
}

// FIXME: int32 overflow over 8k?
static uint32_t gdiff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int width, int height)
{
  if (((intptr_t)sp1 & 15) == 0 && ((intptr_t)sp2 & 15) == 0)
    return aligned_diff(sp1, spitch1, sp2, spitch2, width, height);
  else
    return unaligned_diff(sp1, spitch1, sp2, spitch2, width, height);
}

#define SPOINTER(p) p.operator->()

class   SCSelect : public GenericVideoFilter, public AccessFrame
{
  PClip scene_begin;
  PClip scene_end;
  PClip global_motion;
  uint32_t lastdiff; // FIXME: huge image dimensions require int64?
  uint32_t lnr;
  bool debug;
  double dirmult;

  // lastdiff and lnr variable: not MT friendly
  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_SERIALIZED : 0;
  }

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
        lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(pf), GetPitchY(pf), vi.width, vi.height);
      }
      int olddiff = lastdiff;
      {
        PVideoFrame nf = child->GetFrame(n + 1, env);
        lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(nf), GetPitchY(nf), vi.width, vi.height);
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
    if (!vi.IsYV12() && !(vi.IsYUY2() && planar)) 
      env->ThrowError("SCSelect: only YV12 and planar YUY2 clips are supported");

    CompareVideoInfo(vi, scene_begin->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, scene_end->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, global_motion->GetVideoInfo(), "SCSelect", env);
    
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
  /* New 2.6 requirement!!! */
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