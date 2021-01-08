#define LOGO "RemoveDirt 0.9.2\n"
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

#include "smmintrin.h" // SSE4.1 even if not defined (clang)
#include "emmintrin.h"
#include "immintrin.h"
#include <cstdint>
#include <cassert>
#include <cmath>

//
// Part 1: options at compile time
//

//#define RANGEFILES 

#define FHANDLERS 9
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
#include "common.h"
#include "RemoveGrainT.h"

//
// Part 3: auxiliary functions
//

//
// Part 4: block comparison functions
//

/****************************************************
* C functions
****************************************************/

__forceinline unsigned int SADABS(int x) { return (x < 0) ? -x : x; }

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
static __forceinline uint32_t Sad_C(const uint8_t *pSrc, int nSrcPitch, const uint8_t *pRef, int nRefPitch)
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

template<typename pixel_t, int blksizeX, int blksizeY>
uint32_t SADcompare_C(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/)
{
  return Sad_C<8, 8, pixel_t>(p1, pitch1, p2, pitch2);
}

template<typename pixel_t, int blksizeX, int blksizeY>
uint32_t NSADcompare_C(const BYTE *p1_8, int pitch1, const BYTE *p2_8, int pitch2, int noise)
{
  const pixel_t *p1 = reinterpret_cast<const pixel_t*>(p1_8);
  const pixel_t *p2 = reinterpret_cast<const pixel_t*>(p2_8);
  pitch1 /= sizeof(pixel_t);
  pitch2 /= sizeof(pixel_t);

  int res = 0;
  for (int y = 0; y < blksizeY; y++)
  {
    for (int x = 0; x < blksizeX; x++)
    {
      int diff = std::abs(p1[x] - p2[x]);
      diff -= noise;
      if (diff < 0) diff = 0;
      res += diff;
    }
    p1 += pitch1;
    p2 += pitch2;
  }
  /*
  Code kept for warning:

  This kind of tricky loop - which is like a hand-optimized assembler - was probably good in 2005 
  and helped VS2005 to generate faster code.

  But in 2019 it's way too complicated, the real algorithm is hidden behind the pointer and counter tricks.
  Unoptimizable.

  The clean code above is optimized with SIMD instructions even by Visual Studio.
  More: both SSE4.1 and SSE2 vector code is generated and chosen runtime depending on processor.
  
  // old code:
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
  */
  return res;
}

template<typename pixel_t, int blksizeX, int blksizeY>
uint32_t ExcessPixels_C(const BYTE *p1_8, int pitch1, const BYTE *p2_8, int pitch2, int noise)
{
  const pixel_t *p1 = reinterpret_cast<const pixel_t*>(p1_8);
  const pixel_t *p2 = reinterpret_cast<const pixel_t*>(p2_8);
  pitch1 /= sizeof(pixel_t);
  pitch2 /= sizeof(pixel_t);

  int count = 0;
  for (int y = 0; y < blksizeY; y++)
  {
    for (int x = 0; x < blksizeX; x++)
    {
      int diff = std::abs(p1[x] - p2[x]);
      if (diff > noise) ++count;
    }
    p1 += pitch1;
    p2 += pitch2;
  }
  /*
  Code kept for warning:

  This kind of tricky loop - which is like a hand-optimized assembler - was probably good in 2005
  and helped VS2005 to generate faster code.

  But in 2019 it's of no use, the real algorithm is hidden behind the pointer and counter tricks.
  Unoptimizable.

  The clean code above is optimized with a 8x unrolled loop by Visual Studio for blksizeX==8

  // old code:
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
  */
  return count;
}

/****************************************************
* End of C functions
****************************************************/

/****************************************************
* SIMD functions
****************************************************/

uint32_t SADcompare_simd(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int /*noise*/) {
  // optimizer makes it fast for SIMD
  return Sad_C<8, 8, uint8_t>(p1, pitch1, p2, pitch2);
}

static __forceinline __m128i _mm_loadh_epi64(__m128i x, __m128i *p)
{
  return _mm_castpd_si128(_mm_loadh_pd(_mm_castsi128_pd(x), (double *)p));
}

uint32_t NSADcompare_simd(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
{
  __m128i noise_vector = _mm_set1_epi8(noise);

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

uint32_t ExcessPixels_simd(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise)
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
  uint32_t threshold;
  int noise;
  int   motionblocks;
  unsigned char *blockproperties;
  int   linewidth;
  uint32_t (*blockcompare)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);
  int   hblocks, vblocks;

  int bits_per_pixel;

  uint32_t (*blockcompare_C)(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2, int noise);

  template<typename pixel_t>
  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    constexpr int pixelsize = sizeof(pixel_t);

    motionblocks = 0;

    int inc1 = MOTIONBLOCKHEIGHT * pitch1 - linewidth * pixelsize;
    int inc2 = MOTIONBLOCKHEIGHT * pitch2 - linewidth * pixelsize;
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
        if ((d = blockcompare(p1, pitch1, p2, pitch2, noise) - blockcompare_C(p1, pitch1, p2, pitch2, noise)) != 0)
          debug_printf("blockcompare test fails with difference = %i\n", d);
#endif
        if (blockcompare(p1, pitch1, p2, pitch2, noise) >= threshold)
        {
          properties[0] = MOTION_FLAG1;
          ++motionblocks;
        }

        p1 += MOTIONBLOCKWIDTH * pixelsize;
        p2 += MOTIONBLOCKWIDTH * pixelsize;
        ++properties;
      } while (--i);
      p1 += inc1;
      p2 += inc2;
      ++properties;
    } while (--j);
  }

  MotionDetection(int   width, int height, uint32_t _threshold, int _noise, int _noisy, int _bits_per_pixel, IScriptEnvironment* env) : threshold(_threshold), noise(_noise), bits_per_pixel(_bits_per_pixel)
  {
    linewidth = width;
    hblocks = width / MOTIONBLOCKWIDTH;
    vblocks = height / MOTIONBLOCKHEIGHT;

    const bool use_SSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;

    // old non-SSE2 check
    if (!use_SSE2 & ((hblocks == 0) || (vblocks == 0)))
        env->ThrowError("RemoveDirt: width or height of the clip too small");

    if (bits_per_pixel == 8) {
      blockcompare = use_SSE2 ? SADcompare_simd : SADcompare_C<uint8_t, 8, 8>;
      blockcompare_C = SADcompare_C<uint8_t, 8, 8>;
    }
    else { // 10-16 bits
      // FIXME simd 16, though compiler optimizes well
      blockcompare = SADcompare_C<uint16_t, 8, 8>;
      blockcompare_C = SADcompare_C<uint16_t, 8, 8>;
    }
    
    if (noise > 0)
    {
      if (bits_per_pixel == 8) {
        blockcompare_C = NSADcompare_C<uint8_t, 8, 8>;
        blockcompare = use_SSE2 ? NSADcompare_simd : NSADcompare_C<uint8_t, 8, 8>;
      }
      else {
        blockcompare_C = NSADcompare_C<uint16_t, 8, 8>;
        // FIXME simd 16, though even VS2017 compiler is optimizing it well to SSE4.1/SSE2
        blockcompare = NSADcompare_C<uint16_t, 8, 8>;
      }

      if (_noisy >= 0)
      {
        // noisy is counter, not a SAD-like number
        if (bits_per_pixel == 8) {
          blockcompare_C = ExcessPixels_C<uint8_t, 8, 8>;
          blockcompare = use_SSE2 ? ExcessPixels_simd : ExcessPixels_C<uint8_t, 8, 8>;
        }
        else {
          blockcompare_C = ExcessPixels_C<uint16_t, 8, 8>;
          // FIXME simd 16
          blockcompare = ExcessPixels_C<uint16_t, 8, 8>;
        }
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

  template<typename pixel_t>
  void  markblocks(const BYTE *p1, int pitch1, const BYTE *p2, int pitch2)
  {
    MotionDetection::markblocks<pixel_t>(p1, pitch1, p2, pitch2);
    distblocks = 0;
    if (dist)
    {
      markneighbours();
      (this->*processneighbours)();
    }
  }

  MotionDetectionDist(int   width, int height, int _dist, int _tolerance, int dmode, uint32_t _threshold, int _noise, int _noisy, int _bits_per_pixel, IScriptEnvironment* env)
    : MotionDetection(width, height, _threshold, _noise, _noisy, _bits_per_pixel, env)
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

template<typename pixel_t>
uint32_t horizontal_diff_C(const uint8_t *p, int pitch)
{
  return Sad_C<8, 1, pixel_t>(p, pitch, p + pitch, pitch);
}

template<typename pixel_t>
uint32_t horizontal_diff_simd(const uint8_t *p, int pitch)
{
  // 8 pixels
  if constexpr (sizeof(pixel_t) == 1) {
    auto src1 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p));
    auto src2 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p + pitch));
    return _mm_cvtsi128_si32(_mm_sad_epu8(src1, src2)); // 8 pixels, lower sad result
  }
  else {
    // 10-16 bits
    auto src1 = _mm_load_si128(reinterpret_cast<const __m128i *>(p));
    auto src2 = _mm_load_si128(reinterpret_cast<const __m128i *>(p + pitch));

    // SAD16 of two full vectors
    __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
    __m128i smaller_t = _mm_subs_epu16(src2, src1);
    __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

    auto zero = _mm_setzero_si128();
    // 8 x uint16 absolute differences
    auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
    // sum0_32, sum1_32, sum2_32, sum3_32
    // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
    auto a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
    auto a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
    sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
    // sum here: two 32 bit partial result: sum1 0 sum2 0
    auto sum_hi = _mm_unpackhi_epi64(sum, zero);
    // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
    sum = _mm_add_epi32(sum, sum_hi);
    return _mm_cvtsi128_si32(sum);
  }
}

template<typename pixel_t, int blksizeX>
uint32_t horizontal_diff_chroma_C(const uint8_t *u, const uint8_t *v, int pitch)
{
  assert(blksizeX == 4 || blksizeX == 8);
  return Sad_C<blksizeX, 1, pixel_t>(u, pitch, u + pitch, pitch) + Sad_C<4, 1, pixel_t>(v, pitch, v + pitch, pitch);
  // as of v0.9.2 no internal downscaling occurs for YV24 (for having the subsampled YV12 or YV16 result), cthreshold_h is scaled instead
}

template<typename pixel_t, int blksizeX>
uint32_t horizontal_diff_chroma_simd(const uint8_t *u, const uint8_t *v, int pitch)
{
  assert(blksizeX == 4 || blksizeX == 8);

  if constexpr (sizeof(pixel_t) == 1) {
    // interleave u and v as 2x4 pixels then do sad
    if constexpr (blksizeX == 4) {
      auto src1_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u)));
      auto src1_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v)));
      auto src2_u = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(u + pitch)));
      auto src2_v = _mm_cvtsi32_si128(*(reinterpret_cast<const int *>(v + pitch)));
      return _mm_cvtsi128_si32(_mm_sad_epu8(_mm_unpacklo_epi32(src1_u, src1_v), _mm_unpacklo_epi32(src2_u, src2_v)));
    }
    else {
      // 8 pixels 4:4:4
      auto src1_u = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(u));
      auto src1_v = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(v));
      auto src2_u = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(u + pitch));
      auto src2_v = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(v + pitch));
      return _mm_cvtsi128_si32(_mm_sad_epu8(src1_u, src2_u)) + _mm_cvtsi128_si32(_mm_sad_epu8(src1_v, src2_v));
      //return (sum + 1) >> 1; // 4:4:4: average to have the same magnitude as for YV12 and YV16
      // as of v0.9.2 no internal downscaling occurs for YV24 (for having the subsampled YV12 or YV16 result), cthreshold_h is scaled instead
    }
  }
  else {
    // interleave u and v as 2x4 pixels then do sad16
    if constexpr (blksizeX == 4) {
      auto src1_u = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(u));
      auto src1_v = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(v));
      auto src2_u = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(u + pitch));
      auto src2_v = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(v + pitch));
      auto src1 = _mm_unpacklo_epi16(src1_u, src1_v);
      auto src2 = _mm_unpacklo_epi16(src2_u, src2_v);
      
      // make 16bit SAD
      __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
      __m128i smaller_t = _mm_subs_epu16(src2, src1);
      __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

      auto zero = _mm_setzero_si128();
      // 8 x uint16 absolute differences
      auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
      // sum0_32, sum1_32, sum2_32, sum3_32
      // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
      auto a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
      auto a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
      sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
      // sum here: two 32 bit partial result: sum1 0 sum2 0
      auto sum_hi = _mm_unpackhi_epi64(sum, zero);
      // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
      sum = _mm_add_epi32(sum, sum_hi);
      return _mm_cvtsi128_si32(sum);
    }
    else {
      // 8 pixels 4:4:4
      // u
      auto src1_u = _mm_load_si128(reinterpret_cast<const __m128i *>(u));
      auto src2_u = _mm_load_si128(reinterpret_cast<const __m128i *>(u + pitch));

      // make 16bit SAD
      __m128i greater_t = _mm_subs_epu16(src1_u, src2_u); // unsigned sub with saturation
      __m128i smaller_t = _mm_subs_epu16(src2_u, src1_u);
      __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

      auto zero = _mm_setzero_si128();
      // 8 x uint16 absolute differences
      auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
      // sum0_32, sum1_32, sum2_32, sum3_32

      // v
      auto src1_v = _mm_load_si128(reinterpret_cast<const __m128i *>(v));
      auto src2_v = _mm_load_si128(reinterpret_cast<const __m128i *>(v + pitch));

      // make 16bit SAD
      greater_t = _mm_subs_epu16(src1_v, src2_v); // unsigned sub with saturation
      smaller_t = _mm_subs_epu16(src2_v, src1_v);
      absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

      // 8 x uint16 absolute differences
      auto sum_v = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));

      sum = _mm_add_epi32(sum, sum_v);

      // sum0_32, sum1_32, sum2_32, sum3_32
      // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
      auto a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
      auto a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
      sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
      // sum here: two 32 bit partial result: sum1 0 sum2 0
      auto sum_hi = _mm_unpackhi_epi64(sum, zero);
      // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
      sum = _mm_add_epi32(sum, sum_hi);
      return _mm_cvtsi128_si32(sum);
      //return (_mm_cvtsi128_si32(sum) + 1) >> 1; // 4:4:4: average to have the same magnitude as for YV12 and YV16
      // as of v0.9.2 no internal downscaling occurs for YV24 (for having the subsampled YV12 or YV16 result), cthreshold_h is scaled instead
    }
  }
}

template<typename pixel_t>
static uint32_t vertical_diff_sse2(const uint8_t *p, int32_t pitch)
{
  if constexpr (sizeof(pixel_t) == 1) {
    __m128i xmm0 = _mm_undefined_si128();
    __m128i xmm1 = _mm_undefined_si128();
    // using lower 8 bytes
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 0 * pitch)), 0);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 2 * pitch)), 1); // L2A1 L2A0 L0A1 L0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 1 * pitch)), 0);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 3 * pitch)), 1); // L3A1 L3A0 L1A1 L1A0
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 4 * pitch)), 2);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 6 * pitch)), 3); // L6A1 L6A0 L4A1 L4A0 L2A1 L2A0 L0A1 L0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 5 * pitch)), 2);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 7 * pitch)), 3); // L7A1 L7A0 L5A1 L5A0 L3A1 L3A0 L1A1 L1A0

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
  else {
    // 10-16 bits, pixel_t is uint16_t
    __m128i xmm0 = _mm_undefined_si128();
    __m128i xmm1 = _mm_undefined_si128();
    // using 8x2 bytes
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 0 * pitch)), 0);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 0 * pitch + 2)), 1);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 2 * pitch)), 2); // L2A1 L2A0 L0A1 L0A0
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 2 * pitch + 2)), 3); // L2A1 L2A0 L0A1 L0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 1 * pitch)), 0);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 1 * pitch + 2)), 1);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 3 * pitch)), 2); // L3A1 L3A0 L1A1 L1A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 3 * pitch + 2)), 3); // L3A1 L3A0 L1A1 L1A0
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 4 * pitch)), 4);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 4 * pitch + 2)), 5);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 6 * pitch)), 6); // L6A1 L6A0 L4A1 L4A0 L2A1 L2A0 L0A1 L0A0
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(p + 6 * pitch + 2)), 7); // L6A1 L6A0 L4A1 L4A0 L2A1 L2A0 L0A1 L0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 5 * pitch)), 4);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 5 * pitch + 2)), 5);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 7 * pitch)), 6); // L7A1 L7A0 L5A1 L5A0 L3A1 L3A0 L1A1 L1A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(p + 7 * pitch + 2)), 7); // L7A1 L7A0 L5A1 L5A0 L3A1 L3A0 L1A1 L1A0

    __m128i mask = _mm_undefined_si128();
    mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
    mask = _mm_srli_epi32(mask, 16);    // 0000FFFF 0000FFFF 0000FFFF...

    auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 L6A0 0000 L4A0 0000 L2A0 0000 L0A0
    auto tmp0_odd = _mm_slli_epi32(xmm1, 16);    // L7A0 0000 L5A0 0000 L3A0 0000 L1A0 0000
    auto tmp1_even = _mm_srli_epi32(xmm0, 16);   // 0000 L6A1 0000 L4A1 0000 L2A1 0000 L0A1
    auto tmp1_odd = _mm_subs_epu16(xmm1, mask);  // L7A1 0000 L5A1 0000 L3A1 0000 L1A1 0000  tricky nullify lower 16 bits of dwords

    auto src1 = _mm_or_si128(tmp0_even, tmp0_odd);  // L7A0 L6A0 L5A0 L4A0 L3A0 L2A0 L1A0 L0A0
    auto src2 = _mm_or_si128(tmp1_even, tmp1_odd);  // L7A1 L6A1 L5A1 L4A1 L3A1 L2A1 L1A1 L0A1

    // make 16bit SAD
    __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
    __m128i smaller_t = _mm_subs_epu16(src2, src1);
    __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

    auto zero = _mm_setzero_si128();
    // 8 x uint16 absolute differences
    auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
    // sum0_32, sum1_32, sum2_32, sum3_32
    // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
    auto a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
    auto a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
    sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
    // sum here: two 32 bit partial result: sum1 0 sum2 0
    auto sum_hi = _mm_unpackhi_epi64(sum, zero);
    // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
    sum = _mm_add_epi32(sum, sum_hi);
    return _mm_cvtsi128_si32(sum);
  }
}

// SSE4.1: needed only for uint16_t
#ifdef __clang__
// LLVM 7.0.1
// -Wno-gcc-compat needed for C++ compiler swicthes or otherwise: 
// GCC does not allow '__target__' attribute in this position on a function definition [-Wgcc-compat]
static uint32_t vertical_diff_uint16_sse4(const uint8_t *p, int32_t pitch) __attribute__((__target__("sse4.1")))
#else
static uint32_t vertical_diff_uint16_sse4(const uint8_t *p, int32_t pitch)
#endif
{
  // 10-16 bits, pixel_t is uint16_t
  __m128i xmm0 = _mm_undefined_si128();
  __m128i xmm1 = _mm_undefined_si128();
  // using 8x2 bytes
  // SSE4.1
  xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(p + 0 * pitch)), 0);
  xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(p + 2 * pitch)), 1); // L2A1 L2A0 L0A1 L0A0
  xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(p + 1 * pitch)), 0);
  xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(p + 3 * pitch)), 1); // L3A1 L3A0 L1A1 L1A0
  xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(p + 4 * pitch)), 2);
  xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(p + 6 * pitch)), 3); // L6A1 L6A0 L4A1 L4A0 L2A1 L2A0 L0A1 L0A0
  xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(p + 5 * pitch)), 2);
  xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(p + 7 * pitch)), 3); // L7A1 L7A0 L5A1 L5A0 L3A1 L3A0 L1A1 L1A0

  __m128i mask = _mm_undefined_si128();
  mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
  mask = _mm_srli_epi32(mask, 16);    // 0000FFFF 0000FFFF 0000FFFF...

  auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 L6A0 0000 L4A0 0000 L2A0 0000 L0A0
  auto tmp0_odd = _mm_slli_epi32(xmm1, 16);    // L7A0 0000 L5A0 0000 L3A0 0000 L1A0 0000
  auto tmp1_even = _mm_srli_epi32(xmm0, 16);   // 0000 L6A1 0000 L4A1 0000 L2A1 0000 L0A1
  auto tmp1_odd = _mm_subs_epu16(xmm1, mask);  // L7A1 0000 L5A1 0000 L3A1 0000 L1A1 0000  tricky nullify lower 16 bits of dwords

  auto src1 = _mm_or_si128(tmp0_even, tmp0_odd);  // L7A0 L6A0 L5A0 L4A0 L3A0 L2A0 L1A0 L0A0
  auto src2 = _mm_or_si128(tmp1_even, tmp1_odd);  // L7A1 L6A1 L5A1 L4A1 L3A1 L2A1 L1A1 L0A1

  // make 16bit SAD
  __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
  __m128i smaller_t = _mm_subs_epu16(src2, src1);
  __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

  auto zero = _mm_setzero_si128();
  // 8 x uint16 absolute differences
  auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
  // sum0_32, sum1_32, sum2_32, sum3_32
  // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
  auto a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
  auto a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
  sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
  // sum here: two 32 bit partial result: sum1 0 sum2 0
  auto sum_hi = _mm_unpackhi_epi64(sum, zero);
  // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
  sum = _mm_add_epi32(sum, sum_hi);
  return _mm_cvtsi128_si32(sum);
}

// vertical_diff_yv12_chroma: vertical_diff_chroma<4>
// vertical_diff_yuy2_chroma: vertical_diff_chroma<8>
template<typename pixel_t, int blksizeY>
uint32_t vertical_diff_chroma_core_sse2(const uint8_t *u, const uint8_t *v, int pitch)
{
  assert(blksizeY == 4 || blksizeY == 8);

  if constexpr (sizeof(pixel_t) == 1) {
    __m128i xmm0 = _mm_undefined_si128();
    __m128i xmm1 = _mm_undefined_si128();
    // using lower 8 bytes
    // u
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 0 * pitch)), 0);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 2 * pitch)), 1); // U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 1 * pitch)), 0);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 3 * pitch)), 1); // U3A1 U3A0 U1A1 U1A0
    // v
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 0 * pitch)), 2);
    xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 2 * pitch)), 3); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 1 * pitch)), 2);
    xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 3 * pitch)), 3); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

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
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 0 * pitch)), 0);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 2 * pitch)), 1); // U2A1 U2A0 U0A1 U0A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 1 * pitch)), 0);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 3 * pitch)), 1); // U3A1 U3A0 U1A1 U1A0
      // v
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 0 * pitch)), 2);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 2 * pitch)), 3); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 1 * pitch)), 2);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 3 * pitch)), 3); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

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
      absdiff = _mm_add_epi32(absdiff, absdiff2);
    }

    return _mm_cvtsi128_si32(absdiff);
  }
  else {
    // 10-16 bits
    constexpr int vblocks = (blksizeY == 8) ? 2 : 1;
    auto sumtot = _mm_setzero_si128();
    auto zero = _mm_setzero_si128();
    for (int i = 0; i < vblocks; i++) {
      __m128i xmm0 = _mm_undefined_si128();
      __m128i xmm1 = _mm_undefined_si128();
      // using 2x8 bytes
      // u
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 0 * pitch)), 0);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 0 * pitch + 2)), 1);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 2 * pitch)), 2); // U2A1 U2A0 U0A1 U0A0
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(u + 2 * pitch + 2)), 3); // U2A1 U2A0 U0A1 U0A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 1 * pitch)), 0);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 1 * pitch + 2)), 1);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 3 * pitch)), 2); // U3A1 U3A0 U1A1 U1A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(u + 3 * pitch + 2)), 3); // U3A1 U3A0 U1A1 U1A0
      // v
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 0 * pitch)), 4);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 0 * pitch + 2)), 5);
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 2 * pitch)), 6); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
      xmm0 = _mm_insert_epi16(xmm0, *((int16_t*)(v + 2 * pitch + 2)), 7); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 1 * pitch)), 4);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 1 * pitch + 2)), 5);
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 3 * pitch)), 6); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0
      xmm1 = _mm_insert_epi16(xmm1, *((int16_t*)(v + 3 * pitch + 2)), 7); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

      __m128i mask = _mm_undefined_si128();
      mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
      mask = _mm_srli_epi32(mask, 16);   // 0000FFFF 000FF0FF 00FF00FF...

      auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 V2A0 0000 V0A0 0000 U2A0 0000 U0A0
      auto tmp0_odd = _mm_slli_epi32(xmm1, 16);    // V3A0 0000 V1A0 0000 U3A0 0000 U1A0 0000
      auto tmp1_even = _mm_srli_epi32(xmm0, 16);   // 0000 V2A1 0000 V0A1 0000 U2A1 0000 U0A1
      auto tmp1_odd = _mm_subs_epu16(xmm1, mask);  // V3A1 0000 V1A1 0000 U3A1 0000 U1A1 0000  tricky nullify lower 16 bits of dwords

      auto src1 = _mm_or_si128(tmp0_even, tmp0_odd);  // V3A0 V2A0 V1A0 V0A0 U3A0 U2A0 U1A0 U0A0
      auto src2 = _mm_or_si128(tmp1_even, tmp1_odd);  // V3A1 V2A1 V1A1 V0A1 U3A1 U2A1 U1A1 U0A1

    // make 16bit SAD
      __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
      __m128i smaller_t = _mm_subs_epu16(src2, src1);
      __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

      // 8 x uint16 absolute differences
      auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
      // sum0_32, sum1_32, sum2_32, sum3_32

      sumtot = _mm_add_epi32(sumtot, sum);

      // next 4 lines
      u += 4 * pitch;
      v += 4 * pitch;
    }

    // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
    auto a0_a1 = _mm_unpacklo_epi32(sumtot, zero); // a0 0 a1 0
    auto a2_a3 = _mm_unpackhi_epi32(sumtot, zero); // a2 0 a3 0
    sumtot = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0

    // sumtot here: two 32 bit partial result: sum1 0 sum2 0
    auto sumtot_hi = _mm_unpackhi_epi64(sumtot, zero);
    // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
    sumtot = _mm_add_epi32(sumtot, sumtot_hi);

    int sumtot32 = _mm_cvtsi128_si32(sumtot);

    return sumtot32;
  }
}

// SSE4.1: needed only for uint16_t
template<int blksizeY>
#ifdef __clang__
uint32_t vertical_diff_chroma_core_uint16_sse4(const uint8_t *u, const uint8_t *v, int pitch) __attribute__((__target__("sse4.1")))
#else
uint32_t vertical_diff_chroma_core_uint16_sse4(const uint8_t *u, const uint8_t *v, int pitch)
#endif
{
  assert(blksizeY == 4 || blksizeY == 8);

  // 10-16 bits
  constexpr int vblocks = (blksizeY == 8) ? 2 : 1;

  auto sumtot = _mm_setzero_si128();
  auto zero = _mm_setzero_si128();
  for (int i = 0; i < vblocks; i++) {
    __m128i xmm0 = _mm_undefined_si128();
    __m128i xmm1 = _mm_undefined_si128();
    // using 2x8 bytes
    // SSE4.1
    // u
    xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(u + 0 * pitch)), 0);
    xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(u + 2 * pitch)), 1); // U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(u + 1 * pitch)), 0);
    xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(u + 3 * pitch)), 1); // U3A1 U3A0 U1A1 U1A0
    // v
    xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(v + 0 * pitch)), 2);
    xmm0 = _mm_insert_epi32(xmm0, *((int32_t*)(v + 2 * pitch)), 3); // V2A1 V2A0 V0A1 V0A0 U2A1 U2A0 U0A1 U0A0
    xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(v + 1 * pitch)), 2);
    xmm1 = _mm_insert_epi32(xmm1, *((int32_t*)(v + 3 * pitch)), 3); // V3A1 V3A0 V1A1 V1A0 U3A1 U3A0 U1A1 U1A0

    __m128i mask = _mm_undefined_si128();
    mask = _mm_cmpeq_epi8(mask, mask); // set it all FF
    mask = _mm_srli_epi32(mask, 16);   // 0000FFFF 000FF0FF 00FF00FF...

    auto tmp0_even = _mm_and_si128(xmm0, mask); // 0000 V2A0 0000 V0A0 0000 U2A0 0000 U0A0
    auto tmp0_odd = _mm_slli_epi32(xmm1, 16);    // V3A0 0000 V1A0 0000 U3A0 0000 U1A0 0000
    auto tmp1_even = _mm_srli_epi32(xmm0, 16);   // 0000 V2A1 0000 V0A1 0000 U2A1 0000 U0A1
    auto tmp1_odd = _mm_subs_epu16(xmm1, mask);  // V3A1 0000 V1A1 0000 U3A1 0000 U1A1 0000  tricky nullify lower 16 bits of dwords

    auto src1 = _mm_or_si128(tmp0_even, tmp0_odd);  // V3A0 V2A0 V1A0 V0A0 U3A0 U2A0 U1A0 U0A0
    auto src2 = _mm_or_si128(tmp1_even, tmp1_odd);  // V3A1 V2A1 V1A1 V0A1 U3A1 U2A1 U1A1 U0A1

  // make 16bit SAD
    __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
    __m128i smaller_t = _mm_subs_epu16(src2, src1);
    __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

    // 8 x uint16 absolute differences
    auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
    // sum0_32, sum1_32, sum2_32, sum3_32

    sumtot = _mm_add_epi32(sumtot, sum);

    // next 4 lines
    u += 4 * pitch;
    v += 4 * pitch;
  }

  // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
  auto a0_a1 = _mm_unpacklo_epi32(sumtot, zero); // a0 0 a1 0
  auto a2_a3 = _mm_unpackhi_epi32(sumtot, zero); // a2 0 a3 0
  sumtot = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0

  // sumtot here: two 32 bit partial result: sum1 0 sum2 0
  auto sumtot_hi = _mm_unpackhi_epi64(sumtot, zero);
  // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
  sumtot = _mm_add_epi32(sumtot, sumtot_hi);

  int sumtot32 = _mm_cvtsi128_si32(sumtot);

  return sumtot32;
}

template<typename pixel_t>
uint32_t vertical_diff_core_C(const uint8_t *p8, int pitch)
{
  // always 8 pixels for luma
  const pixel_t *p = reinterpret_cast<const pixel_t*>(p8);
  pitch /= sizeof(pixel_t);
  int   res = 0;
  const int blocksizeY = 8;
  for (int y = 0; y < blocksizeY; y++)
  {
    int diff = std::abs(p[0] - p[1]);
    res += diff;
    p += pitch;
  }
  /* 
  // old code, unoptimizable by VS
  int   i = blocksizeY;
  do
  {
    int diff = p[0] - p[1];
    if (diff < 0) res -= diff;
    else res += diff;
    p += pitch;
  } while (--i);
  */
  return    res;
}

// vertical_diff_yv12_chroma_c: vertical_diff_chroma<4>
// vertical_diff_yuy2_chroma_c: vertical_diff_chroma<8>
template<typename pixel_t, int blksizeY>
uint32_t vertical_diff_chroma_core_C(const uint8_t *u8, const uint8_t *v8, int pitch)
{
  assert(blksizeY == 4 || blksizeY == 8);

  const pixel_t *u = reinterpret_cast<const pixel_t*>(u8);
  const pixel_t *v = reinterpret_cast<const pixel_t*>(v8);
  pitch /= sizeof(pixel_t);

  int res = 0;
  for (int y = 0; y < blksizeY; y++)
  {
    int diff = std::abs(u[0] - u[1]);
    res += diff;
    diff = std::abs(v[0] - v[1]);
    res += diff;
    u += pitch;
    v += pitch;
  }
  // as of v0.9.2 no internal downscaling occurs for YV16/YV24 (for having the subsampled YV12 result), cthreshold_v is scaled instead
  /*
  // old code, unoptimizable by VS
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
  */
  return    res;
}

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
__forceinline void Copy_C(uint8_t *pDst, int nDstPitch, const uint8_t *pSrc, int nSrcPitch)
{
  for (int j = 0; j < nBlkHeight; j++)
  {
    memcpy(pDst, pSrc, nBlkWidth * sizeof(pixel_t));
    pDst += nDstPitch;
    pSrc += nSrcPitch;
  }
}

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
__forceinline void copy_luma_C(BYTE *dest, int dpitch, const BYTE *src, int spitch)
{
  Copy_C<8, 8, pixel_t>(dest, dpitch, src, spitch);
}

// copy_yv12_chroma: copy_chroma_core<4,4>
// copy_yuy2_chroma: copy_chroma_core<4,8>
template<int blksizeX, int blksizeY, typename pixel_t>
void copy_chroma_core(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch)
{
  Copy_C<blksizeX, blksizeY, pixel_t>(destu, dpitch, srcu, spitch);
  Copy_C<blksizeX, blksizeY, pixel_t>(destv, dpitch, srcv, spitch);
}

template<int blksizeY>
void inline colorise(BYTE *u, BYTE *v, int pitch, uint32_t ucolor, uint32_t vcolor)
{
  for(int i = 0; i < blksizeY; i++)
  {
    *(uint32_t*)u = ucolor; // 4 bytes
    *(uint32_t*)v = vcolor;
    u += pitch;
    v += pitch;
  };
}

class   Postprocessing
  : public MotionDetectionDist
{
  int   linewidthUV;
  int   pthreshold; // filter parameter corrected with bit depth factor
  int   cthreshold_h; // filter parameter corrected with bit depth factor for horizontal_diff
  int   cthreshold_v; // filter parameter corrected with bit depth factor for vertical_diff

  uint32_t(*vertical_diff)(const uint8_t *p, int32_t pitch);
  uint32_t(*vertical_diff_C)(const uint8_t *p, int32_t pitch);
  uint32_t(*horizontal_diff)(const BYTE *p, int pitch);
  uint32_t(*horizontal_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);
  void(*copy_luma)(BYTE *dest, int dpitch, const BYTE *src, int spitch);

  uint32_t (*vertical_diff_chroma)(const BYTE *u, const BYTE *v, int pitch);
  void (*copy_chroma)(BYTE *destu, BYTE *destv, int dpitch, const BYTE *srcu, const BYTE *srcv, int spitch);
  uint32_t (*vertical_diff_chroma_C)(const BYTE *u, const BYTE *v, int pitch);

public:
  int       loops;
  int       restored_blocks;

  void(Postprocessing::*postprocessing)(BYTE *dp, int dpitch, BYTE *dpU, BYTE *dpV, int dpitchUV, const BYTE *sp, int spitch, const BYTE *spU, const BYTE *spV, int spitchUV);
  void(Postprocessing::*postprocessing_grey)(BYTE *dp, int dpitch, const BYTE *sp, int spitch);
  void(Postprocessing::*show_motion)(BYTE *u, BYTE *v, int pitchUV);

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

  template<typename pixel_t>
  void  postprocessing_grey_core(BYTE *dp, int dpitch, const BYTE *sp, int spitch)
  {
    constexpr int pixelsize = sizeof(pixel_t); // 1 or 2 bytes

    int bottomdp = 7 * dpitch;
    int bottomsp = 7 * spitch;
    int dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth * pixelsize;
    int sinc = MOTIONBLOCKHEIGHT * spitch - linewidth * pixelsize;

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
          // horizontal offsets are in byte, should be corrected by  * pixelsize
          if ((cl[0] & TO_CLEAN) != 0)
          {
            copy_luma(dp2, dpitch, sp2, spitch);
            cl[0] &= ~TO_CLEAN;

            if (cl[-1] == 0)
            {
#ifdef  TEST_VERTICAL_DIFF
              if (vertical_diff(dp2 + leftdp, dpitch) != vertical_diff_C<8>(dp2 + leftdp, dpitch))
                debug_printf("vertical_diff incorrect\n");
#endif
              if (vertical_diff(dp2 + leftdp * pixelsize, dpitch) > vertical_diff(sp2 + leftsp * pixelsize, spitch) + pthreshold)
              {
                ++to_restore;
                cl[-1] = MOTION_FLAG3;
              }
            }
            if (cl[1] == 0)
            {
              if (vertical_diff(dp2 + rightdp * pixelsize, dpitch) > vertical_diff(sp2 + rightsp * pixelsize, spitch) + pthreshold)
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
          dp2 += rightbldp * pixelsize;
          sp2 += rightblsp * pixelsize;
        } while (--j);
        cl++;
        dp2 += dinc;
        sp2 += sinc;
      } while (--i);
      restored_blocks += to_restore;
    } while (to_restore != 0);
  }


  template<typename pixel_t, int blksizeXchroma, int blksizeYchroma>
  void  postprocessing_core(BYTE *dp, int dpitch, BYTE *dpU, BYTE *dpV, int dpitchUV, const BYTE *sp, int spitch, const BYTE *spU, const BYTE *spV, int spitchUV)
  {
    constexpr int pixelsize = sizeof(pixel_t); // 1 or 2 bytes

    constexpr int Cleftdp = -1;
    constexpr int Crightdp = blksizeXchroma - 1;
    constexpr int Cleftsp = Cleftdp;
    constexpr int Crightsp = Crightdp;

    const int Ctopdp = -dpitchUV;
    const int Ctopsp = -spitchUV;

    const int bottomdp = 7 * dpitch;
    const int bottomsp = 7 * spitch;
    const int Cbottomdp = (blksizeYchroma - 1) * dpitchUV;
    const int Cbottomsp = (blksizeYchroma - 1) * spitchUV;
    int dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth * pixelsize;
    int sinc = MOTIONBLOCKHEIGHT * spitch - linewidth * pixelsize;
    int dincUV = blksizeYchroma * dpitchUV - linewidthUV * pixelsize;
    int sincUV = blksizeYchroma * spitchUV - linewidthUV * pixelsize;

    loops = restored_blocks = 0;

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

      for (int y = 0; y < vblocks; y++)
      {
        for (int x = 0; x < hblocks; x++)
        {
          if ((cl[0] & TO_CLEAN) != 0)
          {
            copy_luma(dp2, dpitch, sp2, spitch);
            copy_chroma(dpU2, dpV2, dpitchUV, spU2, spV2, spitchUV);
            cl[0] &= ~TO_CLEAN;

            // horizontal offsets are in byte, should be corrected by  * pixelsize
            if (cl[-1] == 0)
            {
#ifdef  TEST_VERTICAL_DIFF_CHROMA
              if (vertical_diff_chroma(dpU2 + Cleftdp * pixelsize, dpV2 + Cleftdp * pixelsize, dpitchUV) != vertical_diff_chroma_c(dpU2 + Cleftdp * pixelsize, dpV2 + Cleftdp * pixelsize, dpitchUV))
                debug_printf("vertical_diff_chroma incorrect\n");
#endif
              if ((vertical_diff(dp2 + leftdp * pixelsize, dpitch) > vertical_diff(sp2 + leftsp * pixelsize, spitch) + pthreshold)
                || (vertical_diff_chroma(dpU2 + Cleftdp * pixelsize, dpV2 + Cleftdp * pixelsize, dpitchUV) > vertical_diff_chroma(spU2 + Cleftsp * pixelsize, spV2 + Cleftsp * pixelsize, spitchUV) + cthreshold_v))
              {
                ++to_restore;
                cl[-1] = MOTION_FLAG3;
              }
            }
            if (cl[1] == 0)
            {
              if ((vertical_diff(dp2 + rightdp * pixelsize, dpitch) > vertical_diff(sp2 + rightsp * pixelsize, spitch) + pthreshold)
                || (vertical_diff_chroma(dpU2 + Crightdp * pixelsize, dpV2 + Crightdp * pixelsize, dpitchUV) > vertical_diff_chroma(spU2 + Crightsp * pixelsize, spV2 + Crightsp * pixelsize, spitchUV) + cthreshold_v))
              {
                ++to_restore;
                cl[1] = MOTION_FLAG3;
              }
            }
            if (cl[pline] == 0)
            {
              if ((horizontal_diff(dp2 + topdp, dpitch) > horizontal_diff(sp2 + topsp, spitch) + pthreshold)
                || (horizontal_diff_chroma(dpU2 + Ctopdp, dpV2 + Ctopdp, dpitchUV) > horizontal_diff_chroma(spU2 + Ctopsp, spV2 + Ctopsp, spitchUV) + cthreshold_h))
              {
                ++to_restore;
                cl[pline] = MOTION_FLAG3;
              }
            }
            if (cl[nline] == 0)
            {
              if ((horizontal_diff(dp2 + bottomdp, dpitch) > horizontal_diff(sp2 + bottomsp, spitch) + pthreshold)
                || (horizontal_diff_chroma(dpU2 + Cbottomdp, dpV2 + Cbottomdp, dpitchUV) > horizontal_diff_chroma(spU2 + Cbottomsp, spV2 + Cbottomsp, spitchUV) + cthreshold_h))
              {
                ++to_restore;
                cl[nline] = MOTION_FLAG3;
              }
            }
          }
          ++cl;
          dp2 += rightbldp * pixelsize;
          sp2 += rightblsp * pixelsize;
          dpU2 += blksizeXchroma * pixelsize;
          spU2 += blksizeXchroma * pixelsize;
          dpV2 += blksizeXchroma * pixelsize;
          spV2 += blksizeXchroma * pixelsize;
        }
        cl++;
        dp2 += dinc;
        sp2 += sinc;
        dpU2 += dincUV;
        spU2 += sincUV;
        dpV2 += dincUV;
        spV2 += sincUV;
      }
      restored_blocks += to_restore;
    } while (to_restore != 0);
  }

  template<typename pixel_t, int blksizeXchroma, int blksizeYchroma>
  void  show_motion_core(BYTE *u, BYTE *v, int pitchUV)
  {
    int inc = blksizeYchroma * pitchUV - linewidthUV * sizeof(pixel_t);

    unsigned char *properties = blockproperties;

    int j = vblocks;
    do
    {
      int i = hblocks;
      do
      {
        if (properties[0])
        {
          if constexpr (sizeof(pixel_t) == 1) {
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
            colorise<blksizeYchroma>(u, v, pitchUV, u_color, v_color);
            if (blksizeXchroma == 8)
              colorise<blksizeYchroma>(u + 4, v + 4, pitchUV, u_color, v_color);
          }
          else {
            uint32_t u_color = U_N << (bits_per_pixel - 8);
            uint32_t v_color = V_N << (bits_per_pixel - 8);
            if ((properties[0] & MOTION_FLAG) != 0)
            {
              u_color = U_M << (bits_per_pixel - 8);
              v_color = V_M << (bits_per_pixel - 8);
            }
            if ((properties[0] & MOTION_FLAGP) != 0)
            {
              u_color = U_P << (bits_per_pixel - 8);
              v_color = V_P << (bits_per_pixel - 8);
            }
            u_color = u_color + (u_color << 16); // 2 pixels
            v_color = v_color + (v_color << 16);
            colorise<blksizeYchroma>(u, v, pitchUV, u_color, v_color);
            colorise<blksizeYchroma>(u + 2 * sizeof(pixel_t), v + 2 * sizeof(pixel_t), pitchUV, u_color, v_color);
            if (blksizeXchroma == 8) {
              colorise<blksizeYchroma>(u + 4 * sizeof(pixel_t), v + 4 * sizeof(pixel_t), pitchUV, u_color, v_color);
              colorise<blksizeYchroma>(u + 6 * sizeof(pixel_t), v + 6 * sizeof(pixel_t), pitchUV, u_color, v_color);
            }
          }
        }
        u += blksizeXchroma * sizeof(pixel_t);
        v += blksizeXchroma * sizeof(pixel_t);
        ++properties;
      } while (--i);
      u += inc;
      v += inc;
      ++properties;
    } while (--j);
  }

  Postprocessing(int width, int height, int dist, int tolerance, int dmode, uint32_t threshold, int _noise, int _noisy, int _pthreshold, int _cthreshold, 
    int _xRatioUV, int _yRatioUV, int _bits_per_pixel, IScriptEnvironment* env)
    : MotionDetectionDist(width, height, dist, tolerance, dmode, 
      threshold << (_bits_per_pixel - 8), 
      _noise << (_bits_per_pixel - 8), 
      _noisy, // counter, do not scale
      _bits_per_pixel,
      env)
    , 
    pthreshold(_pthreshold << (_bits_per_pixel - 8)), 
    cthreshold_h((_cthreshold << (_bits_per_pixel - 8)) * 2 / _xRatioUV),
    cthreshold_v((_cthreshold << (_bits_per_pixel - 8)) * 2 / _yRatioUV)
  {
    const bool useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
    const bool useSSE41 = (env->GetCPUFlags() & CPUF_SSE4_1) == CPUF_SSE4_1;

    if (_bits_per_pixel == 8) {
      vertical_diff_C = vertical_diff_core_C<uint8_t>;
      vertical_diff = useSSE2 ? vertical_diff_sse2<uint8_t> : vertical_diff_core_C<uint8_t>;
      copy_luma = copy_luma_C<8, 8, uint8_t>;

      horizontal_diff = useSSE2 ? horizontal_diff_simd<uint8_t> : horizontal_diff_C<uint8_t>;

      if (_yRatioUV == 1) {
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint8_t, 8>;
        vertical_diff_chroma = useSSE2 ? vertical_diff_chroma_core_sse2<uint8_t, 8> : vertical_diff_chroma_core_C<uint8_t, 8>;
      }
      else { // 2
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint8_t, 4>;
        vertical_diff_chroma = useSSE2 ? vertical_diff_chroma_core_sse2<uint8_t, 4> : vertical_diff_chroma_core_C<uint8_t, 4>;
      }

      postprocessing_grey = &Postprocessing::postprocessing_grey_core<uint8_t>;
      if (_xRatioUV == 1) {// YV24
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint8_t, 8> : horizontal_diff_chroma_C<uint8_t, 8>;
      }
      else {
        // YV16, YV12
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint8_t, 4> : horizontal_diff_chroma_C<uint8_t, 4>;
      }

      if (_xRatioUV == 1 && _yRatioUV == 1) { // 4:4:4 YV24
        copy_chroma = copy_chroma_core<8, 8, uint8_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint8_t, 8, 8>;
        show_motion = &Postprocessing::show_motion_core<uint8_t, 8, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 1) {
        // 4:2:2 YV16 YUY2
        copy_chroma = copy_chroma_core<4, 8, uint8_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint8_t, 4, 8>;
        show_motion = &Postprocessing::show_motion_core<uint8_t, 4, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 2) { // 4:2:0 YV12
        copy_chroma = copy_chroma_core<4, 4, uint8_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint8_t, 4, 4>;
        show_motion = &Postprocessing::show_motion_core<uint8_t, 4, 4>;
      }
    }
    else {
      // 10-16 bits
      vertical_diff_C = vertical_diff_core_C<uint16_t>;
      vertical_diff = useSSE41 ? vertical_diff_uint16_sse4 : useSSE2 ? vertical_diff_sse2<uint16_t> : vertical_diff_core_C<uint16_t>;
      copy_luma = copy_luma_C<8, 8, uint16_t>;
      
      horizontal_diff = useSSE2 ? horizontal_diff_simd<uint16_t> : horizontal_diff_C<uint16_t>;

      if (_yRatioUV == 1) {
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint16_t, 8>;
        vertical_diff_chroma = useSSE41 ? vertical_diff_chroma_core_uint16_sse4<8> : useSSE2 ? vertical_diff_chroma_core_sse2<uint16_t, 8> : vertical_diff_chroma_core_C<uint16_t, 8>;
      }
      else { // 2
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint16_t, 4>;
        vertical_diff_chroma = useSSE41 ? vertical_diff_chroma_core_uint16_sse4<4> : useSSE2 ? vertical_diff_chroma_core_sse2<uint16_t, 4> : vertical_diff_chroma_core_C<uint16_t, 4>;
      }

      postprocessing_grey = &Postprocessing::postprocessing_grey_core<uint16_t>;
      if (_xRatioUV == 1) { // YV24
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint16_t, 8> : horizontal_diff_chroma_C<uint16_t, 8>;
      }
      else {// YV16, YV12
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint16_t, 4> : horizontal_diff_chroma_C<uint16_t, 4>;
      }

      if (_xRatioUV == 1 && _yRatioUV == 1) { // 4:4:4 YV24
        copy_chroma = copy_chroma_core<8, 8, uint16_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint16_t, 8, 8>;
        show_motion = &Postprocessing::show_motion_core<uint16_t, 8, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 1) { // 4:2:2 YV16 YUY2
        copy_chroma = copy_chroma_core<4, 8, uint16_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint16_t, 4, 8>;
        show_motion = &Postprocessing::show_motion_core<uint16_t, 4, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 2) { // 4:2:0 YV12
        copy_chroma = copy_chroma_core<4, 4, uint16_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint16_t, 4, 4>;
        show_motion = &Postprocessing::show_motion_core<uint16_t, 4, 4>;
      }
    }

    linewidthUV = linewidth / _xRatioUV;
  }
};


class RemoveDirt : public Postprocessing, public AccessFrame
{
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env);
  bool grey;
  bool show;
  int       blocks;
public:

  int   ProcessFrame(PVideoFrame &dest, PVideoFrame &src, PVideoFrame &previous, PVideoFrame &next, int frame);


  RemoveDirt(int _width, int _height, int dist, int tolerance, int dmode, uint32_t threshold, int noise, int noisy, bool yuy2, int pthreshold, int cthreshold, bool _grey, bool _show, bool debug, 
    int _xRatioUV, int _yRatioUV, int _bits_per_pixel, IScriptEnvironment* env)
    : Postprocessing(_width, _height, dist, tolerance, dmode, threshold, noise, noisy, pthreshold, cthreshold, _xRatioUV, _yRatioUV, _bits_per_pixel, env)
    , AccessFrame(_width, yuy2), grey(_grey), show(_show)
  {
    blocks = debug ? (hblocks * vblocks) : 0;
  }
};

int RemoveDirt::ProcessFrame(PVideoFrame &dest, PVideoFrame &src, PVideoFrame &previous, PVideoFrame &next, int frame)
{
  const BYTE *nextY = GetReadPtrY(next);
  int   nextPitchY = GetPitchY(next);
  if (bits_per_pixel == 8)
    markblocks<uint8_t>(GetReadPtrY(previous), GetPitchY(previous), nextY, nextPitchY);
  else
    markblocks<uint16_t>(GetReadPtrY(previous), GetPitchY(previous), nextY, nextPitchY);

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

  if (grey) 
    (*this.*postprocessing_grey)(destY, destPitchY, srcY, srcPitchY);
  else 
    (*this.*postprocessing)(destY, destPitchY, destU, destV, destPitchUV, srcY, srcPitchY, srcU, srcV, srcPitchUV);

  if (show)
    (*this.*show_motion)(destU, destV, destPitchUV);

  if (blocks) debug_printf("[%u] RemoveDirt: motion blocks = %4u(%2u%%), %4i(%2i%%), %4u(%2u%%), loops = %u\n", frame, motionblocks, (motionblocks * 100) / blocks
    , distblocks, (distblocks * 100) / (int)blocks, restored_blocks, (restored_blocks * 100) / blocks, loops);

  return restored_blocks + distblocks + motionblocks;
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
  PClip after;
  PClip before;
  PClip alternative;
  int       lastframe;
  int       before_offset, after_offset;

  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE /*MT_SERIALIZED*/ : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override
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

  if (vi.BitsPerComponent() > 16)
    env->ThrowError("RemoveDirt: only 8-16 bit color spaces are supported");

  if (vi.IsYUY2() && !args[PLANAR].AsBool(false))
    env->ThrowError("RemoveDirt: native YUY2 not supported, use YV16 or the planar hack with planar=true");

  if (vi.IsYV411())
    env->ThrowError("RemoveDirt: YV411 not supported");

  if (!vi.IsY() && !vi.IsYUV() && !vi.IsYUVA())
    env->ThrowError("RemoveDirt: only Y, planar YUV(A) and planar YUY2 clips are supported");

  int   pthreshold = args[PTHRES].AsInt(DEFAULT_PTHRESHOLD);

  int xRatioUV = 1;
  int yRatioUV = 1;
  if (vi.IsYUY2()) {
    xRatioUV = 2;
    yRatioUV = 1;
  } 
  else if (!vi.IsY()) {
    xRatioUV = 1 << vi.GetPlaneWidthSubsampling(PLANAR_U);
    yRatioUV = 1 << vi.GetPlaneHeightSubsampling(PLANAR_U);
  }

  const bool grey = vi.IsY() || args[GREY].AsBool(false); // fallback to compulsory grey mode

  filter->rd = new RemoveDirt(vi.width, vi.height, args[DIST].AsInt(DEFAULT_DIST), args[TOLERANCE].AsInt(DEFAULT_TOLERANCE), args[DMODE].AsInt(0)
    , args[MTHRES].AsInt(DEFAULT_MTHRESHOLD), args[NOISE].AsInt(0), args[NOISY].AsInt(-1), vi.IsYUY2()
    , pthreshold, args[CTHRES].AsInt(pthreshold)
    , grey, args[SHOW].AsBool(false), args[DEBUG].AsBool(false), xRatioUV, yRatioUV, vi.BitsPerComponent(), env);


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


template<typename pixel_t>
static int64_t calculate_sad_c(const BYTE* cur_ptr, int cur_pitch, const BYTE* other_ptr, int other_pitch, int32_t width, int32_t height)
{
  const pixel_t *ptr1 = reinterpret_cast<const pixel_t *>(cur_ptr);
  const pixel_t *ptr2 = reinterpret_cast<const pixel_t *>(other_ptr);
  cur_pitch /= sizeof(pixel_t);
  other_pitch /= sizeof(pixel_t);

  // for fullframe float may lose precision
  typedef typename std::conditional < std::is_floating_point<pixel_t>::value, double, __int64>::type sum_t;
  // for one row int is enough and faster than int64
  typedef typename std::conditional < std::is_floating_point<pixel_t>::value, float, int>::type sumrow_t;
  sum_t sum = 0;

  for (int y = 0; y < height; ++y) {
    sumrow_t sumrow = 0;
    for (int x = 0; x < width; ++x) {
      sumrow += std::abs(ptr1[x] - ptr2[x]);
    }
    sum += sumrow;
    ptr1 += cur_pitch;
    ptr2 += other_pitch;
  }
  if (std::is_floating_point<pixel_t>::value)
    return (int64_t)(sum * 255); // scale 0..1 based sum to 8 bit range
  else
    return (int64_t)sum; // for int, scaling to 8 bit range is done outside
}

// works for uint8_t, but there is a specific, bit faster function above
template<typename pixel_t>
int64_t calculate_sad_8_or_16_sse2(const BYTE* cur_ptr, int cur_pitch, const BYTE* other_ptr, int other_pitch, int32_t width, int32_t height)
{
  const int rowsize = width * sizeof(pixel_t);
  int mod16_width = rowsize / 16 * 16;

  __m128i zero = _mm_setzero_si128();
  int64_t totalsum = 0; // fullframe SAD exceeds int32 at 8+ bit

  for (int y = 0; y < height; y++)
  {
    __m128i sum = _mm_setzero_si128(); // for one row int is enough
    for (int x = 0; x < mod16_width; x += 16)
    {
      __m128i src1, src2;
      src1 = _mm_load_si128((__m128i *) (cur_ptr + x));   // 16 bytes or 8 words
      src2 = _mm_load_si128((__m128i *) (other_ptr + x));
      if constexpr (sizeof(pixel_t) == 1) {
        sum = _mm_add_epi32(sum, _mm_sad_epu8(src1, src2)); // sum0_32, 0, sum1_32, 0
      }
      else if constexpr (sizeof(pixel_t) == 2) {
        __m128i greater_t = _mm_subs_epu16(src1, src2); // unsigned sub with saturation
        __m128i smaller_t = _mm_subs_epu16(src2, src1);
        __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))
        // 8 x uint16 absolute differences
        sum = _mm_add_epi32(sum, _mm_unpacklo_epi16(absdiff, zero));
        sum = _mm_add_epi32(sum, _mm_unpackhi_epi16(absdiff, zero));
        // sum0_32, sum1_32, sum2_32, sum3_32
      }
    }
    // summing up partial sums
    if constexpr (sizeof(pixel_t) == 2) {
      // at 16 bits: we have 4 integers for sum: a0 a1 a2 a3
      __m128i a0_a1 = _mm_unpacklo_epi32(sum, zero); // a0 0 a1 0
      __m128i a2_a3 = _mm_unpackhi_epi32(sum, zero); // a2 0 a3 0
      sum = _mm_add_epi32(a0_a1, a2_a3); // a0+a2, 0, a1+a3, 0
    }

    // sum here: two 32 bit partial result: sum1 0 sum2 0
    __m128i sum_hi = _mm_unpackhi_epi64(sum, zero);
    // or: __m128i sum_hi = _mm_castps_si128(_mm_movehl_ps(_mm_setzero_ps(), _mm_castsi128_ps(sum)));
    sum = _mm_add_epi32(sum, sum_hi);
    int rowsum = _mm_cvtsi128_si32(sum);

    // rest
    if (mod16_width != rowsize) {
      for (size_t x = mod16_width / sizeof(pixel_t); x < rowsize / sizeof(pixel_t); ++x)
        rowsum += std::abs(reinterpret_cast<const pixel_t *>(cur_ptr)[x] - reinterpret_cast<const pixel_t *>(other_ptr)[x]);
    }

    totalsum += rowsum;

    cur_ptr += cur_pitch;
    other_ptr += other_pitch;
  }
  return totalsum;
}

// SCSelect helper
static int64_t gdiff(const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, int width, int height, int bits_per_pixel,bool useSSE2)
{
  if (bits_per_pixel == 8)
    if(useSSE2)
      return calculate_sad_8_or_16_sse2<uint8_t>(sp1, spitch1, sp2, spitch2, width, height);
    else
      return calculate_sad_c<uint8_t>(sp1, spitch1, sp2, spitch2, width, height);
  else if(bits_per_pixel <= 16)
    if (useSSE2)
      return calculate_sad_8_or_16_sse2<uint16_t>(sp1, spitch1, sp2, spitch2, width, height);
    else
      return calculate_sad_c<uint16_t>(sp1, spitch1, sp2, spitch2, width, height);
  else {
    return calculate_sad_c<float>(sp1, spitch1, sp2, spitch2, width, height);
  }
}

#define SPOINTER(p) p.operator->()

class   SCSelect : public GenericVideoFilter, public AccessFrame
{
  PClip scene_begin;
  PClip scene_end;
  PClip global_motion;
  int64_t lastdiff; // 8k image dimensions require int64 even at 8 bits
  double dirmult;
  bool debug;
  uint32_t lnr;

  bool useSSE2;

  // lastdiff and lnr variable: not MT friendly
  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_SERIALIZED : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override
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
      const bool isRGB = vi.IsRGB();
      PVideoFrame sf = child->GetFrame(n, env);
      if (lnr != n - 1)
      {
        PVideoFrame pf = child->GetFrame(n - 1, env);
        if (isRGB) {
          const int planes[4] = { PLANAR_R, PLANAR_G, PLANAR_B, PLANAR_A };

          lastdiff = 0;
          // RGB: sum all planes
          for (int p = 0; p < 3; p++)
          {
            int plane = planes[p];
            lastdiff += gdiff(sf->GetReadPtr(plane), sf->GetPitch(plane), pf->GetReadPtr(plane), pf->GetPitch(plane), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
          }
        }
        else {
          lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(pf), GetPitchY(pf), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
        }
      }
      int64_t olddiff = lastdiff;
      {
        PVideoFrame nf = child->GetFrame(n + 1, env);
        if (isRGB) {
          // RGB: sum all planes
          const int planes[4] = { PLANAR_R, PLANAR_G, PLANAR_B, PLANAR_A };

          lastdiff = 0;
          for (int p = 0; p < 3; p++)
          {
            int plane = planes[p];
            lastdiff += gdiff(sf->GetReadPtr(plane), sf->GetPitch(plane), nf->GetReadPtr(plane), nf->GetPitch(plane), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
          }
        }
        else {
          lastdiff = gdiff(GetReadPtrY(sf), GetPitchY(sf), GetReadPtrY(nf), GetPitchY(nf), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
        }
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
    // all bit-depths are supported

    if (vi.IsYUY2() && !planar)
      env->ThrowError("SCSelect: native YUY2 not supported, use YV16 or the planar hack with planar=true");

    if (!vi.IsY() && !vi.IsYUV() && !vi.IsYUVA() && !vi.IsPlanarRGB() && !vi.IsPlanarRGBA())
      env->ThrowError("SCSelect: only Y, YUV(A), planar RGB(A) and planar YUY2 clips are supported");

    CompareVideoInfo(vi, scene_begin->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, scene_end->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, global_motion->GetVideoInfo(), "SCSelect", env);
    
    scene_begin->SetCacheHints(CACHE_GENERIC, 0);
    scene_end->SetCacheHints(CACHE_GENERIC, 0);
    if (gcache >= 0) global_motion->SetCacheHints(CACHE_GENERIC, 0);
    if (cache >= 0) child->SetCacheHints(CACHE_GENERIC, cache);

    useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
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
  env->AddFunction("SCSelect", "cccc[dfactor]f[debug]b[planar]b[cache]i[gcache]i", CreateSCSelect, 0);
  env->AddFunction("RestoreMotionBlocks", creatstr, CreateRestoreMotionBlocks, 0);
  debug_printf(LOGO);
  return NULL;
}
#endif  // RANGEFILES