// Avisynth and Vapoursynth filter for removing dirt from film clips
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

#ifdef INTEL_INTRINSICS
#include "smmintrin.h" // SSE4.1 even if not defined (clang)
#include "emmintrin.h"
#include "immintrin.h"
#endif
#include <cstdint>
#include <cassert>
#include <cmath>

//
// Part 1: options at compile time
//

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

#include "avisynth.h"

#ifdef _WIN32
#define VC_EXTRALEAN
#include <Windows.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include "common.h"

#ifdef INTEL_INTRINSICS
// important define for VapourSynth cpufeatures
#define VS_TARGET_CPU_X86 1
#endif

#include "VapourSynth4.h"
#include "VSHelper4.h"
#include "cpufeatures.h"

#include <memory>

class RemoveDirt; // forward

// General parameters which are used in both Avisynth and Vapoursynth

struct RestoreMotionBlocksShared {
  int mthreshold;
  int lastframe;
  int before_offset, after_offset;
  RemoveDirt* rd;

  ~RestoreMotionBlocksShared() {
    // free up any malloc'd buffer;
  }
};

/****************************************************
* C functions
****************************************************/

AVS_FORCEINLINE unsigned int SADABS(int x) { return (x < 0) ? -x : x; }

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
static AVS_FORCEINLINE uint32_t Sad_C(const uint8_t *pSrc, intptr_t nSrcPitch, const uint8_t *pRef, intptr_t nRefPitch)
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
uint32_t SADcompare_C(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int /*noise*/)
{
  return Sad_C<8, 8, pixel_t>(p1, pitch1, p2, pitch2);
}

template<typename pixel_t, int blksizeX, int blksizeY>
uint32_t NSADcompare_C(const BYTE *p1_8, intptr_t pitch1, const BYTE *p2_8, intptr_t pitch2, int noise)
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
  return res;
}

template<typename pixel_t, int blksizeX, int blksizeY>
uint32_t ExcessPixels_C(const BYTE *p1_8, intptr_t pitch1, const BYTE *p2_8, intptr_t pitch2, int noise)
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
  return count;
}

/****************************************************
* End of C functions
****************************************************/

/****************************************************
* SIMD functions
****************************************************/

uint32_t SADcompare_simd(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int /*noise*/) {
  // optimizer makes it fast for SIMD
  return Sad_C<8, 8, uint8_t>(p1, pitch1, p2, pitch2);
}

#ifdef INTEL_INTRINSICS
static AVS_FORCEINLINE __m128i _mm_loadh_epi64(__m128i x, __m128i *p)
{
  return _mm_castpd_si128(_mm_loadh_pd(_mm_castsi128_pd(x), (double *)p));
}

uint32_t NSADcompare_simd(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int noise)
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
#endif

#ifdef INTEL_INTRINSICS
uint32_t ExcessPixels_simd(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int noise)
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
#endif
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
  uint32_t (*blockcompare)(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int noise);
  int   hblocks, vblocks;

  int bits_per_pixel;

  uint32_t (*blockcompare_C)(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2, int noise);

  template<typename pixel_t>
  void  markblocks(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2)
  {
    constexpr int pixelsize = sizeof(pixel_t);

    motionblocks = 0;

    intptr_t inc1 = MOTIONBLOCKHEIGHT * pitch1 - linewidth * pixelsize;
    intptr_t inc2 = MOTIONBLOCKHEIGHT * pitch2 - linewidth * pixelsize;
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

  MotionDetection(int   width, int height, uint32_t _threshold, int _noise, int _noisy, int _bits_per_pixel, bool useSSE2) : threshold(_threshold), noise(_noise), bits_per_pixel(_bits_per_pixel)
  {
    linewidth = width;
    hblocks = width / MOTIONBLOCKWIDTH;
    vblocks = height / MOTIONBLOCKHEIGHT;

    if (bits_per_pixel == 8) {
      blockcompare_C = SADcompare_C<uint8_t, 8, 8>;
#ifdef INTEL_INTRINSICS
      blockcompare = useSSE2 ? SADcompare_simd : SADcompare_C<uint8_t, 8, 8>;
#else
      blockcompare = blockcompare_C;
#endif
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
#ifdef INTEL_INTRINSICS
        blockcompare = useSSE2 ? NSADcompare_simd : NSADcompare_C<uint8_t, 8, 8>;
#else
        blockcompare = blockcompare_C;
#endif
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
#ifdef INTEL_INTRINSICS
          blockcompare = useSSE2 ? ExcessPixels_simd : ExcessPixels_C<uint8_t, 8, 8>;
#else
          blockcompare = blockcompare_C;
#endif
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
  void  markblocks(const BYTE *p1, intptr_t pitch1, const BYTE *p2, intptr_t pitch2)
  {
    MotionDetection::markblocks<pixel_t>(p1, pitch1, p2, pitch2);
    distblocks = 0;
    if (dist)
    {
      markneighbours();
      (this->*processneighbours)();
    }
  }

  MotionDetectionDist(int   width, int height, int _dist, int _tolerance, int dmode, uint32_t _threshold, int _noise, int _noisy, int _bits_per_pixel, bool useSSE2)
    : MotionDetection(width, height, _threshold, _noise, _noisy, _bits_per_pixel, useSSE2)
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
uint32_t horizontal_diff_C(const uint8_t *p, intptr_t pitch)
{
  return Sad_C<8, 1, pixel_t>(p, pitch, p + pitch, pitch);
}

#ifdef INTEL_INTRINSICS

template<typename pixel_t>
uint32_t horizontal_diff_simd(const uint8_t *p, intptr_t pitch)
{
  // 8 pixels
  if constexpr (sizeof(pixel_t) == 1) {
    auto src1 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p));
    auto src2 = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(p + pitch));
    return _mm_cvtsi128_si32(_mm_sad_epu8(src1, src2)); // 8 pixels, lower sad result
  }
  else {
    // 10-16 bits
    auto src1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p));
    auto src2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p + pitch));

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
#endif

template<typename pixel_t, int blksizeX>
uint32_t horizontal_diff_chroma_C(const uint8_t *u, const uint8_t *v, intptr_t pitch)
{
  assert(blksizeX == 4 || blksizeX == 8);
  return Sad_C<blksizeX, 1, pixel_t>(u, pitch, u + pitch, pitch) + Sad_C<4, 1, pixel_t>(v, pitch, v + pitch, pitch);
  // as of v0.9.2 no internal downscaling occurs for YV24 (for having the subsampled YV12 or YV16 result), cthreshold_h is scaled instead
}

#ifdef INTEL_INTRINSICS
template<typename pixel_t, int blksizeX>
uint32_t horizontal_diff_chroma_simd(const uint8_t *u, const uint8_t *v, intptr_t pitch)
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
      auto src1_u = _mm_loadu_si128(reinterpret_cast<const __m128i *>(u));
      auto src2_u = _mm_loadu_si128(reinterpret_cast<const __m128i *>(u + pitch));

      // make 16bit SAD
      __m128i greater_t = _mm_subs_epu16(src1_u, src2_u); // unsigned sub with saturation
      __m128i smaller_t = _mm_subs_epu16(src2_u, src1_u);
      __m128i absdiff = _mm_or_si128(greater_t, smaller_t); //abs(s1-s2)  == (satsub(s1,s2) | satsub(s2,s1))

      auto zero = _mm_setzero_si128();
      // 8 x uint16 absolute differences
      auto sum = _mm_add_epi32(_mm_unpacklo_epi16(absdiff, zero), _mm_unpackhi_epi16(absdiff, zero));
      // sum0_32, sum1_32, sum2_32, sum3_32

      // v
      auto src1_v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(v));
      auto src2_v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(v + pitch));

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
#endif

#ifdef INTEL_INTRINSICS
template<typename pixel_t>
static uint32_t vertical_diff_sse2(const uint8_t *p, intptr_t pitch)
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
#if defined(GCC) || defined(CLANG)
__attribute__((__target__("sse4.1")))
#endif
static uint32_t vertical_diff_uint16_sse4(const uint8_t *p, intptr_t pitch)
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

// YUV420: vertical_diff_chroma<4>
// others: vertical_diff_chroma<8>
template<typename pixel_t, int blksizeY>
uint32_t vertical_diff_chroma_core_sse2(const uint8_t *u, const uint8_t *v, intptr_t pitch)
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
      // no vertical subsampling: planar 422, 444
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
#endif

#ifdef INTEL_INTRINSICS
// SSE4.1: needed only for uint16_t
template<int blksizeY>
#if defined(GCC) || defined(CLANG)
__attribute__((__target__("sse4.1")))
#endif
uint32_t vertical_diff_chroma_core_uint16_sse4(const uint8_t *u, const uint8_t *v, intptr_t pitch)
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
#endif

template<typename pixel_t>
uint32_t vertical_diff_core_C(const uint8_t *p8, intptr_t pitch)
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
  return    res;
}

// VUV420: vertical_diff_chroma<4>
// others: vertical_diff_chroma<8>
template<typename pixel_t, int blksizeY>
uint32_t vertical_diff_chroma_core_C(const uint8_t *u8, const uint8_t *v8, intptr_t pitch)
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
  return    res;
}

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
AVS_FORCEINLINE void Copy_C(uint8_t *pDst, intptr_t nDstPitch, const uint8_t *pSrc, intptr_t nSrcPitch)
{
  for (int j = 0; j < nBlkHeight; j++)
  {
    memcpy(pDst, pSrc, nBlkWidth * sizeof(pixel_t));
    pDst += nDstPitch;
    pSrc += nSrcPitch;
  }
}

template<int nBlkWidth, int nBlkHeight, typename pixel_t>
AVS_FORCEINLINE void copy_luma_C(BYTE *dest, intptr_t dpitch, const BYTE *src, intptr_t spitch)
{
  Copy_C<8, 8, pixel_t>(dest, dpitch, src, spitch);
}

// YUV420: copy_chroma_core<4,4>
// YUV422: copy_chroma_core<4,8>
// YUV444: copy_chroma_core<8,8>
template<int blksizeX, int blksizeY, typename pixel_t>
void copy_chroma_core(BYTE *destu, BYTE *destv, intptr_t dpitch, const BYTE *srcu, const BYTE *srcv, intptr_t spitch)
{
  Copy_C<blksizeX, blksizeY, pixel_t>(destu, dpitch, srcu, spitch);
  Copy_C<blksizeX, blksizeY, pixel_t>(destv, dpitch, srcv, spitch);
}

template<int blksizeY>
void inline colorise(BYTE *u, BYTE *v, intptr_t pitch, uint32_t ucolor, uint32_t vcolor)
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

  uint32_t(*vertical_diff)(const uint8_t *p, intptr_t pitch);
  uint32_t(*vertical_diff_C)(const uint8_t *p, intptr_t pitch);
  uint32_t(*horizontal_diff)(const BYTE *p, intptr_t pitch);
  uint32_t(*horizontal_diff_chroma)(const BYTE *u, const BYTE *v, intptr_t pitch);
  void(*copy_luma)(BYTE *dest, intptr_t dpitch, const BYTE *src, intptr_t spitch);

  uint32_t (*vertical_diff_chroma)(const BYTE *u, const BYTE *v, intptr_t pitch);
  void (*copy_chroma)(BYTE *destu, BYTE *destv, intptr_t dpitch, const BYTE *srcu, const BYTE *srcv, intptr_t spitch);
  uint32_t (*vertical_diff_chroma_C)(const BYTE *u, const BYTE *v, intptr_t pitch);

public:
  int       loops;
  int       restored_blocks;

  void(Postprocessing::*postprocessing)(BYTE *dp, intptr_t dpitch, BYTE *dpU, BYTE *dpV, intptr_t dpitchUV, const BYTE *sp, intptr_t spitch, const BYTE *spU, const BYTE *spV, intptr_t spitchUV);
  void(Postprocessing::*postprocessing_grey)(BYTE *dp, intptr_t dpitch, const BYTE *sp, intptr_t spitch);
  void(Postprocessing::*show_motion)(BYTE *u, BYTE *v, intptr_t pitchUV);

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
  void  postprocessing_grey_core(BYTE *dp, intptr_t dpitch, const BYTE *sp, intptr_t spitch)
  {
    constexpr int pixelsize = sizeof(pixel_t); // 1 or 2 bytes

    intptr_t bottomdp = 7 * dpitch;
    intptr_t bottomsp = 7 * spitch;
    intptr_t dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth * pixelsize;
    intptr_t sinc = MOTIONBLOCKHEIGHT * spitch - linewidth * pixelsize;

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
  void  postprocessing_core(BYTE *dp, intptr_t dpitch, BYTE *dpU, BYTE *dpV, intptr_t dpitchUV, const BYTE *sp, intptr_t spitch, const BYTE *spU, const BYTE *spV, intptr_t spitchUV)
  {
    constexpr int pixelsize = sizeof(pixel_t); // 1 or 2 bytes

    constexpr int Cleftdp = -1;
    constexpr int Crightdp = blksizeXchroma - 1;
    constexpr int Cleftsp = Cleftdp;
    constexpr int Crightsp = Crightdp;

    const intptr_t Ctopdp = -dpitchUV;
    const intptr_t Ctopsp = -spitchUV;

    const intptr_t bottomdp = 7 * dpitch;
    const intptr_t bottomsp = 7 * spitch;
    const intptr_t Cbottomdp = (blksizeYchroma - 1) * dpitchUV;
    const intptr_t Cbottomsp = (blksizeYchroma - 1) * spitchUV;
    intptr_t dinc = MOTIONBLOCKHEIGHT * dpitch - linewidth * pixelsize;
    intptr_t sinc = MOTIONBLOCKHEIGHT * spitch - linewidth * pixelsize;
    intptr_t dincUV = blksizeYchroma * dpitchUV - linewidthUV * pixelsize;
    intptr_t sincUV = blksizeYchroma * spitchUV - linewidthUV * pixelsize;

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
  void  show_motion_core(BYTE *u, BYTE *v, intptr_t pitchUV)
  {
    intptr_t inc = blksizeYchroma * pitchUV - linewidthUV * sizeof(pixel_t);

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
    int _xRatioUV, int _yRatioUV, int _bits_per_pixel, bool useSSE2, bool useSSE41)
    : MotionDetectionDist(width, height, dist, tolerance, dmode, 
      threshold << (_bits_per_pixel - 8), 
      _noise << (_bits_per_pixel - 8), 
      _noisy, // counter, do not scale
      _bits_per_pixel,
      useSSE2)
    , 
    pthreshold(_pthreshold << (_bits_per_pixel - 8)), 
    cthreshold_h((_cthreshold << (_bits_per_pixel - 8)) * 2 / _xRatioUV),
    cthreshold_v((_cthreshold << (_bits_per_pixel - 8)) * 2 / _yRatioUV)
  {

    if (_bits_per_pixel == 8) {
      copy_luma = copy_luma_C<8, 8, uint8_t>;

      vertical_diff_C = vertical_diff_core_C<uint8_t>;
#ifdef INTEL_INTRINSICS
      vertical_diff = useSSE2 ? vertical_diff_sse2<uint8_t> : vertical_diff_core_C<uint8_t>;
#else
      vertical_diff = vertical_diff_C;
#endif

#ifdef INTEL_INTRINSICS
      horizontal_diff = useSSE2 ? horizontal_diff_simd<uint8_t> : horizontal_diff_C<uint8_t>;
#else
      horizontal_diff = horizontal_diff_C<uint8_t>;
#endif

      if (_yRatioUV == 1) {
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint8_t, 8>;
#ifdef INTEL_INTRINSICS
        vertical_diff_chroma = useSSE2 ? vertical_diff_chroma_core_sse2<uint8_t, 8> : vertical_diff_chroma_core_C<uint8_t, 8>;
#else
        vertical_diff_chroma = vertical_diff_chroma_C;
#endif
      }
      else { // 2
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint8_t, 4>;
#ifdef INTEL_INTRINSICS
        vertical_diff_chroma = useSSE2 ? vertical_diff_chroma_core_sse2<uint8_t, 4> : vertical_diff_chroma_core_C<uint8_t, 4>;
#else
        vertical_diff_chroma = vertical_diff_chroma_C;
#endif
      }

      postprocessing_grey = &Postprocessing::postprocessing_grey_core<uint8_t>;
      if (_xRatioUV == 1) {// YV24
#ifdef INTEL_INTRINSICS
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint8_t, 8> : horizontal_diff_chroma_C<uint8_t, 8>;
#else
        horizontal_diff_chroma = horizontal_diff_chroma_C<uint8_t, 8>;
#endif
      }
      else {
#ifdef INTEL_INTRINSICS
        // YV16, YV12
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint8_t, 4> : horizontal_diff_chroma_C<uint8_t, 4>;
#else
        horizontal_diff_chroma = horizontal_diff_chroma_C<uint8_t, 4>;
#endif
      }

      if (_xRatioUV == 1 && _yRatioUV == 1) { // 4:4:4 YV24
        copy_chroma = copy_chroma_core<8, 8, uint8_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint8_t, 8, 8>;
        show_motion = &Postprocessing::show_motion_core<uint8_t, 8, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 1) {
        // 4:2:2 YV16
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
#ifdef INTEL_INTRINSICS
      vertical_diff = useSSE41 ? vertical_diff_uint16_sse4 : useSSE2 ? vertical_diff_sse2<uint16_t> : vertical_diff_core_C<uint16_t>;
#else
      vertical_diff = vertical_diff_C;
#endif
      copy_luma = copy_luma_C<8, 8, uint16_t>;

#ifdef INTEL_INTRINSICS
      horizontal_diff = useSSE2 ? horizontal_diff_simd<uint16_t> : horizontal_diff_C<uint16_t>;
#else
      horizontal_diff = horizontal_diff_C<uint16_t>;
#endif

      if (_yRatioUV == 1) {
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint16_t, 8>;
#ifdef INTEL_INTRINSICS
        vertical_diff_chroma = useSSE41 ? vertical_diff_chroma_core_uint16_sse4<8> : useSSE2 ? vertical_diff_chroma_core_sse2<uint16_t, 8> : vertical_diff_chroma_core_C<uint16_t, 8>;
#else
        vertical_diff_chroma = vertical_diff_chroma_C;
#endif
      }
      else { // 2 YV16 Yv12
        vertical_diff_chroma_C = vertical_diff_chroma_core_C<uint16_t, 4>;
#ifdef INTEL_INTRINSICS
        vertical_diff_chroma = useSSE41 ? vertical_diff_chroma_core_uint16_sse4<4> : useSSE2 ? vertical_diff_chroma_core_sse2<uint16_t, 4> : vertical_diff_chroma_core_C<uint16_t, 4>;
#else
        vertical_diff_chroma = vertical_diff_chroma_C;
#endif
      }

      postprocessing_grey = &Postprocessing::postprocessing_grey_core<uint16_t>;
      if (_xRatioUV == 1) { // YV24
#ifdef INTEL_INTRINSICS
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint16_t, 8> : horizontal_diff_chroma_C<uint16_t, 8>;
#else
        horizontal_diff_chroma = horizontal_diff_chroma_C<uint16_t, 8>;
#endif
      }
      else {// YV16, YV12
#ifdef INTEL_INTRINSICS
        horizontal_diff_chroma = useSSE2 ? horizontal_diff_chroma_simd<uint16_t, 4> : horizontal_diff_chroma_C<uint16_t, 4>;
#else
        horizontal_diff_chroma = horizontal_diff_chroma_C<uint16_t, 4>;
#endif
      }

      if (_xRatioUV == 1 && _yRatioUV == 1) { // 4:4:4 YV24
        copy_chroma = copy_chroma_core<8, 8, uint16_t>;
        postprocessing = &Postprocessing::postprocessing_core<uint16_t, 8, 8>;
        show_motion = &Postprocessing::show_motion_core<uint16_t, 8, 8>;
      }
      else if (_xRatioUV == 2 && _yRatioUV == 1) { // 4:2:2 YV16
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

struct processFrameParams {
  const BYTE* nextY;
  ptrdiff_t nextPitchY;
  const BYTE* previousY;
  ptrdiff_t previousPitchY;
  BYTE* destY;
  BYTE* destU;
  BYTE* destV;
  ptrdiff_t destPitchY;
  ptrdiff_t destPitchUV;
  const BYTE* srcY;
  const BYTE* srcU;
  const BYTE* srcV;
  ptrdiff_t srcPitchY;
  ptrdiff_t srcPitchUV;
};

class RemoveDirt : public Postprocessing
{
  bool show;
  int blocks;

public:

  bool grey;
  int   ProcessFrame(processFrameParams& frameParams, int frame);

  RemoveDirt(int _width, int _height, int dist, int tolerance, int dmode, uint32_t threshold, int noise, int noisy, int pthreshold, int cthreshold, bool _grey, bool _show, bool debug, 
    int _xRatioUV, int _yRatioUV, int _bits_per_pixel, bool useSSE2, bool useSSE41)
    : Postprocessing(_width, _height, dist, tolerance, dmode, threshold, noise, noisy, pthreshold, cthreshold, _xRatioUV, _yRatioUV, _bits_per_pixel, useSSE2, useSSE41)
    , show(_show), grey(_grey)
  {
    blocks = debug ? (hblocks * vblocks) : 0;
  }
};

int RemoveDirt::ProcessFrame(processFrameParams &frameParams, int frame)
{

  if (bits_per_pixel == 8)
    markblocks<uint8_t>(frameParams.previousY, frameParams.previousPitchY, frameParams.nextY, frameParams.nextPitchY);
  else
    markblocks<uint16_t>(frameParams.previousY, frameParams.previousPitchY, frameParams.nextY, frameParams.nextPitchY);

  if (grey) 
    (*this.*postprocessing_grey)(frameParams.destY, frameParams.destPitchY, frameParams.srcY, frameParams.srcPitchY);
  else 
    (*this.*postprocessing)(frameParams.destY, frameParams.destPitchY, frameParams.destU, frameParams.destV, frameParams.destPitchUV, frameParams.srcY, frameParams.srcPitchY, frameParams.srcU, frameParams.srcV, frameParams.srcPitchUV);

  if (show)
    (*this.*show_motion)(frameParams.destU, frameParams.destV, frameParams.destPitchUV);

  if (blocks) debug_printf("[%u] RemoveDirt: motion blocks = %4u(%2u%%), %4i(%2i%%), %4u(%2u%%), loops = %u\n", frame, motionblocks, (motionblocks * 100) / blocks
    , distblocks, (distblocks * 100) / (int)blocks, restored_blocks, (restored_blocks * 100) / blocks, loops);

  return restored_blocks + distblocks + motionblocks;
}

// Avisynth only
class RestoreMotionBlocks : public GenericVideoFilter, private RestoreMotionBlocksShared
{
  friend AVSValue InitRemoveDirt(class RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env);
protected:

  PClip restore;
  PClip after;
  PClip before;
  PClip alternative;

  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override;

public:
  RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative, IScriptEnvironment* env);

  ~RestoreMotionBlocks()
  {
    delete rd;
  }
};

// Avisynth only
// keep them equivalent: RestoreMotionBlocks::GetFrame (Avisynth) restoreMotionBlocksGetFrame (VapourSynth)
PVideoFrame RestoreMotionBlocks::GetFrame(int n, IScriptEnvironment* env)
{
  if ((n + before_offset < 0) || (n + after_offset > lastframe)) return alternative->GetFrame(n, env);
  PVideoFrame pf = before->GetFrame(n + before_offset, env);
  PVideoFrame df = child->GetFrame(n, env); // src, the filtered
  PVideoFrame rf = restore->GetFrame(n, env);
  PVideoFrame nf = after->GetFrame(n + after_offset, env);
  env->MakeWritable(&df); // preserves frame properties of v8

  processFrameParams frameParams;

  frameParams.nextY = nf->GetReadPtr(PLANAR_Y);
  frameParams.nextPitchY = nf->GetPitch(PLANAR_Y);
  frameParams.previousY = pf->GetReadPtr(PLANAR_Y);
  frameParams.previousPitchY = pf->GetPitch(PLANAR_Y);
  frameParams.destY = df->GetWritePtr(PLANAR_Y);
  frameParams.destPitchY = df->GetPitch(PLANAR_Y);
  frameParams.srcY = rf->GetReadPtr(PLANAR_Y);
  frameParams.srcPitchY = rf->GetPitch(PLANAR_Y);
  if (!rd->grey) {
    frameParams.destU = df->GetWritePtr(PLANAR_U);
    frameParams.destV = df->GetWritePtr(PLANAR_V);
    frameParams.destPitchUV = df->GetPitch(PLANAR_U);
    frameParams.srcU = rf->GetReadPtr(PLANAR_U);
    frameParams.srcV = rf->GetReadPtr(PLANAR_V);
    frameParams.srcPitchUV = rf->GetPitch(PLANAR_U);
  }

  if (rd->ProcessFrame(frameParams, n) > mthreshold)
    return alternative->GetFrame(n, env);
  else return df;
}

// Avisynth only 
// VapourSynth See also in restoreMotionBlocksCreate
RestoreMotionBlocks::RestoreMotionBlocks(PClip filtered, PClip _restore, PClip neighbour, PClip neighbour2, PClip _alternative, IScriptEnvironment* env)
  : GenericVideoFilter(filtered), restore(_restore), after(neighbour), before(neighbour2), alternative(_alternative)
{
  // sets frame number limits and replaces undefined (null) PClips with defaults from before/after
  lastframe = vi.num_frames - 1;
  before_offset = after_offset = 0;
  if (after == NULL)
  {
    after = restore;
    before = restore;
    before_offset = -1;
    after_offset = 1;
  }
  if (before == NULL)
  {
    before = after;
    before_offset = -1;
    after_offset = 1;
  }
  if (alternative == NULL)
    alternative = restore;

  CompareVideoInfo(vi, restore->GetVideoInfo(), "RemoveDirt", env);
  CompareVideoInfo(vi, before->GetVideoInfo(), "RemoveDirt", env);
  CompareVideoInfo(vi, after->GetVideoInfo(), "RemoveDirt", env);
}

enum restore_motion_blocks_creatargs { SRC, RESTORE, AFTER, BEFORE, ALTERNATIVE, PLANAR, SHOW, DEBUG, GMTHRES, MTHRES, NOISE, NOISY, DIST, TOLERANCE, DMODE, PTHRES, CTHRES, GREY };

// Avisynth only.
// For VapourSynth see restoreMotionBlocksCreate
AVSValue InitRemoveDirt(RestoreMotionBlocks *filter, AVSValue args, IScriptEnvironment* env)
{
  VideoInfo &vi = filter->vi;

  if (vi.BitsPerComponent() > 16)
    env->ThrowError("RemoveDirt: only 8-16 bit color spaces are supported");

  if (vi.IsYUY2()) {
    env->ThrowError("RemoveDirt: native YUY2 not supported, not even by the planar hack with planar=true");
  }

  if (vi.IsYV411())
    env->ThrowError("RemoveDirt: YV411 not supported");

  if (!vi.IsY() && !vi.IsYUV() && !vi.IsYUVA())
    env->ThrowError("RemoveDirt: only Y and planar YUV(A)");

  int xRatioUV = 1;
  int yRatioUV = 1;
  if (!vi.IsY()) {
    xRatioUV = 1 << vi.GetPlaneWidthSubsampling(PLANAR_U);
    yRatioUV = 1 << vi.GetPlaneHeightSubsampling(PLANAR_U);
  }

  const int dist = args[DIST].AsInt(DEFAULT_DIST);
  const int tolerance = args[TOLERANCE].AsInt(DEFAULT_TOLERANCE);
  const int dmode = args[DMODE].AsInt(0);
  const int mthreshold = args[MTHRES].AsInt(DEFAULT_MTHRESHOLD);
  const int noise = args[NOISE].AsInt(0);
  const int noisy = args[NOISY].AsInt(-1);
  const int pthreshold = args[PTHRES].AsInt(DEFAULT_PTHRESHOLD);
  const int cthreshold = args[CTHRES].AsInt(pthreshold);
  const bool grey = vi.IsY() || args[GREY].AsBool(false); // fallback to compulsory grey mode
  const bool show = args[SHOW].AsBool(false);
  const bool debug = args[DEBUG].AsBool(false);
  const int bitpercomponent = vi.BitsPerComponent();
  const int gmthreshold = args[GMTHRES].AsInt(DEFAULT_GMTHRESHOLD);

#ifdef INTEL_INTRINSICS
  const bool useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
  const bool useSSE41 = (env->GetCPUFlags() & CPUF_SSE4_1) == CPUF_SSE4_1;
#else
  const bool useSSE2 = false;
  const bool useSSE41 = false;
#endif

  filter->rd = new RemoveDirt(vi.width, vi.height, dist, tolerance, dmode
    , mthreshold , noise, noisy
    , pthreshold, cthreshold
    , grey, show, debug, xRatioUV, yRatioUV, bitpercomponent, useSSE2, useSSE41);


  filter->mthreshold = (gmthreshold * filter->rd->hblocks * filter->rd->vblocks) / 100;
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
static int64_t calculate_sad_c(const BYTE* cur_ptr, intptr_t cur_pitch, const BYTE* other_ptr, intptr_t other_pitch, int32_t width, int32_t height)
{
  const pixel_t *ptr1 = reinterpret_cast<const pixel_t *>(cur_ptr);
  const pixel_t *ptr2 = reinterpret_cast<const pixel_t *>(other_ptr);
  cur_pitch /= sizeof(pixel_t);
  other_pitch /= sizeof(pixel_t);

  // for fullframe float may lose precision
  typedef typename std::conditional < std::is_floating_point<pixel_t>::value, double, int64_t>::type sum_t;
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

#ifdef INTEL_INTRINSICS
// works for uint8_t, but there is a specific, bit faster function above
template<typename pixel_t>
int64_t calculate_sad_8_or_16_sse2(const BYTE* cur_ptr, intptr_t cur_pitch, const BYTE* other_ptr, intptr_t other_pitch, int32_t width, int32_t height)
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
#endif


// SCSelect helper
static int64_t gdiff(const BYTE *sp1, intptr_t spitch1, const BYTE *sp2, intptr_t spitch2, int width, int height, int bits_per_pixel,bool useSSE2)
{
  if (bits_per_pixel == 8)
#ifdef INTEL_INTRINSICS
    if(useSSE2)
      return calculate_sad_8_or_16_sse2<uint8_t>(sp1, spitch1, sp2, spitch2, width, height);
    else
#endif
      return calculate_sad_c<uint8_t>(sp1, spitch1, sp2, spitch2, width, height);
  else if(bits_per_pixel <= 16) // VapourSynth: "half" is not supported
#ifdef INTEL_INTRINSICS
    if (useSSE2)
      return calculate_sad_8_or_16_sse2<uint16_t>(sp1, spitch1, sp2, spitch2, width, height);
    else
#endif
      return calculate_sad_c<uint16_t>(sp1, spitch1, sp2, spitch2, width, height);
  else { 
    // VapourSynth: theoretical 32 bit integer is not supported
    return calculate_sad_c<float>(sp1, spitch1, sp2, spitch2, width, height);
  }
}

struct SCSelectShared {
  double dirmult;
  bool debug;
  bool useSSE2;
  bool isRGB;
  int bits_per_pixel;
  int numFrames;

  // These are handled in multithreading friendly way
  int64_t lastdiff; // 8k image dimensions require int64 even at 8 bits
  int lnr;
};

class SCSelect : public GenericVideoFilter, private SCSelectShared
{
  PClip scene_begin;
  PClip scene_end;
  PClip global_motion;

  // lastdiff and lnr variable: not MT friendly
  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_SERIALIZED : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env) override;

public:
  SCSelect(PClip clip, PClip _scene_begin, PClip _scene_end, PClip _global_motion, double dfactor, bool _debug, bool planar, int cache, int gcache, IScriptEnvironment* env)
    : GenericVideoFilter(clip), scene_begin(_scene_begin), scene_end(_scene_end), global_motion(_global_motion)
  {
    dirmult = dfactor;
    debug = _debug;
    lnr = -2;
    lastdiff = 0;
    useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
    isRGB = vi.IsRGB();
    bits_per_pixel = vi.BitsPerComponent();
    numFrames = vi.num_frames;

    // all bit-depths are supported
    if (!vi.IsY() && !vi.IsYUV() && !vi.IsYUVA() && !vi.IsPlanarRGB() && !vi.IsPlanarRGBA())
      env->ThrowError("SCSelect: only Y, YUV(A), planar RGB(A) clips are supported");

    CompareVideoInfo(vi, scene_begin->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, scene_end->GetVideoInfo(), "SCSelect", env);
    CompareVideoInfo(vi, global_motion->GetVideoInfo(), "SCSelect", env);

    // explicite cache hints for classic Avisynth 2.6
    scene_begin->SetCacheHints(CACHE_GENERIC, 0);
    scene_end->SetCacheHints(CACHE_GENERIC, 0);
    if (gcache >= 0) global_motion->SetCacheHints(CACHE_GENERIC, gcache);
    if (cache >= 0) child->SetCacheHints(CACHE_GENERIC, cache);
  }
};

// Avisynth only
// keep them equivalent: SCSelect::GetFrame (Avisynth) sCSelectGetFrame (VapourSynth)
PVideoFrame SCSelect::GetFrame(int n, IScriptEnvironment* env)
{
  PClip selected;
  const char* debugmsg;
  if (n == 0)
  {
  set_begin:
    debugmsg = "[%u] SCSelect: scene begin\n";
    selected = scene_begin;
  }
  else if (n >= numFrames)
  {
  set_end:
    debugmsg = "[%u] SCSelect: scene end\n";
    selected = scene_end;
  }
  else
  {
    PVideoFrame sf = child->GetFrame(n, env);
    if (lnr != n - 1) // check out-of-sequence call, calculate diff only if needed. This is why no MT is allowed (filter has global state)
    {
      PVideoFrame pf = child->GetFrame(n - 1, env);
      if (isRGB) {
        const int planes[4] = { PLANAR_R, PLANAR_G, PLANAR_B, PLANAR_A };

        lastdiff = 0;
        // RGB: sum all planes
        for (int p = 0; p < 3; p++)
        {
          int plane = planes[p];
          lastdiff += gdiff(sf->GetReadPtr(plane), sf->GetPitch(plane), pf->GetReadPtr(plane), pf->GetPitch(plane), vi.width, vi.height, bits_per_pixel, useSSE2);
        }
      }
      else {
        lastdiff = gdiff(sf->GetReadPtr(PLANAR_Y), sf->GetPitch(PLANAR_Y), pf->GetReadPtr(PLANAR_Y), pf->GetPitch(PLANAR_Y), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
      }
    }
    int64_t olddiff = lastdiff;
    {
      PVideoFrame nf = child->GetFrame(n + 1, env); // in VapourSynth part reading beyond numFrames is guarded
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
        lastdiff = gdiff(sf->GetReadPtr(PLANAR_Y), sf->GetPitch(PLANAR_Y), nf->GetReadPtr(PLANAR_Y), nf->GetPitch(PLANAR_Y), vi.width, vi.height, vi.BitsPerComponent(), useSSE2);
      }
      lnr = n;
    }
    if (dirmult * olddiff < lastdiff) goto set_end;
    if (dirmult * lastdiff < olddiff) goto set_begin;
    debugmsg = "[%u] SCSelect: global motion\n";
    selected = global_motion;
  }
  if (debug) debug_printf(debugmsg, n);
  return selected->GetFrame(n, env);
}

AVSValue __cdecl CreateSCSelect(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  enum ARGS { CLIP, SBEGIN, SEND, GMOTION, DFACTOR, DEBUG, PLANAR, CACHE, GCACHE };
  return new SCSelect(args[CLIP].AsClip(), args[SBEGIN].AsClip(), args[SEND].AsClip(), args[GMOTION].AsClip(), args[DFACTOR].AsFloat(4.0)
    , args[DEBUG].AsBool(false), args[PLANAR].AsBool(false), args[CACHE].AsInt(2), args[GCACHE].AsInt(0), env);
};

/* Avisynth >= 2.6 requirement!!! */
// Declare and initialise server pointers static storage.
const AVS_Linkage *AVS_linkage = 0;

// DLL entry point called from LoadPlugin() to setup a user plugin.
extern "C" __declspec(dllexport) const char* __stdcall
AvisynthPluginInit3(IScriptEnvironment* env, const AVS_Linkage* const vectors) {
  // Save the server pointers.
  AVS_linkage = vectors;
  env->AddFunction("SCSelect", "cccc[dfactor]f[debug]b[planar]b[cache]i[gcache]i", CreateSCSelect, 0);
  // keep sync'd with restore_motion_blocks_creatargs enum
  env->AddFunction("RestoreMotionBlocks", "cc[neighbour]c[neighbour2]c[alternative]c[planar]b[show]b[debug]b[gmthreshold]i[mthreshold]i[noise]i[noisy]i[dist]i[tolerance]i[dmode]i[pthreshold]i[cthreshold]i[grey]b"
  , CreateRestoreMotionBlocks, 0);
  return NULL;
}

// VapourSynth support from here

// Avs/Vs common parameters ("Shared"), with VS-specific additions.
struct RestoreMotionBlocksVSData : public RestoreMotionBlocksShared {
  // VSNode is like PClip in Avisynth
  VSNode* node; // in Avisynth: child
  VSNode* restore; // from _restore
  VSNode* after; // from neighbour
  VSNode* before; // from neighbour2
  VSNode* alternative; // from _alternative
  int width;
  int height;
};

// VapourSynth only
// keep them equivalent: RestoreMotionBlocks::GetFrame (Avisynth) restoreMotionBlocksGetFrame (VapourSynth)
static const VSFrame* VS_CC restoreMotionBlocksGetFrame(int n, int activationReason, void* instanceData, void** frameData, VSFrameContext* frameCtx, VSCore* core, const VSAPI* vsapi) {
  RestoreMotionBlocksVSData* d = (RestoreMotionBlocksVSData*)instanceData;

  if (activationReason == arInitial) {
    // request the very same frame numbers and clips as they appear in activationReason == arAllFramesReady
    if ((n + d->before_offset < 0) || (n + d->after_offset > d->lastframe)) {
      vsapi->requestFrameFilter(n, d->alternative, frameCtx);
    }
    else {
      vsapi->requestFrameFilter(n, d->node, frameCtx);
      vsapi->requestFrameFilter(n, d->restore, frameCtx);
      vsapi->requestFrameFilter(n + d->after_offset, d->after, frameCtx);
      vsapi->requestFrameFilter(n + d->before_offset, d->before, frameCtx);
      vsapi->requestFrameFilter(n, d->alternative, frameCtx);
    }
  }
  else if (activationReason == arAllFramesReady) {

    if ((n + d->before_offset < 0) || (n + d->after_offset > d->lastframe)) 
      return vsapi->getFrameFilter(n, d->alternative, frameCtx); // Avisynth: return alternative->GetFrame(n, env);

    const VSFrame* pf = vsapi->getFrameFilter(n + d->before_offset, d->before, frameCtx); // PVideoFrame pf = before->GetFrame(n + before_offset, env);
    const VSFrame* src = vsapi->getFrameFilter(n, d->node, frameCtx); // PVideoFrame df = child->GetFrame(n, env); // src, the filtered
    const VSFrame* rf = vsapi->getFrameFilter(n, d->restore, frameCtx); // PVideoFrame rf = restore->GetFrame(n, env);
    const VSFrame* nf = vsapi->getFrameFilter(n + d->after_offset, d->after, frameCtx); // PVideoFrame nf = after->GetFrame(n + after_offset, env);
    
    VSFrame* df = vsapi->copyFrame(src, core); // ~Avisynth's env->MakeWritable(&df);
    vsapi->freeFrame(src);

    processFrameParams frameParams;

    frameParams.nextY = vsapi->getReadPtr(nf, 0); // nf->GetReadPtr(PLANAR_Y);
    frameParams.nextPitchY = vsapi->getStride(nf, 0); // nf->GetPitch(PLANAR_Y);
    frameParams.previousY = vsapi->getReadPtr(pf, 0); // pf->GetReadPtr(PLANAR_Y);
    frameParams.previousPitchY = vsapi->getStride(pf, 0); // pf->GetPitch(PLANAR_Y);
    frameParams.destY = vsapi->getWritePtr(df, 0); // df->GetWritePtr(PLANAR_Y);
    frameParams.destPitchY = vsapi->getStride(df, 0); // df->GetPitch(PLANAR_Y);
    frameParams.srcY = vsapi->getReadPtr(rf, 0); // rf->GetReadPtr(PLANAR_Y);
    frameParams.srcPitchY = vsapi->getStride(rf, 0); // rf->GetPitch(PLANAR_Y);
    if (!d->rd->grey) {
      frameParams.destU = vsapi->getWritePtr(df, 1); // df->GetWritePtr(PLANAR_U);
      frameParams.destV = vsapi->getWritePtr(df, 2); // df->GetWritePtr(PLANAR_V);
      frameParams.destPitchUV = vsapi->getStride(df, 1); // df->GetPitch(PLANAR_U);
      frameParams.srcU = vsapi->getReadPtr(rf, 1); // rf->GetReadPtr(PLANAR_U);
      frameParams.srcV = vsapi->getReadPtr(rf, 2); // rf->GetReadPtr(PLANAR_V);
      frameParams.srcPitchUV = vsapi->getStride(rf, 1); // rf->GetPitch(PLANAR_U);
    }

    if (d->rd->ProcessFrame(frameParams, n) > d->mthreshold) {
      vsapi->freeFrame(rf);
      vsapi->freeFrame(nf);
      vsapi->freeFrame(pf);
      return vsapi->getFrameFilter(n, d->alternative, frameCtx); // return alternative->GetFrame(n, env);
    }

    vsapi->freeFrame(rf);
    vsapi->freeFrame(nf);
    vsapi->freeFrame(pf);
    return df;
  }

  return nullptr;
}

static void VS_CC restoreMotionBlocksFree(void* instanceData, VSCore* core, const VSAPI* vsapi) {
  RestoreMotionBlocksVSData* d = (RestoreMotionBlocksVSData*)instanceData;
  // nodes are not smart pointers like PClip in Avisynth, has to free up manually
  vsapi->freeNode(d->node); // aka child/src in Avisynth
  vsapi->freeNode(d->restore);
  vsapi->freeNode(d->after);
  vsapi->freeNode(d->before);
  vsapi->freeNode(d->alternative);
  delete d;
}

#define RETERROR(x) do { vsapi->mapSetError(out, (x)); 	vsapi->freeNode(d->node); vsapi->freeNode(d->restore); vsapi->freeNode(d->after); vsapi->freeNode(d->before); vsapi->freeNode(d->alternative); return; } while (0)

static void VS_CC restoreMotionBlocksCreate(const VSMap* in, VSMap* out, void* userData, VSCore* core, const VSAPI* vsapi) {
  std::unique_ptr<RestoreMotionBlocksVSData> d(new RestoreMotionBlocksVSData());
  int err;

  d->node = vsapi->mapGetNode(in, "clip", 0, &err); // src, not optional
  const VSVideoInfo* vi = vsapi->getVideoInfo(d->node);

  if (!vsh::isConstantVideoFormat(vi))
    RETERROR("RestoreMotionBlocks: Video must be constant format!");

  uint32_t fid = vsapi->queryVideoFormatID(vi->format.colorFamily, vi->format.sampleType, vi->format.bitsPerSample, vi->format.subSamplingW, vi->format.subSamplingH, core);

  // Check for valid bit depths (8 to 16) and Y/YUV formats (grey, 4:2:0, 4:2:2, 4:4:4)
  if ((vi->format.bitsPerSample < 8 || vi->format.bitsPerSample > 16) ||
    (fid != pfYUV420P8 && fid != pfYUV420P10 && fid != pfYUV420P12 && fid != pfYUV420P14 && fid != pfYUV420P16 &&
      fid != pfYUV422P8 && fid != pfYUV422P10 && fid != pfYUV422P12 && fid != pfYUV422P14 && fid != pfYUV422P16 &&
      fid != pfYUV444P8 && fid != pfYUV444P10 && fid != pfYUV444P12 && fid != pfYUV444P14 && fid != pfYUV444P16 &&
      fid != pfGray8 && fid != pfGray10 && fid != pfGray12 && fid != pfGray14 && fid != pfGray16)) {
    RETERROR("RestoreMotionBlocks: Video must be grey, YUV 4:2:0, 4:2:2, or 4:4:4 with bit depths 8-16!");
  }

  // the other clips. Fixme: check if null
  d->restore = vsapi->mapGetNode(in, "restore", 0, nullptr); // not optional
  d->before = vsapi->mapGetNode(in, "neighbour", 0, &err); // optional
  if (err) d->before = nullptr;
  d->after = vsapi->mapGetNode(in, "neighbour2", 0, &err); // optional
  if (err) d->after = nullptr;
  d->alternative = vsapi->mapGetNode(in, "alternative", 0, &err); // optional
  if (err) d->alternative = nullptr;

  // Clip replacement like in RestoreMotionBlocks::RestoreMotionBlocks

  // sets frame number limits and replaces undefined (null) PClips with defaults from before/after
  d->lastframe = vi->numFrames - 1;
  d->before_offset = d->after_offset = 0;
  if (d->after == nullptr)
  {
    d->after = d->restore;
    vsapi->addNodeRef(d->after);
    d->before = d->restore;
    vsapi->addNodeRef(d->before);
    d->before_offset = -1;
    d->after_offset = 1;
  }
  if (d->before == nullptr)
  {
    d->before = d->after;
    vsapi->addNodeRef(d->before);
    d->before_offset = -1;
    d->after_offset = 1;
  }
  if (d->alternative == nullptr) {
    d->alternative = d->restore;
    vsapi->addNodeRef(d->alternative);
  }

  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->restore)))
    RETERROR("RestoreMotionBlocks: Video formats or size do not match: restore (2nd clip param) clip!");

  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->before)))
    RETERROR("RestoreMotionBlocks: Video formats or size do not match: neighbour (before) clip!");
  
  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->after)))
    RETERROR("RestoreMotionBlocks: Video formats or size do not match: neighbour2 (after) clip!");

  d->width = vi->width;
  d->height = vi->height;

  // parameter read out like in InitRemoveDirt

  int xRatioUV = 1;
  int yRatioUV = 1;
  if (vi->format.colorFamily == cfYUV) { // Avisynth: !vi.IsY()) !cfGray
    xRatioUV = 1 << vi->format.subSamplingW;
    yRatioUV = 1 << vi->format.subSamplingH;
  }

  // int dist = args[DIST].AsInt(DEFAULT_DIST);
  int dist = vsapi->mapGetIntSaturated(in, "dist", 0, &err);
  if (err) dist = DEFAULT_DIST;

  // int tolerance = args[TOLERANCE].AsInt(DEFAULT_TOLERANCE);
  int tolerance = vsapi->mapGetIntSaturated(in, "tolerance", 0, &err);
  if (err) tolerance = DEFAULT_TOLERANCE;

  // int dmode = args[DMODE].AsInt(0);
  int dmode = vsapi->mapGetIntSaturated(in, "dmode", 0, &err);
  if (err) dmode = 0;

  // int mthreshold = args[MTHRES].AsInt(DEFAULT_MTHRESHOLD);
  int mthreshold = vsapi->mapGetIntSaturated(in, "mthreshold", 0, &err);
  if (err) mthreshold = DEFAULT_MTHRESHOLD;

  // int noise = args[NOISE].AsInt(0);
  int noise = vsapi->mapGetIntSaturated(in, "noise", 0, &err);
  if (err) noise = 0;

  // int noisy = args[NOISY].AsInt(-1);
  int noisy = vsapi->mapGetIntSaturated(in, "noisy", 0, &err);
  if (err) noisy = -1;

  // int pthreshold = args[PTHRES].AsInt(DEFAULT_PTHRESHOLD);
  int pthreshold = vsapi->mapGetIntSaturated(in, "pthreshold", 0, &err);
  if (err) pthreshold = DEFAULT_PTHRESHOLD;

  // int cthreshold = args[CTHRES].AsInt(pthreshold);
  int cthreshold = vsapi->mapGetIntSaturated(in, "cthreshold", 0, &err);
  if (err) cthreshold = pthreshold;

  // bool grey = vi.IsY() || args[GREY].AsBool(false); // fallback to compulsory grey mode
  bool grey = (1 == vsapi->mapGetIntSaturated(in, "grey", 0, &err));
  if (err) grey = false;
  if (vi->format.colorFamily == cfGray) grey = true;

  // bool show = args[SHOW].AsBool(false);
  bool show = (1 == vsapi->mapGetIntSaturated(in, "show", 0, &err));
  if (err) show = false;

  // bool debug = args[DEBUG].AsBool(false);
  bool debug = (1 == vsapi->mapGetIntSaturated(in, "debug", 0, &err));
  if (err) debug = false;

  // int bitpercomponent = vi.BitsPerComponent();
  int bitpercomponent = vi->format.bitsPerSample;

  // int gmthreshold = args[GMTHRES].AsInt(DEFAULT_GMTHRESHOLD);
  int gmthreshold = vsapi->mapGetIntSaturated(in, "gmthreshold", 0, &err);
  if (err) gmthreshold = DEFAULT_GMTHRESHOLD;

#ifdef INTEL_INTRINSICS
  const CPUFeatures* cpuf = getCPUFeatures();
  // fake Avisynth for reference
  int flags = CPUF_FPU | CPUF_MMX | CPUF_INTEGER_SSE | CPUF_SSE | CPUF_SSE2; // minimum to run VS
  if (cpuf->sse3)      flags |= CPUF_SSE3;
  if (cpuf->ssse3)     flags |= CPUF_SSSE3;
  if (cpuf->sse4_1)    flags |= CPUF_SSE4_1;
  if (cpuf->sse4_2)    flags |= CPUF_SSE4_2;
  if (cpuf->avx)       flags |= CPUF_AVX;
  if (cpuf->avx2)      flags |= CPUF_AVX2;
  if (cpuf->fma3)      flags |= CPUF_FMA3;
  if (cpuf->f16c)      flags |= CPUF_F16C;
  if (cpuf->avx512_f)  flags |= CPUF_AVX512F;
  if (cpuf->avx512_bw) flags |= CPUF_AVX512BW;
  if (cpuf->avx512_dq) flags |= CPUF_AVX512DQ;
  if (cpuf->avx512_cd) flags |= CPUF_AVX512CD;
  if (cpuf->avx512_vl) flags |= CPUF_AVX512VL;

  // avs: 
  // const bool useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
  // const bool useSSE41 = (env->GetCPUFlags() & CPUF_SSE4_1) == CPUF_SSE4_1;
  const bool useSSE2 = (flags & CPUF_SSE2) == CPUF_SSE2; // VS: always. Minimum req.
  const bool useSSE41 = (flags & CPUF_SSE4_1) == CPUF_SSE4_1;
#else
  const bool useSSE2 = false;
  const bool useSSE41 = false;
#endif

  d->rd = new RemoveDirt(vi->width, vi->height, dist, tolerance, dmode
    , mthreshold, noise, noisy
    , pthreshold, cthreshold
    , grey, show, debug, xRatioUV, yRatioUV, bitpercomponent, useSSE2, useSSE41);


  d->mthreshold = (gmthreshold * d->rd->hblocks * d->rd->vblocks) / 100;

  VSFilterDependency deps[] = { { d->node, rpStrictSpatial},
                                { d->restore, rpGeneral},
                                { d->before, rpGeneral},
                                { d->after, rpGeneral},
                                { d->alternative, rpGeneral}
  }; /* Depending the the request patterns you may want to change this */
  // rpStrictSpatial is not good since (except d->node) frame numbers n+/-1 can be requested as well; restore can be cloned to the other clips.
  vsapi->createVideoFilter(out, "RestoreMotionBlocks", vi, restoreMotionBlocksGetFrame, restoreMotionBlocksFree, fmParallel, deps, 5, d.release(), core);
}


// Avs/Vs common parameters ("Shared"), with VS-specific additions.
struct SCSelectVSData : public SCSelectShared {
  // VSNode is like PClip in Avisynth
  VSNode* node; // in Avisynth: child
  VSNode* scene_begin;
  VSNode* scene_end;
  VSNode* global_motion;
  int width;
  int height;
};

// VapourSynth only
// keep them equivalent: SCSelect::GetFrame (Avisynth) sCSelectGetFrame (VapourSynth)
static const VSFrame* VS_CC sCSelectGetFrame(int n, int activationReason, void* instanceData, void** frameData, VSFrameContext* frameCtx, VSCore* core, const VSAPI* vsapi) {
  SCSelectVSData* d = (SCSelectVSData*)instanceData;

  if (activationReason == arInitial) {
    // request the very same frame numbers and clips as they appear in activationReason == arAllFramesReady
    if (n == 0) {
      vsapi->requestFrameFilter(n, d->scene_begin, frameCtx);
    }
    else if (n >= d->numFrames) {
      vsapi->requestFrameFilter(n, d->scene_end, frameCtx);
    }
    else {
      vsapi->requestFrameFilter(n, d->node, frameCtx);
      
      // n-1 is always requested, we don't know whether another thread's arAllFramesReady
      // has modified lnr since our arInitial
      //if (d->lnr != n - 1) // no!
      vsapi->requestFrameFilter(n - 1, d->node, frameCtx);
      if (n + 1 < d->numFrames)
        vsapi->requestFrameFilter(n + 1, d->node, frameCtx);
      vsapi->requestFrameFilter(n, d->scene_begin, frameCtx);
      vsapi->requestFrameFilter(n, d->scene_end, frameCtx);
      vsapi->requestFrameFilter(n, d->global_motion, frameCtx);
    }
  }
  else if (activationReason == arAllFramesReady) {

    VSNode *selected;
    const char* debugmsg;
    if (n == 0)
    {
    set_begin:
      debugmsg = "[%u] SCSelect: scene begin\n";
      selected = d->scene_begin;
    }
    else if (n >= d->numFrames)
    {
    set_end:
      debugmsg = "[%u] SCSelect: scene end\n";
      selected = d->scene_end;
    }
    else
    {
      const VSFrame* sf = vsapi->getFrameFilter(n, d->node, frameCtx); // PVideoFrame sf = child->GetFrame(n, env);

      if (d->lnr != n - 1) // check out-of-sequence call, calculate diff only if needed. This is why no MT is allowed (filter has global state)
      {
        const VSFrame* pf = vsapi->getFrameFilter(n - 1, d->node, frameCtx); // PVideoFrame pf = child->GetFrame(n - 1, env);
        if (d->isRGB) {

          d->lastdiff = 0;
          // RGB: sum all planes
          for (int p = 0; p < 3; p++)
          {
            d->lastdiff += gdiff(
              vsapi->getReadPtr(sf, p), // sf->GetReadPtr(plane),
              vsapi->getStride(sf, p), // sf->GetPitch(plane),
              vsapi->getReadPtr(pf, p), // pf->GetReadPtr(plane), 
              vsapi->getStride(pf, p), // pf->GetPitch(plane), 
              d->width, d->height, d->bits_per_pixel, d->useSSE2);
          }
        }
        else {
          d->lastdiff = gdiff(
            vsapi->getReadPtr(sf, 0), // sf->GetReadPtr(PLANAR_Y), 
            vsapi->getStride(sf, 0), // sf->GetPitch(PLANAR_Y), 
            vsapi->getReadPtr(pf, 0), // pf->GetReadPtr(PLANAR_Y), 
            vsapi->getStride(pf, 0), // pf->GetPitch(PLANAR_Y), 
            d->width, d->height, d->bits_per_pixel, d->useSSE2);
        }
        vsapi->freeFrame(pf);
      }
      int64_t olddiff = d->lastdiff;
      {
        const VSFrame* nf = vsapi->getFrameFilter(n + 1 < d->numFrames ? n + 1 : n, d->node, frameCtx); // PVideoFrame nf = child->GetFrame(n + 1, env);
        
        if (d->isRGB) {
          // RGB: sum all planes

          d->lastdiff = 0;
          for (int p = 0; p < 3; p++)
          {
            d->lastdiff += gdiff(
              vsapi->getReadPtr(sf, p), // sf->GetReadPtr(plane),
              vsapi->getStride(sf, p), // sf->GetPitch(plane),
              vsapi->getReadPtr(nf, p), // nf->GetReadPtr(plane), 
              vsapi->getStride(nf, p), // nf->GetPitch(plane), 
              d->width, d->height, d->bits_per_pixel, d->useSSE2);
          }
        }
        else {
          d->lastdiff = gdiff(
            vsapi->getReadPtr(sf, 0), // sf->GetReadPtr(PLANAR_Y), 
            vsapi->getStride(sf, 0), // sf->GetPitch(PLANAR_Y), 
            vsapi->getReadPtr(nf, 0), // nf->GetReadPtr(PLANAR_Y), 
            vsapi->getStride(nf, 0), // nf->GetPitch(PLANAR_Y), 
            d->width, d->height, d->bits_per_pixel, d->useSSE2);
        }
        d->lnr = n;
        vsapi->freeFrame(nf);
      }
      vsapi->freeFrame(sf);

      if (d->dirmult * olddiff < d->lastdiff) goto set_end;
      if (d->dirmult * d->lastdiff < olddiff) goto set_begin;
      debugmsg = "[%u] SCSelect: global motion\n";
      selected = d->global_motion;
    }
    if (d->debug) debug_printf(debugmsg, n);
    
    const VSFrame *result = vsapi->getFrameFilter(n, selected, frameCtx); // return selected->GetFrame(n, env);
    return result;

  }

  return nullptr;
}

static void VS_CC sCSelectFree(void* instanceData, VSCore* core, const VSAPI* vsapi) {
  SCSelectVSData* d = (SCSelectVSData*)instanceData;
  // nodes are not smart pointers like PClip in Avisynth, has to free up manually
  vsapi->freeNode(d->node); // aka child/src in Avisynth
  vsapi->freeNode(d->scene_begin);
  vsapi->freeNode(d->scene_end);
  vsapi->freeNode(d->global_motion);
  delete d;
}

#define RETERROR_SCSELECT(x) do { vsapi->mapSetError(out, (x)); 	vsapi->freeNode(d->node); vsapi->freeNode(d->scene_begin); vsapi->freeNode(d->scene_end); vsapi->freeNode(d->global_motion); return; } while (0)

static void VS_CC sCSelectCreate(const VSMap* in, VSMap* out, void* userData, VSCore* core, const VSAPI* vsapi) {
  std::unique_ptr<SCSelectVSData> d(new SCSelectVSData());
  int err;

  d->node = vsapi->mapGetNode(in, "clip", 0, &err); // src, not optional
  const VSVideoInfo* vi = vsapi->getVideoInfo(d->node);

  if (!vsh::isConstantVideoFormat(vi))
    RETERROR_SCSELECT("SCSelect: Video must be constant format!");

  // Check for valid bit depths (8 to 16 bit Integer and 32 bit float)
  if ((vi->format.bitsPerSample < 8 || vi->format.bitsPerSample > 16) && vi->format.sampleType == stInteger)
    RETERROR_SCSELECT("SCSelect: For integer formats only 8-16 bit is supported!");
  if (vi->format.bitsPerSample != 32 && vi->format.sampleType == stFloat)
    RETERROR_SCSELECT("SCSelect: For float formats only 32 bit is supported!");

  d->scene_begin = vsapi->mapGetNode(in, "sbegin", 0, nullptr); // not optional
  d->scene_end = vsapi->mapGetNode(in, "send", 0, nullptr); // not optional
  d->global_motion = vsapi->mapGetNode(in, "gmotion", 0, nullptr); // not optional

  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->scene_begin)))
    RETERROR_SCSELECT("RestoreMotionBlocks: Video formats or size do not match: sbegin clip!");

  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->scene_end)))
    RETERROR_SCSELECT("RestoreMotionBlocks: Video formats or size do not match: send clip!");

  if (!vsh::isSameVideoInfo(vi, vsapi->getVideoInfo(d->global_motion)))
    RETERROR_SCSELECT("RestoreMotionBlocks: Video formats or size do not match: gmotion clip!");

  d->width = vi->width;
  d->height = vi->height;
  d->isRGB = vi->format.colorFamily == cfRGB; // avs: vi.isRGB()
  d->bits_per_pixel = vi->format.bitsPerSample; // avs: vi.BitsPerComponent()
  d->numFrames = vi->numFrames;

  // args[DFACTOR].AsFloat(4.0)
  d->dirmult = vsapi->mapGetFloat(in, "dfactor", 0, &err);
  if (err) d->dirmult = 4.0;

  // args[DEBUG].AsBool(false)
  d->debug = (1 == vsapi->mapGetIntSaturated(in, "debug", 0, &err));
  if (err) d->debug = false;

  // Initialize not multithreading friendly variables
  d->lnr = -2;
  d->lastdiff = 0;


#ifdef INTEL_INTRINSICS
  const CPUFeatures* cpuf = getCPUFeatures();
  // fake Avisynth for reference
  int flags = CPUF_FPU | CPUF_MMX | CPUF_INTEGER_SSE | CPUF_SSE | CPUF_SSE2; // minimum to run VS
  if (cpuf->sse3)      flags |= CPUF_SSE3;
  if (cpuf->ssse3)     flags |= CPUF_SSSE3;
  if (cpuf->sse4_1)    flags |= CPUF_SSE4_1;
  if (cpuf->sse4_2)    flags |= CPUF_SSE4_2;
  if (cpuf->avx)       flags |= CPUF_AVX;
  if (cpuf->avx2)      flags |= CPUF_AVX2;
  if (cpuf->fma3)      flags |= CPUF_FMA3;
  if (cpuf->f16c)      flags |= CPUF_F16C;
  if (cpuf->avx512_f)  flags |= CPUF_AVX512F;
  if (cpuf->avx512_bw) flags |= CPUF_AVX512BW;
  if (cpuf->avx512_dq) flags |= CPUF_AVX512DQ;
  if (cpuf->avx512_cd) flags |= CPUF_AVX512CD;
  if (cpuf->avx512_vl) flags |= CPUF_AVX512VL;

  // avs: 
  // const bool useSSE2 = (env->GetCPUFlags() & CPUF_SSE2) == CPUF_SSE2;
  // const bool useSSE41 = (env->GetCPUFlags() & CPUF_SSE4_1) == CPUF_SSE4_1;
  d->useSSE2 = (flags & CPUF_SSE2) == CPUF_SSE2; // VS: always. Minimum req.
#else
  d->useSSE2 = false;
#endif

  VSFilterDependency deps[] = { { d->node, rpGeneral},
                                { d->scene_begin, rpStrictSpatial},
                                { d->scene_end, rpStrictSpatial},
                                { d->global_motion, rpStrictSpatial}
  }; /* Depending the the request patterns you may want to change this */
  // rpStrictSpatial is not good for d->node, frame numbers n+/-1 can be requested
  vsapi->createVideoFilter(out, "SCSelect", vi, sCSelectGetFrame, sCSelectFree, fmUnordered, deps, 4, d.release(), core);
  // in Avisynth this is MT_SERIALIZED. Here in VapourSynth: fmUnordered
}

//////////////////////////////////////////
// Init

VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin* plugin, const VSPLUGINAPI* vspapi) {
  vspapi->configPlugin("com.vapoursynth.removedirt", "removedirt", "RemoveDirt for Avisynth/Vapoursynth", VS_MAKE_VERSION(1, 0), VAPOURSYNTH_API_VERSION, 0, plugin);
  vspapi->registerFunction("RestoreMotionBlocks", "clip:vnode;restore:vnode;neighbour:vnode:opt;neighbour2:vnode:opt;alternative:vnode:opt;show:int:opt;debug:int:opt;gmthreshold:int:opt;mthreshold:int:opt;noise:int:opt;noisy:int:opt;dist:int:opt;tolerance:int:opt;dmode:int:opt;pthreshold:int:opt;cthreshold:int:opt;grey:int:opt;"
    , "clip:vnode;", restoreMotionBlocksCreate, nullptr, plugin);

  // no "planar", "cache", "gcache" like in Avisynth which kept them for script compatibility
  vspapi->registerFunction("SCSelect", "clip:vnode;sbegin:vnode;send:vnode;gmotion:vnode;dfactor:float:opt;debug:int:opt;"
    , "clip:vnode;", sCSelectCreate, nullptr, plugin);
}
