// TemporalRepair from RemoveGrainT 1.0 package"
//
// An Avisynth plugin for removing grain from progressive video
//
// 2007 By Rainer Wittmann <gorw@gmx.de>
// 2019 Additional work by Ferenc Pintér
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

#include "avisynth.h"


#include <Windows.h>
#include <stdio.h>
#include <stdarg.h>
#include <cassert>
#include "planar.h"
#include "common.h"
#include "RemoveGrainT.h"
#include "stdint.h"
#include "emmintrin.h"
#include "immintrin.h"
#include <math.h>

#define	SSE_INCREMENT	16

#ifndef X86_64
// for inline asm - old stuff kept for comparison
#define	SSE_SHIFT		4
#define	SSE_MOVE		movdqu
#define	SSE3_MOVE		movdqu
//#define	SSE3_MOVE		lddqu
#define	SSE_RMOVE		movdqa
#define	SSE0			xmm0
#define	SSE1			xmm1
#define	SSE2			xmm2
#define	SSE3			xmm3
#define	SSE4			xmm4
#define	SSE5			xmm5
#define	SSE6			xmm6
#define	SSE7			xmm7
#endif

__forceinline void RepairPixel(uint8_t *dest, const uint8_t *src1, const uint8_t *src2, const uint8_t *previous, const uint8_t *next)
{
	auto src_next = _mm_loadu_si128(reinterpret_cast<const __m128i*>(next));
	auto src_prev = _mm_loadu_si128(reinterpret_cast<const __m128i*>(previous));
	auto src_1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src1));
	auto src_2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src2));

	auto min_np2 = _mm_min_epu8(_mm_min_epu8(src_next, src_prev), src_2);
	auto max_np2 = _mm_max_epu8(_mm_max_epu8(src_next, src_prev), src_2);
	auto reg1 = _mm_max_epu8(min_np2, src_1);
	auto result = _mm_min_epu8(reg1, max_np2);

	_mm_storeu_si128(reinterpret_cast<__m128i*>(dest), result);
}

static void temporal_repair_mode0_sse2(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  assert(width >= 16);

  const int wmod16 = width / 16 * 16;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < wmod16; x += 16)
    {
      RepairPixel(dp + x, sp1 + x, sp2 + x, pp + x, np + x);
    }
    // remainder, surely unaligned, overlaps but still simd
    if (width > wmod16) {
      const int last16 = width - 16;
      RepairPixel(dp + last16, sp1 + last16, sp2 + last16, pp + last16, np + last16);
    }
    dp += dpitch;
    sp1 += spitch1;
    sp2 += spitch2;
    pp += ppitch;
    np += npitch;
  }
}

#ifndef X86_64
__forceinline int RepairPixel_c(int src_1, int src_2, int src_prev, int src_next)
{

	auto min_np2 = min(min(src_next, src_prev), src_2); //_mm_min_epu8(_mm_min_epu8(src_next, src_prev), src_2);
	auto max_np2 = max(max(src_next, src_prev), src_2);
	auto reg1 = max(min_np2, src_1);
	return min(reg1, max_np2);
}

static void temporal_repair_mode0_c(BYTE* dp, int dpitch, const BYTE* sp1, int spitch1, const BYTE* sp2, int spitch2, const BYTE* pp, int ppitch, const BYTE* np, int npitch, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			dp[x] = RepairPixel_c(sp1[x], sp2[x], pp[x], np[x]);
		}
		dp += dpitch;
		sp1 += spitch1;
		sp2 += spitch2;
		pp += ppitch;
		np += npitch;
	}
}

#define	RepairPixel_old(dest, src1, src2, previous, next, reg1, reg2, reg3, reg4)	\
__asm	SSE3_MOVE	reg1,			next				\
__asm	SSE3_MOVE	reg3,			previous			\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	SSE3_MOVE	reg4,			src2				\
__asm	pminub		reg1,			reg3				\
__asm	pmaxub		reg2,			reg3				\
__asm	pminub		reg1,			reg4				\
__asm	SSE3_MOVE	reg3,			src1				\
__asm	pmaxub		reg2,			reg4				\
__asm	pmaxub		reg1,			reg3				\
__asm	pminub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1


static void temporal_repair_mode0_sse2_old(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  int blocks = --width / SSE_INCREMENT;
  int remainder = (width & (SSE_INCREMENT - 1)) - (SSE_INCREMENT - 1);
  width -= SSE_INCREMENT - 1;
  dpitch -= width;
  spitch1 -= width;
  spitch2 -= width;
  ppitch -= width;
  npitch -= width;
  __asm	mov			ebx, pp
  __asm	mov			edx, sp1
  __asm	mov			esi, sp2
  __asm	mov			edi, dp
  __asm	mov			eax, np
  __asm	mov			ecx, blocks
  __asm	align		16
  __asm	_loop:
  RepairPixel_old([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3)
    __asm	add			eax, SSE_INCREMENT
  __asm	add			esi, SSE_INCREMENT
  __asm	add			edi, SSE_INCREMENT
  __asm	add			edx, SSE_INCREMENT
  __asm	add			ebx, SSE_INCREMENT
  __asm	loop		_loop
  // the last pixels
  __asm	add			esi, remainder
  __asm	add			edi, remainder
  __asm	add			edx, remainder
  __asm	mov			ecx, blocks
  __asm	add			ebx, remainder
  __asm	add			eax, remainder
  RepairPixel_old([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3)
    __asm	add			esi, spitch2
  __asm	add			edi, dpitch
  __asm	add			edx, spitch1
  __asm	add			ebx, ppitch
  __asm	add			eax, npitch
  __asm	dec			height
  __asm	jnz			_loop
}
#endif

// SSE4.1 simulation for SSE2
// false: a, true: b
__forceinline __m128i _MM_BLENDV_EPI8(__m128i const& a, __m128i const& b, __m128i const& selector) {
  return _mm_or_si128(_mm_and_si128(selector, b), _mm_andnot_si128(selector, a));
}

__forceinline void BRepairPixel(uint8_t *dest, const uint8_t *src1, const uint8_t *src2, const uint8_t *previous, const uint8_t *next)
{
	__m128i reg3, reg5;
	auto src_next = _mm_loadu_si128(reinterpret_cast<const __m128i*>(next));
	auto src_prev = _mm_loadu_si128(reinterpret_cast<const __m128i*>(previous));
  auto src_1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src1));
  auto src_2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src2));
	
  auto max_np = _mm_max_epu8(src_next, src_prev);
	auto min_np = _mm_min_epu8(src_next, src_prev);

	auto diff_src2_minnp = _mm_subs_epu8(src_2, min_np);
  reg5 = _mm_adds_epu8(_mm_adds_epu8(diff_src2_minnp, diff_src2_minnp) , min_np); // diff_src2_minnp * 2 - min_np
  
  auto diff_maxnp_src2 = _mm_subs_epu8(max_np, src_2);
  reg3 = _mm_subs_epu8(max_np, _mm_adds_epu8(diff_maxnp_src2, diff_maxnp_src2)); // max_np - 2 * diff_maxnp_src2

	reg5 = _mm_min_epu8(reg5, max_np);
	reg3 = _mm_max_epu8(reg3, min_np);

	auto equ1 = _mm_cmpeq_epi8(min_np, reg5);
	auto equ2 = _mm_cmpeq_epi8(max_np, reg3);
  auto equ = _mm_or_si128(equ1, equ2);// _mm_max_epu8(equ1, equ2); // pracically or. mask set if min_np == reg5 or max_np == reg3 FIXME

	reg5 = _mm_min_epu8(reg5, src_1);
	reg5 = _mm_max_epu8(reg5, reg3);

  // real SSE4.1 _mm_blendv_epi8: 2830 vs 2960 FPS, worth using it
  auto result = _MM_BLENDV_EPI8(reg5, src_2, equ); // keep src2 where equal

	_mm_storeu_si128(reinterpret_cast<__m128i*>(dest), result);
}

static void btemporal_repair_mode4_sse2(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  assert(width >= 16);

  const int wmod16 = width / 16 * 16;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < wmod16; x += 16)
    {
      BRepairPixel(dp + x, sp1 + x, sp2 + x, pp + x, np + x);
    }
    // remainder, surely unaligned, overlaps but still simd
    const int last16 = width - 16;
    BRepairPixel(dp + last16, sp1 + last16, sp2 + last16, pp + last16, np + last16);
    dp += dpitch;
    sp1 += spitch1;
    sp2 += spitch2;
    pp += ppitch;
    np += npitch;
  }
}

__forceinline int subs_int(int x, int y) {
  return x < y ? 0 : x - y;
}

__forceinline int adds_uint8(int x, int y) {
  int sum = x + y;
  return sum > 255 ? 255 : sum;
}

__forceinline int BRepairPixel_c(int src_1, int src_2, int src_prev, int src_next)
{
  auto max_np = max(src_next, src_prev);
  auto min_np = min(src_next, src_prev);

  auto diff_src2_minnp = subs_int(src_2, min_np);
  auto reg5 = adds_uint8(adds_uint8(diff_src2_minnp, diff_src2_minnp), min_np); // diff_src2_minnp * 2 - min_np

  auto diff_maxnp_src2 = subs_int(max_np, src_2);
  auto reg3 = subs_int(max_np, adds_uint8(diff_maxnp_src2, diff_maxnp_src2)); // max_np - 2 * diff_maxnp_src2

  reg5 = min(reg5, max_np);
  reg3 = max(reg3, min_np);

  auto equ = min_np == reg5 || max_np == reg3;

  reg5 = min(reg5, src_1);
  reg5 = max(reg5, reg3);

  return equ ? src_2 : reg5;
}

static void btemporal_repair_mode4_c(BYTE* dp, int dpitch, const BYTE* sp1, int spitch1, const BYTE* sp2, int spitch2, const BYTE* pp, int ppitch, const BYTE* np, int npitch, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			dp[x] = BRepairPixel_c(sp1[x], sp2[x], pp[x], np[x]);
		}
		dp += dpitch;
		sp1 += spitch1;
		sp2 += spitch2;
		pp += ppitch;
		np += npitch;
	}
}


#ifndef X86_64

#define	BRepairPixel_old(dest, src1, src2, previous, next, reg1, reg2, reg3, reg4, reg5, reg6)	\
__asm	SSE3_MOVE	reg1,			next				\
__asm	SSE3_MOVE	reg3,			previous			\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	SSE3_MOVE	reg4,			src2				\
__asm	pmaxub		reg2,			reg3				\
__asm	SSE_RMOVE	reg5,			reg4				\
__asm	pminub		reg1,			reg3				\
__asm	SSE_RMOVE	reg6,			reg2				\
__asm	psubusb		reg5,			reg1				\
__asm	psubusb		reg6,			reg4				\
__asm	SSE_RMOVE	reg3,			reg2				\
__asm	paddusb		reg5,			reg5				\
__asm	paddusb		reg6,			reg6				\
__asm	paddusb		reg5,			reg1				\
__asm	psubusb		reg3,			reg6				\
__asm	pminub		reg5,			reg2				\
__asm	pmaxub		reg3,			reg1				\
__asm	SSE3_MOVE	reg6,			src1				\
__asm	pcmpeqb		reg1,			reg5				\
__asm	pcmpeqb		reg2,			reg3				\
__asm	pminub		reg5,			reg6				\
__asm	pmaxub		reg1,			reg2				\
__asm	pmaxub		reg5,			reg3				\
__asm	pminub		reg4,			reg1				\
__asm	psubusb		reg5,			reg1				\
__asm	pmaxub		reg4,			reg5				\
__asm	SSE_MOVE	dest,			reg4

static	void	btemporal_repair_mode4_old(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  int	blocks = --width / SSE_INCREMENT;
  int	remainder = (width & (SSE_INCREMENT - 1)) - (SSE_INCREMENT - 1);
  width -= SSE_INCREMENT - 1;
  dpitch -= width;
  spitch1 -= width;
  spitch2 -= width;
  ppitch -= width;
  npitch -= width;
  __asm	mov			ebx, pp
  __asm	mov			edx, sp1
  __asm	mov			esi, sp2
  __asm	mov			edi, dp
  __asm	mov			eax, np
  __asm	mov			ecx, blocks
  __asm	align		16
  __asm	_loop:
  BRepairPixel_old([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5)
    __asm	add			eax, SSE_INCREMENT
  __asm	add			esi, SSE_INCREMENT
  __asm	add			edi, SSE_INCREMENT
  __asm	add			edx, SSE_INCREMENT
  __asm	add			ebx, SSE_INCREMENT
  __asm	loop		_loop
  // the last pixels
  __asm	add			esi, remainder
  __asm	add			edi, remainder
  __asm	mov			ecx, blocks
  __asm	add			ebx, remainder
  __asm	add			eax, remainder
  BRepairPixel_old([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5)
    __asm	add			esi, spitch2
  __asm	add			edi, dpitch
  __asm	add			edx, spitch1
  __asm	add			ebx, ppitch
  __asm	add			eax, npitch
  __asm	dec			height
  __asm	jnz			_loop
}
#endif

class	TemporalRepair : public GenericVideoFilter, public PlanarAccess
{
  void(*trepair)(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height);
  unsigned int last_frame;
  PClip orig;
  int opt;

  // MT mode Registration for Avisynth+
  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_NICE_FILTER : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if (n <= 0 || n - 1 >= last_frame)
      return child->GetFrame(n, env);
    PVideoFrame pf = orig->GetFrame(n - 1, env);
    PVideoFrame sf = orig->GetFrame(n, env);
    PVideoFrame nf = orig->GetFrame(n + 1, env);
    PVideoFrame cf = child->GetFrame(n, env);
    PVideoFrame df = env->NewVideoFrame(vi);

    for(int i = 0; i < planecount; i++)
      trepair(GetWritePtr(df, i), GetPitch(df, i), GetReadPtr(cf, i), GetPitch(cf, i), GetReadPtr(sf, i), GetPitch(sf, i), GetReadPtr(pf, i), GetPitch(pf, i), GetReadPtr(nf, i), GetPitch(nf, i), width[i], height[i]);
 
    return df;
  }

public:
  TemporalRepair(PClip clip, PClip oclip, int mode, bool grey, bool planar, int opt, IScriptEnvironment* env) : 
    GenericVideoFilter(clip), 
    PlanarAccess(vi), 
    orig(oclip), opt(opt)
  {
    if (!planar && !vi.IsPlanar())
      env->ThrowError("TemporalRepair: only planar color spaces are supported");

    CompareVideoInfo(vi, orig->GetVideoInfo(), "TemporalRepair", env);

    assert(mode == 0 || mode == 4);
    if (opt == 0) // 0: C
      trepair = mode == 4 ? btemporal_repair_mode4_c : temporal_repair_mode0_c;
    else if (opt != 99) // SSE2 new simd
      trepair = mode == 4 ? btemporal_repair_mode4_sse2 : temporal_repair_mode0_sse2;
#ifndef X86_64
    else if (opt == 99) // 99: old
      trepair = mode == 4 ? btemporal_repair_mode4_old : temporal_repair_mode0_sse2_old;
#endif
    last_frame = vi.num_frames >= 2 ? vi.num_frames - 2 : 0;
    if (grey)
      planecount = 1;
  }
};

__forceinline void get_lu(__m128i &lower, __m128i &upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  auto src_next = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next));
  auto src_prev = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous));
  auto src_curr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current));
  auto max_np = _mm_max_epu8(src_next, src_prev);
  auto min_np = _mm_min_epu8(src_next, src_prev);
  upper = _mm_subs_epu8(max_np, src_curr);
  lower = _mm_subs_epu8(src_curr, min_np);
}

__forceinline void get_lu_c(int& lower, int& upper, int src_prev, int src_curr, int src_next)
{
  auto max_np = max(src_next, src_prev);
  auto min_np = min(src_next, src_prev);
  upper = subs_int(max_np, src_curr);
  lower = subs_int(src_curr, min_np);
}

#ifndef X86_64
// reg& reg& ptr ptr ptr reg reg
#define	get_lu_old(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE3_MOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2
#endif

__forceinline void SmoothTRepair(uint8_t *dest, __m128i &lower, __m128i &upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  auto src_curr = _mm_loadu_si128(reinterpret_cast<const __m128i*>(current));
  auto src_prev = _mm_loadu_si128(reinterpret_cast<const __m128i*>(previous));
  auto src_next = _mm_loadu_si128(reinterpret_cast<const __m128i*>(next));
  auto src_dest = _mm_loadu_si128(reinterpret_cast<const __m128i*>(dest));

  auto tmp_u = _mm_adds_epu8(upper, src_curr);
  auto tmp_l = _mm_subs_epu8(src_curr, lower);

  auto tmp_max = _mm_max_epu8(_mm_max_epu8(tmp_u, src_prev), src_next);
  auto tmp_min = _mm_min_epu8(_mm_min_epu8(tmp_l, src_prev), src_next);
  
  auto result = _mm_max_epu8(_mm_min_epu8(tmp_max, src_dest), tmp_min);
  
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), result);
}

__forceinline void SmoothTRepair_c(BYTE &dest, int lower, int upper, const int src_prev, const int src_curr, const int src_next)
{
  auto src_dest = dest;

  auto tmp_u = adds_uint8(upper, src_curr);
  auto tmp_l = subs_int(src_curr, lower);

  auto tmp_max = max(max(tmp_u, src_prev), src_next);
  auto tmp_min = min(min(tmp_l, src_prev), src_next);

  auto result = max(min(tmp_max, src_dest), tmp_min);

  dest = result;
}

#ifndef X86_64
// ptr reg reg ptr ptr ptr reg reg
#define	SmoothTRepair_old(dest, lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	reg1,			current				\
__asm	SSE3_MOVE	reg2,			previous			\
__asm	paddusb		upper,			reg1				\
__asm	psubusb		reg1,			lower				\
__asm	pmaxub		upper,			reg2				\
__asm	SSE3_MOVE	lower,			next				\
__asm	pminub		reg1,			reg2				\
__asm	pmaxub		upper,			lower				\
__asm	SSE3_MOVE	reg2,			dest				\
__asm	pminub		reg1,			lower				\
__asm	pminub		upper,			reg2				\
__asm	pmaxub		upper,			reg1				\
__asm	SSE_MOVE	dest,			upper
#endif

void smooth_temporal_repair1_sse2(BYTE *dp, const BYTE *previous, const BYTE *sp, const BYTE *next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X
  
  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1;
  // remainder: corrects pointers back to process exactly the 16 last pixels 
  const int hblocks = (width - 1) / 16;
  const int remainder = ((width - 1) & 15) - 15;

  for(int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {
    __m128i lowermax = _mm_undefined_si128();
    __m128i uppermax = _mm_undefined_si128();
    __m128i lower = _mm_undefined_si128();
    __m128i upper = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      get_lu(lowermax, uppermax, previous, sp, next);
      get_lu(lower, upper, previous + 1, sp + 1, next + 1);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2, sp + 2, next + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch, sp + 2 * ofpitch, next + 2 * nfpitch);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch + 1, sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch + 2, sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + pfpitch, sp + ofpitch, next + nfpitch);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + pfpitch + 2, sp + ofpitch + 2, next + nfpitch + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      SmoothTRepair(dp, lowermax, uppermax, previous + pfpitch + 1, sp + ofpitch + 1, next + nfpitch + 1);
      dp += 16;
      previous += 16;
      sp += 16;
      next += 16;
    }

    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    sp += remainder;
    previous += remainder;
    next += remainder;
    dp += remainder;

    get_lu(lowermax, uppermax, previous, sp, next);
    get_lu(lower, upper, previous + 1, sp + 1, next + 1);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2, sp + 2, next + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch, sp + 2 * ofpitch, next + 2 * nfpitch);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch + 1, sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch + 2, sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + pfpitch, sp + ofpitch, next + nfpitch);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + pfpitch + 2, sp + ofpitch + 2, next + nfpitch + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    SmoothTRepair(dp, lowermax, uppermax, previous + pfpitch + 1, sp + ofpitch + 1, next + nfpitch + 1);

    dp += pitches[0];
    previous += pitches[1];
    sp += pitches[2];
    next += pitches[3];
  }
}

void smooth_temporal_repair1_c(BYTE* dp, const BYTE* previous, const BYTE* sp, const BYTE* next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1;

  for (int y = 0; y < height; y++)
  {

    for (int x = 0; x < width; x++)
    {
      int lowermax, uppermax;
      int lower, upper;    
      get_lu_c(lowermax, uppermax, previous[x], sp[x], next[x]);
      get_lu_c(lower, upper, previous[x + 1], sp[x + 1], next[x + 1]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2], sp[x + 2], next[x + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch], sp[x + 2 * ofpitch], next[x + 2 * nfpitch]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch + 1], sp[x + 2 * ofpitch + 1], next[x + 2 * nfpitch + 1]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch + 2], sp[x + 2 * ofpitch + 2], next[x + 2 * nfpitch + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + pfpitch], sp[x + ofpitch], next[x + nfpitch]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + pfpitch + 2], sp[x + ofpitch + 2], next[x + nfpitch + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      SmoothTRepair_c(dp[x], lowermax, uppermax, previous[x + pfpitch + 1], sp[x + ofpitch + 1], next[x + nfpitch + 1]);
    }

    dp += pitches[0];
    previous += pitches[1];
    sp += pitches[2];
    next += pitches[3];
  }
}


#ifndef X86_64
void	smooth_temporal_repair1_old(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{
  __asm	mov			eax, hblocks
  __asm	mov			ecx, eax
  __asm	mov			edx, previous
  __asm	mov			esi, _sp
  __asm	shl			eax, SSE_SHIFT
  __asm	mov			edi, dp
  __asm	add			eax, remainder
  __asm	mov			ebx, pitch
  __asm	sub			pitch, eax
  __asm	lea			edi, [edi + ebx + 1]
    __asm	mov			eax, next
  __asm	align		16
  __asm	middle_loop:
  get_lu_old(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
    __asm	add			esi, SSE_INCREMENT
  __asm	add			edx, SSE_INCREMENT
  __asm	add			eax, SSE_INCREMENT
  __asm	add			edi, SSE_INCREMENT
  __asm	dec			ecx
  __asm	jnz			middle_loop
  // the last pixels
  __asm	add			esi, remainder
  __asm	add			edx, remainder
  __asm	add			eax, remainder
  __asm	add			edi, remainder
  get_lu_old(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
    __asm	add			esi, pitch
  __asm	add			edx, pitch
  __asm	add			eax, pitch
  __asm	add			edi, pitch
  __asm	dec			height
  __asm	mov			ecx, hblocks
  __asm	jnz			middle_loop
}
#endif

/*
__forceinline void get_lu_reg(__m128i &lower, __m128i &upper, const uint8_t *previous, __m128i &current, const uint8_t *next)
{
  __m128i reg1, reg2;

  upper = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm	SSE3_MOVE	upper, next
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm	SSE3_MOVE	reg1, previous
  reg2 = upper; // __asm	SSE_RMOVE	reg2, upper
  // only difference between _get_lu and _get_lu_reg: current is cached to a register for reuse
  lower = current; // __asm	SSE_RMOVE	lower,			current
  upper = _mm_max_epu8(upper, reg1); // __asm	pmaxub		upper, reg1
  reg2 = _mm_min_epu8(reg2, reg1); // __asm	pminub		reg2, reg1
  upper = _mm_subs_epu8(upper, lower); // __asm	psubusb		upper, lower
  lower = _mm_subs_epu8(lower, reg2); // __asm	psubusb		lower, reg2
}
*/

#ifndef X86_64
// reg& reg& ptr reg& ptr reg reg
#define	get_lu_reg_old(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE_RMOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2
#endif

__forceinline void SmoothTRepair2(uint8_t *dest, __m128i lower, __m128i upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i lower1 = _mm_undefined_si128();
  __m128i upper1 = _mm_undefined_si128();

  auto src_curr = _mm_loadu_si128(reinterpret_cast<const __m128i*>(current));
  auto src_dest = _mm_loadu_si128(reinterpret_cast<const __m128i*>(dest));

  get_lu(lower1, upper1, previous, current, next);
  auto uppermax = _mm_max_epu8(upper, upper1);
  auto lowermax = _mm_max_epu8(lower, lower1);
  auto upperlowermax = _mm_max_epu8(uppermax, lowermax);

  auto tmp1 = _mm_adds_epu8(src_curr, upperlowermax);
  auto tmp2 = _mm_subs_epu8(src_curr, upperlowermax);

  auto result = _mm_max_epu8(_mm_min_epu8(tmp1, src_dest), tmp2);
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), result);
}

__forceinline void SmoothTRepair2_c(BYTE &dest, int lower, int upper, int src_prev, int src_curr, int src_next)
{
  int lower1;
  int upper1;

  auto src_dest = dest;

  get_lu_c(lower1, upper1, src_prev, src_curr, src_next);
  auto uppermax = max(upper, upper1);
  auto lowermax = max(lower, lower1);
  auto upperlowermax = max(uppermax, lowermax);

  auto tmp1 = adds_uint8(src_curr, upperlowermax);
  auto tmp2 = subs_int(src_curr, upperlowermax);

  auto result = max(min(tmp1, src_dest), tmp2);
  dest = result;
}


#ifndef X86_64
#define	SmoothTRepair2_old(dest, lower, upper, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
__asm	SSE3_MOVE	reg1,			current				\
		get_lu_reg_old(reg4, reg5, previous, reg1, next, reg2, reg3)	\
__asm	pmaxub		upper,			reg5				\
__asm	pmaxub		lower,			reg4				\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	pmaxub		upper,			lower				\
__asm	SSE3_MOVE	reg3,			dest				\
__asm	paddusb		reg1,			upper				\
__asm	psubusb		reg2,			upper				\
__asm	pminub		reg1,			reg3				\
__asm	pmaxub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1
#endif

void	smooth_temporal_repair2_sse2(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1; //  __asm	lea			edi, [edi + pitch + 1]
  // remainder: corrects pointers back to process exactly the 16 last pixels 
  const int hblocks = (width - 1) / 16;
  const int remainder = ((width - 1) & 15) - 15;

  for (int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {

    __m128i lowermax = _mm_undefined_si128();
    __m128i uppermax = _mm_undefined_si128();
    __m128i lower = _mm_undefined_si128();
    __m128i upper = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      get_lu(lowermax, uppermax, previous, _sp, next);
      get_lu(lower, upper, previous + 1, _sp + 1, next + 1);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2, _sp + 2, next + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + pfpitch, _sp + ofpitch, next + nfpitch);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      get_lu(lower, upper, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
      uppermax = _mm_max_epu8(uppermax, upper);
      lowermax = _mm_max_epu8(lowermax, lower);
      SmoothTRepair2(dp, lowermax, uppermax, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);

      _sp += 16;
      previous += 16;
      next += 16;
      dp += 16;
    }

    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    _sp += remainder;
    previous += remainder;
    next += remainder;
    dp += remainder;

    get_lu(lowermax, uppermax, previous, _sp, next);
    get_lu(lower, upper, previous + 1, _sp + 1, next + 1);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2, _sp + 2, next + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + pfpitch, _sp + ofpitch, next + nfpitch);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    get_lu(lower, upper, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
    uppermax = _mm_max_epu8(uppermax, upper);
    lowermax = _mm_max_epu8(lowermax, lower);
    SmoothTRepair2(dp, lowermax, uppermax, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);

    dp += pitches[0];
    previous += pitches[1];
    _sp += pitches[2];
    next += pitches[3];
  }
}

#ifndef X86_64
// obsolate, assumes that pitches are all equal, which is not always true
void	smooth_temporal_repair2_old(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{
  __asm	mov			eax, hblocks
  __asm	mov			ecx, eax
  __asm	mov			edx, previous
  __asm	mov			esi, _sp
  __asm	shl			eax, SSE_SHIFT
  __asm	mov			edi, dp
  __asm	add			eax, remainder
  __asm	mov			ebx, pitch
  __asm	sub			pitch, eax
  __asm	lea			edi, [edi + ebx + 1]
    __asm	mov			eax, next
  __asm	align		16
  __asm	middle_loop:
  get_lu_old(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair2_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
    __asm	add			esi, SSE_INCREMENT
  __asm	add			edx, SSE_INCREMENT
  __asm	add			eax, SSE_INCREMENT
  __asm	add			edi, SSE_INCREMENT
  __asm	dec			ecx
  __asm	jnz			middle_loop
  // the last pixels
  __asm	add			esi, remainder
  __asm	add			edx, remainder
  __asm	add			eax, remainder
  __asm	add			edi, remainder
  get_lu_old(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair2_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
    __asm	add			esi, pitch
  __asm	add			edx, pitch
  __asm	add			eax, pitch
  __asm	add			edi, pitch
  __asm	dec			height
  __asm	mov			ecx, hblocks
  __asm	jnz			middle_loop
}
#endif

void	smooth_temporal_repair2_c(BYTE* dp, const BYTE* previous, const BYTE* sp, const BYTE* next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int lowermax, uppermax;
      int lower, upper;     
      get_lu_c(lowermax, uppermax, previous[x], sp[x], next[x]);
      get_lu_c(lower, upper, previous[x + 1], sp[x + 1], next[x + 1]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2], sp[x + 2], next[x + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch], sp[x + 2 * ofpitch], next[x + 2 * nfpitch]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch + 1], sp[x + 2 * ofpitch + 1], next[x + 2 * nfpitch + 1]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + 2 * pfpitch + 2], sp[x + 2 * ofpitch + 2], next[x + 2 * nfpitch + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + pfpitch], sp[x + ofpitch], next[x + nfpitch]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      get_lu_c(lower, upper, previous[x + pfpitch + 2], sp[x + ofpitch + 2], next[x + nfpitch + 2]);
      uppermax = max(uppermax, upper);
      lowermax = max(lowermax, lower);
      SmoothTRepair2_c(dp[x], lowermax, uppermax, previous[x + pfpitch + 1], sp[x + ofpitch + 1], next[x + nfpitch + 1]);
    }

    dp += pitches[0];
    previous += pitches[1];
    sp += pitches[2];
    next += pitches[3];
  }
}

__forceinline void get2diff(__m128i &pdiff, __m128i &ndiff, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  auto src_curr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current));
  auto src_prev = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous));
  auto src_next = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next));

  auto diff_cp = _mm_subs_epu8(src_curr, src_prev);
  auto diff_pc = _mm_subs_epu8(src_prev, src_curr);
  pdiff = _mm_or_si128(diff_pc, diff_cp); // abs(c-p)

  auto diff_cn = _mm_subs_epu8(src_curr, src_next);
  auto diff_nc = _mm_subs_epu8(src_next, src_curr);
  ndiff = _mm_or_si128(diff_nc, diff_cn); // abs(c-n)
}

__forceinline void get2diff_c(int& pdiff, int& ndiff, int src_prev, int src_curr, int src_next)
{
  pdiff = abs(src_curr - src_prev); 
  ndiff = abs(src_curr - src_next);
}

#ifndef X86_64
#define	get2diff_old(pdiff, ndiff, previous, current, next, reg1, reg2, reg3)	\
__asm	SSE3_MOVE	reg3,			current				\
__asm	SSE3_MOVE	pdiff,			previous			\
__asm	SSE_RMOVE	reg1,			reg3				\
__asm	SSE3_MOVE	ndiff,			next				\
__asm	SSE_RMOVE	reg2,			reg3				\
__asm	psubusb		reg1,			pdiff				\
__asm	psubusb		reg2,			ndiff				\
__asm	psubusb		ndiff,			reg3				\
__asm	psubusb		pdiff,			reg3				\
__asm	pmaxub		pdiff,			reg1				\
__asm	pmaxub		ndiff,			reg2
#endif


__forceinline void SmoothTRepair3(uint8_t *dest, __m128i &pmax, __m128i &nmax, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i pdiff = _mm_undefined_si128();
  __m128i ndiff = _mm_undefined_si128();

  auto src_curr = _mm_loadu_si128(reinterpret_cast<const __m128i*>(current));

  get2diff(pdiff, ndiff, previous, current, next);
  pmax = _mm_max_epu8(pmax, pdiff);
  nmax = _mm_max_epu8(nmax, ndiff);

  pmax = _mm_min_epu8(pmax, nmax);
  auto src_dest = _mm_loadu_si128(reinterpret_cast<const __m128i *>(dest));
  auto tmp_upper = _mm_adds_epu8(src_curr, pmax);
  auto tmp_lower = _mm_subs_epu8(src_curr, pmax);
  auto result = _mm_max_epu8(_mm_min_epu8(tmp_upper, src_dest), tmp_lower);
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), result);
}

__forceinline void SmoothTRepair3_c(BYTE &dest, int& pmax, int& nmax, int src_prev, int src_curr, int src_next)
{
  int pdiff;
  int ndiff;

  get2diff_c(pdiff, ndiff, src_prev, src_curr, src_next);
  pmax = max(pmax, pdiff);
  nmax = max(nmax, ndiff);

  pmax = min(pmax, nmax);
  auto src_dest = dest;
  auto tmp_upper = adds_uint8(src_curr, pmax);
  auto tmp_lower = subs_int(src_curr, pmax);
  auto result = max(min(tmp_upper, src_dest), tmp_lower);
  dest = result;
}

#ifndef X86_64
#define	SmoothTRepair3_old(dest, pmax, nmax, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
		get2diff_old(reg4, reg5, previous, current, next, reg2, reg3, reg1)	\
__asm	pmaxub		pmax,			reg4				\
__asm	pmaxub		nmax,			reg5				\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	pminub		pmax,			nmax				\
__asm	SSE3_MOVE	reg3,			dest				\
__asm	paddusb		reg1,			pmax				\
__asm	psubusb		reg2,			pmax				\
__asm	pminub		reg1,			reg3				\
__asm	pmaxub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1
#endif


void smooth_temporal_repair3_sse2(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1; //  __asm	lea			edi, [edi + pitch + 1]
  // remainder: corrects pointers back to process exactly the 16 last pixels 
  const int hblocks = (width - 1) / 16;
  const int remainder = ((width - 1) & 15) - 15;

  for (int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {
    __m128i pdiffmax = _mm_undefined_si128();
    __m128i ndiffmax = _mm_undefined_si128();
    __m128i pdiff = _mm_undefined_si128();
    __m128i ndiff = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      get2diff(pdiffmax, ndiffmax, previous, _sp, next);
      get2diff(pdiff, ndiff, previous + 1, _sp + 1, next + 1);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + 2, _sp + 2, next + 2);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff); // __asm	pmaxub		pdiffmax, pdiff // BUG Fixed by PF 2019, this line was missing
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + pfpitch, _sp + ofpitch, next + nfpitch);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      get2diff(pdiff, ndiff, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
      pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
      ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
      SmoothTRepair3(dp, pdiffmax, ndiffmax, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);
      _sp += 16;
      previous += 16;
      next += 16;
      dp += 16;
    }
    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    _sp += remainder;
    previous += remainder;
    next += remainder;
    dp += remainder;

    get2diff(pdiffmax, ndiffmax, previous, _sp, next);
    get2diff(pdiff, ndiff, previous + 1, _sp + 1, next + 1);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + 2, _sp + 2, next + 2);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff); // __asm	pmaxub		pdiffmax, pdiff // BUG Fixed by PF 2019, this line was missing
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + pfpitch, _sp + ofpitch, next + nfpitch);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    get2diff(pdiff, ndiff, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
    pdiffmax = _mm_max_epu8(pdiffmax, pdiff);
    ndiffmax = _mm_max_epu8(ndiffmax, ndiff);
    SmoothTRepair3(dp, pdiffmax, ndiffmax, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);


    dp += pitches[0];
    previous += pitches[1];
    _sp += pitches[2];
    next += pitches[3];
  }
}

void smooth_temporal_repair3_c(BYTE* dp, const BYTE* previous, const BYTE* sp, const BYTE* next, intptr_t* pitches, int width, int height)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1;

  for (int y = 0; y < height; y++)
  {
    int pdiffmax;
    int ndiffmax;
    int pdiff;
    int ndiff;
    for (int x = 0; x < width; x++)
    {
      get2diff_c(pdiffmax, ndiffmax, previous[x], sp[x], next[x]);
      get2diff_c(pdiff, ndiff, previous[x + 1], sp[x + 1], next[x + 1]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + 2], sp[x + 2], next[x + 2]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + 2 * pfpitch], sp[x + 2 * ofpitch], next[x + 2 * nfpitch]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + 2 * pfpitch + 1], sp[x + 2 * ofpitch + 1], next[x + 2 * nfpitch + 1]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + 2 * pfpitch + 2], sp[x + 2 * ofpitch + 2], next[x + 2 * nfpitch + 2]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + pfpitch], sp[x + ofpitch], next[x + nfpitch]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      get2diff_c(pdiff, ndiff, previous[x + pfpitch + 2], sp[x + ofpitch + 2], next[x + nfpitch + 2]);
      pdiffmax = max(pdiffmax, pdiff);
      ndiffmax = max(ndiffmax, ndiff);
      SmoothTRepair3_c(dp[x], pdiffmax, ndiffmax, previous[x + pfpitch + 1], sp[x + ofpitch + 1], next[x + nfpitch + 1]);
    }

    dp += pitches[0];
    previous += pitches[1];
    sp += pitches[2];
    next += pitches[3];
  }
}

#ifndef X86_64
void	smooth_temporal_repair3_old(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{
  __asm	mov			eax, hblocks
  __asm	mov			ecx, eax
  __asm	mov			edx, previous
  __asm	mov			esi, _sp
  __asm	shl			eax, SSE_SHIFT
  __asm	mov			edi, dp
  __asm	add			eax, remainder
  __asm	mov			ebx, pitch
  __asm	sub			pitch, eax
  __asm	lea			edi, [edi + ebx + 1]
    __asm	mov			eax, next
  __asm	align		16
  __asm	middle_loop:
  get2diff_old(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
    get2diff_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2 // BUG Fixed by PF 2019, this line was missing
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  SmoothTRepair3_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
    __asm	add			esi, SSE_INCREMENT
  __asm	add			edx, SSE_INCREMENT
  __asm	add			eax, SSE_INCREMENT
  __asm	add			edi, SSE_INCREMENT
  __asm	dec			ecx
  __asm	jnz			middle_loop
  // the last pixels
  __asm	add			esi, remainder
  __asm	add			edx, remainder
  __asm	add			eax, remainder
  __asm	add			edi, remainder
  get2diff_old(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
    get2diff_old(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff_old(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  SmoothTRepair3_old([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
    __asm	add			esi, pitch
  __asm	add			edx, pitch
  __asm	add			eax, pitch
  __asm	add			edi, pitch
  __asm	dec			height
  __asm	mov			ecx, hblocks
  __asm	jnz			middle_loop
}
#endif

class SmoothTemporalRepair : public GenericVideoFilter, public PlanarAccess
{
  //HomogeneousChild oclip; // remove "have the same pitch" hack
  PClip oclip;

  // each clip can have its own pitch
  void(*st_repair)(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int width, int height);

  unsigned last_frame;
  int opt;
  
  // MT mode Registration for Avisynth+
  int __stdcall SetCacheHints(int cachehints, int frame_range) override {
    return cachehints == CACHE_GET_MTMODE ? MT_NICE_FILTER : 0;
  }

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if (n <= 0 || n - 1 >= last_frame)
      return child->GetFrame(n, env);
    PVideoFrame sf = child->GetFrame(n, env);
    /* no more HomogeneousChild, does not guarantee equal pitches
    PVideoFrame pf = oclip->GetFrame(n - 1, env);
    PVideoFrame of = oclip->GetFrame(n, env);
    PVideoFrame nf = oclip->GetFrame(n + 1, env);
    */
    PVideoFrame pf = oclip->GetFrame(n - 1, env);
    PVideoFrame of = oclip->GetFrame(n, env);
    PVideoFrame nf = oclip->GetFrame(n + 1, env);
    PVideoFrame df = env->NewVideoFrame(vi);

    for (int i = 0; i < planecount; i++)
    {
      int sfpitch = GetPitch(sf, i);

      BYTE* dp = GetWritePtr(df, i);
      int dppitch = GetPitch(df, i);
      int pfpitch = GetPitch(pf, i);
      int ofpitch = GetPitch(of, i);
      int nfpitch = GetPitch(nf, i);

      // copy the plane from sp to dp
      env->BitBlt(dp, dppitch, GetReadPtr(sf, i), sfpitch, width[i], height[i]); // FIX PF 2019: use sfpitch

      intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

      st_repair(dp, GetReadPtr(pf, i), GetReadPtr(of, i), GetReadPtr(nf, i),
        // pitch, // FIXED: eliminate HomogeneousChild. Here only one pitch is used (for speed reasons(?)) through the HomogeneousChild hack (pf, of and nf pitch is forced to have the same pitch)
        // The assumption that NewVideoFrame is giving fixed pitches is wrong for Avs+ due to frame cacheing and frame buffer reuse mechanism
        pitches, // pitch array
        width[i] - 2, height[i] - 2);
    }
    return df;
  }
public:
  SmoothTemporalRepair(PClip clip, PClip _oclip, int mode, bool grey, bool planar, int opt, IScriptEnvironment* env) : 
    GenericVideoFilter(clip), 
    PlanarAccess(vi), 
    //oclip(_oclip, grey, env),  // no more HomogenousChild
    oclip(_oclip),
    opt(opt)
  {
    if (!planar && !vi.IsPlanar())
      env->ThrowError("TemporalRepair: only planar color spaces are supported");

    CompareVideoInfo(vi, _oclip->GetVideoInfo(), "TemporalRepair", env);

    if (vi.IsY())
      grey = true;

    // PF 2019: different parameters for old inline asm, they are not accessible anymore, no HomogenousClip
    switch (mode)
    {
    case 1:
      if (opt == 0)
        st_repair = smooth_temporal_repair1_c;
      else
        st_repair = smooth_temporal_repair1_sse2;
      break;
    case 2:
      if (opt == 0)
        st_repair = smooth_temporal_repair2_c;
      else
        st_repair = smooth_temporal_repair2_sse2;
      break;
    default:
      if (opt == 0)
        st_repair = smooth_temporal_repair3_c;
      else
        st_repair = smooth_temporal_repair3_sse2;
      break;
    }

    if (grey) 
      planecount = 1;

    last_frame = vi.num_frames - 2;
    if ((int)last_frame < 0) last_frame = 0;

    /*FIXME: check min dimensions
      env->ThrowError("TemporalRepair: the width or height of the clip is too small");*/
  }
};

#define MAXTMODE	4	

bool spatial[MAXTMODE + 1] = { false, true, true, true, false };

AVSValue __cdecl CreateTemporalRepair(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  enum ARGS { CLIP, OCLIP, MODE, SMOOTH, GREY, PLANAR, OPT };
  PClip clip = args[CLIP].AsClip();
  PClip oclip = args[OCLIP].AsClip();
  bool grey = args[GREY].AsBool(false);
  // mode and smooth are the same
  int mode = args[MODE].AsInt(args[SMOOTH].AsInt(0));
  if ((unsigned)mode > MAXTMODE) env->ThrowError("TemporalRepair: illegal mode %i", mode);
  bool planar = args[PLANAR].AsBool(false);
  int opt = args[OPT].AsInt(-1);

  return spatial[mode] ? (AVSValue) new SmoothTemporalRepair(clip, oclip, mode, grey, planar, opt, env)
    : (AVSValue) new TemporalRepair(clip, oclip, mode, grey, planar, opt, env);

};

