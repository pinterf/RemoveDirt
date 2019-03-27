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

#define	SSE_INCREMENT	16

#ifndef X86_64
// for inline asm
#define	SSE_SHIFT		4
#define	SSE_MOVE		movdqu
#define	SSE3_MOVE		movdqu
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

__forceinline void _RepairPixel(uint8_t *dest, const uint8_t *src1, const uint8_t *src2, const uint8_t *previous, const uint8_t *next)
{
  __m128i reg1, reg2, reg3, reg4;
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm SSE3_MOVE reg1, next
  reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm SSE3_MOVE reg3, previous
  reg2 = reg1; // __asm SSE_RMOVE reg2, reg1
  reg4 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src2)); // __asm SSE3_MOVE reg4, src2
  reg1 = _mm_min_epu8(reg1, reg3); // __asm pminub reg1, reg3
  reg2 = _mm_max_epu8(reg2, reg3); // __asm pmaxub reg2, reg3
  reg1 = _mm_min_epu8(reg1, reg4); // __asm pminub reg1, reg4
  reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src1)); // __asm SSE3_MOVE reg3, src1
  reg2 = _mm_max_epu8(reg2, reg4); // __asm pmaxub reg2, reg4
  reg1 = _mm_max_epu8(reg1, reg3); // __asm pmaxub reg1, reg3
  reg1 = _mm_min_epu8(reg1, reg2); // __asm pminub reg1, reg2
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), reg1); // __asm SSE_MOVE dest, reg1
}

static void _temporal_repair(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  assert(width >= 16);

  const int wmod16 = width / 16 * 16;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < wmod16; x += 16)
    {
      _RepairPixel(dp + x, sp1 + x, sp2 + x, pp + x, np + x);
    }
    // remainder, surely unaligned, overlaps but still simd
    const int last16 = width - 16;
    _RepairPixel(dp + last16, sp1 + last16, sp2 + last16, pp + last16, np + last16);
    dp += dpitch;
    sp1 += spitch1;
    sp2 += spitch2;
    pp += ppitch;
    np += npitch;
  }
}

#ifndef X86_64

#define	RepairPixel(dest, src1, src2, previous, next, reg1, reg2, reg3, reg4)	\
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


static void temporal_repair(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
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
  RepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3)
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
  RepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3)
    __asm	add			esi, spitch2
  __asm	add			edi, dpitch
  __asm	add			edx, spitch1
  __asm	add			ebx, ppitch
  __asm	add			eax, npitch
  __asm	dec			height
  __asm	jnz			_loop
}
#endif

__forceinline void _BRepairPixel(uint8_t *dest, const uint8_t *src1, const uint8_t *src2, const uint8_t *previous, const uint8_t *next)
{
  __m128i reg1, reg2, reg3, reg4, reg5, reg6;
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm SSE3_MOVE reg1, next
  reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm SSE3_MOVE reg3, previous
  reg2 = reg1; // __asm SSE_RMOVE reg2, reg1
  reg4 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src2)); // __asm SSE3_MOVE reg4, src2
  reg2 = _mm_max_epu8(reg2, reg3); //__asm pmaxub  reg2, reg3
  reg5 = reg4; // __asm SSE_RMOVE reg5, reg4
  reg1 = _mm_min_epu8(reg1, reg3); // __asm pminub  reg1, reg3
  reg6 = reg2; // __asm SSE_RMOVE reg6, reg2
  reg5 = _mm_subs_epu8(reg5, reg1); // __asm psubusb reg5, reg1
  reg6 = _mm_subs_epu8(reg6, reg4); // __asm psubusb reg6, reg4
  reg3 = reg2; // __asm SSE_RMOVE reg3, reg2
  reg5 = _mm_adds_epu8(reg5, reg5); // __asm paddusb reg5, reg5
  reg6 = _mm_adds_epu8(reg6, reg6); // __asm paddusb reg6, reg6
  reg5 = _mm_adds_epu8(reg5, reg1); // __asm paddusb reg5, reg1
  reg3 = _mm_subs_epu8(reg3, reg6); // __asm psubusb reg3, reg6
  reg5 = _mm_min_epu8(reg5, reg2); // __asm pminub  reg5, reg2
  reg3 = _mm_max_epu8(reg3, reg1); // __asm pmaxub  reg3, reg1
  reg6 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src1)); // __asm SSE3_MOVE reg6, src1
  reg1 = _mm_cmpeq_epi8(reg1, reg5); // __asm pcmpeqb reg1, reg5
  reg2 = _mm_cmpeq_epi8(reg2, reg3); // __asm pcmpeqb reg2, reg3
  reg5 = _mm_min_epu8(reg5, reg6); // __asm pminub  reg5, reg6
  reg1 = _mm_max_epu8(reg1, reg2); // __asm pmaxub  reg1, reg2
  reg5 = _mm_max_epu8(reg5, reg3); // __asm pmaxub  reg5, reg3
  reg4 = _mm_min_epu8(reg4, reg1); // __asm pminub  reg4, reg1
  reg5 = _mm_subs_epu8(reg5, reg1); // __asm psubusb reg5, reg1
  reg4 = _mm_max_epu8(reg4, reg5); //__asm pmaxub  reg4, reg5
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), reg4); // __asm SSE_MOVE dest, reg4
}

static void _btemporal_repair(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
  assert(width >= 16);

  const int wmod16 = width / 16 * 16;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < wmod16; x += 16)
    {
      _BRepairPixel(dp + x, sp1 + x, sp2 + x, pp + x, np + x);
    }
    // remainder, surely unaligned, overlaps but still simd
    const int last16 = width - 16;
    _BRepairPixel(dp + last16, sp1 + last16, sp2 + last16, pp + last16, np + last16);
    dp += dpitch;
    sp1 += spitch1;
    sp2 += spitch2;
    pp += ppitch;
    np += npitch;
  }
}

#ifndef X86_64

#define	BRepairPixel(dest, src1, src2, previous, next, reg1, reg2, reg3, reg4, reg5, reg6)	\
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

static	void	btemporal_repair(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
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
  BRepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5)
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
  BRepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5)
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
  int optOld;
  bool optSSE2;

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if (n <= 0 || n - 1 >= last_frame)
      return child->GetFrame(n, env);
    PVideoFrame pf = orig->GetFrame(n - 1, env);
    PVideoFrame sf = orig->GetFrame(n, env);
    PVideoFrame nf = orig->GetFrame(n + 1, env);
    PVideoFrame cf = child->GetFrame(n, env);
    PVideoFrame df = env->NewVideoFrame(vi);

    int i = planes;
    do
    {
      trepair(GetWritePtr(df, i), GetPitch(df, i), GetReadPtr(cf, i), GetPitch(cf, i), GetReadPtr(sf, i), GetPitch(sf, i), GetReadPtr(pf, i), GetPitch(pf, i), GetReadPtr(nf, i), GetPitch(nf, i), width[i], height[i]);
    } while (--i >= 0);
    return df;
  }

public:
  TemporalRepair(PClip clip, PClip oclip, int mode, bool grey, bool planar, int optOld, bool optSSE2, IScriptEnvironment *env) : GenericVideoFilter(clip), PlanarAccess(vi, planar && grey), orig(oclip), optOld(optOld), optSSE2(optSSE2)
  {
    CompareVideoInfo(vi, orig->GetVideoInfo(), "TemporalRepair", env);
    // mode == 0 or 4
#ifndef X86_64
    if(optOld == 0)
      trepair = mode ? btemporal_repair : temporal_repair;
    else
      trepair = mode ? _btemporal_repair : _temporal_repair;
#else
    trepair = mode ? _btemporal_repair : _temporal_repair;
#endif
    last_frame = vi.num_frames >= 2 ? vi.num_frames - 2 : 0;
    if (grey)
      planes = 0;
  }

  //~TemporalRepair(){}
};

__forceinline void _get_lu(__m128i &lower, __m128i &upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i reg1, reg2;

  upper = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm	SSE3_MOVE	upper, next
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm	SSE3_MOVE	reg1, previous
  reg2 = upper; // __asm	SSE_RMOVE	reg2, upper
  lower = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current)); // __asm	SSE3_MOVE	lower, current
  upper = _mm_max_epu8(upper, reg1); // __asm	pmaxub		upper, reg1
  reg2 = _mm_min_epu8(reg2, reg1); // __asm	pminub		reg2, reg1
  upper = _mm_subs_epu8(upper, lower); // __asm	psubusb		upper, lower
  lower = _mm_subs_epu8(lower, reg2); // __asm	psubusb		lower, reg2
}

#ifndef X86_64
// reg& reg& ptr ptr ptr reg reg
#define	get_lu(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE3_MOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2
#endif

__forceinline void _SmoothTRepair(uint8_t *dest, __m128i &lower, __m128i &upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i reg1, reg2;

  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current)); // __asm	SSE3_MOVE	reg1, current
  reg2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm	SSE3_MOVE	reg2, previous
  upper = _mm_adds_epu8(upper, reg1); // __asm	paddusb		upper, reg1
  reg1 = _mm_subs_epu8(reg1, lower); // __asm	psubusb		reg1, lower
  upper = _mm_max_epu8(upper, reg2); // __asm	pmaxub		upper, reg2
  lower = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm	SSE3_MOVE	lower, next
  reg1 = _mm_min_epu8(reg1, reg2); // __asm	pminub		reg1, reg2
  upper = _mm_max_epu8(upper, lower); // __asm	pmaxub		upper, lower
  reg2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(dest)); // __asm	SSE3_MOVE	reg2, dest
  reg1 = _mm_min_epu8(reg1, lower); // __asm	pminub		reg1, lower
  upper = _mm_min_epu8(upper, reg2); // __asm	pminub		upper, reg2
  upper = _mm_max_epu8(upper, reg1); // __asm	pmaxub		upper, reg1
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), upper); // __asm	SSE_MOVE	dest, upper
}

#ifndef X86_64
// ptr reg reg ptr ptr ptr reg reg
#define	SmoothTRepair(dest, lower, upper, previous, current, next, reg1, reg2)	\
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

void _smooth_temporal_repair1(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int hblocks, int height, int remainder)
{
  // #   X    X
  // X new_dp X
  // X   X    X
  
  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, _sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1; //  __asm	lea			edi, [edi + pitch + 1]
  // remainder: corrects pointers back to process exactly the 16 last pixels 

  for(int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {
    __m128i SSE0 = _mm_undefined_si128();
    __m128i SSE1 = _mm_undefined_si128();
    __m128i SSE2 = _mm_undefined_si128();
    __m128i SSE3 = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      _get_lu(SSE0, SSE1, previous, _sp, next);
      _get_lu(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _SmoothTRepair(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);
      dp += 16; //  __asm	add			edi, SSE_INCREMENT
      previous += 16; // __asm	add			edx, SSE_INCREMENT
      _sp += 16; // __asm	add			esi, SSE_INCREMENT
      next += 16; // __asm	add			eax, SSE_INCREMENT
    }

    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    _sp += remainder; // __asm	add			esi, remainder
    previous += remainder; // __asm	add			edx, remainder
    next += remainder; //  __asm	add			eax, remainder
    dp += remainder; // __asm	add			edi, remainder

    _get_lu(SSE0, SSE1, previous, _sp, next);
    _get_lu(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _SmoothTRepair(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);

    dp += pitches[0]; // __asm	add			edi, pitch
    previous += pitches[1]; // __asm	add			edx, pitch
    _sp += pitches[2]; // __asm	add			esi, pitch
    next += pitches[3]; // __asm	add			eax, pitch
  }
}

#ifndef X86_64
void	smooth_temporal_repair1(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
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
  get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
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
  get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
    __asm	add			esi, pitch
  __asm	add			edx, pitch
  __asm	add			eax, pitch
  __asm	add			edi, pitch
  __asm	dec			height
  __asm	mov			ecx, hblocks
  __asm	jnz			middle_loop
}
#endif

__forceinline void _get_lu_reg(__m128i &lower, __m128i &upper, const uint8_t *previous, __m128i &current, const uint8_t *next)
{
  __m128i reg1, reg2;

  upper = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm	SSE3_MOVE	upper, next
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm	SSE3_MOVE	reg1, previous
  reg2 = upper; // __asm	SSE_RMOVE	reg2, upper
  // only difference between _get_lu and _get_lu_reg: here register
  lower = current; // __asm	SSE_RMOVE	lower,			current
  upper = _mm_max_epu8(upper, reg1); // __asm	pmaxub		upper, reg1
  reg2 = _mm_min_epu8(reg2, reg1); // __asm	pminub		reg2, reg1
  upper = _mm_subs_epu8(upper, lower); // __asm	psubusb		upper, lower
  lower = _mm_subs_epu8(lower, reg2); // __asm	psubusb		lower, reg2
}

#ifndef X86_64
// reg& reg& ptr reg& ptr reg reg
#define	get_lu_reg(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE_RMOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2
#endif

__forceinline void _SmoothTRepair2(uint8_t *dest, __m128i &lower, __m128i &upper, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i reg1, reg2, reg3;
  __m128i reg4 = _mm_undefined_si128();
  __m128i reg5 = _mm_undefined_si128();

  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current)); // __asm	SSE3_MOVE	reg1, current
  _get_lu_reg(reg4, reg5, previous, reg1, next); // get_lu_reg(reg4, reg5, previous, reg1, next, reg2, reg3)
  upper = _mm_max_epu8(upper, reg5); // __asm	pmaxub		upper, reg5
  lower = _mm_max_epu8(lower, reg4); // __asm	pmaxub		lower, reg4
  reg2 = reg1; // __asm	SSE_RMOVE	reg2, reg1
  upper = _mm_max_epu8(upper, lower); // __asm	pmaxub		upper, lower
  reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(dest)); // __asm	SSE3_MOVE	reg3, dest
  reg1 = _mm_adds_epu8(reg1, upper); // __asm	paddusb		reg1, upper
  reg2 = _mm_subs_epu8(reg2, upper); // __asm	psubusb		reg2, upper
  reg1 = _mm_min_epu8(reg1, reg3); // __asm	pminub		reg1, reg3
  reg1 = _mm_max_epu8(reg1, reg2); // __asm	pmaxub		reg1, reg2
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), reg1); // __asm	SSE_MOVE	dest, reg1
}

#ifndef X86_64
#define	SmoothTRepair2(dest, lower, upper, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
__asm	SSE3_MOVE	reg1,			current				\
		get_lu_reg(reg4, reg5, previous, reg1, next, reg2, reg3)	\
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

void	_smooth_temporal_repair2(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int hblocks, int height, int remainder)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, _sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1; //  __asm	lea			edi, [edi + pitch + 1]
  // remainder: corrects pointers back to process exactly the 16 last pixels 

  for (int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {
    __m128i SSE0 = _mm_undefined_si128();
    __m128i SSE1 = _mm_undefined_si128();
    __m128i SSE2 = _mm_undefined_si128();
    __m128i SSE3 = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      _get_lu(SSE0, SSE1, previous, _sp, next);
      _get_lu(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _get_lu(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      _SmoothTRepair2(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);
      _sp += 16; // __asm	add			esi, SSE_INCREMENT
      previous += 16; // __asm	add			edx, SSE_INCREMENT
      next += 16; // __asm	add			eax, SSE_INCREMENT
      dp += 16; //  __asm	add			edi, SSE_INCREMENT
    }

    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    _sp += remainder; // __asm	add			esi, remainder
    previous += remainder; // __asm	add			edx, remainder
    next += remainder; //  __asm	add			eax, remainder
    dp += remainder; // __asm	add			edi, remainder

    _get_lu(SSE0, SSE1, previous, _sp, next);
    _get_lu(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _get_lu(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    _SmoothTRepair2(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);

    dp += pitches[0]; // __asm	add			edi, pitch
    previous += pitches[1]; // __asm	add			edx, pitch
    _sp += pitches[2]; // __asm	add			esi, pitch
    next += pitches[3]; // __asm	add			eax, pitch
  }
}

#ifndef X86_64
void	smooth_temporal_repair2(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
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
  get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair2([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
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
  get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
    get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
    __asm	pmaxub		SSE1, SSE3
  __asm	pmaxub		SSE0, SSE2
  SmoothTRepair2([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
    __asm	add			esi, pitch
  __asm	add			edx, pitch
  __asm	add			eax, pitch
  __asm	add			edi, pitch
  __asm	dec			height
  __asm	mov			ecx, hblocks
  __asm	jnz			middle_loop
}
#endif

__forceinline void _get2diff(__m128i &pdiff, __m128i &ndiff, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i reg1, reg2;

  __m128i reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current)); // __asm	SSE3_MOVE	reg3,			current
  pdiff = _mm_loadu_si128(reinterpret_cast<const __m128i *>(previous)); // __asm	SSE3_MOVE	pdiff, previous
  reg1 = reg3; // __asm	SSE_RMOVE	reg1, reg3
  ndiff = _mm_loadu_si128(reinterpret_cast<const __m128i *>(next)); // __asm	SSE3_MOVE	ndiff, next
  reg2 = reg3; // __asm	SSE_RMOVE	reg2, reg3
  reg1 = _mm_subs_epu8(reg1, pdiff); // __asm	psubusb		reg1, pdiff
  reg2 = _mm_subs_epu8(reg2, ndiff); // __asm	psubusb		reg2, ndiff
  ndiff = _mm_subs_epu8(ndiff, reg3); // __asm	psubusb		ndiff, reg3
  pdiff = _mm_subs_epu8(pdiff, reg3); // __asm	psubusb		pdiff, reg3
  pdiff = _mm_max_epu8(pdiff, reg1); // __asm	pmaxub		pdiff, reg1
  ndiff = _mm_max_epu8(ndiff, reg2); // __asm	pmaxub		ndiff, reg2
}


#ifndef X86_64
#define	get2diff(pdiff, ndiff, previous, current, next, reg1, reg2, reg3)	\
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


__forceinline void _SmoothTRepair3(uint8_t *dest, __m128i &pmax, __m128i &nmax, const uint8_t *previous, const uint8_t *current, const uint8_t *next)
{
  __m128i reg2, reg3;

  __m128i reg1 = _mm_undefined_si128();
  __m128i reg4 = _mm_undefined_si128();
  __m128i reg5 = _mm_undefined_si128();

  _get2diff(reg4, reg5, previous, current, next);
  reg1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(current));
  pmax = _mm_max_epu8(pmax, reg4); // __asm	pmaxub		pmax, reg4
  nmax = _mm_max_epu8(nmax, reg5); // __asm	pmaxub		nmax, reg5
  reg2 = reg1; //  __asm	SSE_RMOVE	reg2, reg1
  pmax = _mm_min_epu8(pmax, nmax); // __asm	pminub		pmax, nmax
  reg3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(dest)); // __asm	SSE3_MOVE	reg3, dest
  reg1 = _mm_adds_epu8(reg1, pmax); // __asm	paddusb		reg1, pmax
  reg2 = _mm_subs_epu8(reg1, pmax); // __asm	psubusb		reg2, pmax
  reg1 = _mm_min_epu8(reg1, reg3); // __asm	pminub		reg1, reg3
  reg1 = _mm_max_epu8(reg1, reg2); // __asm	pmaxub		reg1, reg2
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dest), reg1); // __asm	SSE_MOVE	dest, reg1
}

#ifndef X86_64
#define	SmoothTRepair3(dest, pmax, nmax, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
		get2diff(reg4, reg5, previous, current, next, reg2, reg3, reg1)	\
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


void _smooth_temporal_repair3(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int hblocks, int height, int remainder)
{
  // #   X    X
  // X new_dp X
  // X   X    X

  // memo: const intptr_t pitches[4] = { dppitch, pfpitch, ofpitch, nfpitch };

  // #: original_dp, previous, _sp, next
  const intptr_t dppitch = pitches[0];
  const intptr_t pfpitch = pitches[1];
  const intptr_t ofpitch = pitches[2];
  const intptr_t nfpitch = pitches[3];

  dp = dp + dppitch + 1; //  __asm	lea			edi, [edi + pitch + 1]
  // remainder: corrects pointers back to process exactly the 16 last pixels 

  for (int i = 0; i < 4; i++)
    pitches[i] -= (hblocks * 16 + remainder);

  for (int y = 0; y < height; y++)
  {
    __m128i SSE0 = _mm_undefined_si128();
    __m128i SSE1 = _mm_undefined_si128();
    __m128i SSE2 = _mm_undefined_si128();
    __m128i SSE3 = _mm_undefined_si128();
    for (int x = 0; x < hblocks; x++)
    {
      _get2diff(SSE0, SSE1, previous, _sp, next);
      _get2diff(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2 // BUG Fixed by PF 2019, this line was missing
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _get2diff(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
      SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
      SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
      _SmoothTRepair3(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);
      _sp += 16; // __asm	add			esi, SSE_INCREMENT
      previous += 16; // __asm	add			edx, SSE_INCREMENT
      next += 16; // __asm	add			eax, SSE_INCREMENT
      dp += 16; //  __asm	add			edi, SSE_INCREMENT
    }
    // the last pixels. Do simd to the rightmost 16 pixels, overlap is possible but no problem
    _sp += remainder; // __asm	add			esi, remainder
    previous += remainder; // __asm	add			edx, remainder
    next += remainder; //  __asm	add			eax, remainder
    dp += remainder; // __asm	add			edi, remainder

    _get2diff(SSE0, SSE1, previous, _sp, next);
    _get2diff(SSE2, SSE3, previous + 1, _sp + 1, next + 1);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + 2, _sp + 2, next + 2);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + 2 * pfpitch, _sp + 2 * ofpitch, next + 2 * nfpitch);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + 2 * pfpitch + 1, _sp + 2 * ofpitch + 1, next + 2 * nfpitch + 1);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + 2 * pfpitch + 2, _sp + 2 * ofpitch + 2, next + 2 * nfpitch + 2);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2 // BUG Fixed by PF 2019, this line was missing
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + pfpitch, _sp + ofpitch, next + nfpitch);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _get2diff(SSE2, SSE3, previous + pfpitch + 2, _sp + ofpitch + 2, next + nfpitch + 2);
    SSE0 = _mm_max_epu8(SSE0, SSE2); // __asm	pmaxub		SSE0, SSE2
    SSE1 = _mm_max_epu8(SSE1, SSE3); // __asm	pmaxub		SSE1, SSE3
    _SmoothTRepair3(dp, SSE0, SSE1, previous + pfpitch + 1, _sp + ofpitch + 1, next + nfpitch + 1);

    dp += pitches[0]; // __asm	add			edi, pitch
    previous += pitches[1]; // __asm	add			edx, pitch
    _sp += pitches[2]; // __asm	add			esi, pitch
    next += pitches[3]; // __asm	add			eax, pitch
  }
}

#ifndef X86_64
void	smooth_temporal_repair3(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
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
  get2diff(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
    get2diff(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2 // BUG Fixed by PF 2019, this line was missing
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  SmoothTRepair3([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
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
  get2diff(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
    get2diff(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx], [esi + 2 * ebx], [eax + 2 * ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx + 1], [esi + 2 * ebx + 1], [eax + 2 * ebx + 1], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + 2 * ebx + 2], [esi + 2 * ebx + 2], [eax + 2 * ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  get2diff(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
    __asm	pmaxub		SSE0, SSE2
  __asm	pmaxub		SSE1, SSE3
  SmoothTRepair3([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
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
  void(*st_repair)(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, intptr_t* pitches, int hblocks, int height, int remainder);

  int height2[3], hblocks[3], remainder[3];
  unsigned last_frame;
  int optOld;
  bool optSSE2;

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if (n <= 0 || n - 1 >= last_frame)
      return child->GetFrame(n, env);
    PVideoFrame sf = child->GetFrame(n, env);
    /* no more HomogeneousChild
    PVideoFrame pf = oclip->GetFrame(n - 1, env);
    PVideoFrame of = oclip->GetFrame(n, env);
    PVideoFrame nf = oclip->GetFrame(n + 1, env);
    */
    PVideoFrame pf = oclip->GetFrame(n - 1, env);
    PVideoFrame of = oclip->GetFrame(n, env);
    PVideoFrame nf = oclip->GetFrame(n + 1, env);
    PVideoFrame df = env->NewVideoFrame(vi);

    int i = planes;
    do
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
        // pitch, // FIXME: eliminate HomogeneousChild. Here only one pitch is used (for speed reasons(?)) through the HomogeneousChild hack (pf, of and nf pitch is forced to have the same pitch)
        // The assumption that NewVideoFrame is giving fixed pitches is wrong for Avs+ due to frame cacheing and frame buffer reuse mechanism
        pitches, // pitch array
        hblocks[i], height2[i], remainder[i]);
    } while (--i >= 0);
    return df;
  }
public:
  SmoothTemporalRepair(PClip clip, PClip _oclip, int mode, bool grey, bool planar, int optOld, bool optSSE2, IScriptEnvironment* env) : GenericVideoFilter(clip), PlanarAccess(vi), 
    //oclip(_oclip, grey, env),  // no more HomogenousChild
    oclip(_oclip),
    optOld(optOld), optSSE2(optSSE2)
  {
    if (vi.IsYV12() + planar == 0) // FIXME: proper check
      env->ThrowError("TemporalRepair: only planar color spaces are supported");
    CompareVideoInfo(vi, _oclip->GetVideoInfo(), "TemporalRepair", env);

    // PF 2019: different parameters for old inline asm, they are not accessible anymore, no HomogenousClip
    switch (mode)
    {
    case 1:
#ifndef X86_64
      /*
      if (optOld == 0)
        st_repair = smooth_temporal_repair1; // old inline asm
      else
        */
      st_repair = _smooth_temporal_repair1;
#else
      st_repair = _smooth_temporal_repair1;
#endif
      break;
    case 2:
#ifndef X86_64
      /*
      if (optOld == 0)
        st_repair = smooth_temporal_repair2; // old inline asm
      else
        st_repair = _smooth_temporal_repair2;
      */
      st_repair = _smooth_temporal_repair2;
#else
      st_repair = _smooth_temporal_repair2;
#endif
      break;
    default:
#ifndef X86_64
      /*
      if (optOld == 0)
        st_repair = smooth_temporal_repair3; // old inline asm
      else if(optOld == 1)
        st_repair = _smooth_temporal_repair3;
      */
      st_repair = _smooth_temporal_repair3;
#else
#if 0
      if (optOld == 0 || optOld == 1)
        st_repair = _smooth_temporal_repair3;
#endif
      st_repair = _smooth_temporal_repair3;
#endif
      break;
    }

    if (grey) planes = 0;

    last_frame = vi.num_frames - 2;
    if ((int)last_frame < 0) last_frame = 0;

    int	i = planes;
    do
    {
      height2[i] = height[i] - 2; // 3x3 matrices, top and bottom line is unaffected
      unsigned w = width[i] - 3;
      hblocks[i] = w / SSE_INCREMENT;
      remainder[i] = (w & (SSE_INCREMENT - 1)) - (SSE_INCREMENT - 1);
    } while (--i >= 0);

    if ((hblocks[planes] <= 0) || (height2[planes] <= 0))
      env->ThrowError("TemporalRepair: the width or height of the clip is too small");
  }
  //~SmoothTemporalRepair(){}
};

#define MAXTMODE	4	

bool spatial[MAXTMODE + 1] = { false, true, true, true, false };

AVSValue __cdecl CreateTemporalRepair(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  enum ARGS { CLIP, OCLIP, MODE, SMOOTH, GREY, PLANAR, OPTOLD, OPTSSE2 };
  PClip clip = args[CLIP].AsClip();
  PClip oclip = args[OCLIP].AsClip();
  bool grey = args[GREY].AsBool(false);
  // mode and smooth are the same
  int mode = args[MODE].AsInt(args[SMOOTH].AsInt(0));
  if ((unsigned)mode > MAXTMODE) env->ThrowError("TemporalRepair: illegal mode %i", mode);
  bool planar = args[PLANAR].AsBool(false);
  int optOld = args[OPTSSE2].AsInt(0);
  bool optSSE2 = args[OPTSSE2].AsBool(true); // RFU
  return spatial[mode] ? (AVSValue) new SmoothTemporalRepair(clip, oclip, mode, grey, planar, optOld, optSSE2, env)
    : (AVSValue) new TemporalRepair(clip, oclip, mode, grey, planar, optOld, optSSE2, env);
};

