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

#ifndef X86_64

#include <Windows.h>
#include <stdio.h>
#include <stdarg.h>
#include "planar.h"
#include "common.h"
#include "RemoveGrainT.h"

#define	SSE_INCREMENT	16
#define	SSE_SHIFT		4
#define	SSE_MOVE		movdqu
#define	SSE3_MOVE		movdqu
#define	SSE_RMOVE		movdqa
#define	SSE_MASKMOV		MASKMOVDQU
#define	SSE0			xmm0
#define	SSE1			xmm1
#define	SSE2			xmm2
#define	SSE3			xmm3
#define	SSE4			xmm4
#define	SSE5			xmm5
#define	SSE6			xmm6
#define	SSE7			xmm7

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

class	TemporalRepair : public GenericVideoFilter, public PlanarAccess
{
  void(*trepair)(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height);
  unsigned last_frame;
  PClip orig;

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
  TemporalRepair(PClip clip, PClip oclip, int mode, bool grey, bool planar, IScriptEnvironment *env) : GenericVideoFilter(clip), PlanarAccess(vi, planar && grey), orig(oclip)
  {
    CompareVideoInfo(vi, orig->GetVideoInfo(), "TemporalRepair", env);
    trepair = mode ? btemporal_repair : temporal_repair;
    last_frame = vi.num_frames >= 2 ? vi.num_frames - 2 : 0;
    if (grey)
      planes = 0;
  }

  //~TemporalRepair(){}
};

#define	get_lu(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE3_MOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2

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

#define	get_lu_reg(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE_RMOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2

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

class SmoothTemporalRepair : public GenericVideoFilter, public PlanarAccess
{
  HomogeneousChild oclip; // "have the same pitch" hack

  void(*st_repair)(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder);

  int height2[3], hblocks[3], remainder[3];
  unsigned last_frame;

  PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
  {
    if (n <= 0 || n - 1 >= last_frame)
      return child->GetFrame(n, env);
    PVideoFrame sf = child->GetFrame(n, env);
    PVideoFrame pf = oclip->GetFrame(n - 1, env);
    PVideoFrame of = oclip->GetFrame(n, env);
    PVideoFrame nf = oclip->GetFrame(n + 1, env);
    PVideoFrame df = env->NewVideoFrame(vi);

    int i = planes;
    do
    {
      BYTE* dp = GetWritePtr(df, i);
      int	pitch = GetPitch(df, i); // FIXME: df can have different pitch!
      // copy the plane from sp to dp
      env->BitBlt(dp, pitch, GetReadPtr(sf, i), pitch, width[i], height[i]);
      st_repair(dp, GetReadPtr(pf, i), GetReadPtr(of, i), GetReadPtr(nf, i),
        pitch, // FIXME: elimate HomogeneousChild. Here only one pitch is used (for speed reasons(?)) through the HomogeneousChild hack (pf, of and nf pitch is forced to have the same pitch)
        hblocks[i], height2[i], remainder[i]);
    } while (--i >= 0);
    return df;
  }
public:
  SmoothTemporalRepair(PClip clip, PClip _oclip, int mode, bool grey, bool planar, IScriptEnvironment* env) : GenericVideoFilter(clip), PlanarAccess(vi), oclip(_oclip, grey, env)
  {
    if (vi.IsYV12() + planar == 0) // FIXME: proper check
      env->ThrowError("TemporalRepair: only planar color spaces are supported");
    CompareVideoInfo(vi, _oclip->GetVideoInfo(), "TemporalRepair", env);

    switch (mode)
    {
    case 1:
      st_repair = smooth_temporal_repair1;
      break;
    case 2:
      st_repair = smooth_temporal_repair2;
      break;
    default:
      st_repair = smooth_temporal_repair3;
    }

    if (grey) planes = 0;

    last_frame = vi.num_frames - 2;
    if ((int)last_frame < 0) last_frame = 0;

    int	i = planes;
    do
    {
      height2[i] = height[i] - 2;
      // unsigned	w = width[i] - 1 - 2*smooth;
      unsigned	w = width[i] - 3;
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
  enum ARGS { CLIP, OCLIP, MODE, SMOOTH, GREY, PLANAR };
  PClip clip = args[CLIP].AsClip();
  PClip oclip = args[OCLIP].AsClip();
  bool grey = args[GREY].AsBool(false);
  int mode = args[MODE].AsInt(args[SMOOTH].AsInt(0));
  if ((unsigned)mode > MAXTMODE) env->ThrowError("TemporalRepair: illegal mode %i", mode);
  bool planar = args[PLANAR].AsBool(false);
  return spatial[mode] ? (AVSValue) new SmoothTemporalRepair(clip, oclip, mode, grey, planar, env)
    : (AVSValue) new TemporalRepair(clip, oclip, mode, grey, planar, env);
};
#endif // x64
