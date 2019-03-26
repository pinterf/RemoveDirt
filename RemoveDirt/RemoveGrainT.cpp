#define LOGO		"RemoveGrainT 1.0\n"
// An Avisynth plugin for removing grain from progressive video
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

#define	MODIFYPLUGIN	1// creat Repair plugin instead of RemoveGrain, 0 = compatible with RemoveGrain
//#define	SHARPEN	1
//#define	BLUR 1
//#define	SSE2_TEST		// ISSE2 version that can be used side by side with the SSE version
//#define	DEBUG_NAME		// for debugging
//#define	ISSE	2		// P4, Athlon 64, Sempron 3100
#define	ISSE	3		// Prescott P4	
//#define	CVERSION		// for debugging only
#define	ALIGNPITCH
#define	SMOOTH2

#define	DEFAULT_MODE	2
#define	DEFAULT_RGLIMIT	0

#define VC_EXTRALEAN 
#include <Windows.h>
#include <stdio.h>
#include <stdarg.h>

#include "avisynth.h"
#include "planar.h"
#include "AvsRecursion.h"

static	IScriptEnvironment	*AVSenvironment;

#ifdef	SSE2_TEST
#ifndef	ISSE
#define	ISSE	2
#endif
#ifndef	DEBUG_NAME
#define	DEBUG_NAME
#endif
#endif

#ifdef	CVERSION
#ifndef	DEBUG_NAME
#define	DEBUG_NAME
#endif
#endif

#ifndef	ISSE
#define	ISSE	1
#endif

#if		ISSE > 1			
#define	CPUFLAGS		CPUF_SSE2
#else
#define	CPUFLAGS		CPUF_INTEGER_SSE
#endif

#if	defined(SHARPEN) || defined(BLUR)
#define	SHLUR
#endif

#if	defined(SHLUR) && defined(MODIFYPLUGIN)
#error "SHARPEN or BLUR cannot be combined with MODIFYPLUGIN"
#endif

#if	1
#define DPRINTF_SIZE	200

static	void	debug_printf(const char *format, ...)
{
	char	buffer[DPRINTF_SIZE];
	va_list	args;
	va_start(args, format);
	vsprintf_s(buffer, DPRINTF_SIZE, format, args);
	va_end(args);
	OutputDebugString(buffer);	
}
#endif

#define	COMPARE_MASK	(~24)

static	void CompareVideoInfo(VideoInfo &vi1, const VideoInfo &vi2, const char *progname)
{	
	if( (vi1.width != vi2.width) || (vi1.height != vi2.height) || ( (vi1.pixel_type & COMPARE_MASK) != (vi2.pixel_type & COMPARE_MASK) ))
	{
#if	1
		debug_printf("widths = %u, %u, heights = %u, %u, color spaces = %X, %X\n"
						, vi1.width, vi2.width, vi1.height, vi2.height, vi1.pixel_type, vi2.pixel_type);
#endif
		AVSenvironment->ThrowError("%s: clips must be of equal type", progname);
	}
	if(vi1.num_frames > vi2.num_frames) vi1.num_frames = vi2.num_frames;
}

#ifdef	TESTCOMPARE
unsigned	testcompare(const BYTE *dp, int dpitch, const BYTE *pp, int ppitch, int width, int height)
{
	int i = height;
	--dp; --pp;
	unsigned	diffsum = 0;
	do
	{
		int j = width;
		do
		{
			int	diff = dp[j] - pp[j];
			if( diff < 0 ) diff = -diff;
			diffsum += diff;
		} while( --j );
		dp += dpitch;
		pp += ppitch;
	} while( --i );
	return	diffsum;
}
#endif	// TESTCOMPARE

#if		ISSE > 1
#define	SSE_INCREMENT	16
#define	SSE_SHIFT		4
#define	SSE_MOVE		movdqu
#if		ISSE > 2
#define	SSE3_MOVE		lddqu
#else
#define	SSE3_MOVE		movdqu
#endif
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
#define	SSE_EMMS	
#else
#define	SSE_INCREMENT	8
#define	SSE_SHIFT		3
#define	SSE_MOVE		movq
#define	SSE3_MOVE		movq
#define	SSE_RMOVE		movq
#define	SSE_MASKMOV		MASKMOVQ
#define	SSE0			mm0
#define	SSE1			mm1
#define	SSE2			mm2
#define	SSE3			mm3
#define	SSE4			mm4
#define	SSE5			mm5
#define	SSE6			mm6
#define	SSE7			mm7
#define	SSE_EMMS		__asm	emms
#endif	// ISSE

#if	BLUR == 1
#define	blur(center, min, max, reg1, reg2)\
__asm	SSE_RMOVE	reg2,				center			\
__asm	psubusb		max,				center			\
__asm	psubusb		reg2,				min				\
__asm	SSE_RMOVE	reg1,				max				\
__asm	SSE_RMOVE	min,				reg2			\
__asm	psubusb		max,				reg2			\
__asm	psubusb		min,				reg1			\
__asm	psrlw		max,				1				\
__asm	psrlw		min,				1				\
__asm	pminub		reg2,				max				\
__asm	pminub		reg1,				min				\
__asm	paddusb		center,				reg2			\
__asm	psubusb		center,				reg1
#elif	BLUR == 2
__asm	pminub		center,				max				\
__asm	pmaxub		center,				min				\
__asm	SSE_RMOVE	reg2,				center			\
__asm	psubusb		max,				center			\
__asm	psubusb		reg2,				min				\
__asm	SSE_RMOVE	reg1,				max				\
__asm	SSE_RMOVE	min,				reg2			\
__asm	psubusb		max,				reg2			\
__asm	psubusb		min,				reg1			\
__asm	psrlw		max,				1				\
__asm	psrlw		min,				1				\
__asm	pminub		reg2,				max				\
__asm	pminub		reg1,				min				\
__asm	paddusb		center,				reg2			\
__asm	psubusb		center,				reg1
#endif

#ifdef	SHARPEN

static	const __declspec(align(SSE_INCREMENT)) unsigned short rshift[3][SSE_INCREMENT / 2] =
{
	{
		0,0,0,0		
#if	SSE_INCREMENT == 16
		,0,0,0,0		
#endif
	},
	{
		1,0,0,0		
#if	SSE_INCREMENT == 16
		, 0,0,0,0			
#endif
	},
	{
		2,0,0,0		
#if	SSE_INCREMENT == 16
		, 0,0,0,0
#endif
	}
};

#define	SHIFT_MASK0	255
#define	SHIFT_MASK1	127
#define	SHIFT_MASK2	63
static	const __declspec(align(SSE_INCREMENT)) BYTE	shift_mask[3][SSE_INCREMENT] =
{
	{
		SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0		
#if	SSE_INCREMENT == 16
		, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0, SHIFT_MASK0
#endif
	},
	{
		SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1		
#if	SSE_INCREMENT == 16
		, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1, SHIFT_MASK1
#endif
	},
	{
		SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2		
#if	SSE_INCREMENT == 16
		, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2, SHIFT_MASK2
#endif
	}
};

#if	SHARPEN == 1
// only sharpen
#define	sharpen(center, min, max, rshift, SHIFT_MASK1, reg1, reg2)\
__asm	SSE_RMOVE	reg2,				center			\
__asm	psubusb		max,				center			\
__asm	psubusb		reg2,				min				\
__asm	SSE_RMOVE	reg1,				max				\
__asm	SSE_RMOVE	min,				reg2			\
__asm	psubusb		max,				reg2			\
__asm	psubusb		min,				reg1			\
__asm	psrlw		reg2,				rshift			\
__asm	psrlw		reg1,				rshift			\
__asm	pand		reg2,				SHIFT_MASK1		\
__asm	pand		reg1,				SHIFT_MASK1		\
__asm	pminub		reg2,				max				\
__asm	pminub		reg1,				min				\
__asm	psubusb		center,				reg2			\
__asm	paddusb		center,				reg1

#elif	SHARPEN == 2
// clip and sharpen
#define	sharpen(center, min, max, rshift, SHIFT_MASK1, reg1, reg2)\
__asm	pminub		center,				max				\
__asm	pmaxub		center,				min				\
__asm	SSE_RMOVE	reg2,				center			\
__asm	psubusb		max,				center			\
__asm	psubusb		reg2,				min				\
__asm	SSE_RMOVE	reg1,				max				\
__asm	SSE_RMOVE	min,				reg2			\
__asm	psubusb		max,				reg2			\
__asm	psubusb		min,				reg1			\
__asm	psrlw		reg2,				rshift			\
__asm	psrlw		reg1,				rshift			\
__asm	pand		reg2,				SHIFT_MASK1		\
__asm	pand		reg1,				SHIFT_MASK1		\
__asm	pminub		reg2,				max				\
__asm	pminub		reg1,				min				\
__asm	psubusb		center,				reg2			\
__asm	paddusb		center,				reg1
#endif
#endif	// SHARPEN

#ifdef	BLUR
#define	sharpen(center, min, max, rshift, SHIFT_MASK1, reg1, reg2)	blur(center, min, max, reg1, reg2)
#endif

#ifdef	SHARPEN

void	do_nothing(BYTE *dp, int dpitch, const BYTE *sp, int spitch, int hblocks, int remainder, int incpitch, int height, int strength)
{
}

void	copy_plane(BYTE *dp, int dpitch, const BYTE *sp, int spitch, int hblocks, int remainder, int incpitch, int height, int strength)
{
	AVSenvironment->BitBlt(dp, dpitch, sp, spitch, hblocks * SSE_INCREMENT + 2 * (SSE_INCREMENT + 1) + remainder, height);
}

#else // SHARPEN

void	do_nothing(BYTE *dp, int dpitch, const BYTE *sp, int spitch, int hblocks, int remainder, int incpitch, int height)
{
}

void	copy_plane(BYTE *dp, int dpitch, const BYTE *sp, int spitch, int hblocks, int remainder, int incpitch, int height)
{
	AVSenvironment->BitBlt(dp, dpitch, sp, spitch, hblocks * SSE_INCREMENT + 2 * (SSE_INCREMENT + 1) + remainder, height);
}
#endif	// SHARPEN

#ifdef	MODIFYPLUGIN

#if	ISSE > 1
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
#else
#define	RepairPixel(dest, src1, src2, previous, next, reg1, reg2, reg3, reg4)	\
__asm	SSE3_MOVE	reg1,			next				\
__asm	SSE3_MOVE	reg3,			previous			\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	SSE3_MOVE	reg4,			src2				\
__asm	pminub		reg1,			reg3				\
__asm	pmaxub		reg2,			reg3				\
__asm	pminub		reg1,			reg4				\
__asm	pmaxub		reg2,			reg4				\
__asm	pmaxub		reg1,			src1				\
__asm	pminub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1
#endif

static	void	temporal_repair(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height)
{
	int	blocks = --width / SSE_INCREMENT;
	int	remainder = (width & (SSE_INCREMENT - 1)) - (SSE_INCREMENT - 1);
	width -= SSE_INCREMENT - 1;
	dpitch -= width;
	spitch1 -= width;
	spitch2 -= width;
	ppitch -= width;
	npitch -= width; 
__asm	mov			ebx,				pp
__asm	mov			edx,				sp1
__asm	mov			esi,				sp2
__asm	mov			edi,				dp
__asm	mov			eax,				np
__asm	mov			ecx,				blocks
__asm	align		16
__asm	_loop:
		RepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3)
__asm	add			eax,				SSE_INCREMENT
__asm	add			esi,				SSE_INCREMENT
__asm	add			edi,				SSE_INCREMENT
__asm	add			edx,				SSE_INCREMENT
__asm	add			ebx,				SSE_INCREMENT
__asm	loop		_loop
// the last pixels
__asm	add			esi,				remainder
__asm	add			edi,				remainder
__asm	add			edx,				remainder
__asm	mov			ecx,				blocks
__asm	add			ebx,				remainder
__asm	add			eax,				remainder
		RepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3) 
__asm	add			esi,				spitch2
__asm	add			edi,				dpitch
__asm	add			edx,				spitch1
__asm	add			ebx,				ppitch
__asm	add			eax,				npitch
__asm	dec			height
__asm	jnz			_loop
}

#if	ISSE > 1
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
#else
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
__asm	pcmpeqb		reg1,			reg5				\
__asm	pcmpeqb		reg2,			reg3				\
__asm	pminub		reg5,			src1				\
__asm	pmaxub		reg1,			reg2				\
__asm	pmaxub		reg5,			reg3				\
__asm	pminub		reg4,			reg1				\
__asm	psubusb		reg5,			reg1				\
__asm	pmaxub		reg4,			reg5				\
__asm	SSE_MOVE	dest,			reg4
#endif	// ISSE > 1

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
__asm	mov			ebx,				pp
__asm	mov			edx,				sp1
__asm	mov			esi,				sp2
__asm	mov			edi,				dp
__asm	mov			eax,				np
__asm	mov			ecx,				blocks
__asm	align		16
__asm	_loop:
		BRepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5)
__asm	add			eax,				SSE_INCREMENT
__asm	add			esi,				SSE_INCREMENT
__asm	add			edi,				SSE_INCREMENT
__asm	add			edx,				SSE_INCREMENT
__asm	add			ebx,				SSE_INCREMENT
__asm	loop		_loop
// the last pixels
__asm	add			esi,				remainder
__asm	add			edi,				remainder
__asm	mov			ecx,				blocks
__asm	add			ebx,				remainder
__asm	add			eax,				remainder
		BRepairPixel([edi], [edx], [esi], [ebx], [eax], SSE0, SSE1, SSE2, SSE3, SSE4, SSE5) 
__asm	add			esi,				spitch2
__asm	add			edi,				dpitch
__asm	add			edx,				spitch1
__asm	add			ebx,				ppitch
__asm	add			eax,				npitch
__asm	dec			height
__asm	jnz			_loop
}

class	TemporalRepair : public GenericVideoFilter, public PlanarAccess
{
	void			(*trepair)(BYTE *dp, int dpitch, const BYTE *sp1, int spitch1, const BYTE *sp2, int spitch2, const BYTE *pp, int ppitch, const BYTE *np, int npitch, int width, int height);
	unsigned		last_frame;
	PClip			orig;
	
	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
	{
		if( ((unsigned)(n - 1) >= last_frame) ) return child->GetFrame(n, env);
		PVideoFrame	pf = orig->GetFrame(n - 1, env);
		PVideoFrame	sf = orig->GetFrame(n, env);
		PVideoFrame	nf = orig->GetFrame(n + 1, env);
		PVideoFrame cf = child->GetFrame(n, env);
		PVideoFrame	df = env->NewVideoFrame(vi);

		int i = planes;
		do
		{
			trepair(GetWritePtr(df, i), GetPitch(df, i), GetReadPtr(cf, i), GetPitch(cf, i), GetReadPtr(sf, i), GetPitch(sf, i), GetReadPtr(pf, i), GetPitch(pf, i), GetReadPtr(nf, i), GetPitch(nf, i), width[i], height[i]);
		} while( --i >= 0 );
		SSE_EMMS
		return df;
	}

public:
	TemporalRepair(PClip clip, PClip oclip, int mode, bool grey, bool planar) : GenericVideoFilter(clip), PlanarAccess(vi, planar && grey), orig(oclip)
	{
		CompareVideoInfo(vi, orig->GetVideoInfo(), "TemporalRepair");
		//child->SetCacheHints(CACHE_RANGE, 0);
		//orig->SetCacheHints(CACHE_RANGE, 2);
		trepair = mode ? btemporal_repair : temporal_repair;
		last_frame = vi.num_frames - 2;
		if( (int) last_frame < 0 ) last_frame = 0;
		if( grey ) planes = 0;
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

#if	ISSE > 1
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
#else
#define	SmoothTRepair(dest, lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	reg1,			current				\
__asm	SSE3_MOVE	reg2,			previous			\
__asm	paddusb		upper,			reg1				\
__asm	psubusb		reg1,			lower				\
__asm	pmaxub		upper,			reg2				\
__asm	SSE3_MOVE	lower,			next				\
__asm	pminub		reg1,			reg2				\
__asm	pmaxub		upper,			lower				\
__asm	pminub		reg1,			lower				\
__asm	pminub		upper,			dest				\
__asm	pmaxub		upper,			reg1				\
__asm	SSE_MOVE	dest,			upper
#endif

void	smooth_temporal_repair1(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{	
__asm	mov			eax,				hblocks
__asm	mov			ecx,				eax
__asm	mov			edx,				previous
__asm	mov			esi,				_sp
__asm	shl			eax,				SSE_SHIFT
__asm	mov			edi,				dp
__asm	add			eax,				remainder
__asm	mov			ebx,				pitch
__asm	sub			pitch,				eax
__asm	lea			edi,				[edi + ebx + 1]
__asm	mov			eax,				next
__asm	align		16
__asm	middle_loop:
		get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
		get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		SmoothTRepair([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
__asm	add			esi,				SSE_INCREMENT
__asm	add			edx,				SSE_INCREMENT
__asm	add			eax,				SSE_INCREMENT
__asm	add			edi,				SSE_INCREMENT
__asm	dec			ecx
__asm	jnz			middle_loop
// the last pixels
__asm	add			esi,				remainder
__asm	add			edx,				remainder
__asm	add			eax,				remainder
__asm	add			edi,				remainder
		get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
		get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		SmoothTRepair([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE6, SSE7)
__asm	add			esi,				pitch
__asm	add			edx,				pitch
__asm	add			eax,				pitch
__asm	add			edi,				pitch
__asm	dec			height
__asm	mov			ecx,				hblocks
__asm	jnz			middle_loop
}

#ifdef	SMOOTH2

#define	get_lu_reg(lower, upper, previous, current, next, reg1, reg2)	\
__asm	SSE3_MOVE	upper,			next				\
__asm	SSE3_MOVE	reg1,			previous			\
__asm	SSE_RMOVE	reg2,			upper				\
__asm	SSE_RMOVE	lower,			current				\
__asm	pmaxub		upper,			reg1				\
__asm	pminub		reg2,			reg1				\
__asm	psubusb		upper,			lower				\
__asm	psubusb		lower,			reg2

#if	ISSE > 1
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
#else
#define	SmoothTRepair2(dest, lower, upper, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
__asm	SSE3_MOVE	reg1,			current				\
		get_lu_reg(reg4, reg5, previous, reg1, next, reg2, reg3)	\
__asm	pmaxub		upper,			reg5				\
__asm	pmaxub		lower,			reg4				\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	pmaxub		upper,			lower				\
__asm	paddusb		reg1,			upper				\
__asm	psubusb		reg2,			upper				\
__asm	pminub		reg1,			dest				\
__asm	pmaxub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1
#endif

void	smooth_temporal_repair2(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{	
__asm	mov			eax,				hblocks
__asm	mov			ecx,				eax
__asm	mov			edx,				previous
__asm	mov			esi,				_sp
__asm	shl			eax,				SSE_SHIFT
__asm	mov			edi,				dp
__asm	add			eax,				remainder
__asm	mov			ebx,				pitch
__asm	sub			pitch,				eax
__asm	lea			edi,				[edi + ebx + 1]
__asm	mov			eax,				next
__asm	align		16
__asm	middle_loop:
		get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
		get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		SmoothTRepair2([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
__asm	add			esi,				SSE_INCREMENT
__asm	add			edx,				SSE_INCREMENT
__asm	add			eax,				SSE_INCREMENT
__asm	add			edi,				SSE_INCREMENT
__asm	dec			ecx
__asm	jnz			middle_loop
// the last pixels
__asm	add			esi,				remainder
__asm	add			edx,				remainder
__asm	add			eax,				remainder
__asm	add			edi,				remainder
		get_lu(SSE0, SSE1, [edx], [esi], [eax], SSE6, SSE7)
		get_lu(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		get_lu(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
__asm	pmaxub		SSE0,				SSE2
		SmoothTRepair2([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
__asm	add			esi,				pitch
__asm	add			edx,				pitch
__asm	add			eax,				pitch
__asm	add			edi,				pitch
__asm	dec			height
__asm	mov			ecx,				hblocks
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

#if	ISSE > 1
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
#else
#define	SmoothTRepair3(dest, pmax, nmax, previous, current, next, reg1, reg2, reg3, reg4, reg5)	\
		get2diff(reg4, reg5, previous, current, next, reg2, reg3, reg1)	\
__asm	pmaxub		pmax,			reg4				\
__asm	pmaxub		nmax,			reg5				\
__asm	SSE_RMOVE	reg2,			reg1				\
__asm	pminub		pmax,			nmax				\
__asm	paddusb		reg1,			pmax				\
__asm	psubusb		reg2,			pmax				\
__asm	pminub		reg1,			dest				\
__asm	pmaxub		reg1,			reg2				\
__asm	SSE_MOVE	dest,			reg1
#endif

void	smooth_temporal_repair3(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next, int pitch, int hblocks, int height, int remainder)
{	
__asm	mov			eax,				hblocks
__asm	mov			ecx,				eax
__asm	mov			edx,				previous
__asm	mov			esi,				_sp
__asm	shl			eax,				SSE_SHIFT
__asm	mov			edi,				dp
__asm	add			eax,				remainder
__asm	mov			ebx,				pitch
__asm	sub			pitch,				eax
__asm	lea			edi,				[edi + ebx + 1]
__asm	mov			eax,				next
__asm	align		16
__asm	middle_loop:
		get2diff(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
		get2diff(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		SmoothTRepair3([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
__asm	add			esi,				SSE_INCREMENT
__asm	add			edx,				SSE_INCREMENT
__asm	add			eax,				SSE_INCREMENT
__asm	add			edi,				SSE_INCREMENT
__asm	dec			ecx
__asm	jnz			middle_loop
// the last pixels
__asm	add			esi,				remainder
__asm	add			edx,				remainder
__asm	add			eax,				remainder
__asm	add			edi,				remainder
		get2diff(SSE0, SSE1, [edx], [esi], [eax], SSE5, SSE6, SSE7)
		get2diff(SSE2, SSE3, [edx + 1], [esi + 1], [eax + 1], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2], [esi + 2], [eax + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx], [esi + 2*ebx], [eax + 2*ebx], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx + 1], [esi + 2*ebx + 1], [eax + 2*ebx + 1], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + 2*ebx + 2], [esi + 2*ebx + 2], [eax + 2*ebx + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + ebx], [esi + ebx], [eax + ebx], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		get2diff(SSE2, SSE3, [edx + ebx + 2], [esi + ebx + 2], [eax + ebx + 2], SSE5, SSE6, SSE7)
__asm	pmaxub		SSE0,				SSE2
__asm	pmaxub		SSE1,				SSE3
		SmoothTRepair3([edi], SSE0, SSE1, [edx + ebx + 1], [esi + ebx + 1], [eax + ebx + 1], SSE4, SSE5, SSE6, SSE7, SSE3)
__asm	add			esi,				pitch
__asm	add			edx,				pitch
__asm	add			eax,				pitch
__asm	add			edi,				pitch
__asm	dec			height
__asm	mov			ecx,				hblocks
__asm	jnz			middle_loop
}
#endif	// SMOOTH2

class SmoothTemporalRepair : public GenericVideoFilter, public PlanarAccess
{
	HomogeneousChild	oclip;

#ifdef	SMOOTH2
	void	(*st_repair)(BYTE *dp, const BYTE *previous, const BYTE *_sp, const BYTE *next,int pitch, int hblocks, int height, int remainder);
#else
#define		st_repair	smooth_temporal_repair1
#endif

	int		height2[3], hblocks[3], remainder[3];
	unsigned		last_frame;

	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env)
	{
		if( ((unsigned)(n - 1) >= last_frame) ) return child->GetFrame(n, env);
		PVideoFrame sf = child->GetFrame(n, env);
		PVideoFrame pf = oclip->GetFrame(n - 1, env);
		PVideoFrame of = oclip->GetFrame(n, env);
		PVideoFrame nf = oclip->GetFrame(n + 1, env);
		PVideoFrame df = env->NewVideoFrame(vi);
		
		int	i = planes;
		do
		{
			BYTE* dp = GetWritePtr(df,i);
			int	pitch = GetPitch(df, i);
			// copy the plane from sp to dp
			env->BitBlt(dp, pitch, GetReadPtr(sf, i), pitch, width[i], height[i]);
			st_repair(dp, GetReadPtr(pf, i), GetReadPtr(of, i), GetReadPtr(nf, i), pitch, hblocks[i], height2[i], remainder[i]);
		} while( --i >= 0 );
		SSE_EMMS
		return df;
	}
public:
	SmoothTemporalRepair(PClip clip, PClip _oclip, int mode, bool grey, bool planar, IScriptEnvironment* env) : GenericVideoFilter(clip), PlanarAccess(vi), oclip(_oclip, grey, env)
	{
		if( vi.IsYV12() + planar == 0 ) AVSenvironment->ThrowError("TemporalRepair: only planar color spaces are supported");
		CompareVideoInfo(vi, _oclip->GetVideoInfo(), "TemporalRepair");
		//_oclip->SetCacheHints(CACHE_RANGE, 2);
		//child->SetCacheHints(CACHE_NOTHING, 0);

#ifdef	SMOOTH2	
		switch( mode )
		{
			case 1 :
				st_repair = smooth_temporal_repair1;
				break;
			case 2 :
				st_repair = smooth_temporal_repair2;
				break;
			default :
				st_repair = smooth_temporal_repair3;
		}
#endif	// SMOOTH2

		if( grey ) planes = 0;

		last_frame = vi.num_frames - 2;
		if( (int) last_frame < 0 ) last_frame = 0;
		
		int	i = planes;
		do
		{
			height2[i] = height[i] - 2;	
			// unsigned	w = width[i] - 1 - 2*smooth;
			unsigned	w = width[i] - 3;
			hblocks[i] = w / SSE_INCREMENT;
			remainder[i] = (w & (SSE_INCREMENT - 1)) - (SSE_INCREMENT - 1);
		} while( --i >= 0 );

		if( (hblocks[planes] <= 0) || (height2[planes] <= 0) ) 
				AVSenvironment->ThrowError("TemporalRepair: the width or height of the clip is too small");
	}
	//~SmoothTemporalRepair(){}
};

#define	MAXTMODE	4	

bool	spatial[MAXTMODE + 1] = {false, true, true, true, false };

AVSValue __cdecl CreateTemporalRepair(AVSValue args, void* user_data, IScriptEnvironment* env)
{
	enum ARGS { CLIP, OCLIP, MODE, SMOOTH, GREY, PLANAR };
	PClip	clip = args[CLIP].AsClip();
	PClip	oclip = args[OCLIP].AsClip();
	bool	grey = args[GREY].AsBool(false);
	int		mode = args[MODE].AsInt(args[SMOOTH].AsInt(0));
	if( (unsigned) mode > MAXTMODE ) env->ThrowError("TemporalRepair: illegal mode %i", mode);
	bool	planar = args[PLANAR].AsBool(false);
	return	spatial[mode] ? (AVSValue) new SmoothTemporalRepair(clip, oclip, mode, grey, planar, env)
										: (AVSValue) new TemporalRepair(clip, oclip, mode, grey, planar);
};

#else	// MODIFYPLUGIN
#endif // MODIFYPLUGIN

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit2(IScriptEnvironment* env)
{
#ifdef	MODIFYPLUGIN
#ifdef	DEBUG_NAME
	env->AddFunction("DTemporalRepair", "cc[mode]i[smooth]i[grey]b[planar]b", CreateTemporalRepair, 0);
#else
	env->AddFunction("TemporalRepair", "cc[mode]i[smooth]i[grey]b[planar]b", CreateTemporalRepair, 0);
#endif
#elif SHARPEN == 1
#elif SHARPEN > 1
#else // MODIFYPLUGIN
#endif // MODIFYPLUGIN
	AVSenvironment = env;
	if( (CPUFLAGS & env->GetCPUFlags()) != CPUFLAGS ) 
#if	ISSE > 1
		env->ThrowError("RemoveGrainT needs an SSE2 capable cpu!\n");
#else
		env->ThrowError("RemoveGrainT needs an SSE capable cpu!\n");
#endif
#if	0
	debug_printf(LOGO);
#endif
	return 0;
}
