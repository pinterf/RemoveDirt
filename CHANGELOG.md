# RemoveDirt - pfmod

## Change log

- (20250108 v1.1)
  - Fix SCSelect (VapourSynth)

- (20250108 v1.0)
  - Remove YUY2 support (not even with "planar hack")
  - VapourSynth support (API 4) (dual Avisynth/VapourSynth plugin)
    VS version supports the very same basic parameters like the Avisynth version except
    some historical compatibility parameters:
    - RestoreMotionBlocks is omitting "planar"
    - SCSelect is omitting "planar", "cache", "gcache"

- (20210223 v0.9.3)
  - Fix a crash, which can occur on non mod8 sources

- (20210108)
  - No functional change since v0.9.2
  - Linux/GCC port
  - CMake build system, automatic C-only version for non-intel 
  - TemporalRepair port which was moved to RgTools in 2019, deleted from here.

- (20190328-0405 v0.9.x WIP)
  - Add TemporalRepair from RepairT.DLL
  - Reports MT mode for Avisynth+: MT_NICE_FILTER
  - fix a small bug in mode 3 SSE2
  - x64 version, rewrite to SIMD intrinsics
  - create pure C functions 
  - Fix mode 1-3 which relied on having the same pitch for multiple different frames
  - support avs 2.6 8 bit color spaces
  - Move to LLVM 8.0: http://releases.llvm.org/download.html#8.0.0, pre-built libraries Windows (64-bit)
    Uninstall old LLVM anyway
    Install/Add LLVM for System Path

- (20190324 v0.9.2)
  - RestoreMotionBlocks: 10-16 bit support. Relevant threshold and noise parameters are bit depth independent.
  - minor speedup
  - SCSelect: add support for 10-16 bits and 32 bit float clips
  - SCSelect: add support for planar RGB
  - FIX: SCSelect: make it work properly for large frames (>8MPixel)
  - FIX: SCSelect: Makes use the whole frame: now counts the rightmost non-mod32 pixels as well.
  - Clang 7.0.1 support (LLVM) with Visual Studio 2017 (15.9.9) (LLVM 8.0.0: not tried yet)
    - Install LLVM 7.0.1 (http://releases.llvm.org/download.html, Windows pre-built libraries)
    - Install Clang Power Tools & LLVM Compiler Toolchain
      - https://marketplace.visualstudio.com/items?itemName=caphyon.ClangPowerTools
        https://www.clangpowertools.com/CHANGELOG.html 
      - https://marketplace.visualstudio.com/items?itemName=LLVMExtensions.llvm-toolchain
  - update html docs
  - add clang-built DLLs to the released version

- (20190314 v0.9.1)
  - project moved to github: https://github.com/pinterf/RemoveDirt
  - built using Visual Studio 2017
  - x64 build for Avisynth+
  - Added version resource to DLL
  - Changed to AVS 2.6 plugin interface
  - Fix: RestoreMotionBlocks: grey=false: it was copying 8x4 chroma pixels for YV12 and 8x8 for YUY2 instead of 4x4 and 8x4 blocks
  - Fix: SCSelect: Old v0.9 SSE2 code omitted difference checking for every second 8 columns
  - Removed MMX code, now requires SSE2
  - Reports MT Modes for Avisynth+: MT_SERIALIZED for SCSelect
  - Reports MT Modes for Avisynth+: MT_MULTI_INSTANCE for RestoreMotionBlocks (may not be any faster)
  - Added Y, YV16 and YV24 support besides existing YV12 and planar-hacked-YUY2
  - Note: This mod does not support other filters appearing in 1.0beta.

- (20190312)
  Initial source v0.9 (2005/05/07) moved to VS2017 project 