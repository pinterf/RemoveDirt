# RemoveDirt - pfmod
A plugin for removing dirt from film clips.

Port of classic RemoveDirt 0.9 to Avisynth v2.6 interface (x86/x64), adding new color spaces, bugfixes.

Functions in plugin: RestoreMotionBlocks and SCSelect.
For Linux build instructions see bottom of readme.

Note: Previous v0.9 DLL versions named differently (RemoveDirtT.DLL, RemoveDirtSSE2.DLL) should be deleted from your plugin folder.

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

Build instructions
==================
VS2019: 
  use IDE

Windows GCC (mingw installed by msys2):
  from the 'build' folder under project root:

  del ..\CMakeCache.txt
  cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=on
  @rem test: cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=off
  cmake --build . --config Release  

Linux
  from the 'build' folder under project root:
  ENABLE_INTEL_SIMD is automatically off for non x86 arhitectures
  
  rm CMakeCache.txt
  cmake ..
  cmake --build . --config Release    
  sudo make install

  test for C only on x86 arhitectures:
  cmake .. -DENABLE_INTEL_SIMD:bool=off

Links
=====
Project: https://github.com/pinterf/RemoveDirt
Forum: https://forum.doom9.org/showthread.php?t=176199
Additional info: http://avisynth.nl/index.php/RemoveDirt