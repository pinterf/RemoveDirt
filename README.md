# RemoveDirt - pfmod

Port of classic RemoveDirt 0.9 to Avisynth v2.6 interface (x86/x64), adding new color spaces

- (20190319 v0.9.??)
  - RestoreMotionBlocks: 10-16 bit support. Relevant threshold and noise parameters work the same as for 8 bits.
  - SCSelect: add support for 10-16 bits and 32 bit float clips
  - SCSelect: add support for planar RGB
  - FIX: SCSelect: make it work properly for large frames (>8MPixel)
  - FIX: SCSelect: Count with rightmost non-mod32 pixels

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

- (20190312)
  Initial source v0.9 (2005/05/07) moved to VS2017 project 

Links
=====
http://avisynth.nl/index.php/RemoveDirt