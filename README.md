# RemoveDirt - pfmod
A plugin for removing dirt from film clips.

Port of classic RemoveDirt 0.9 to Avisynth v2.6 interface (x86/x64), adding new color spaces, bugfixes by Ferenc Pintér.

Functions in plugin: RestoreMotionBlocks and SCSelect.
For Linux build instructions see bottom of readme.

Note: Previous v0.9 DLL versions named differently (RemoveDirtT.DLL, RemoveDirtSSE2.DLL) should be deleted from your plugin folder.

Links
-----

Project: https://github.com/pinterf/RemoveDirt
Forum: https://forum.doom9.org/showthread.php?t=176199
Additional info: http://avisynth.nl/index.php/RemoveDirt

Build instructions
------------------

### Windows Visual Studio MSVC

use IDE

### Windows GCC

(mingw installed by msys2)
From the 'build' folder under project root:

```
del ..\CMakeCache.txt
cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=on
@rem test: cmake .. -G "MinGW Makefiles" -DENABLE_INTEL_SIMD:bool=off
cmake --build . --config Release  
```

### Linux GCC

* Clone repo and build
  
        git clone https://github.com/pinterf/RemoveDirt
        cd RemoveDirt
        cmake -B build -S .
        cmake --build build

  Useful hints:        
  
  build after clean: 

  ```
  cmake --build build --clean-first
  ```
  
  Force no x86 assembler support 
  
  Note: ENABLE_INTEL_SIMD is automatically off for non-x86 architectures

  ```
  cmake -B build -S . -DENABLE_INTEL_SIMD:bool=off
  ```
  
  Delete CMake cache
  
  ```
  rm build/CMakeCache.txt
  ```
  
  

* Find binaries at

        build/RemoveDirt/libremovedirt.so

* Install binaries

        cd build
        sudo make install

