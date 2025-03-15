# apollo4pevb

Current code is for toggling the three onboard LEDs.

Setting up:
- place ```templates``` in examples folder of Ambiq Apollo 4 SDK.
- write the code in src/main.c
- cd to ```gcc```.
- configure the `Makefile` by setting `TARGET` which is name of the project, and `TOP_DIR` which is the path to the SDK
- compile using ```make```. The binary files is created as `bin/main.bin`.
- Flash this binary file using JFlashLite
- Apollo 4 Plus EVB config for JFlashLite:
  - Device: AMAP42KP-KBR
  - Interface: SWD
  - Speed: 4000kHz
  - Prog. addr.: 0x00018000
 
 
