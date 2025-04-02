# apollo4pevb

Current code is for 
Setting up:
- place ```templates``` in examples folder of Ambiq Apollo 4 SDK.
- write the code in src/main.c
- cd to ```gcc```.
- configure the `Makefile` by setting `TARGET` which is name of the project, and `TOP_DIR` which is the path to the SDK
- add any paths to include libraries
- compile using ```make```. The binary files is created as `bin/main.bin`.
- Flash this binary file using JFlashLite
- Apollo 4 Plus EVB config for JFlashLite:
  - Device: AMAP42KP-KBR
  - Interface: SWD
  - Speed: 4000kHz
  - Prog. addr.: 0x00018000
 
 Available Codes:
 - `src/spi.c`: SPI code for ADC ADS7042
 - `src/uart.h`: UART code
 - `src/led.c`: Sample LED code for toggling the three onboard LEDs.
