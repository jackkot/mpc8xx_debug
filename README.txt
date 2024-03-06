This repository contains python implementation of Freescale MPC8xx debug port communication.
Code is very dirty and unstructured but allow to
- download/upload flash firmware
- get/set processor registers and read/write memory
- run arbitrary code in internal sram
- compile C code to minimalistic programs ready to in ram execution
- set breakpoints
- fetch stack trace

This project was written to port u-boot and linux to custom board based such processor.
Code tested on python 3.10 with MPC852T (MPC866 family) processor.
Low lewel transport is SPI with sreset GPIO. FT232H via pyftdi is using.

Also this repository contains CH341A pure python spi realization. But debug library for
this transport is abandoned on half way.
