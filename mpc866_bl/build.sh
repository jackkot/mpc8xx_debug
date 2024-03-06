#!/bin/bash

set -euo pipefail
set -x

ofile="prog"
CROSS_COMPILE="powerpc-linux-gnu-"
CFLAGS="-Os -std=gnu99 -ffunction-sections -fdata-sections -nostdinc"
LDFLAGS="-m elf32ppc -T linker.lds --gc-sections -Ttext 0xfff03000 -nostdlib"
#CFLAGS="-Os -std=gnu99 -Wl,-M,--verbose -static"
#LDFLAGS="-m elf32ppc -T linker.lds -Ttext 0xfff03000 --verbose  -static"

${CROSS_COMPILE}gcc $CFLAGS -D__ASSEMBLY__ -c crt0.S
${CROSS_COMPILE}gcc $CFLAGS -D__ASSEMBLY__ -c crtsavres.S
${CROSS_COMPILE}gcc $CFLAGS -c main.c
${CROSS_COMPILE}ld $LDFLAGS crt0.o main.o crtsavres.o -o "$ofile.elf"
${CROSS_COMPILE}objcopy -O binary "$ofile.elf" "$ofile.bin"
${CROSS_COMPILE}objdump -D "$ofile.elf" > "$ofile.hexdump"
