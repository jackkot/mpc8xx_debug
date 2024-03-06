#!/bin/bash

set -euo pipefail
set -x

ofile="prog"

rm -f "$ofile.elf" "$ofile.bin" "$ofile.hexdump"
rm -f *.o
