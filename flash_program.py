from mpc8xx_debug import *
import sys

print("Reset")
enter_debug()
print("wait_ready")
wait_ready()
print("wait_debug")
if not wait_stop(timeout=1):
    exit()

print_icr()
print_sprs()


if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    # filename = "image.bin"
    # filename = "/home/winix/src/other/bivme2/new_kernel/u-boot-v2017.05/u-boot.bin"
    filename = "/home/winix/src/other/bivme2/new_kernel/u-boot-v2024.01/u-boot.bin"

loadaddr = 0x0
file_offset = 0x0
progsize = None

# set_spr(149, 0xFFFFFFFF & ~(1<<21)) # set DER to ALL except DECIE
set_spr(149, 0xFFFFFFFF) # set DER to ALL
# set_spr(149, 0x2002000F) # set DER to (CHSTPE | TRE | LBRKE | IBRKE | EBRKE | DPIE)
# set_spr(149, 0x0002000F) # set DER to (TRE | LBRKE | IBRKE | EBRKE | DPIE)

# reset BRx/ORx to reset state
write_ram(0xfff00100, [0x00000401, 0x00000ff4] + [0]*6*2) # BRx/ORx

load_firmware("mpc866_bl/prog.bin")
flash_program_from_file(filename, loadaddr, ofs=file_offset, size=progsize)

finish_debug()
