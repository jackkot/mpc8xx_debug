from mpc8xx_debug import *

print("Reset")
enter_debug()
print("wait_ready")
wait_ready()
print("wait_debug")
if not wait_stop(timeout=1):
    exit()

print_icr()
print_sprs()
# print("BRx/ORx"); print_ram(0xfff00100, 14)

# reset BRx/ORx to reset state
write_ram(0xfff00100, [0x00000401, 0x00000ff4] + [0]*6*2) # BRx/ORx

# set_spr(149, 0xFFFFFFFF & ~(1<<21)) # set DER to ALL except DECIE
# set_spr(149, 0xFFFFFFFF) # set DER to ALL
# set_spr(149, 0x2002000F) # set DER to (CHSTPE | TRE | LBRKE | IBRKE | EBRKE | DPIE)
# set_spr(149, 0x0002000F) # set DER to (TRE | LBRKE | IBRKE | EBRKE | DPIE)


print("Run")
go_to_addr(0x100)
# exit()

if not wait_stop():
    exit()

print_icr()
print_sprs()
print_gprs()


print("Reset")
enter_debug()
print("wait_ready")
wait_ready()
print("wait_debug")
if not wait_stop(timeout=1):
    exit()


# print("BRx/ORx"); print_ram(0xfff00100, 14)
# print_ram(0xfff02000, 10)

# exit()

gp = get_gpr(2)
# print("gd:")
# print_ram(gp, 144//4, print_abs=False)

# bd = read_ram(gp+0)[0]
# print("bd:")
# print_ram(bd, 64//4, print_abs=False)


# reloc_ofs = read_ram(gp+20*4)[0]
# print_target_stack(dbgsym_file='/home/winix/src/other/bivme2/new_kernel/u-boot-v2024.01/u-boot', reloc_ofs=reloc_ofs)

reloc_ofs = 0
set_msr(0x30)
print_target_stack(dbgsym_file='/home/winix/src/other/bivme2/new_kernel/linux-3.2.102/vmlinux', reloc_ofs=reloc_ofs)


finish_debug()
