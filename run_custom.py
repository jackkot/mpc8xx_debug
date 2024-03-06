from mpc8xx_debug import *

print("Reset")
enter_debug()
print("wait_ready")
wait_ready()
print("wait_ready")
if not wait_stop():
    exit()

set_spr(638, 0xFFF00000)
print(f">> IMMR = {get_spr(638):08x}")

write_ram(0xFFF00004, [0xFFFFFF03]) # SYPCR # Disable watchdog

print_icr()
print_sprs()
print_gprs()
print("BRx/ORx"); print_ram(0xfff00100, 7*2, print_abs=False)
# dump_all_immr()

# print_ram_inside(0xfff02800, 4)
# tx = read_ram(0xfff0280c, 1)[0]
# write_ram_inside(tx, 0x30)
# write_ram_inside(0xfff02808, 0xa0000001)
# print_sprs()


# print_ram(0xfff02800, 4)
# instr(rfi())



# reloc_ofs = read_ram(gp+20*4)[0]
# print_target_stack(dbgsym_file='/home/winix/src/other/bivme2/new_kernel/u-boot-v2024.01/u-boot', reloc_ofs=reloc_ofs)

reloc_ofs = 0
set_msr(0x30)
print_target_stack(dbgsym_file='/home/winix/src/other/bivme2/new_kernel/linux-3.2.102/vmlinux', reloc_ofs=reloc_ofs)



finish_debug()
exit()


# # infinite loop
# write_ram_fast(load_addr, [
#     0x48000000, # b .+0
# ])
# go_to_addr(load_addr)
# if not wait_trap():
#     exit()
# exit()



# write_ram(0xfff00100, [0x00000401, 0x00000ff4] + [0]*6*2) # BRx/ORx

load_firmware("mpc866_bl/prog.bin")

# # run_prog_simple()
# run_prog_memcpy(dst=ram_buf, src=0x20000, cnt=8)
run_prog_memcpy(dst=ram_buf, src=0x40000100, cnt=3)
print_ram(ram_buf, count=0x4)
# print_ram_inside(addr=0x20000, cnt=3)

write_ram(0xfff00100, [0x40000401, 0x80000ff4]) # BR0/OR0
print("BRx/ORx"); print_ram(0xfff00100, 14)


flash_base = 0x40000000
run_prog_flash_read(flash_base+0x0)
run_prog_flash_read(flash_base+0x1)
run_prog_flash_read(flash_base+0x2)
run_prog_flash_read(flash_base+0x3)

run_prog_flash_write(flash_base+0x000, 0xF0) # reset
# run_prog_flash_write(flash_base+0x000, 0xF0) # reset
# run_prog_flash_write(flash_base+0x000, 0xFF) # reset

run_prog_flash_write(flash_base+0x0AA, 0x98) # CFI Query
data = [0]*0x10
for i in range(0x10, 0x4f+1):
    data += [run_prog_flash_read(flash_base+(i<<1))]
run_prog_flash_write(flash_base+0x000, 0xF0) # reset
print_hex(data)

# flash_cmd(0xAAA, 0x90) # Autoselect
# for i in range(0x00, 0x03+1):
#     run_prog_flash_read(flash_base+(i<<1))
# run_prog_flash_read(flash_base+(0x0<<1))
# # run_prog_flash_read(flash_base+(0x0<<1))
# # run_prog_flash_read(flash_base+(0x0<<1))
# # run_prog_flash_read(flash_base+(0x1<<1))
# run_prog_flash_write(flash_base+0x000, 0xF0) # reset

# run_prog_flash_read(flash_base+(0x0<<1))


finish_debug()
