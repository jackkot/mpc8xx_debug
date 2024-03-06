import time
from itertools import islice
from collections.abc import Sequence
from struct import Struct
from traceback import print_stack
import subprocess

from ch341 import CH341A


DEBUG=False
# DEBUG=True
last_cmd = 0xffffffff

load_addr = 0xFFF03000
ram_buf = 0xFFF02000
flash_base = 0x0


def cdiv(x, y):
    return (x + y - 1) // y


def batched(iterable, n):
    it = iter(iterable)
    while True:
        batch = tuple(islice(it, n))
        if not batch:
            break
        yield batch


def print_hex(data):
    for i, d in enumerate(data):
        if i % 8 == 0:
            if i != 0:
                print("")
            print(f"{i:02x}: ", end="")
        print(f"{d:02x} ", end="")
    print("")


def print_mem_buf(addr, data):
    for i, d in enumerate(data):
        print(f"{addr+i*4:08x}: {d:08x} ")


def measure_time(func, count=1):
    def wrapper(*arg, **kw):
        from datetime import timedelta
        import time
        start = time.perf_counter()
        res = func(*arg, **kw)        
        t = time.perf_counter()-start
        print(f"Ready {func.__name__} in {timedelta(seconds=t)}, {count/t:.3f} tr/s")
        return res
    return wrapper



dbg_port = CH341A()
rev = dbg_port.get_revision()
print(f"Device revision is {rev[0]}.{rev[1]}.{rev[2]}")
dbg_port.reset()
# dbg_port.gpio_setdir(0)
# exit()

# dbg_port.gpio_setdir(0x3F)
# dbg_port.gpio_setbits(0x1)
# print(f"{dbg_port.gpio_getbits():x}")
# dbg_port.gpio_setbits(0x0)
# print(f"{dbg_port.gpio_getbits():x}")

# dbg_port.spi_init()
# res = dbg_port.spi_transfer([0x2, 3,4,5,6], do_cs=False)
# print_hex(res)

# print(f"Read data: ", end="")

# dbg_port.spi_cs(True)
# dbg_port.spi_cs(False)
# res = dbg_port.spi_transfer([0x00, 0x0, 0x0, 0x0, 0x0], do_cs=False)
# print_hex(res)


p = dbg_port.SpiPin
state = p.CSn
read_data = 0

def set(bits):
    global state
    state |= bits
    dbg_port.gpio_setbits(state)

def clr(bits):
    global state
    state &= ~bits
    dbg_port.gpio_setbits(state)

def enter_debug():
    dbg_port.gpio_setdir(p.MOSI | p.CSn | p.CLK)
    # CLK = 1 - set immediatly debug mode
    # MOSI = 0 - set asynchronous clocked mode
    s = p.CLK
    dbg_port.gpio_setbits(s) # enter soft reset
    # time.sleep(0.5)
    # # assert and deassert HRESET here
    # time.sleep(1.5)
    dbg_port.gpio_setbits(p.CSn | s) # exit soft reset
    dbg_port.gpio_setbits(p.CSn)  # Deassert CLK and MOSI to defaults


def wait_ready():
    ret = True    
    while ret:
        dbg_port.gpio_setbits(p.CSn | p.CLK)
        ret = True if dbg_port.gpio_getbits() & p.MISO else False
        dbg_port.gpio_setbits(p.CSn)


def wait_debug(timeout=None):
    start = time.time()
    
    try:
        status = 1
        while status == 1:
            res = instr([nop(), nop()], silent=True)
            status = res[1][0]
            cmd_nop(silent=True)
            if timeout is not None and time.time() - start > timeout:
                print("wait_debug timeout")
                return False
    except KeyboardInterrupt:
        cmd_nop(silent=True)
        print("wait_debug interrupted")
        return False
        
    return True
    

def do_debug_transfer(mode, ctrl, data, silent=False):
    global last_cmd

    if not isinstance(data, Sequence):
        data = [data]

    ctrl_byte = 0
    if mode: ctrl_byte |= 0x2
    if ctrl: ctrl_byte |= 0x1

    wbuf = []
    for d in data:
        wbuf += [ctrl_byte | 0x4] # start bit
        if mode:
            d <<= 25
        wbuf += Struct('>I').pack(d)

    rbuf = dbg_port.spi_transfer(wbuf, do_cs=False)

    res = tuple(Struct('>BI').iter_unpack(bytes(rbuf)))
    if not silent:
        for i, (status, rdata) in enumerate(res):
            if status not in [0, 3]:
                if i>0:
                    err_ctrl = ctrl_byte
                    err_cmd = data[i-1]
                else:
                    err_ctrl = last_cmd >> 32
                    err_cmd = last_cmd & 0xFFFFFFFF
                print(f"!!! error status: {status} in cmd {err_ctrl:x}.{err_cmd:08x}")
                print_stack()
            if DEBUG:
                print(f"send: {ctrl_byte:x}.{data[i]:08x}; recv: {status:x}.{rdata:08x}")

    last_cmd = (ctrl_byte<<32) | data[-1]
    return res


def instr(data, *v, **kv):
    return do_debug_transfer(0, 0, data, *v, **kv)



def op_immd(op, ra, rb, data):
    ret = op
    ret |= (ra & 0x1F) << 21
    ret |= (rb & 0x1F) << 16
    ret |= (data & 0xFFFF) << 0
    return ret

def lis(dest_gpr, data):
    return op_immd(0x3C000000, dest_gpr, 0, data)

def li(dest_gpr, data):
    return op_immd(0x38000000, dest_gpr, 0, data)

def ori(dest_gpr, src_gpr, data):
    return op_immd(0x60000000, src_gpr, dest_gpr, data)

def addi(dest_gpr, src_gpr, data):
    return op_immd(0x38000000, dest_gpr, src_gpr, data)

def subi(dest_gpr, src_gpr, data):
    addi(dest_gpr, src_gpr, -data)

def stw(src_gpr, ofs, dst_addr_gpr):
    return op_immd(0x90000000, src_gpr, dst_addr_gpr, ofs)

def stwu(src_gpr, ofs, dst_addr_gpr):
    return stw(src_gpr, ofs, dst_addr_gpr) | 0x04000000

def lwz(dst_gpr, ofs, src_addr_gpr):
    return op_immd(0x80000000, dst_gpr, src_addr_gpr, ofs)

def lwzu(dst_gpr, ofs, src_addr_gpr):
    return lwz(dst_gpr, ofs, src_addr_gpr) | 0x04000000

def mfmsr(dest_gpr):
    return op_immd(0x7C0000A6, dest_gpr, 0, 0)

def mtmsr(gpr):
    return op_immd(0x7C000124, gpr, 0, 0)

def mfspr(dest_gpr, spr):
    spr_data = ((spr & 0x1F) << 16) | ((spr & 0x3E0) << 6)
    return op_immd(0x7C0002A6, dest_gpr, 0, 0) | spr_data

def mtspr(spr, src_gpr):
    spr_data = ((spr & 0x1F) << 16) | ((spr & 0x3E0) << 6)
    return op_immd(0x7C0003A6, src_gpr, 0, 0) | spr_data

def trap():
    return 0x7FE00008

def twi(op, gpr, val):
    return op_immd(0x0c000000, op, gpr, val)

def rfi():
    return 0x4C000064

def b(ofs):
    ofs &= 0x3FFFFFC
    return 0x48000000 | ofs

def bl(ofs):
    return b(ofs) | 0x1

def blr(ofs):
    return 0x4e800020

def nop():
    return 0x60000000





def set_gpr(gpr, data):
    instr([
        lis(gpr, data >> 16),
        ori(gpr, gpr, data),
    ])

def get_gpr(gpr):
    res = instr([
        mtspr(630, gpr),
        nop(),
    ])
    return res[-1][1]

def set_spr(spr, data, gpr1=12):
    instr([
        lis(gpr1, data >> 16),
        ori(gpr1, gpr1, data),
        mtspr(spr, gpr1),
    ])

def get_spr(spr, gpr1=12):
    res = instr([
        mfspr(gpr1, spr),
        mtspr(630, gpr1),
        nop(),
    ])
    return res[-1][1]

def set_msr(data, gpr1=12):
    instr([
        lis(gpr1, data >> 16),
        ori(gpr1, gpr1, data),
        mtmsr(gpr1),
    ])

def get_msr(gpr1=12):
    res = instr([
        mfmsr(gpr1),
        mtspr(630, gpr1),
        nop(),
    ])
    return res[-1][1]

def write_ram(addr, data, gpr1=12, gpr2=11):
    if not isinstance(data, Sequence):
        data = [data]
    addr -= 4 # due to stwu

    wbuf = [
        lis(gpr1, addr >> 16),
        ori(gpr1, gpr1, addr),
    ]
    for d in data:
        wbuf += [
            lis(gpr2, d >> 16),
            ori(gpr2, gpr2, d),
            stwu(gpr2, 4, gpr1),
        ]
    instr(wbuf)

def write_ram_fast(addr, data):
    addr -= 4 # due to fast download procedure

    set_gpr(30, addr)
    cmd_set_download_mode(True)
    cmd_data(data)
    cmd_set_download_mode(False)

def read_ram(addr, count=1, gpr1=12, gpr2=11):
    addr -= 4 # due to lwzu
    wbuf = [
        lis(gpr1, addr >> 16),
        ori(gpr1, gpr1, addr),
    ]
    for _ in range(count):
        wbuf += [
            lwzu(gpr2, 4, gpr1),
            mtspr(630, gpr2),
        ]
    wbuf += [nop()]
    ret = instr(wbuf)
    # print(ret)
    ret = tuple(x[1] for x in ret[4::2]) # each 2 word started from 4
    return ret

def print_ram(addr, count=1, gpr1=12, gpr2=11):
    res = read_ram(addr, count, gpr1, gpr2)
    print_mem_buf(addr, res)





def cmd_nop(*v, **kv):
    return do_debug_transfer(1,1,0x0, *v,**kv)

def cmd_hard_reset():
    do_debug_transfer(1,1,0x1)
    dbg_port.gpio_setdir(0)

def cmd_soft_reset():
    do_debug_transfer(1,1,0x1)
    dbg_port.gpio_setdir(0)

def cmd_data(data):
    do_debug_transfer(0,1,data)

def cmd_set_download_mode(val):
    if val:
        do_debug_transfer(1,1,0x63)
    else:
        do_debug_transfer(1,1,0x43)
        cmd_data([0x0])

def cmd_nmi_trap():
    do_debug_transfer(1,1,0x1F)
    do_debug_transfer(1,1,0x7F)


def bp_set_breakpoint(addr):
    set_spr(144, addr) # CMPA
    set_spr(158, (0x4<<29) | (0x2<<18) | (0x1<<11)) # ICTRL = CTA=Equal,IW0=Match A,SIW0EN


def bp_continue(next=True):
    if next:
        set_spr(26, get_spr(26)+4) # SRR0
    get_spr(148) # reset ICR
    instr( rfi() ) # go

def bp_go():
    bp_continue(next=False)


def print_gpr(r):
    print(f">> r{r} = {get_gpr(r):08x}")

def print_gprs():
    for i in range(0, 31+1):
        print_gpr(i)

def print_sprs():
    print(f">> MSR = {get_msr():08x}")
    print(f">> LR = {get_spr(8):08x}")
    print(f">> DAR = {get_spr(19):08x}")
    print(f">> BAR = {get_spr(159):08x}")
    print(f">> DER = {get_spr(149):08x}")
    print(f">> SRR0 = {get_spr(26):08x}")
    print(f">> SRR1 = {get_spr(27):08x}")

def print_icr():
    print(f">> ICR = {get_spr(148):08x}")

        
def check_icr():
    icr = get_spr(148) # it resets automatically
    # print(f">> ICR = {icr:08x}")

    # ICR[0] - Development port interrupt bit
    # ICR[23] - Program interrupt bit
    if icr & ~((1<<23) | (1<<0)) or icr == 0: 
        print(f"Exception occured (ICR = {icr:08x})")
        print_sprs()
        raise Exception("ICR error")
    
    if icr == 0: 
        print(f"ICR is 0. Maybe power is off?")
        raise Exception("ICR error")


def load_firmware(file):
    print("Load firmware...", end='', flush=True)
    with open(file, 'rb') as f:
        prog_bin = f.read()
    prog = [d[0] for d in Struct(">I").iter_unpack(prog_bin)]
    write_ram_fast(load_addr, prog)
    print(" done")

def run_prog(ep, gprs={}):
    set_gpr(31, ep) # select entry point
    for r,v in gprs.items():
        set_gpr(r, v)

    set_spr(149, 0xFFFFFFFF) # set DER to ALL
    # set_spr(149, 0x2180000F) # set DER to (CHSTPE | ALIE | PRIE | LBRKE | IBRKE | EBRKE | DPIE)
    # set_spr(149, 0x2180200F) # set DER to (CHSTPE | ALIE | PRIE | ITLBMSE? | LBRKE | IBRKE | EBRKE | DPIE)
    # set_spr(149, 0x2002000F) # set DER to (CHSTPE | TRE | LBRKE | IBRKE | EBRKE | DPIE)

    set_spr(26, load_addr) # set SRR0 (Entry Point)
    get_spr(148) # reset ICR

    instr( rfi() ) # go
    wait_debug() # wait for complete
    check_icr() # check if it was fault


def run_prog_simple():
    run_prog(0)

def run_prog_memcpy(dst, src, cnt=1):
    run_prog(1, gprs={3: dst, 4: src, 5: cnt})

def mem_read_inside(addr, cnt=1):
    run_prog_memcpy(dst=ram_buf, src=addr, cnt=cnt)
    return read_ram(ram_buf, count=cnt)

def mem_print_inside(addr, cnt=1):
    run_prog_memcpy(dst=ram_buf, src=addr, cnt=cnt)
    res = read_ram(ram_buf, cnt)
    print_mem_buf(addr, res)


def save_mem_to_file(filename, src, size):
    cnt = (size+3)//4 # rounded up
    chunk = 0x1000//4

    chunk_cnt = cdiv(cnt, chunk)

    data = []
    for i in range(chunk_cnt):
        cur_cnt = min(cnt, chunk)
        print(f"[{i+1}/{chunk_cnt}] Read {cur_cnt*4} bytes from address 0x{src:08x}")
        data += mem_read_inside(src, cur_cnt)
        cnt -= cur_cnt
        src += cur_cnt*4
    
    data = Struct(f">{len(data)}I").pack(*data)
    data = data[:size]

    # print(data)
    # for d in data: print(f"{d:02x} ")

    with open(filename, 'wb') as f:
        f.write(data)



def run_prog_flash_write(addr, data):
    print(f"flash write({addr:03x}, {data:02x})")
    run_prog(2, gprs={3: addr, 4: data})
    
def run_prog_flash_read(addr):
    run_prog(3, gprs={3: addr})
    data = read_ram(ram_buf, 1)[0] & 0xFF
    print(f"flash read ({addr:03x}) = {data:02x}")
    return data

def run_prog_flash_erase(flash, addr, size):
    run_prog(4, gprs={3: flash, 4: addr, 5: size})
    # for x in read_ram(ram_buf, 10): print(f"{x:08x}")
    return True if read_ram(ram_buf, 1)[0] == 0 else False

def run_prog_flash_program(flash, addr, data, size):
    run_prog(5, gprs={3: flash, 4: addr, 5: data, 6: size})
    return True if read_ram(ram_buf, 1)[0] == 0 else False


def flash_cmd(addr, cmd):
    run_prog_flash_write(0xAAA, 0xAA)
    run_prog_flash_write(0x555, 0x55)
    run_prog_flash_write(addr, cmd)

def flash_erase_sector(addr, size=1):
    print(f"Erase sector at address 0x{addr:08x}")
    ret = run_prog_flash_erase(flash_base, addr, size)
    if not ret:
        raise Exception("Flash erasing failed")

def flash_write_buf(addr, wbuf):
    max_chunk = 0x1000

    size = len(wbuf)
    chunk_cnt = cdiv(size, max_chunk)

    flash_erase_sector(addr, size)

    for i, buf in enumerate(batched(wbuf, max_chunk)):
        cur_size = len(buf)
        print(f"[{i+1}/{chunk_cnt}] Write {cur_size} bytes to address 0x{addr:08x}")

        buf_aligned = bytes(buf).ljust(cdiv(cur_size, 4) * 4, b'\x00')
        pkt_data = [d[0] for d in Struct(">I").iter_unpack(buf_aligned)]
        write_ram_fast(ram_buf, pkt_data)

        ret = run_prog_flash_program(flash_base, addr, ram_buf, cur_size)
        if not ret:
            raise Exception("Flash programming failed")
        addr += cur_size

def flash_program_from_file(filename, dst, ofs=0, size=-1):
    with open(filename, 'rb') as f:
        data = f.read()
    
    if size == -1:
        size = len(data)

    wbuf = data[ofs:ofs+size]
    flash_write_buf(dst, wbuf)

def flash_program_array(dst, data):
    if not isinstance(data, Sequence):
        data = [data]

    wbuf = Struct(f">{len(data)}I").pack(*data)
    flash_write_buf(dst, wbuf)




def jump_to_flash_with_debug(addr = 0x100, timeout=None):
    write_ram_fast(load_addr, [
        lis(2, addr>>16),
        ori(2, 2, addr),
        mtspr(8, 2),
        0x4e800020, # blr
    ])

    # set_spr(149, 0xFFFFFFFF & ~(1<<20)) # set DER to ALL except DECIE
    # set_spr(149, 0xFFFFFFFF) # set DER to ALL
    set_spr(149, 0x2180000F) # set DER to (CHSTPE | ALIE | PRIE | LBRKE | IBRKE | EBRKE | DPIE)
    # set_spr(149, 0x2180200F) # set DER to (CHSTPE | ALIE | PRIE | ITLBMSE? | LBRKE | IBRKE | EBRKE | DPIE)
    # set_spr(149, 0x2002000F) # set DER to (CHSTPE | TRE | LBRKE | IBRKE | EBRKE | DPIE)

    set_spr(26, load_addr) # set SRR0 (Entry Point)
    get_spr(148) # reset ICR

    instr( rfi() ) # go
    # exit()

    ret = wait_debug(timeout) # wait for program exception
    if not ret:
        return False

    print_icr()
    print_sprs()
    return True

def addr2line(file, addr):
    if file is None:
        return f"0x{addr:08x}\n"

    reloc_off = read_ram(get_gpr(2)+64)[0]
    addr = (addr - reloc_off) & 0xffffffff
    return subprocess.check_output([
            'addr2line', '-pifCa', '-e', file, f"0x{addr:08x}"
        ]).decode()

def print_target_stack(file):
    print(f".:", addr2line(file, get_spr(26)), end='') # SRR0
    print(f"0:", addr2line(file, get_spr(8)), end='') # LR
    
    stack_frame = read_ram(get_gpr(1))[0]
    i = 1
    while stack_frame:
        print(f"{i}:", addr2line(file, read_ram(stack_frame+4)[0]-4), end='')
        stack_frame = read_ram(stack_frame)[0]
        i += 1


print("Reset")
enter_debug()
print("wait_ready")
wait_ready()
print("wait_debug")
wait_debug()


set_spr(638, 0xFFF00000)
print(f">> IMMR = {get_spr(638):08x}")
# print(f">> r1 = {get_gpr(1):08x}")
# print(f">> r3 = {get_gpr(3):08x}")
# print(f">> r4 = {get_gpr(4):08x}")

write_ram(0xFFF00004, [0xFFFFFF03]) # SYPCR # Disable watchdog

# measure_time(write_ram_fast, 2048)(0xFFF02000, [0xFFFFFFFF]*2048) #549 tr/s
# measure_time(write_ram, 2048)(0xFFF02000, [0xFFFFFFFF]*2048) #187 tr/s
# measure_time(read_ram, 2048)(0xFFF02000, 2048) #282 tr/s

print("")
print("Init:")
print_icr()
print_sprs()
# write_ram(0xfff00100, [0x00000401, 0x00000ff4] + [0]*6*2) # BRx/ORx
# print("BRx/ORx"); print_ram(0xfff00100, 14)
print("")


print("Prepare:")

# write_ram_fast(ram_buf, [0xFFFFFFFF]*(0x1000//4)) # clear ram_buf
# measure_time(write_ram_fast, 0x2000)(ram_buf, [0xFFFFFFFF]*(0x2000//4)) # 2244 b/s

do_this = False



# # infinite loop
# write_ram_fast(load_addr, [
#     0x48000000, # b .+0
# ])
# ret = jump_to_flash_with_debug(load_addr)
# if not ret:
#     enter_debug()
#     # cmd_nmi_trap()
#     if not wait_debug(timeout=2):
#         exit()
# exit()



# # do_this = True
# if do_this:
#     load_firmware("mpc866_bl/prog.bin")
#     flash_program_from_file("/tftpboot/boot/image.bin", 0x0000_0000)
#     exit()







# # do_this = True
# if do_this:
#     ret = jump_to_flash_with_debug(0x100, timeout=None)
#     if not ret:
#         cmd_nmi_trap()
#         if not wait_debug(timeout=2):
#             exit()

#     print("BRx/ORx"); print_ram(0xfff00100, 14)
#     # write_ram(ram_buf, [0x12345678])
#     print_ram(ram_buf, 10)
#     print_gprs()
    
#     print_gpr(1)
#     print("Stack:")
#     # sp = get_gpr(1)
#     # gp = get_gpr(2)
#     # print_ram(sp, (gp-sp)//4)
#     # print("gd:")
#     # print_ram(gp, (0xfff02F00-gp)//4)
#     # print("gd end")
    
#     print_target_stack('/home/winix/src/other/bivme2/new_kernel/u-boot-v2017.05/u-boot')
#     # print()
#     exit()







# # do_this = True
# if do_this:
#     load_firmware("mpc866_bl/prog.bin")

#     # # run_prog_simple()
#     # run_prog_memcpy(dst=ram_buf, src=0x20000, cnt=8)
#     run_prog_memcpy(dst=ram_buf, src=0x40000100, cnt=3)
#     print_ram(ram_buf, count=0x4)
#     # mem_print_inside(addr=0x20000, cnt=3)

#     write_ram(0xfff00100, [0x40000401, 0x80000ff4]) # BR0/OR0
#     print("BRx/ORx"); print_ram(0xfff00100, 14)

    
#     flash_base = 0x40000000
#     run_prog_flash_read(flash_base+0x0)
#     run_prog_flash_read(flash_base+0x1)
#     run_prog_flash_read(flash_base+0x2)
#     run_prog_flash_read(flash_base+0x3)
    
#     run_prog_flash_write(flash_base+0x000, 0xF0) # reset
#     # run_prog_flash_write(flash_base+0x000, 0xF0) # reset
#     # run_prog_flash_write(flash_base+0x000, 0xFF) # reset
    
#     # run_prog_flash_write(flash_base+0x0AA, 0x98) # CFI Query
#     # data = [0]*0x10
#     # for i in range(0x10, 0x4f+1):
#     #     data += [run_prog_flash_read(flash_base+(i<<1))]
#     # run_prog_flash_write(flash_base+0x000, 0xF0) # reset
#     # print_hex(data)
    
#     flash_cmd(0xAAA, 0x90) # Autoselect
#     for i in range(0x00, 0x03+1):
#         run_prog_flash_read(flash_base+(i<<1))
#     run_prog_flash_read(flash_base+(0x0<<1))
#     # run_prog_flash_read(flash_base+(0x0<<1))
#     # run_prog_flash_read(flash_base+(0x0<<1))
#     # run_prog_flash_read(flash_base+(0x1<<1))
#     run_prog_flash_write(flash_base+0x000, 0xF0) # reset

#     run_prog_flash_read(flash_base+(0x0<<1))
#     exit()





# # Breakpoint Example
# set_gpr(3, 2)
# write_ram_fast(load_addr, [
#     li(3, 0x1),
#     nop(),
#     li(3, 0x0),
#     nop(),
#     # trap(),
#     b(-4*4),
# ])

# set_spr(149, 0xFFFFFFFF) # set DER to ALL

# set_spr(26, load_addr) # set SRR0 (Entry Point)
# set_spr(27, 0x00000002) # set SRR1 to MSR (RI)


# bp_set_breakpoint(load_addr)
# # print(f">> CMPA = {get_spr(144):08x}")
# # print(f">> ICTRL = {get_spr(158):08x}")

# for i in range(2):
#     print("Run:")
#     if i==0:
#         bp_go()
#     else:
#         bp_continue()
#     # cmd_nmi_trap()

#     wait_debug() # wait for complete

#     print_gpr(3)
#     print_icr()
#     print_sprs()


# # Programs Example

# # infinite loop
# write_ram_fast(load_addr, [
#     0x48000000, # b .+0
# ])


# # immediatly exit
# write_ram_fast(load_addr, [
#     trap(),
# ])

# write_ram_fast(load_addr, [
#     li(0, 0x0),
#     lis(12, 0xFFF0),
#     lis(3, 0x1122),
#     ori(3, 3, 0x3345),
#     stw(3, 0x3000, 12),

#     lis(3,    0x0000),
#     ori(3, 3, 0x0100),
#     # mtspr(8, 3), # LR

#     lwz(4, 0x0, 3),
#     stw(4, 0x3008, 12),

#     # 0x48000001, # bl .+0
#     trap(),
# ])

# # memcpy(dst:r3, src:r4, cnt:r10)
# write_ram_fast(load_addr, [
#     li(10, 0x10//4),
#     0x7D4903A6, # mtctr   10

#     lis(4, 0),
#     ori(4,4, 0x100),
    
#     lis(3, 0xfff0),
#     ori(3,3, 0x2000),

#     0x3884FFFC, # addi    4,4,-4
#     0x3863FFFC, # addi    3,3,-4
#     0x84A40004, # g: lwzu    5,4(4)
#     0x94A30004, # stwu    5,4(3)
#     0x4200FFF8, # bdnz    g

#     trap(),
# ])

# # jump()
# write_ram_fast(load_addr, [
#     0x3c40ff80, # lis    r2,0xFF80
#     0x60420100, # ori    r2,r2,0x0100
#     0x7c4803a6, # mtspr  LR,r2
#     0x4e800020, # blr
# ])





# # Firmware Progs Example

# load_firmware("mpc866_bl/prog.bin")

# save_mem_to_file("bootloader.bin", 0x0, 128*1024)
# save_mem_to_file("a.bin", 0x100, 6*1024)
# measure_time(save_mem_to_file, 8*1024)("a.bin", 0x100, 8*1024)
# measure_time(save_mem_to_file, 128*1024)("bootloader.bin", 0x0, 128*1024) # 1093 b/s



# mem_print_inside(addr=0x20000, cnt=4)

# flash_erase_sector(0x0002_0000)

# flash_program_array(0x0002_0000, [
#     0x48000000, # b .+0
# ])

# measure_time(flash_program_array, 0x800*4)(0x0002_0000, [0x11223344]*0x800) # 1894 b/s

# flash_program_from_file("/tftpboot/boot/image.bin", 0x0000_0000)

# mem_print_inside(addr=0x20000, cnt=4)


# print()
# run_prog_flash_read(0x20000)
# run_prog_flash_read(0x20001)

# print()

# mem_print_inside(addr=0x20000, cnt=4)



# flash_erase_sector(0x0000_0000, size=1)
# flash_erase_sector(0x0000_0000, size=4*8*1024)
# print_ram(ram_buf, 4)
# flash_program_array(0x0001_0000, [0]*(64*1024//4))
# flash_program_array(0x0000_0000, [0]*(4*8*1024//4))
# flash_program_array(0x0000_0000, [0x55]*(4*8*1024//4))

# mem_print_inside(addr=0x0, cnt=2)
# print()
# for i in range(1, 16+1):
#     mem_print_inside(addr=i*0x2000-8, cnt=4)
#     print()


# save_mem_to_file("tmp.bin", 0x0, 128*1024)




cmd_nop()
# cmd_soft_reset()
# cmd_hard_reset()


dbg_port.close()
