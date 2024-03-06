import time
from itertools import islice
from collections.abc import Sequence
from struct import Struct
from traceback import print_stack
import subprocess
import signal
from contextlib import contextmanager

# from ch341 import CH341A
from pyftdi.spi import SpiController
from pyftdi.gpio import GpioAsyncController


DEBUG=False
# DEBUG=True
dev_url = "ftdi://::1/"
spi_freq = 0.5e6
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


def print_mem_buf(addr, data, print_abs=True):
    for i, d in enumerate(data):
        if print_abs:
            a = f"{addr+i*4:08x}"
        else:
            a = f"{i:4d}"
        print(f"{a}: {d:08x} ")


@contextmanager
def measure_time(count=1):
    from datetime import timedelta
    import time
    start = time.perf_counter()
    try:
        yield
    finally:
        t = time.perf_counter()-start
        print(f"Ready in {timedelta(seconds=t)}", end='')
        if count > 1:
            print(f", {count/t:.3f} tr/s")
        else:
            print()



class DelayedKeyboardInterrupt:

    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.signal(signal.SIGINT, self.handler)
                
    def handler(self, sig, frame):
        self.signal_received = (sig, frame)
    
    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)


PIN_SRESET = 1 << 4
PIN_HRESET = 1 << 5
PIN_FROZEN = 1 << 6

spi_ctrl = SpiController()
spi_ctrl.configure(
    dev_url,
    direction=PIN_SRESET|PIN_HRESET,
    initial=PIN_SRESET|PIN_HRESET,
    frequency=spi_freq,
    cs_count=1,
    turbo=True,
    debug=True,
)
spi = spi_ctrl.get_port(0, mode=0)
gpio = spi_ctrl.get_gpio()
usb_dev = spi_ctrl.ftdi.usb_dev

print(f"[{usb_dev.idVendor:04x}:{usb_dev.idProduct:04x}] {usb_dev.manufacturer} {usb_dev.product} ({usb_dev.serial_number})")
print(f"Chip: {spi_ctrl.ftdi.ic_name}")
spi_ctrl.ftdi.reset()


def enter_debug(hard_reset=False, immediatly_stop=True):

    def set_gpio(pin, val):
        v = spi_ctrl.read_gpio(True) & spi_ctrl.direction
        if val:
            v |= pin
        else:
            v &= ~pin
        spi_ctrl.write_gpio(v)

    # CLK = immediatly_stop - set immediatly debug mode
    # MOSI = 0 - set asynchronous clocked mode
    cfg = spi_ctrl.SCK_BIT if immediatly_stop else 0
    spi_ctrl.force_control(spi_ctrl.frequency, bytes([cfg])) # set soft reset state pins
    set_gpio(PIN_SRESET, False) # assert SRESET
    time.sleep(1.1)
    if hard_reset:
        set_gpio(PIN_HRESET, False) # assert HRESET
        time.sleep(0.1)
        set_gpio(PIN_HRESET, True) # deassert HRESET
        time.sleep(0.1)
    set_gpio(PIN_SRESET, True) # deassert SRESET
    time.sleep(0.1)
    spi_ctrl.force_control(spi_ctrl.frequency, bytes([0x00]))


def finish_debug():
    cmd_nop()
    spi_ctrl.close()
    gpio = GpioAsyncController()
    gpio.configure(dev_url, direction=0x00)
    gpio.close(freeze=True)




def wait_ready():
    while True:
        with DelayedKeyboardInterrupt():
            res = spi.exchange(bytes([0]), readlen=1, duplex=True, start=False, stop=False)
        if res[0] == 0:
            return


def wait_debug(timeout=None):
    start = time.time()
    
    try:
        while True:
            res = instr([nop(), nop()], silent=True)
            status = res[1][0]
            cmd_nop(silent=True)
            if status != 1:
                return True
            if timeout is not None and time.time() - start > timeout:
                print("wait_debug timeout")
                return False
    except KeyboardInterrupt:
        cmd_nop(silent=True)
        print("wait_debug interrupted")
        return False
            

def do_debug_transfer(mode, ctrl, data, silent=False, synchronious=False):
    global last_cmd
    chunk_size = 5 if synchronious else 4096

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

    rbuf = []
    
    for b in batched(wbuf, chunk_size):
        b = bytes(b)
        if synchronious:
            with DelayedKeyboardInterrupt():
                wait_ready()
        with DelayedKeyboardInterrupt():
            rbuf += spi.exchange(b, readlen=len(b), duplex=True, start=False, stop=False)

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
                if status & 0x80:
                    raise Exception(f"status overrun ({status:x}) in cmd {err_ctrl:x}.{err_cmd:08x}")
                print(f"!!! error status: {status:x} in cmd {err_ctrl:x}.{err_cmd:08x}")
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





@contextmanager
def borrow_gprs(*gpr_list):
    gprs = {gpr: get_gpr(gpr) for gpr in gpr_list}
    try:
        yield
    finally:
        for gpr, val in gprs.items():
            set_gpr(gpr, val)


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

def set_spr(spr, data):
    gpr1 = 12
    with borrow_gprs(gpr1):
        instr([
            lis(gpr1, data >> 16),
            ori(gpr1, gpr1, data),
            mtspr(spr, gpr1),
        ])

def get_spr(spr):
    gpr1=12
    with borrow_gprs(gpr1):
        res = instr([
            mfspr(gpr1, spr),
            mtspr(630, gpr1),
            nop(),
        ])
        return res[-1][1]

def set_msr(data):
    gpr1=12
    with borrow_gprs(gpr1):
        instr([
            lis(gpr1, data >> 16),
            ori(gpr1, gpr1, data),
            mtmsr(gpr1),
        ])

def get_msr():
    gpr1=12
    with borrow_gprs(gpr1):
        res = instr([
            mfmsr(gpr1),
            mtspr(630, gpr1),
            nop(),
        ])
        return res[-1][1]

def write_ram(addr, data):
    gpr1, gpr2 = 12, 11

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
        
    with borrow_gprs(gpr1, gpr2):
        instr(wbuf)

def write_ram_fast(addr, data):
    addr -= 4 # due to fast download procedure
    with borrow_gprs(30, 31):
        set_gpr(30, addr)
        cmd_set_download_mode(True)
        cmd_data(data)
        cmd_set_download_mode(False)

def read_ram(addr, count=1):
    gpr1, gpr2 = 12, 11
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
    
    with borrow_gprs(gpr1, gpr2):
        ret = instr(wbuf)

    # print(ret)
    ret = tuple(x[1] for x in ret[4::2]) # each 2nd word started from 4th
    return ret

def print_ram(addr, count=1, print_abs=True):
    res = read_ram(addr, count)
    print_mem_buf(addr, res, print_abs=print_abs)


def write_ram_inside(addr, data):
    load_addr = 0xfff02000

    if not isinstance(data, Sequence):
        data = [data]

    write_ram(load_addr, [
        stwu(30, 4, 31),
        trap(),
    ])
    with borrow_gprs(30, 31):
        set_gpr(31, addr - 4)
        for d in data:
            set_gpr(30, d)
            go_to_addr(load_addr)
            if not wait_stop(timeout=1):
                raise Exception("write_ram_inside is hang")


def read_ram_inside(addr, cnt=1):
    load_addr = 0xfff02000

    write_ram(load_addr, [
        lwzu(30, 4, 31),
        trap(),
    ])

    rdata = []
    with borrow_gprs(30, 31):
        set_gpr(31, addr - 4)
        for _ in range(cnt):
            go_to_addr(load_addr)
            if not wait_stop(timeout=1):
                raise Exception("write_ram_inside is hang")
            rdata += [get_gpr(30)]
    
    return rdata


def print_ram_inside(addr, cnt=1):
    res = read_ram_inside(addr, cnt)
    print_mem_buf(addr, res)



def cmd_nop(*v, **kv):
    return do_debug_transfer(1,1,0x0, *v,**kv)

def cmd_hard_reset():
    do_debug_transfer(1,1,0x1)

def cmd_soft_reset():
    do_debug_transfer(1,1,0x1)

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
    set_spr(27, 0) # set SRR1 (next MSR)
    get_spr(148) # reset ICR

    instr( rfi() ) # go
    wait_debug() # wait for complete
    check_icr() # check if it was fault


def run_prog_simple():
    run_prog(0)

def run_prog_memcpy(dst, src, cnt=1):
    run_prog(1, gprs={3: dst, 4: src, 5: cnt})

def save_mem_to_file(filename, src, size):
    cnt = (size+3)//4 # rounded up
    chunk = 0x1000//4

    chunk_cnt = cdiv(cnt, chunk)

    data = []
    for i in range(chunk_cnt):
        cur_cnt = min(cnt, chunk)
        print(f"[{i+1}/{chunk_cnt}] Read {cur_cnt*4} bytes from address 0x{src:08x}")
        data += read_ram_inside(src, cur_cnt)
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

def flash_program_from_file(filename, dst, ofs=0, size=None):
    with open(filename, 'rb') as f:
        data = f.read()
    
    if size is None:
        size = len(data)

    wbuf = data[ofs:ofs+size]
    flash_write_buf(dst, wbuf)

def flash_program_array(dst, data):
    if not isinstance(data, Sequence):
        data = [data]

    wbuf = Struct(f">{len(data)}I").pack(*data)
    flash_write_buf(dst, wbuf)




def go_to_addr(addr = 0x100, msr=None):
    # write_ram_fast(load_addr, [
    #     lis(2, addr>>16),
    #     ori(2, 2, addr),
    #     mtspr(8, 2),
    #     0x4e800020, # blr
    # ])

    # set_spr(26, load_addr) # set SRR0 (Entry Point)

    set_spr(26, addr) # set SRR0 (Entry Point)
    if msr:
        set_spr(27, msr) # set SRR1 (next MSR)
    get_spr(148) # reset ICR

    instr( rfi() ) # go


def try_stop(timeout=1):
    print("Try NMI trap stop")
    cmd_nmi_trap()
    if not wait_debug(timeout):
        print("Try SRESET stop")
        enter_debug()
        if not wait_debug(timeout):
            print("Failed")
            return False
    return True

def wait_stop(timeout=None, exit_timeout=1):
    if not wait_debug(timeout):
        return try_stop(exit_timeout)
    return True


def addr2line(dbgsym_file, addr, reloc_ofs=0):
    if dbgsym_file is None:
        return f"0x{addr:08x}\n"

    addr = (addr - reloc_ofs) & 0xffffffff
    return subprocess.check_output([
            'addr2line', '-pifCa', '-e', dbgsym_file, f"0x{addr:08x}"
        ]).decode()

def print_target_stack(dbgsym_file, reloc_ofs=0):
    # read_ram_f = read_ram_inside
    read_ram_f = read_ram
    print(f".:", addr2line(dbgsym_file, get_spr(26), reloc_ofs), end='') # SRR0
    print(f"0:", addr2line(dbgsym_file, get_spr(8), reloc_ofs), end='') # LR
    
    def get_stack(sp):
        frame = read_ram_f(sp)[0]
        while frame:
            old_frame = frame
            frame, lr = read_ram_f(frame, 2)
            if old_frame == frame:
                print("stack is looped!")
                return
            yield lr

    for i, lr in enumerate(get_stack(get_gpr(1))):
        print(f"{1+i}:", addr2line(dbgsym_file, lr-4, reloc_ofs), end='')


def dump_all_immr():

    def print_ram1(addr, s):
        addr |= 0xfff00000
        a = addr
        a &= 0xFFFFFFFC
        d = read_ram(a, 1)[0]
        d >>= 8*(addr&0x3)
        d &= (1<<(8*(s+1)))-1
        dstr = f"{d:0{2*s}x}".rjust(8)
        print(f"{addr:08x}: {dstr}", end='')

    print("General System Interface Unit")
    print("SIUMCR          = ", end=''); print_ram1(0x000, 4); print("    (SIU module configuration register)")
    print("SYPCR           = ", end=''); print_ram1(0x004, 4); print("    (System protection control register)")
    print("SWSR            = ", end=''); print_ram1(0x00E, 2); print("    (Software service register)")
    print("SIPEND          = ", end=''); print_ram1(0x010, 4); print("    (SIU interrupt pending register)")
    print("SIMASK          = ", end=''); print_ram1(0x014, 4); print("    (SIU interrupt mask register)")
    print("SIEL            = ", end=''); print_ram1(0x018, 4); print("    (SIU interrupt edge/level register)")
    print("SIVEC           = ", end=''); print_ram1(0x01C, 4); print("    (SIU interrupt vector register)")
    print("TESR            = ", end=''); print_ram1(0x020, 4); print("    (Transfer error status register)")
    print("SDCR            = ", end=''); print_ram1(0x030, 4); print("    (SDMA configuration register)")

    print()
    print("PCMCIA")
    print("PBR0            = ", end=''); print_ram1(0x080, 4); print("    (PCMCIA interface base register 0)")
    print("POR0            = ", end=''); print_ram1(0x084, 4); print("    (PCMCIA interface option register 0)")
    print("PBR1            = ", end=''); print_ram1(0x088, 4); print("    (PCMCIA interface base register 1)")
    print("POR1            = ", end=''); print_ram1(0x08C, 4); print("    (PCMCIA interface option register 1)")
    print("PBR2            = ", end=''); print_ram1(0x090, 4); print("    (PCMCIA interface base register 2)")
    print("POR2            = ", end=''); print_ram1(0x094, 4); print("    (PCMCIA interface option register 2)")
    print("PBR3            = ", end=''); print_ram1(0x098, 4); print("    (PCMCIA interface base register 3)")
    print("POR3            = ", end=''); print_ram1(0x09C, 4); print("    (PCMCIA interface option register 3)")
    print("PBR4            = ", end=''); print_ram1(0x0A0, 4); print("    (PCMCIA interface base register 4)")
    print("POR4            = ", end=''); print_ram1(0x0A4, 4); print("    (PCMCIA interface option register 4)")
    print("PBR5            = ", end=''); print_ram1(0x0A8, 4); print("    (PCMCIA interface base register 5)")
    print("POR5            = ", end=''); print_ram1(0x0AC, 4); print("    (PCMCIA interface option register 5)")
    print("PBR6            = ", end=''); print_ram1(0x0B0, 4); print("    (PCMCIA interface base register 6)")
    print("POR6            = ", end=''); print_ram1(0x0B4, 4); print("    (PCMCIA interface option register 6)")
    print("PBR7            = ", end=''); print_ram1(0x0B8, 4); print("    (PCMCIA interface base register 7)")
    print("POR7            = ", end=''); print_ram1(0x0BC, 4); print("    (PCMCIA interface option register 7)")
    print("PGCRA           = ", end=''); print_ram1(0x0E0, 4); print("    (PCMCIA interface general control register A)")
    print("PGCRB           = ", end=''); print_ram1(0x0E4, 4); print("    (PCMCIA interface general control register B)")
    print("PSCR            = ", end=''); print_ram1(0x0E8, 4); print("    (PCMCIA interface status changed register)")
    print("PIPR            = ", end=''); print_ram1(0x0F0, 4); print("    (PCMCIA interface input pins register)")
    print("PER             = ", end=''); print_ram1(0x0F8, 4); print("    (PCMCIA interface enable register)")

    print()
    print("Memory Controller")
    print("BR0             = ", end=''); print_ram1(0x100, 4); print("    (Base register bank 0)")
    print("OR0             = ", end=''); print_ram1(0x104, 4); print("    (Option register bank 0)")
    print("BR1             = ", end=''); print_ram1(0x108, 4); print("    (Base register bank 1)")
    print("OR1             = ", end=''); print_ram1(0x10C, 4); print("    (Option register bank 1)")
    print("BR2             = ", end=''); print_ram1(0x110, 4); print("    (Base register bank 2)")
    print("OR2             = ", end=''); print_ram1(0x114, 4); print("    (Option register bank 2)")
    print("BR3             = ", end=''); print_ram1(0x118, 4); print("    (Base register bank 3)")
    print("OR3             = ", end=''); print_ram1(0x11C, 4); print("    (Option register bank 3)")
    print("BR4             = ", end=''); print_ram1(0x120, 4); print("    (Base register bank 4)")
    print("OR4             = ", end=''); print_ram1(0x124, 4); print("    (Option register bank 4)")
    print("BR5             = ", end=''); print_ram1(0x128, 4); print("    (Base register bank 5)")
    print("OR5             = ", end=''); print_ram1(0x12C, 4); print("    (Option register bank 5)")
    print("BR6             = ", end=''); print_ram1(0x130, 4); print("    (Base register bank 6)")
    print("OR6             = ", end=''); print_ram1(0x134, 4); print("    (Option register bank 6)")
    print("BR7             = ", end=''); print_ram1(0x138, 4); print("    (Base register bank 7)")
    print("OR7             = ", end=''); print_ram1(0x13C, 4); print("    (Option register bank 7)")
    print("MAR             = ", end=''); print_ram1(0x164, 4); print("    (Memory address register)")
    print("MCR             = ", end=''); print_ram1(0x168, 4); print("    (Memory command register)")
    print("MAMR            = ", end=''); print_ram1(0x170, 4); print("    (Machine A mode register)")
    print("MBMR            = ", end=''); print_ram1(0x174, 4); print("    (Machine B mode register)")
    print("MSTAT           = ", end=''); print_ram1(0x178, 2); print("    (Memory status register)")
    print("MPTPR           = ", end=''); print_ram1(0x17A, 2); print("    (Memory periodic timer prescaler)")
    print("MDR             = ", end=''); print_ram1(0x17C, 4); print("    (Memory data register)")

    print()
    print("System Integration Timers")
    print("TBSCR           = ", end=''); print_ram1(0x200, 2); print("    (Timebase status and control register)")
    print("TBREFA          = ", end=''); print_ram1(0x204, 4); print("    (Timebase reference register A)")
    print("TBREFB          = ", end=''); print_ram1(0x208, 4); print("    (Timebase reference register B)")
    print("PISCR           = ", end=''); print_ram1(0x240, 2); print("    (Periodic interrupt status and control register)")
    print("PITC            = ", end=''); print_ram1(0x244, 4); print("    (Periodic interrupt count register)")
    print("PITR            = ", end=''); print_ram1(0x248, 4); print("    (Periodic interrupt timer register)")

    print()
    print("Clocks and Reset")
    print("SCCR            = ", end=''); print_ram1(0x280, 4); print("    (System clock reset control register)")
    print("PLPRCR          = ", end=''); print_ram1(0x284, 4); print("    (PLL and reset control register)")
    print("RSR             = ", end=''); print_ram1(0x288, 4); print("    (Reset status register)")

    print()
    print("System Integration Timers Keys")
    print("TBSCRK          = ", end=''); print_ram1(0x300, 4); print("    (Timebase status and control register key)")
    print("TBREFAK         = ", end=''); print_ram1(0x304, 4); print("    (Timebase reference register A key)")
    print("TBREFBK         = ", end=''); print_ram1(0x308, 4); print("    (Timebase reference register B key)")
    print("TBK             = ", end=''); print_ram1(0x30C, 4); print("    (Timebase/decrementer register key)")
    print("PISCRK          = ", end=''); print_ram1(0x340, 4); print("    (Periodic interrupt status and control register key)")
    print("PITCK           = ", end=''); print_ram1(0x344, 4); print("    (Periodic interrupt count register key)")

    print()
    print("Clocks and Reset Keys")
    print("SCCRK           = ", end=''); print_ram1(0x380, 4); print("    (System clock control key)")
    print("PLPRCRK         = ", end=''); print_ram1(0x384, 4); print("    (PLL and reset control register key)")
    print("RSRK            = ", end=''); print_ram1(0x388, 4); print("    (Reset status register key)")

    print()
    print("I2C Controller")
    print("I2MOD           = ", end=''); print_ram1(0x860, 1); print("    (I2C mode register)")
    print("I2ADD           = ", end=''); print_ram1(0x864, 1); print("    (I2C address register)")
    print("I2BRG           = ", end=''); print_ram1(0x868, 1); print("    (I2C BRG register)")
    print("I2COM           = ", end=''); print_ram1(0x86C, 1); print("    (I2C command register)")
    print("I2CER           = ", end=''); print_ram1(0x870, 1); print("    (I2C event register)")
    print("I2CMR           = ", end=''); print_ram1(0x874, 1); print("    (I2C mask register)")

    print()
    print("DMA")
    print("SDAR            = ", end=''); print_ram1(0x904, 4); print("    (SDMA address register)")
    print("SDSR            = ", end=''); print_ram1(0x908, 1); print("    (SDMA status register)")
    print("SDMR            = ", end=''); print_ram1(0x90C, 1); print("    (SDMA mask register)")
    print("IDSR1           = ", end=''); print_ram1(0x910, 1); print("    (IDMA1 status register)")
    print("IDMR1           = ", end=''); print_ram1(0x914, 1); print("    (IDMA1 mask register)")
    print("IDSR2           = ", end=''); print_ram1(0x918, 1); print("    (IDMA2 status register)")
    print("IDMR2           = ", end=''); print_ram1(0x91C, 1); print("    (IDMA2 mask register)")

    print()
    print("Communications Processor Module Interrupt Control")
    print("CIVR            = ", end=''); print_ram1(0x930, 2); print("    (CPM interrupt vector register)")
    print("CICR            = ", end=''); print_ram1(0x940, 4); print("    (CPM interrupt configuration register)")
    print("CIPR            = ", end=''); print_ram1(0x944, 4); print("    (CPM interrupt pending register)")
    print("CIMR            = ", end=''); print_ram1(0x948, 4); print("    (CPM interrupt mask register)")
    print("CISR            = ", end=''); print_ram1(0x94C, 4); print("    (CPM in-service register)")

    print()
    print("Input/Output Port")
    print("PADIR           = ", end=''); print_ram1(0x950, 2); print("    (Port A data direction register)")
    print("PAPAR           = ", end=''); print_ram1(0x952, 2); print("    (Port A pin assignment register)")
    print("PAODR           = ", end=''); print_ram1(0x954, 2); print("    (Port A open drain register)")
    print("PADAT           = ", end=''); print_ram1(0x956, 2); print("    (Port A data register)")
    print("PCDIR           = ", end=''); print_ram1(0x960, 2); print("    (Port C data direction register)")
    print("PCPAR           = ", end=''); print_ram1(0x962, 2); print("    (Port C pin assignment register)")
    print("PCSO            = ", end=''); print_ram1(0x964, 2); print("    (Port C special options register)")
    print("PCDAT           = ", end=''); print_ram1(0x966, 2); print("    (Port C data register)")
    print("PCINT           = ", end=''); print_ram1(0x968, 2); print("    (Port C interrupt control register)")
    print("PDDIR           = ", end=''); print_ram1(0x970, 2); print("    (Port D data direction register)")
    print("PDPAR           = ", end=''); print_ram1(0x972, 2); print("    (Port D pin assignment register)")
    print("PDDAT           = ", end=''); print_ram1(0x976, 2); print("    (Port D data register)")
    print("UTMODE          = ", end=''); print_ram1(0x978, 4); print("    (UTOPIA mode register  42.2/-1)")

    print()
    print("CPM General-Purpose Timers")
    print("TGCR            = ", end=''); print_ram1(0x980, 2); print("    (Timer global configuration register)")
    print("TMR1            = ", end=''); print_ram1(0x990, 2); print("    (Timer 1 mode register)")
    print("TMR2            = ", end=''); print_ram1(0x992, 2); print("    (Timer 2 mode register)")
    print("TRR1            = ", end=''); print_ram1(0x994, 2); print("    (Timer 1 reference register)")
    print("TRR2            = ", end=''); print_ram1(0x996, 2); print("    (Timer 2 reference register)")
    print("TCR1            = ", end=''); print_ram1(0x998, 2); print("    (Timer 1 capture register)")
    print("TCR2            = ", end=''); print_ram1(0x99A, 2); print("    (Timer 2 capture register)")
    print("TCN1            = ", end=''); print_ram1(0x99C, 2); print("    (Timer 1 counter)")
    print("TCN2            = ", end=''); print_ram1(0x99E, 2); print("    (Timer 2 counter)")
    print("TMR3            = ", end=''); print_ram1(0x9A0, 2); print("    (Timer 3 mode register)")
    print("TMR4            = ", end=''); print_ram1(0x9A2, 2); print("    (Timer 4 mode register)")
    print("TRR3            = ", end=''); print_ram1(0x9A4, 2); print("    (Timer 3 reference register)")
    print("TRR4            = ", end=''); print_ram1(0x9A6, 2); print("    (Timer 4 reference register)")
    print("TCR3            = ", end=''); print_ram1(0x9A8, 2); print("    (Timer 3 capture register)")
    print("TCR4            = ", end=''); print_ram1(0x9AA, 2); print("    (Timer 4 capture register)")
    print("TCN3            = ", end=''); print_ram1(0x9AC, 2); print("    (Timer 3 counter)")
    print("TCN4            = ", end=''); print_ram1(0x9AE, 2); print("    (Timer 4 counter)")
    print("TER1            = ", end=''); print_ram1(0x9B0, 2); print("    (Timer 1 event register)")
    print("TER2            = ", end=''); print_ram1(0x9B2, 2); print("    (Timer 2 event register)")
    print("TER3            = ", end=''); print_ram1(0x9B4, 2); print("    (Timer 3 event register)")
    print("TER4            = ", end=''); print_ram1(0x9B6, 2); print("    (Timer 4 event register)")

    print()
    print("Communications Processor (CP)")
    print("CPCR            = ", end=''); print_ram1(0x9C0, 2); print("    (CP command register)")
    print("RCCR            = ", end=''); print_ram1(0x9C4, 2); print("    (RISC controller configuration register)")
    print("RMDS            = ", end=''); print_ram1(0x9C7, 1); print("    (RISC microcode development support control register)")
    print("RCTR1           = ", end=''); print_ram1(0x9CC, 2); print("    (RISC controller trap register 1)")
    print("RCTR2           = ", end=''); print_ram1(0x9CE, 2); print("    (RISC controller trap register 2)")
    print("RCTR3           = ", end=''); print_ram1(0x9D0, 2); print("    (RISC controller trap register 3)")
    print("RCTR4           = ", end=''); print_ram1(0x9D2, 2); print("    (RISC controller trap register 4)")
    print("RTER            = ", end=''); print_ram1(0x9D6, 2); print("    (RISC timer event register)")
    print("RTMR            = ", end=''); print_ram1(0x9DA, 2); print("    (RISC timers mask register)")

    print()
    print("Baud Rate Generators")
    print("BRGC1           = ", end=''); print_ram1(0x9F0, 4); print("    (BRG1 configuration register)")
    print("BRGC2           = ", end=''); print_ram1(0x9F4, 4); print("    (BRG2 configuration register)")
    print("BRGC3           = ", end=''); print_ram1(0x9F8, 4); print("    (BRG3 configuration register)")
    print("BRGC4           = ", end=''); print_ram1(0x9FC, 4); print("    (BRG4 configuration register)")

    print()
    print("Serial Communications Controller 1 (SCC1)")
    print("GSMR_L1         = ", end=''); print_ram1(0xA00, 4); print("    (SCC1 general mode register)")
    print("GSMR_H1         = ", end=''); print_ram1(0xA04, 4); print("    (SCC1 general mode register)")
    print("PSMR1           = ", end=''); print_ram1(0xA08, 2); print("    (SCC1 protocol specific mode register)")
    print("TODR1           = ", end=''); print_ram1(0xA0C, 2); print("    (SCC1 transmit-on-demand register)")
    print("DSR1            = ", end=''); print_ram1(0xA0E, 2); print("    (SCC1 data synchronization register)")
    print("SCCE1           = ", end=''); print_ram1(0xA10, 2); print("    (SCC1 event register)")
    print("SCCM1           = ", end=''); print_ram1(0xA14, 2); print("    (SCC1 mask register)")
    print("SCCS1           = ", end=''); print_ram1(0xA17, 1); print("    (SCC1 status register)")

    print()
    print("Serial Communications Controller 2 (SCC2)")
    print("GSMR_L2         = ", end=''); print_ram1(0xA20, 4); print("    (SCC2 general mode register)")
    print("GSMR_H2         = ", end=''); print_ram1(0xA24, 4); print("    (SCC2 general mode register)")
    print("PSMR2           = ", end=''); print_ram1(0xA28, 2); print("    (SCC2 protocol-specific mode register)")
    print("TODR2           = ", end=''); print_ram1(0xA2C, 2); print("    (SCC2 transmit on demand register)")
    print("DSR2            = ", end=''); print_ram1(0xA2E, 2); print("    (SCC2 data synchronization register)")
    print("SCCE2           = ", end=''); print_ram1(0xA30, 2); print("    (SCC2 event register)")
    print("SCCM2           = ", end=''); print_ram1(0xA34, 2); print("    (SCC2 mask register)")
    print("SCCS2           = ", end=''); print_ram1(0xA37, 1); print("    (SCC2 status register)")
    print("GSMR_L3         = ", end=''); print_ram1(0xA40, 4); print("    (SCC3 general mode register)")
    print("GSMR_H3         = ", end=''); print_ram1(0xA44, 4); print("    (SCC3 general mode register)")
    print("PSMR3           = ", end=''); print_ram1(0xA48, 2); print("    (SCC3 protocol specific mode register)")
    print("TODR3           = ", end=''); print_ram1(0xA4C, 2); print("    (SCC3 transmit on demand register)")
    print("DSR3            = ", end=''); print_ram1(0xA4E, 2); print("    (SCC3 data synchronization register)")
    print("SCCE3           = ", end=''); print_ram1(0xA50, 2); print("    (SCC3 event register)")
    print("SCCM3           = ", end=''); print_ram1(0xA54, 2); print("    (SCC3 mask register)")
    print("SCCS3           = ", end=''); print_ram1(0xA57, 1); print("    (SCC3 status register)")
    print("GSMR_L4         = ", end=''); print_ram1(0xA60, 4); print("    (SCC4 general mode register)")
    print("GSMR_H4         = ", end=''); print_ram1(0xA64, 4); print("    (SCC4 general mode register)")
    print("PSMR4           = ", end=''); print_ram1(0xA68, 2); print("    (SCC4 protocol specific mode register)")
    print("TODR4           = ", end=''); print_ram1(0xA6C, 2); print("    (SCC4 transmit on demand register)")
    print("DSR4            = ", end=''); print_ram1(0xA6E, 2); print("    (SCC4 data synchronization register)")
    print("SCCE4           = ", end=''); print_ram1(0xA70, 2); print("    (SCC4 event register)")
    print("SCCM4           = ", end=''); print_ram1(0xA74, 2); print("    (SCC4 mask register)")
    print("SCCS4           = ", end=''); print_ram1(0xA77, 1); print("    (SCC4 status register)")

    print()
    print("Serial Management Controller 1 (SMC1)")
    print("SMCMR1          = ", end=''); print_ram1(0xA82, 2); print("    (SMC1 mode register)")
    print("SMCE1           = ", end=''); print_ram1(0xA86, 1); print("    (SMC1 event register)")
    print("SMCM1           = ", end=''); print_ram1(0xA8A, 1); print("    (SMC1 mask register)")
    print("SMCE2           = ", end=''); print_ram1(0xA96, 1); print("    (SMC2 event register)")
    print("SMCM2           = ", end=''); print_ram1(0xA9A, 1); print("    (SMC2 mask register)")

    print()
    print("Serial Peripheral Interface (SPI)")
    print("SPMODE          = ", end=''); print_ram1(0xAA0, 2); print("    (SPI mode register)")
    print("SPIE            = ", end=''); print_ram1(0xAA6, 1); print("    (SPI event register)")
    print("SPIM            = ", end=''); print_ram1(0xAAA, 1); print("    (SPI mask register)")
    print("SPCOM           = ", end=''); print_ram1(0xAAD, 1); print("    (SPI command register)")

    print()
    print("Parallel Interface Port (PIP) and Port B")
    print("PIPC            = ", end=''); print_ram1(0xAB2, 2); print("    (PIP configuration register)")
    print("PTPR            = ", end=''); print_ram1(0xAB6, 2); print("    (PIP timing parameters register)")
    print("PBDIR           = ", end=''); print_ram1(0xAB8, 4); print("    (Port B data direction register)")
    print("PBPAR           = ", end=''); print_ram1(0xABC, 4); print("    (Port B pin assignment register)")
    print("PBODR           = ", end=''); print_ram1(0xAC0, 4); print("    (Port B open drain register)")
    print("PBDAT           = ", end=''); print_ram1(0xAC4, 4); print("    (Port B data register)")

    print()
    print("Serial Interface (SI)")
    print("SIMODE          = ", end=''); print_ram1(0xAE0, 4); print("    (SI mode register)")
    print("SIGMR           = ", end=''); print_ram1(0xAE4, 1); print("    (SI global mode register)")
    print("SISTR           = ", end=''); print_ram1(0xAE6, 1); print("    (SI status register)")
    print("SICMR           = ", end=''); print_ram1(0xAE7, 1); print("    (SI command register)")
    print("SICR            = ", end=''); print_ram1(0xAEC, 4); print("    (SI clock route register)")
    print("SIRP            = ", end=''); print_ram1(0xAF0, 4); print("    (Serial interface RAM pointer register)")
    # print("SIRAM           = ", end=''); print_ram1(0xC00, 512); print("    (SI routing RAM)")

    print()
    print("Fast Ethernet Controller (FEC)")
    print("ADDR_LOW        = ", end=''); print_ram1(0xE00, 4); print("    ()")
    print("ADDR_HIGH       = ", end=''); print_ram1(0xE04, 4); print("    ()")
    print("HASH_TABLE_HIGH = ", end=''); print_ram1(0xE08, 4); print("    ()")
    print("HASH_TABLE_LOW  = ", end=''); print_ram1(0xE0C, 4); print("    ()")
    print("R_DES_START     = ", end=''); print_ram1(0xE10, 4); print("    ()")
    print("X_DES_START     = ", end=''); print_ram1(0xE14, 4); print("    ()")
    print("R_BUFF_SIZE     = ", end=''); print_ram1(0xE18, 4); print("    ()")
    print("ECNTRL          = ", end=''); print_ram1(0xE40, 4); print("    ()")
    print("IEVENT          = ", end=''); print_ram1(0xE44, 4); print("    ()")
    print("IMASK           = ", end=''); print_ram1(0xE48, 4); print("    ()")
    print("IVEC            = ", end=''); print_ram1(0xE4C, 4); print("    ()")
    print("R_DES_ACTIVE    = ", end=''); print_ram1(0xE50, 4); print("    ()")
    print("X_DES_ACTIVE    = ", end=''); print_ram1(0xE54, 4); print("    ()")
    print("MII_DATA        = ", end=''); print_ram1(0xE80, 4); print("    ()")
    print("MII_SPEED       = ", end=''); print_ram1(0xE84, 4); print("    ()")
    print("R_BOUND         = ", end=''); print_ram1(0xECC, 4); print("    ()")
    print("R_FSTART        = ", end=''); print_ram1(0xED0, 4); print("    ()")
    print("X_WMRK          = ", end=''); print_ram1(0xEE4, 4); print("    ()")
    print("X_FSTART        = ", end=''); print_ram1(0xEEC, 4); print("    ()")
    print("FUN_CODE        = ", end=''); print_ram1(0xF34, 4); print("    ()")
    print("R_CNTRL         = ", end=''); print_ram1(0xF44, 4); print("    ()")
    print("R_HASH          = ", end=''); print_ram1(0xF48, 4); print("    ()")
    print("X_CNTRL         = ", end=''); print_ram1(0xF84, 4); print("    ()")

    # print()
    # print("Dual-Port RAM (DPRAM)")
    # print("DPRAM1          = ", end=''); print_ram1(0x200, 4096); print("    (Dual-port system RAM)")
    # print("DPRAM2          = ", end=''); print_ram1(0x300, 3072); print("    (Dual-port system RAM expansion)")
    # print("PRAM            = ", end=''); print_ram1(0x3C0, 1024); print("    (Dual-port parameter RAM)")



# print("Reset")
# enter_debug()
# print("wait_ready")
# wait_ready()
# print("wait_debug")
# if not wait_stop(timeout=1):
#     exit()

# set_spr(638, 0xFFF00000)
# print(f">> IMMR = {get_spr(638):08x}")
# # print(f">> r1 = {get_gpr(1):08x}")
# # print(f">> r3 = {get_gpr(3):08x}")
# # print(f">> r4 = {get_gpr(4):08x}")

# write_ram(0xFFF00004, [0xFFFFFF03]) # SYPCR # Disable watchdog

# with measure_time(count=0x2000): write_ram_fast(0xFFF02000, [0xFFFFFFFF]*(0x2000//4)) # 39 kB/s
# with measure_time(count=0x2000): write_ram(0xFFF02000, [0xFFFFFFFF]*(0x2000//4)) # 14 kB/s
# with measure_time(count=0x2000): read_ram(0xFFF02000, 0x2000//4) # 20 kB/s

# print("")
# print("Init:")
# print_icr()
# print_sprs()
# # write_ram(0xfff00100, [0x00000401, 0x00000ff4] + [0]*6*2) # BRx/ORx
# # print("BRx/ORx"); print_ram(0xfff00100, 14)
# print("")


# print("Prepare:")

# write_ram_fast(ram_buf, [0xFFFFFFFF]*(0x1000//4)) # clear ram_buf


# do_this = False


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




# cmd_soft_reset()
# cmd_hard_reset()


# finish_debug()
