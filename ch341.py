import usb.core
import usb.util
import enum
from itertools import islice
from collections.abc import Sequence
import signal


def swap_byte(x):
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa)
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc)
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0)
    return x


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


class CH341A():

    class Endpoints(enum.IntEnum):
        BULK_WRITE = 0x02
        BULK_READ  = 0x82

    class SpiPin(enum.IntEnum):
        CSn  = 1<<0
        CLK  = 1<<3
        MOSI = 1<<5
        MISO = 1<<7

    class Cmd(enum.IntEnum):
        # Vendor define
        VENDOR_WRITE_TYPE   = 0x40  # vendor write command
        VENDOR_READ_TYPE    = 0XC0  # vendor read command

        PARA_INIT           = 0xB1  # Init Parallel    
        I2C_STATUS          = 0x52  # get I2C status
        I2C_COMMAND         = 0x53  # send I2C command

        BUF_CLEAR           = 0xB2  # clear uncompleted data
        I2C_CMD_X           = 0x54  # send I2C command
        DELAY_MS            = 0x5E
        VENDOR_VERSION      = 0x5F  # get version of chip

        CMD_PARA_R0         = 0xAC  # read data0 from parport
        CMD_PARA_R1         = 0xAD  # read data1 from parport
        CMD_PARA_W0         = 0xA6  # write data0 to parport
        CMD_PARA_W1         = 0xA7  # write data1 to parport
        CMD_PARA_STS        = 0xA0  # get status of parport

        # CH341 COMMAND
        CMD_SET_OUTPUT      = 0xA1  # set parport output
        CMD_IO_ADDR         = 0xA2  # MEM IO Addr
        CMD_PRINT_OUT       = 0xA3  # print output
        CMD_SPI_STREAM      = 0xA8  # SPI command
        CMD_SIO_STREAM      = 0xA9  # SIO command
        CMD_I2C_STREAM      = 0xAA  # I2C command
        CMD_UIO_STREAM      = 0xAB  # UIO command

        CMD_I2C_STM_START   = 0x74
        CMD_I2C_STM_STOP    = 0x75
        CMD_I2C_STM_OUT     = 0x80
        CMD_I2C_STM_IN      = 0xC0
        CMD_I2C_STM_MAX     = 0x20
        CMD_I2C_STM_SET     = 0x60 # bit 2: SPI with two data pairs D5,D4=out, D7,D6=in
        CMD_I2C_STM_US      = 0x40
        CMD_I2C_STM_MS      = 0x50
        CMD_I2C_STM_DLY     = 0x0F
        CMD_I2C_STM_END     = 0x00

        CMD_UIO_STM_IN      = 0x00  #  UIO Interface In ( D0 ~ D7 )
        CMD_UIO_STM_DIR     = 0x40  #  UIO interface Dir( set dir of D0~D5 )
        CMD_UIO_STM_OUT     = 0x80  #  UIO Interface Output(D0~D5)
        CMD_UIO_STM_US      = 0xC0  #  UIO Interface Delay Command( us )
        CMD_UIO_STM_END     = 0x20  #  UIO Interface End Command

        # request
        DEBUG_READ          = 0x95  # read two regs
        DEBUG_WRITE         = 0x9A  # write two regs

        USB20_CMD_SPI_BLCK_RD = 0xC3
        
    class I2C_Speed(enum.IntEnum):
        STM_I2C_20K         = 0x00
        STM_I2C_100K        = 0x01
        STM_I2C_400K        = 0x02
        STM_I2C_750K        = 0x03
        STM_SPI_DBL         = 0x04


    DIR_MASK = 0x3F #D6,D7 - input, D0-D5 - output
    MAX_PACKET_SIZE = 32


    def __init__(self):
        self.open()


    def open(self):
        dev = usb.core.find(idVendor=0x1a86, idProduct=0x5512)
        if dev is None:
            raise ValueError('Device is not connected')
        
        dev.default_timeout = 1000

        try:
            cfg = dev.get_active_configuration()
        except usb.core.USBError:
            dev.set_configuration()
            cfg = dev.get_active_configuration()
        
        itf = cfg[(0,0)]
        
        # try:
        #     itf.set_altsetting()
        # except usb.core.USBError as e:
        #     raise
            
        self.dev: usb.core.Device = dev
        self.cfg: usb.core.Configuration = cfg
        self.itf: usb.core.Interface = itf
        
        self.ep_in:  usb.core.Endpoint = usb.util.find_descriptor(itf, bEndpointAddress=self.Endpoints.BULK_READ)
        self.ep_out: usb.core.Endpoint = usb.util.find_descriptor(itf, bEndpointAddress=self.Endpoints.BULK_WRITE)

        # self.PACKET_LENGTH = 


    def close(self):
        usb.util.dispose_resources(self.dev)


    def reset(self):
        self.dev.reset()
        self.open()


    def read(self, *v, **kv):
        return self.ep_in.read(*v, **kv)


    def write(self, *v, **kv):
        return self.ep_out.write(*v, **kv)


    def gpio_setdir(self, dir_bits):
        dir_bits &= self.DIR_MASK
        buf = [
            self.Cmd.CMD_UIO_STREAM,
            self.Cmd.CMD_UIO_STM_DIR | dir_bits,
            self.Cmd.CMD_UIO_STM_END
        ]
        self.write(buf)


    def gpio_setbits(self, bits):
        bits &= self.DIR_MASK
        buf = [
            self.Cmd.CMD_UIO_STREAM,
            self.Cmd.CMD_UIO_STM_OUT | bits,
            self.Cmd.CMD_UIO_STM_END
        ]
        self.write(buf)


    def gpio_getbits(self):
        buf = [
            self.Cmd.CMD_UIO_STREAM,
            self.Cmd.CMD_UIO_STM_IN,
            self.Cmd.CMD_UIO_STM_END
        ]
        self.write(buf)
        buf = self.read(1)
        return buf[0]


    def get_revision(self):
        rev = self.dev.bcdDevice
        return (
            (rev>>8) & 0xFF,
            (rev>>4) & 0xF,
            (rev>>0) & 0xF,
        )


    def i2c_init(self):
        self.i2c_set_speed(self.I2C_Speed.STM_I2C_100K)


    def i2c_set_speed(self, speed: I2C_Speed):
        buf = [
    		self.Cmd.CMD_I2C_STREAM,
            self.Cmd.CMD_I2C_STM_SET | (speed & 0x7),
            self.Cmd.CMD_I2C_STM_END
        ]
        self.write(buf)


    def spi_init(self):
        pin = self.SpiPin
        spd = self.I2C_Speed
        self.gpio_setdir(pin.MOSI | pin.CSn | pin.CLK)
        # self.gpio_setbits(pin.MOSI | pin.CSn)
        self.i2c_set_speed(0)


    def spi_cs(self, sel):
        pin = self.SpiPin
        val = pin.MOSI
        if not sel:
            val |= pin.CSn
        self.gpio_setbits(val)



    def spi_transfer(self, size_or_buffer, do_cs = True):
        read_data = []

        if isinstance(size_or_buffer, Sequence):
            data = size_or_buffer
        else: # here we consider it is a integer
            data = [0xFF for _ in range(size_or_buffer)]

        if do_cs:
            self.spi_cs(True)

        for pkt_data in batched(data, self.MAX_PACKET_SIZE - 1):
            buf = [self.Cmd.CMD_SPI_STREAM]
            buf += [swap_byte(d) for d in pkt_data]
            with DelayedKeyboardInterrupt():
                self.write(buf)
                read_data += self.read(self.MAX_PACKET_SIZE - 1)
        
        if do_cs:
            self.spi_cs(False)

        return [swap_byte(d) for d in read_data]
