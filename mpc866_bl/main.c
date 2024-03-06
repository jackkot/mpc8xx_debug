void __exit();
int *ram_base = (void *)0xfff02000;
int ram_count = 0x1000/4;

static void imemcpy(int *dst, int *src, int cnt) {
    for (int i=0; i<cnt; i++) {
        dst[i] = src[i];
    }
}

static void imemset(int *dst, int fill, int cnt) {
    for (int i=0; i<cnt; i++) {
        dst[i] = fill;
    }
}

void ep_simple() {
    ram_base[0] = 0x11223344;
    __exit();
}

void ep_memcpy(void *dst, void *src, int cnt) {
    // int cnt = (size+3)/4; /* rounded up */
    imemset(dst, 0x0, cnt);
    imemcpy(dst, src, cnt);
    // ram_base[0] = 1;
    __exit();
}

// int ram_ptr=1;

// #define NEXT_RAM_PTR ((ram_ptr==(ram_count-1))?ram_ptr:ram_ptr++)
    
static void flash_raw_write(volatile char *flash, unsigned int ofs, char data) {
    // ram_base[NEXT_RAM_PTR] = (unsigned int)(flash+ofs);
    // ram_base[NEXT_RAM_PTR] = data;
    flash[ofs] = data;
}
static char flash_raw_read(volatile char *flash, unsigned int ofs) {
    char data = flash[ofs];
    // ram_base[NEXT_RAM_PTR] = 0x80000000 | (unsigned int)(flash+ofs);
    // ram_base[NEXT_RAM_PTR] = data;
    return data;
}

static int flash_wait(char *flash, unsigned int addr)
{
    for (int timeout=0; timeout<20000000; timeout++) {
        if (flash_raw_read(flash, addr) == flash_raw_read(flash, addr)) {
            return 0;
        }
    }
    return -1;
}



static int flash_cmd(char *flash, unsigned int addr, char data) {
    flash_raw_write(flash, 0xaaa, 0xaa); // unlock 1
    flash_raw_write(flash, 0x555, 0x55); // unlock 2
    flash_raw_write(flash, addr, data); // cmd
}


static int flash_erase_sector(char *flash, unsigned int sector_start_addr) {
    flash_cmd(flash, 0xaaa, 0x80); // Erase Prologue
    flash_cmd(flash, sector_start_addr, 0x30); // Erase Sector
    return flash_wait(flash, sector_start_addr); // Wait for complete
}

static int flash_write_byte(char *flash, unsigned int addr, char data) {
    flash_cmd(flash, 0xaaa, 0xa0); // Program
    flash_raw_write(flash, addr, data);
    return flash_wait(flash, addr); // Wait for complete
}

static int flash_write(char *flash, unsigned int addr, char *data, unsigned int size) {
    int err;
    for (int ofs=0; ofs<size; ofs++) {
        err = flash_write_byte(flash, addr+ofs, data[ofs]);
        if (err) {
            return err;
        }
    }
    return 0;
}

void ep_flash_erase(char *flash, unsigned int addr, unsigned int size) {
    int err;
    // ram_ptr = 1;

    for(int a=addr; a<addr+size; a++) {
        char d = flash_raw_read(flash, a);
        if (d != 0xff) {
            err = flash_erase_sector(flash, a);
            if (err) {
                ram_base[0] = 1;
                __exit();
            }
        }
    }
    ram_base[0] = 0;
    __exit();
}

void ep_flash_program(char *flash, unsigned int addr, char *data, unsigned int size) {
    int err;
    // ram_ptr = 1;

    err = flash_write(flash, addr, data, size);
    ram_base[0] = err ? 1 : 0;
    __exit();
}

void ep_flash_write_one(char *addr, char data) {
    *addr = data;
    __exit();
}

void ep_flash_read_one(char *addr) {
    ram_base[0] = *addr;
    __exit();
}
