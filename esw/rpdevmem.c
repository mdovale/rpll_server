/*
 * Minimal devmem-like utility for RedPitaya register access.
 * Use when devmem is not available on the target.
 *
 * Usage:
 *   rpdevmem [addr]           - read 32-bit at addr (default 0x42000008)
 *   rpdevmem addr value       - write value to addr
 *   rpdevmem -d [base]        - dump 16 bytes from base (default 0x42000000)
 *   rpdevmem -t [addr]        - test: write 0xffffffff then 0, report read-back
 *
 * Requires root. Addresses are physical (e.g. 0x42000000 = CFG_RST).
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_SIZE 4096
#define PAGE_MASK (PAGE_SIZE - 1)

static void* map_base;
static size_t map_len;

static void* map_phys(unsigned addr, size_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return NULL;
    }
    unsigned offset = addr & PAGE_MASK;
    unsigned page_addr = addr & ~PAGE_MASK;
    map_len = (len + offset + PAGE_MASK) & ~PAGE_MASK;
    map_base = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_addr);
    close(fd);
    if (map_base == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    return (char*)map_base + offset;
}

static void unmap_phys(void)
{
    if (map_base != MAP_FAILED && map_base != NULL)
        munmap(map_base, map_len);
}

static uint32_t read_u32(const void* ptr)
{
    uint32_t v;
    memcpy(&v, ptr, sizeof(v));
    return v;
}

static void write_u32(void* ptr, uint32_t v)
{
    memcpy(ptr, &v, sizeof(v));
}

int main(int argc, char** argv)
{
    unsigned addr = 0x42000008;  /* pll_flag_open (AXI GPIO Data2) by default */
    int do_write = 0;
    unsigned value = 0;
    int do_dump = 0;
    int do_test = 0;
    unsigned dump_base = 0x42000000;

    if (argc >= 2 && strcmp(argv[1], "-d") == 0) {
        do_dump = 1;
        if (argc >= 3)
            dump_base = (unsigned)strtoul(argv[2], NULL, 0);
    } else if (argc >= 2 && strcmp(argv[1], "-t") == 0) {
        do_test = 1;
        if (argc >= 3)
            addr = (unsigned)strtoul(argv[2], NULL, 0);
    } else if (argc >= 2) {
        addr = (unsigned)strtoul(argv[1], NULL, 0);
        if (argc >= 3) {
            value = (unsigned)strtoul(argv[2], NULL, 0);
            do_write = 1;
        }
    }

    if (do_test) {
        void* p = map_phys(addr, 4);
        if (!p)
            return 1;
        printf("Test 0x%08x: write 0xffffffff -> ", addr);
        write_u32(p, 0xFFFFFFFF);
        __sync_synchronize();
        printf("read 0x%08x; write 0 -> ", read_u32(p));
        write_u32(p, 0);
        __sync_synchronize();
        printf("read 0x%08x\n", read_u32(p));
        unmap_phys();
        return 0;
    }

    if (do_dump) {
        void* p = map_phys(dump_base, 16);
        if (!p)
            return 1;
        printf("0x%08x: %08x %08x %08x %08x\n", dump_base,
               read_u32((const char*)p + 0),
               read_u32((const char*)p + 4),
               read_u32((const char*)p + 8),
               read_u32((const char*)p + 12));
        unmap_phys();
        return 0;
    }

    void* p = map_phys(addr, 4);
    if (!p)
        return 1;

    if (do_write) {
        write_u32(p, value);
        __sync_synchronize();
        printf("Wrote 0x%x to 0x%08x\n", value, addr);
    }
    printf("0x%08x: 0x%08x (%u)\n", addr, read_u32(p), read_u32(p));
    unmap_phys();
    return 0;
}
