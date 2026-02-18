#include <hardware/clocks.h>
// #include <hardware/dma.h>
#include <hardware/pio.h>
#include "hardware/sync.h"
// #include <hardware/structs/systick.h>
// #include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/multicore.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <setjmp.h>
#include <stdio.h>
#include <arm_acle.h>
#include <cmsis_gcc.h>

extern int stdio_usb_in_chars(char* buf, int length);

#ifdef __cplusplus
}
#endif

#include "watch.pio.h"

#define G_RW 20
#define G_E  21
#define G_Q  22
#define G_NMI 26
#define G_RESET 27
#define G_HALT 28
#define G_SLENB 29

#define G_D0 0
#define G_A0 32

#define G_LED  25 // 27 // 28 // 25
#define SET_LED(X) gpio_put(G_LED, (X))

#include <array>
#include <atomic>
#include <cstdint>

using byte = unsigned char;

#define INCLUDING
#include "disk11_rom.c" // byte disk11_rom[8192]

using IOReader = byte(*)(uint addr);
using IOWriter = void(*)(uint addr, byte d);

IOReader Readers[256];
IOWriter Writers[256];

byte DefaultReader(uint addr) { gpio_set_dir_out_masked(0); return 43; }
void DefaultWriter(uint addr, byte d) { gpio_set_dir_out_masked(0); }

#define MARK_FLOPPY_LATCH        0x100
#define MARK_FLOPPY_COMMAND      0x200
#define MARK_FLOPPY_TRACK        0x300
#define MARK_FLOPPY_SECTOR       0x400
#define MARK_FLOPPY_MASK         0xFFFFFF00

// Code from fast to slow main.
#define SLOW_SEND_NMI        150

// Code to tethered PC.
//
// Length is explicit:
#define C_LOGGING    130
#define C_DISK_READ    173
#define C_DISK_WRITE   174
//
// Length is implicit:
#define C_PUTCHAR 193
#define C_RAM2_WRITE 195
#define C_CYCLE_RD3  211

// Commands into the FIFO to the slow core
#define FIFO_READ  0x01000000u
#define FIFO_ROM   0x02000000u
#define FIFO_WRITE 0x03000000u
#define FIFO_NMI   0x04000000u
#define FIFO_FLOPPY_COMMAND 0x05000000u

byte floppy_latch;
byte floppy_command;
byte floppy_status;
byte floppy_track;
byte floppy_sector;
byte *floppy_ptr;
byte *floppy_limit;
byte floppy_buf[256];

byte ram[64 * 1024];

#define volatile_sio_hw ((volatile sio_hw_t *)SIO_BASE)

static inline bool volatile_gpio_get(uint gpio) {
#if NUM_BANK0_GPIOS <= 32
    return volatile_sio_hw->gpio_in & (1u << gpio);
#else
    if (gpio < 32) {
        return volatile_sio_hw->gpio_in & (1u << gpio);
    } else {
        return volatile_sio_hw->gpio_hi_in & (1u << (gpio - 32));
    }
#endif
}


void InitializePins() {
    for (uint i = 0; i <= 22; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }
    for (uint i = 25; i <= 29; i++) {
        gpio_init(i);
        switch (i) {
        case G_RESET:
            gpio_set_dir(i, GPIO_IN);
            gpio_set_pulls(i, /*up=*/true, /*down=*/false);
            break;
        default:
            gpio_set_dir(i, GPIO_IN);
            gpio_set_pulls(i, /*up=*/true, /*down=*/false);
            gpio_set_dir(i, GPIO_OUT);
            gpio_put(i, 1);
            break;
        }
    }

    for (uint i = 30; i <= 47; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }

    // LED off.
    gpio_init(G_LED);
    gpio_set_dir(G_LED, GPIO_OUT);
    SET_LED(0);
}

constexpr uint N = 1000;
uint32_t Record[N];

void HaltOn() {
        gpio_set_dir(G_HALT, true);
        gpio_put(G_HALT, false);
}
void HaltOff() {
        sleep_us(100);
        gpio_put(G_HALT, true);
        gpio_set_dir(G_HALT, false);
}

void Fatal(const char* s) {
    for (const char* p = "FATAL: "; *p; p++) {
        putchar(C_PUTCHAR);
        putchar(*p);
    }
    for (const char* p = s; *p; p++) {
        putchar(C_PUTCHAR);
        putchar(*p);
    }
    printf("\nFATAL: %s\n", s);
    while (1) continue;
}

void ReceiveSectorData() {
    char c = 0;
    int rc;
    do {
        rc = stdio_usb_in_chars(&c, 1);
    } while (rc == PICO_ERROR_NO_DATA);

    if (byte(c) != 0xAD) Fatal("bad c");

    int needed = 7;
    char* p = (char*)floppy_buf; // first write with unneeded header
    while (needed > 0) {
        rc = stdio_usb_in_chars(p, needed);
        if (rc == PICO_ERROR_NO_DATA) continue;

        p += rc;
        needed -= rc;
    }

    needed = 256;
    p = (char*)floppy_buf; // overwrite with good data
    while (needed > 0) {
        rc = stdio_usb_in_chars(p, needed);
        if (rc == PICO_ERROR_NO_DATA) continue;

        p += rc;
        needed -= rc;
    }
}

void background() {
    while(1) {
            uint x = multicore_fifo_pop_blocking();
            HaltOn();
            switch (x>>24) {
                case 0:
                    putchar_raw(x);
                    break;

                case FIFO_READ>>24: // read cycle
                    // printf("+%04Xr%02X ", 0xFFFF & (x>>8), 0xFF&x);
                    // putchar('r');
                    putchar_raw(C_CYCLE_RD3);
                    putchar_raw(x>>16);
                    putchar_raw(x>>8);
                    putchar_raw(x);
                    break;

                case FIFO_ROM>>24: // ROM cycle
                    // printf("\nT %04X:%02X ", (x>>8), 0xFF&x);
                    putchar_raw(C_CYCLE_RD3);
                    putchar_raw(x>>16);
                    putchar_raw(x>>8);
                    putchar_raw(x);
                    break;

                case FIFO_WRITE>>24: // write cycle
                    // putchar('w');
                    // printf("+%04Xw%02X ", (x>>8), 0xFF&x);
                    putchar_raw(C_RAM2_WRITE);
                    putchar_raw(x>>16);
                    putchar_raw(x>>8);
                    putchar_raw(x);
                    break;

                case FIFO_NMI>>24:
                    gpio_put(G_NMI, 0);  // Activate NMI
                    sleep_us(2);  // for more than a cycle
                    gpio_put(G_NMI, 1);  // Activate NMI
                                         //
                    // putchar('Q');

                    putchar_raw(C_LOGGING);
                    putchar_raw(4 + 128);
                    putchar_raw('N');
                    putchar_raw('M');
                    putchar_raw('I');
                    putchar_raw('\n');
                    break;
                case FIFO_FLOPPY_COMMAND>>24:
                    switch (x&0xFF) {
                        case 0x17: // seek track
                            floppy_track = floppy_buf[0];
                            break;

                        case 0x80: // read sector
                            printf(" %d:%d", floppy_track, floppy_sector);
                            putchar_raw(C_DISK_READ);
                            putchar_raw(5 + 128);
                            putchar_raw('f');
                            putchar_raw(x);
                            putchar_raw(floppy_latch);
                            putchar_raw(floppy_track);
                            putchar_raw(floppy_sector);

                            ReceiveSectorData();
                            floppy_ptr = floppy_buf;

                            printf(" ");
                            break;
                    }
                    break;
                default:
                    printf("\nWUT? FIFO %x\n", x);
            }
            sleep_us(5);
            HaltOff();
    }
}

#define STALL_WHILE(PIN,HL,CH) {                                 \
    uint spin = 0;                                               \
    /* Read twice, and check twice, to avoid glitches. */        \
    while (volatile_gpio_get(PIN) == HL || volatile_gpio_get(PIN)==HL) {++spin;}   \
    /*if (spin<1) multicore_fifo_push_blocking(CH);*/            \
}

void foreground() {
    // Disable interrupts in this "fast" core.
    save_and_disable_interrupts();

    while (true) {

#if 0
        volatile uint early, extra;
        {
            uint spin = 0;
            do {
                // Read twice, and check twice, to avoid glitches.
                early = SIO_HW->gpio_in;
                extra = SIO_HW->gpio_in;
                ++spin;
            } while (extra != early || (early & (1<<G_E)) == 0);
            // if (spin<2) multicore_fifo_push_blocking('v');
        }
#else
        STALL_WHILE(G_E, 0, 'v');
        volatile uint early = volatile_sio_hw->gpio_in;
#endif
        uint addr = sio_hw->gpio_hi_in & 0xFFFF;
        bool writing = ((early & (1u << G_RW)) == 0);

        if (writing) {
          // CPU WRITING -- we RX
            STALL_WHILE(G_Q , 1, 'p');
            byte late = (byte)sio_hw->gpio_in;  // late grab of data
            ram[addr] = late;

            IOWriter w = 0;
            if (addr >= 0xFF00) {
                w = Writers[addr&0x00FF];
                if (w) multicore_fifo_push_blocking(FIFO_WRITE | (addr<<8) | late);
                if (w) w(addr, late);
            }

            STALL_WHILE(G_E , 1, 'q');

          // END WRITING
        } else {
          // CPU READING -- we TX
          if (addr >= 0xFF00) {
              IOReader r = Readers[addr&0x00FF];
              if (r) {
                byte data = r(addr);
                gpio_set_dir_out_masked(0xFF);
                gpio_put_masked(0xFF, data);

                STALL_WHILE(G_E , 1, 'r');

                gpio_set_dir_in_masked(0xFF);
                // multicore_fifo_push_blocking(FIFO_READ | (addr<<8) | data);
              }
          } else if (0xC000 <= addr && addr < 0xE000) {
              byte data = disk11_rom[addr & 0x1FFF];

              gpio_set_dir_out_masked(0xFF);
              gpio_put_masked(0xFF, data);

              // multicore_fifo_push_blocking(FIFO_ROM | (addr<<8) | data);
              STALL_WHILE(G_E , 1, 's');

              gpio_set_dir_in_masked(0xFF);

#if 0
          } else if (addr < 0x8000) {
              byte data = ram[addr];

              gpio_put(G_SLENB, 0);
              gpio_set_dir_out_masked(0xFF);
              gpio_put_masked(0xFF, data);

              STALL_WHILE(G_E , 1, 't');
              gpio_put(G_SLENB, 1);

              gpio_set_dir_in_masked(0xFF);
#endif
          } else {
              STALL_WHILE(G_E , 1, 'u');
          }
          // END READING
        } // end if (writing) else
    } // while true
}

byte Read42(uint addr) { return 42; }

void FloppyWriteLatch(uint a, byte d) {
    floppy_latch = d;
    // multicore_fifo_push_blocking(MARK_FLOPPY_LATCH | d);
    //multicore_fifo_push_blocking('L');
}
void FloppyWriteCommand(uint a, byte d) {

    floppy_status = ((d & 0xF0) == 0x80) ? 0x02: 0x00; // YAK

    floppy_ptr = floppy_buf; // Reset pointer.
    if (d == 0x17) floppy_track = floppy_buf[0]; // was losing critical race

    multicore_fifo_push_blocking(FIFO_FLOPPY_COMMAND | d);
    //multicore_fifo_push_blocking('C');
}
void FloppyWriteTrack(uint a, byte d) {
    floppy_track = d;
    //multicore_fifo_push_blocking('T');
}
void FloppyWriteSector(uint a, byte d) {
    floppy_sector = d;
    //multicore_fifo_push_blocking('S');
}
void FloppyWriteData(uint a, byte d) {
    //multicore_fifo_push_blocking('D');
    *floppy_ptr++ = d;
    if ((floppy_latch&0x80)!=0 && floppy_ptr >= floppy_limit) multicore_fifo_push_blocking(FIFO_NMI);
}
byte FloppyReadStatus(uint a) {
    //multicore_fifo_push_blocking('$');
    byte z = floppy_status;
    floppy_status &= 1;  // Clear all except BUSY.
    return z;
}
byte FloppyReadData(uint a) {
    //multicore_fifo_push_blocking('#');
    byte z = *floppy_ptr++;
    if ((floppy_latch&0x80)!=0 && floppy_ptr >= floppy_limit) {
            floppy_ptr = floppy_buf;
            multicore_fifo_push_blocking(FIFO_NMI);
    }
    return z;
}

int main() {
  InitializePins();
  set_sys_clock_khz(250000, true);  // overclock
  stdio_usb_init();

  // TEST:  PRINT PEEK(&HFF70) -> 42
  Readers[0x70] = Read42;

  // Floppy
  Writers[0x40] = FloppyWriteLatch;
  Writers[0x48] = FloppyWriteCommand;
  Writers[0x49] = FloppyWriteTrack;
  Writers[0x4A] = FloppyWriteSector;
  Writers[0x4B] = FloppyWriteData;

  Readers[0x48] = FloppyReadStatus;
  Readers[0x4B] = FloppyReadData;

  floppy_limit = 256 + floppy_buf;

  // foreground must be fast.
  multicore_launch_core1(foreground);

  background(); // background on core 0 handles interrupts.
}
