/*
 * watcher-march will work on double-speed coco3 
 * if we are at 250 or 270 MHz (but not 150 or 300).
 * But does not use any PIO yet.
 * */
#define CENTIPEDE_ADD_16K 1
#define CENTIPEDE_24D_EQ 1
#define CENTIPEDE_INVERT_EQ 1
#define CENTIPEDE_FAST_EQ 1
#define CENTIPEDE_CTS    18
#define CENTIPEDE_SCS    19

#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define FORCE_INLINE inline __attribute__((always_inline))

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

#define G_RW 20
#define G_E  21
#define G_Q  22

#if CENTIPEDE_24D_EQ // J6: 1-2 5-6 (Direct E and Q)

#define G_LED  25
#define G_SND  26
#define G_CART 27
#define G_SLENB 28
#define G_HALT 29
#define G_NMI  30
#define G_RESET 31

#else // Original Centipede experiment (all wire-wrapped)

#define G_LED  25
#define G_NMI 26
#define G_RESET 27
#define G_HALT 28
#define G_SLENB 29

#endif

#define G_D0 0
#define G_A0 32

#define SET_LED(X) gpio_put(G_LED, (X))

#include <array>
#include <atomic>
#include <cstdint>

using byte = unsigned char;

#define INCLUDING
#include "disk11_rom.c" // byte disk11_rom[8192]...
// #include "hdbdw3cc3_rom.c"
// #include "hdbchs_rom.c"
// #include "hdbsdc_rom.c"

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
#define FIFO_W_256   0x06000000u  // finished 256 bytes of written data
#define FIFO_FLOPPY_LATCH 0x07000000u

byte floppy_latch;
byte floppy_command;
byte floppy_status;
byte floppy_track;
byte floppy_sector;
byte *floppy_ptr;

byte floppy_buf[256];
#define floppy_limit   (256+floppy_buf)

byte ram[64 * 1024];

#define volatile_sio_hw ((volatile sio_hw_t *)SIO_BASE)

FORCE_INLINE bool inline_volatile_gpio_get(uint pin) {
#if NUM_BANK0_GPIOS <= 32
    return volatile_sio_hw->gpio_in & (1u << pin);
#else
    if (pin < 32) {
        return volatile_sio_hw->gpio_in & (1u << pin);
    } else {
        return volatile_sio_hw->gpio_hi_in & (1u << (pin - 32));
    }
#endif
}

FORCE_INLINE void force_inline_multicore_fifo_push_blocking(uint32_t data) {
    // We wait for the fifo to have some space
    while (!multicore_fifo_wready())
        tight_loop_contents();

    sio_hw->fifo_wr = data;

    // Fire off an event to the other core
    __sev();
}


#if 0
template <typename T, uint32_t Size>
class CrossCoreFIFO {
    static_assert((Size & (Size - 1)) == 0, "Size must be a power of 2");

public:
    CrossCoreFIFO() : head(0), tail(0) {}

    // Called by the Producer Core
    FORCE_INLINE bool push(const T& item) {
        uint32_t next_head = (head.load(std::memory_order_relaxed) + 1) & (Size - 1);

        if (next_head == tail.load(std::memory_order_acquire)) {
            return false; // Buffer full
        }

        data[head.load(std::memory_order_relaxed)] = item;

        // Ensure data is written before head is updated
        head.store(next_head, std::memory_order_release);
        return true;
    }

    // Called by the Consumer Core
    FORCE_INLINE bool pop(T& item) {
        uint32_t current_tail = tail.load(std::memory_order_relaxed);

        if (current_tail == head.load(std::memory_order_acquire)) {
            return false; // Buffer empty
        }

        item = data[current_tail];

        // Ensure data is read before tail is updated
        tail.store((current_tail + 1) & (Size - 1), std::memory_order_release);
        return true;
    }

private:
    std::array<T, Size> data;
    std::atomic<uint32_t> head; // Written by Producer
    std::atomic<uint32_t> tail; // Written by Consumer
};
#endif

void INPUT(int i) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
}
void OUTPUT(int i, int x) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_OUT);
        gpio_put(i, x);
}

void InitializePins() {
    for (uint i = 0; i <= 22; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, false, false);
    }
    OUTPUT( G_LED   , 1 );
#if G_SND
    INPUT( G_SND   );
#endif
#if G_CART
    OUTPUT( G_CART  , 1);
#endif

    // OUTPUT(G_SLENB, 0);
    gpio_init(G_SLENB);
    gpio_set_dir(G_SLENB, GPIO_OUT);
    gpio_put(G_SLENB, 0);
    gpio_set_dir(G_SLENB, GPIO_IN);
    gpio_set_pulls(G_SLENB, false, false);

    // OUTPUT( G_HALT  , 0);
    gpio_init(G_HALT);
    gpio_set_dir(G_HALT, GPIO_OUT);
    gpio_put(G_HALT, 0);
    gpio_set_dir(G_HALT, GPIO_IN);
    gpio_set_pulls(G_HALT, false, false);

    // OUTPUT( G_NMI   , 0);
    gpio_init(G_NMI);
    gpio_set_dir(G_NMI, GPIO_OUT);
    gpio_put(G_NMI, 0);
    gpio_set_dir(G_NMI, GPIO_IN);
    gpio_set_pulls(G_NMI, false, false);

    INPUT( G_RESET  );
    gpio_set_pulls(G_RESET, /*up=*/true, /*down=*/false);

    for (uint i = 32; i <= 47; i++) {
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
        gpio_set_dir(G_HALT, GPIO_OUT);
        // gpio_put(G_HALT, false);
}
void HaltOff() {
        // sleep_us(100);
        // gpio_put(G_HALT, true);
        gpio_set_dir(G_HALT, GPIO_IN);
}

void Fatal(const char* s, int x) {
    for (const char* p = "FATAL: "; *p; p++) {
        putchar(C_PUTCHAR);
        putchar(*p);
    }
    for (const char* p = s; *p; p++) {
        putchar(C_PUTCHAR);
        putchar(*p);
    }
    printf("\nFATAL(%d.): %s\n", x, s);
    while (1) continue;
}

void SendSectorData() {
    for (uint i = 0; i < 256; i++) {
        putchar_raw(floppy_buf[i]);
    }
}

void ReceiveSectorData() {
    char c = 0;
    int rc;
    do {
        rc = stdio_usb_in_chars(&c, 1);
    } while (rc == PICO_ERROR_NO_DATA);

    if (byte(c) != 0xAD) {
        printf(" ReceiveSectorData: rc=%d. c=%d. \n", rc, c);
        Fatal("bad c", (byte)c);
    }

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
                    gpio_set_dir(G_NMI, GPIO_OUT);
                    // gpio_put(G_NMI, 0);  // Activate NMI
                    sleep_us(2);  // for more than a cycle
                    // gpio_put(G_NMI, 1);  // Release NMI
                    gpio_set_dir(G_NMI, GPIO_IN);
                                         //
                    // putchar('Q');

                    putchar_raw(C_LOGGING);
                    putchar_raw(4 + 128);
                    putchar_raw('N');
                    putchar_raw('M');
                    putchar_raw('I');
                    putchar_raw('\n');
                    break;

                case FIFO_FLOPPY_LATCH>>24:
                    {
                        static uint last_latch;
                        if (x != last_latch) {
                            printf(" _%02x ", (x&0xFF));
                            last_latch = x;
                        }
                    }
                    break;

                case FIFO_FLOPPY_COMMAND>>24:
                    printf(" f!%02x ", (x&0xFF));
                    switch (x&0xFF) {
                        case 0x17: // seek track
                            floppy_track = floppy_buf[0];
                            break;

                        case 0x80: // read sector
                            printf(" %dr%d", floppy_track, floppy_sector);
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

                        case 0xA0: // write sector
                            printf(" %dw%d", floppy_track, floppy_sector);
                            putchar_raw(C_DISK_WRITE);
                            putchar_raw(0xC4);
                            putchar_raw(5 + 128);
                            putchar_raw('f');
                            putchar_raw(x);
                            putchar_raw(floppy_latch);
                            putchar_raw(floppy_track);
                            putchar_raw(floppy_sector);

                            floppy_ptr = floppy_buf;

                            break;
                    }
                    break;
                case FIFO_W_256>>24:
                            SendSectorData();
                            floppy_ptr = floppy_buf;

                            printf(" [sent] ");
                    break;
                default:
                    printf("\nWUT? FIFO %x\n", x);
            }
            // sleep_us(5);
            HaltOff();
    }
}


#define STALL_WHILE(PIN,HL,IGNORED) {                                 \
    while (inline_volatile_gpio_get(PIN) == HL) { tight_loop_contents(); }                      \
}


#define SAY(C) force_inline_multicore_fifo_push_blocking((C)&255)

void foreground() {
    // Disable interrupts in this "fast" core.
    save_and_disable_interrupts();

    while (true) {

        STALL_WHILE(G_E, CENTIPEDE_INVERT_EQ, 'v');

        volatile uint signals = volatile_sio_hw->gpio_in;
        uint abus = sio_hw->gpio_hi_in & 0xFFFF;

        constexpr uint NEG_CTS = (1 << CENTIPEDE_CTS);
        constexpr uint NEG_SCS = (1 << CENTIPEDE_SCS);
        constexpr uint NEG_SELECTS = NEG_CTS | NEG_SCS;

        if (LIKELY((signals & NEG_SELECTS) == NEG_SELECTS)) { // Not Special Select

            if (LIKELY((signals & (1u << G_RW)) != 0)) {
              // NORMAL CPU READING -- we TX
              if (abus >= 0xFF00) {
                  IOReader r = Readers[abus&0x00FF];
                  if (r) {
                    byte dbus = r(abus);
                    gpio_set_dir_out_masked(0xFF);
                    gpio_put_masked(0xFF, dbus);
                  } else {
                  }
#if CENTIPEDE_ADD_16K
                } else if (abus < 0x8000) {
                    byte dbus = ram[abus];

                    gpio_set_dir(G_SLENB, GPIO_OUT);
                    // gpio_put(G_SLENB, 0);

                    gpio_set_dir_out_masked(0xFF);
                    gpio_put_masked(0xFF, dbus);
#endif
                } else {
                    ;
                }

                STALL_WHILE(G_E , not CENTIPEDE_INVERT_EQ, 'r');
                busy_wait_at_least_cycles(6);
                gpio_set_dir_in_masked(0xFF);

                // gpio_put(G_SLENB, 1);
                gpio_set_dir(G_SLENB, GPIO_IN);

              // END NORMAL READING
            } else {
              // NORMAL CPU WRITING -- we RX
                STALL_WHILE(G_Q , not CENTIPEDE_INVERT_EQ, 'p');
                byte dbus = (byte)sio_hw->gpio_in;  // late grab of data
                ram[abus] = dbus;

                IOWriter w = 0;
                if (abus >= 0xFF00) {
                    w = Writers[abus&0x00FF];
                    if (w) w(abus, dbus);
                }

                STALL_WHILE(G_E , not CENTIPEDE_INVERT_EQ, 'q');

              // END NORMAL WRITING
            } // end if (writing) else

        } else { // Is Special Select
            if (LIKELY((signals & (1u << G_RW)) != 0)) { // Special CPU READING -- we TX
              byte dbus;

              if (LIKELY((signals & NEG_CTS) == 0)) { // READ CTS
                //SAY('F');
                dbus = disk11_rom[abus & 0x1FFF];

              } else { // READ SCS
                //SAY('R');
                  dbus = ram[abus];
                  switch (abus & 15) {
                      case 0x8: // ReadStatus
    dbus = floppy_status;
    floppy_status &= 1;  // Clear all except BUSY.
                          break;
                      case 0xB: // ReadData
    dbus = *floppy_ptr++;
    if ((floppy_latch&0x80)!=0 && floppy_ptr >= floppy_limit) {
            floppy_ptr = floppy_buf;
            force_inline_multicore_fifo_push_blocking(FIFO_NMI);
    }
                          break;
                      default:
                          break;
                  }
              }

              gpio_set_dir_out_masked(0xFF);
              gpio_put_masked(0xFF, dbus);
              STALL_WHILE(G_E , not CENTIPEDE_INVERT_EQ, 's');
                busy_wait_at_least_cycles(6);
              gpio_set_dir_in_masked(0xFF);
            } else { // Special CPU WRITING -- we RX
              //SAY('W');
              STALL_WHILE(G_Q , not CENTIPEDE_INVERT_EQ, 'p');
              byte dbus = (byte)sio_hw->gpio_in;  // grab dbus after q drops
              ram[abus] = dbus;

              if (LIKELY((signals & NEG_SCS) == 0)) {
                  // WRITE SCS
                  switch (abus & 15) {
                      case 0x0: // WriteLatch
    floppy_latch = dbus;
    force_inline_multicore_fifo_push_blocking(FIFO_FLOPPY_LATCH | dbus);
                          break;
                      case 0x8: // WriteCommand
    floppy_status = ((dbus & 0xF0) == 0x80) || ((dbus & 0xF0) == 0xA0) ? 0x02: 0x00; // YAK

    floppy_ptr = floppy_buf; // Reset pointer.
    if (dbus == 0x17) floppy_track = floppy_buf[0]; // was losing critical race

    force_inline_multicore_fifo_push_blocking(FIFO_FLOPPY_COMMAND | dbus);
                          break;
                      case 0x9: // WriteTrack
    floppy_track = dbus;
                          break;
                      case 0xA: // WriteSector
    floppy_sector = dbus;
                          break;
                      case 0xB: // WriteData
    *floppy_ptr++ = dbus;
    if ((floppy_latch&0x80)!=0 && floppy_ptr >= floppy_limit) {
        force_inline_multicore_fifo_push_blocking(FIFO_W_256);
        force_inline_multicore_fifo_push_blocking(FIFO_NMI);
    }
                          break;
                      default:
                          break;
                  }
              } // end write SCS

              STALL_WHILE(G_E , not CENTIPEDE_INVERT_EQ, 's');
            } // end read or write
        } // end if special
    } // end while true
} // end foreground()

int main() {
  InitializePins();
  set_sys_clock_khz(250000, true);  // overclock
  stdio_usb_init();

  // foreground must be fast.
  multicore_launch_core1(foreground);

  background(); // background on core 0 handles interrupts.
}
