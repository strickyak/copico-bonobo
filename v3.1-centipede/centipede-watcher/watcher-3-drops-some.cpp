#include <hardware/clocks.h>
// #include <hardware/dma.h>
// #include <hardware/pio.h>
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

#ifdef __cplusplus
}
#endif

using byte = uint8_t;

#define G_RW 20
#define G_E  21
#define G_Q  22

#define G_D0 0
#define G_A0 32

#define G_LED  25 // 27 // 28 // 25
#define SET_LED(X) gpio_put(G_LED, (X))

#if 0
#define CYCLES_TO_CAPTURE 1024
uint64_t Capture[CYCLES_TO_CAPTURE];
uint16_t Before[CYCLES_TO_CAPTURE];
uint16_t After[CYCLES_TO_CAPTURE];
#endif

#include <array>
#include <atomic>
#include <cstdint>

template <typename T, uint32_t Size>
class CrossCoreFIFO {
    static_assert((Size & (Size - 1)) == 0, "Size must be a power of 2");

public:
    CrossCoreFIFO() : head(0), tail(0) {}

    // Called by the Producer Core
    bool push(const T& item) {
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
    bool pop(T& item) {
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

#if 0
void InitInPin(int pin) {
  // gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_set_pulls(pin, /*up=*/true, /*down=*/false);
}
void OutPin(int pin, bool value) {
  // gpio_init(pin);  // GPIO needs to own the output pin.
  // gpio_put(pin, value);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, value);
}
#endif

void InitializePins() {
    for (uint i = 0; i <= 22; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }
    for (uint i = 25; i <= 29; i++) {
        // 25=(LED), 26=/NMI, 27=/RESET, 28=/HALT, 29=/SLENB
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_set_pulls(i, /*up=*/true, /*down=*/false);
    }
    for (uint i = 30; i <= 47; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
    }

    // LED off.
    gpio_init(G_LED);
    gpio_set_dir(G_LED, GPIO_OUT);
    SET_LED(0);
}

// byte ram[64 * 1024];
//byte sent[64 * 1024];

CrossCoreFIFO<byte, 128*1024 > fifo;

#if 0
void SendRamlet(uint addr, byte data) {
    putchar_raw(195);
    putchar_raw((addr>>8)&255);
    putchar_raw(addr&255);
    putchar_raw(data);
}
#endif
void SendRamletFifo(uint addr, byte data) {
    fifo.push(195);
    fifo.push((addr>>8)&255);
    fifo.push(addr&255);
    fifo.push(data);
}

void core1_main_fifo() {
    while (true) {
        byte x;
        bool ok = fifo.pop(x);
        if (ok) {
            putchar_raw(x);
            // if (x==195) printf("\n");
            // printf("%02x ", x);
        }
    }
}

#if 0
void core1_main() {
    while (true) {
        SendRamlet(0xFF23, ram[0xFF23]);
        __DSB(); // Data Synchronization Barrier (DSB)
        for (uint addr=0x200; addr < 0x400; addr++) {
            SendRamlet(addr, ram[addr]);
        }
        sleep_ms(100);
    }
}
#endif

int core0_main() {
  while (true) {

    //for (uint i = 0; i < CYCLES_TO_CAPTURE; i++) {
        // Wait for a rise of E.
        uint16_t before = 0;
        while (gpio_get(G_E) == false) before++;
        //Before[i] = before;

        // Wait for a fall of E, grabbing pins.
        uint16_t after = 0;
        uint64_t pins = 0;
        while (gpio_get(G_E) == true) {
            pins = gpio_get_all64();
            after++;
        }
        // pins = gpio_get_all64();
        //Capture[i] = pins;
        //After[i] = after;
        {
            bool write = (pins & (1lu << G_RW)) == 0;
            if (write) {
                uint addr = (pins >> 32) & 0xFFFFu;
                byte data = pins & 0xFFu;
                /// ram[addr] = data;

                if (0x400 <= addr && addr < 0x600) {
                    // printf("(%04x:%02x %d %d)", addr, data, before, after);
                    SendRamletFifo(addr, data);
                }
                // __DSB();

            }
        }
    //}
  }
}
int main() {
  set_sys_clock_khz(250000, true);  // overclock
  stdio_usb_init();
  printf("*** This is Centipede.\n");

  InitializePins();
#if 1
  multicore_launch_core1(core1_main_fifo);
#endif
  core0_main();
}
