#include <hardware/clocks.h>
// #include <hardware/dma.h>
// #include <hardware/pio.h>
// #include <hardware/structs/systick.h>
// #include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <setjmp.h>
#include <stdio.h>

using byte = uint8_t;

#define G_RW 20
#define G_E  21
#define G_Q  22

#define G_D0 0
#define G_A0 32

#define G_LED  25 // 27 // 28 // 25
#define SET_LED(X) gpio_put(G_LED, (X))

#define CYCLES_TO_CAPTURE 1024
uint64_t Capture[CYCLES_TO_CAPTURE];
uint16_t Before[CYCLES_TO_CAPTURE];
uint16_t After[CYCLES_TO_CAPTURE];

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
        // gpio_set_dir(i, GPIO_OUT);
        // gpio_put(i, true); // /NMI, /RESET, /HALT, /SLENB
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

byte ram[64 * 1024];
byte sent[64 * 1024];

void core1_main() {
    while (true) {
        for (int i=0; i < 80; i++) {
            putchar('z');
        }
        putchar('\n');
    }
}

int main() {
  set_sys_clock_khz(250000, true);  // overclock
  stdio_usb_init();
  printf("*** This is Centipede.\n");

  InitializePins();
#if 0
  multicore_launch_core1(core1_main);
#endif

  while (true) {

    for (uint i = 0; i < CYCLES_TO_CAPTURE; i++) {
        // Wait for a rise of E.
        uint16_t before = 0;
        while (gpio_get(G_E) == false) before++;
        Before[i] = before;

        // Wait for a fall of E, grabbing pins.
        uint16_t after = 0;
        uint64_t pins = 0;
        while (gpio_get(G_E) == true) {
            pins = gpio_get_all64();
            after++;
        }
        Capture[i] = pins;
        After[i] = after;
#if 0
        {
            bool write = (pins & (1lu << G_RW)) == 0;
            if (write) {
                uint addr = (pins >> 32) & 0xFFFFu;
                uint data = pins & 0xFFu;
                ram[addr] = data;
            }
        }
#endif
    }
#if 1
    for (uint i = 0; i < CYCLES_TO_CAPTURE; i++) {

        uint64_t pins = Capture[i];

        uint addr = (pins >> 32) & 0xFFFFu;
        bool write = (pins & (1lu << G_RW)) == 0;
        uint data = pins & 0xFFu;

        printf("%4d: %04x %c %02x (%d,%d)\n", i, addr, write?'W':'R', data, Before[i], After[i]);
    }
#endif
  }
}
