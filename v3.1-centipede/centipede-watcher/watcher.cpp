#include <hardware/clocks.h>
// #include <hardware/dma.h>
// #include <hardware/pio.h>
// #include <hardware/structs/systick.h>
// #include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <setjmp.h>
#include <stdio.h>

#define G_RW 20
#define G_E  21
#define G_Q  22

#define G_D0 0
#define G_A0 32

#define G_LED 25
#define SET_LED(X) gpio_put(G_LED, (X))

#define CYCLES_TO_CAPTURE 1024
uint64_t Capture[CYCLES_TO_CAPTURE];

void InitInPin(int pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
}
void OutPin(int pin, bool value) {
  gpio_init(pin);  // GPIO needs to own the output pin.
  gpio_put(pin, value);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, value);
}

void InitializePins() {
    // LED off.
    gpio_init(G_LED);
    gpio_set_dir(G_LED, GPIO_OUT);
    SET_LED(0);

    for (uint i = 0; i <= 22; i++) {
        InitInPin(i);
    }
    for (uint i = 30; i <= 47; i++) {
        InitInPin(i);
    }
    OutPin(26, true);  // /NMI
    OutPin(27, true);  // /RESET
    OutPin(28, true);  // /HALT
    OutPin(29, true);  // /SLENB
}

int main() {
  stdio_usb_init();
  printf("*** This is Centipede.\n");

  InitializePins();

  while (true) {

    SET_LED(1);
    sleep_ms(150);
    SET_LED(0);
    sleep_ms(350);
    SET_LED(1);
    sleep_ms(150);
    SET_LED(0);
    sleep_ms(350);

    if (gpio_get(G_Q)) {
        printf("1");
    } else {
        printf("0");
    }

    for (uint i = 0; i < CYCLES_TO_CAPTURE; i++) {
        // Wait for a rise of Q.
        while (gpio_get(G_Q) == false) continue;

        Capture[i] = gpio_get_all64();

        // Wait for a fall of Q.
        while (gpio_get(G_Q) == true) continue;
    }

    for (uint i = 0; i < CYCLES_TO_CAPTURE; i++) {

        // Wait for a rise of Q.
        while (gpio_get(G_Q) == false) continue;

        uint64_t pins = gpio_get_all64();

        // Wait for a fall of Q.
        while (gpio_get(G_Q) == true) continue;

        uint addr = (pins >> 32) & 0xFFFFu;
        bool write = (pins & (1lu << G_RW)) == 0;
        uint data = pins & 0xFFu;

        printf("%4d: %04x %c %02x\n", i, addr, write?'W':'R', data);
    }
  }
}
