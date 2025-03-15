// bonobo.cpp -- for the Copico-Bonobo v2.4 -- strick@yak.net
//
// SPDX-License-Identifier: MIT

// HINT: $ minicom -b 115200 -D /dev/ttyACM0

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/structs/systick.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <stdio.h>

#include <functional>

extern "C" {
extern int stdio_usb_in_chars(char* buf, int length);
}
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#define force_inline inline __attribute__((always_inline))

typedef unsigned char byte;
typedef unsigned int word;

#include "control.pio.h"
#include "status.pio.h"

byte hi(word w) {
    return (byte)(w>>8);
}
byte lo(word w) {
    return (byte)w;
}
word hilo(byte hi, byte lo) {
    return (word(hi)<<8) | word(lo);
}

void putbyte(byte x) { putchar_raw(x); }

bool getbyte(byte* ptr) {
      int rc = stdio_usb_in_chars((char*)ptr, 1);
      return (rc != PICO_ERROR_NO_DATA);
}

void InPin(int p) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_IN);
}
void OutPin(int p, uint value) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, value);
}

constexpr uint LED_PIN = 25;
void LED(uint x) { gpio_put(LED_PIN, x); }

uint WAIT_GET(PIO pio, uint sm) {
  while (pio_sm_is_rx_fifo_empty(pio, sm)) continue;

  return pio_sm_get(pio, sm);
}

void PUT(PIO pio, uint sm, uint x) {
  pio_sm_put(pio, sm, x);
}

void InitialBlinks() {
  for (uint blink = 0; blink < 5; blink++) {
    LED(0);
    sleep_ms(500);
    LED(1);
    sleep_ms(500);
    printf("blink %d\n", blink);
  }
}

void StartPio1() {
  constexpr uint sm0 = 0, sm1 = 1;

  pio_clear_instruction_memory(pio1);

  const uint offset0 = pio_add_program(pio1, &control_pio_program);
  control_pio_init(pio1, sm0, offset0);
  pio_sm_clear_fifos(pio1, sm0);
  // pio_sm_restart(pio1, sm0);

  const uint offset1 = pio_add_program(pio1, &status_pio_program);
  status_pio_init(pio1, sm1, offset1);
  pio_sm_clear_fifos(pio1, sm1);
  // pio_sm_restart(pio1, sm1);
}

int main() {
  for (uint i = 0; i < 16; i++) InPin(i);
  OutPin(6, 1); // cart
  OutPin(7, 1); // nmi
  OutPin(16, 1); // direction
  OutPin(17, 1); // halt
  OutPin(18, 1); // slenb
  OutPin(19, 1); // spoon
  InPin(20); // reset
  InPin(21); // E clock
  InPin(22); // Write Control
  InPin(26); // Read Control
  InPin(27); // Write Data
  InPin(28); // Read Data
  OutPin(LED_PIN, 1);

  stdio_usb_init();
  printf("*** HELLO BONOBO\n");

  InitialBlinks();
  StartPio1();
  int lit = 0;
  int count = 0;
  while (true) {
    constexpr uint sm0 = 0, sm1 = 1;
    uint c = WAIT_GET(pio1, sm0);
    PUT(pio1, sm1, c);

    printf(" (%d) ", c);

    LED(lit);
    lit = !lit;
    count++;
    if ((count&31) == 0) printf("\n");
  }
}
