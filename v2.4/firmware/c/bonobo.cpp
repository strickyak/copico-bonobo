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

void InPin(int p, uint value) {
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
  //const PIO pio = pio0;
  //constexpr uint sm = 0;

  while (pio_sm_is_rx_fifo_empty(pio, sm)) continue;

  return pio_sm_get(pio, sm);
}

void PUT(uint x) {
  const PIO pio = pio0;
  constexpr uint sm = 0;
  pio_sm_put(pio, sm, x);
}

void StartPio(PIO pio) {
  constexpr uint sm0 = 0;

  pio_clear_instruction_memory(pio);

  const uint offset = pio_add_program(pio, &control_pio_program);
  control_pio_init(pio, sm0, offset);
}

struct repeating_timer TimerData;
volatile uint TimerTicks;
volatile bool TimerFired;
bool TimerCallback(repeating_timer_t* rt) {
  TimerTicks++;
  TimerFired = true;
  return true;
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

int main() {
  OutPin(16, 1); // direction
  OutPin(17, 1); // halt
  OutPin(18, 1); // slenb
  OutPin(19, 1); // spoon
  InPin(20, 1); // reset
  InPin(21, 1); // E clock
  InPin(22, 1); // Write Control
  InPin(26, 1); // Read Control
  InPin(27, 1); // Write Data
  InPin(28, 1); // Read Data
  OutPin(LED_PIN, 1);

  stdio_usb_init();
  printf("*** HELLO BONOBO\n");

  // alarm_pool_init_default();
  // add_repeating_timer_us(16666 /* 60 Hz */, TimerCallback, nullptr, &TimerData);

  InitialBlinks();
  StartPio(pio1);
  int lit = 0;
  int count = 0;
  while (true) {
    constexpr uint sm0 = 0;
    uint c = WAIT_GET(pio1, sm0);
    printf(" (%d) ", c);
    LED(lit);
    lit = !lit;
    count++;
    if ((count&31) == 0) printf("\n");
  }
}
