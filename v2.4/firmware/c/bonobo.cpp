// bonobo.cpp -- for the Copico-Bonobo v2.4 -- strick@yak.net
//
// SPDX-License-Identifier: MIT

#include <hardware/clocks.h>
#include <hardware/exception.h>
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

#include "bonobo24.pio.h"

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
      int rc = stdio_usb_in_chars(ptr, 1);
      return (rc != PICO_ERROR_NO_DATA);
}

void OutPin(int p, uint value) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, value);
}

constexpr uint LED_PIN = 25;
void LED(uint x) { gpio_put(LED_PIN, x); }

#if 0
uint WAIT_GET() {
  const PIO pio = pio0;
  constexpr uint sm = 0;

  while (pio_sm_is_rx_fifo_empty(pio, sm)) continue;

  return pio_sm_get(pio, sm);
}

void PUT(uint x) {
  const PIO pio = pio0;
  constexpr uint sm = 0;
  pio_sm_put(pio, sm, x);
}

void StartPio() {
  const PIO pio = pio0;
  constexpr uint sm = 0;

  pio_clear_instruction_memory(pio);
  const uint offset = pio_add_program(pio, &tpio_program);
  tpio_program_init(pio, sm, offset);
}
#endif

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
    Log("blink");
  }
}

int main() {
  stdio_usb_init();
  cyw43_arch_init();
  OutPin(LED_PIN, 1);

  alarm_pool_init_default();
  add_repeating_timer_us(16666 /* 60 Hz */, TimerCallback, nullptr, &TimerData);

  InitialBlinks();
  GET_STUCK();
}
