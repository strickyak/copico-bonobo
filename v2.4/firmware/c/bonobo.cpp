// bonobo.cpp -- for the Copico-Bonobo v2.4 -- strick@yak.net
//
// SPDX-License-Identifier: MIT

// HINT: $ minicom -b 115200 -D /dev/ttyACM0

#include <hardware/clocks.h>
#include <hardware/dma.h>
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

#include "on_reset.pio.h"
#include "spoon.pio.h"
#include "control.pio.h"
#include "status.pio.h"
#include "read.pio.h"
#include "write.pio.h"

#include "life-semi.decb.for-c.h"
#include "bootdata.h"

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

bool TryGetFromFifo(PIO pio, uint sm, byte* octet_ptr) {
  if (pio_sm_is_rx_fifo_empty(pio, sm)) return false;

  *octet_ptr = pio_sm_get(pio, sm);
  return true;
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

template <uint N>
class CircBuf {
 private:
  byte buf[N];
  uint nextIn, nextOut;

 public:
  CircBuf() : nextIn(0), nextOut(0) {}

  uint NumBytesBuffered() {
    if (nextOut <= nextIn)
      return nextIn - nextOut;
    else
      return nextIn + N - nextOut;
  }

  byte Peek() { return buf[nextOut]; }

  byte Take() {
    byte z = buf[nextOut];
    ++nextOut;
    if (nextOut >= N) nextOut = 0;
    return z;
  }

  void Put(byte x) {
    buf[nextIn] = x;
    ++nextIn;
    if (nextIn >= N) nextIn = 0;
  }
};
CircBuf<20000> McpBuf;

void StartPio1() {
  constexpr uint sm0 = 0, sm1 = 1, sm2 = 2, sm3 = 3;

  pio_clear_instruction_memory(pio1);

  const uint offset0 = pio_add_program(pio1, &control_pio_program);
  control_pio_init(pio1, sm0, offset0);
  pio_sm_clear_fifos(pio1, sm0);
  // pio_sm_restart(pio1, sm0);

  const uint offset1 = pio_add_program(pio1, &status_pio_program);
  status_pio_init(pio1, sm1, offset1);
  pio_sm_clear_fifos(pio1, sm1);

  const uint offset2 = pio_add_program(pio1, &read_pio_program);
  read_pio_init(pio1, sm2, offset2);
  pio_sm_clear_fifos(pio1, sm2);

  const uint offset3 = pio_add_program(pio1, &write_pio_program);
  read_pio_init(pio1, sm3, offset3);
  pio_sm_clear_fifos(pio1, sm3);
}

void Panic() {
    printf("\nPanic!\n");
    while (true) {
        // Triple blinking, a second apart.
        LED(1);
        sleep_ms(200);
        LED(0);
        sleep_ms(200);
        LED(1);
        sleep_ms(200);
        LED(0);
        sleep_ms(200);
        LED(1);
        sleep_ms(200);

        LED(0);
        sleep_ms(1000);
    }
}

// READ PIO DMA: The first read is lost, from bigger[0].
// The next 256 reads are read by 256 read cycles, dma_buffer[0-255].
// Then one final read cycle clears the pipe, counts as 257,
// but does not cause "direction" to change,
// so what is going on in the state machine!?

constexpr uint HACK = 1; // Mark where our DMA ReadToCoco is Off by One and Hacked.
byte bigger[256 + HACK];
#define dma_buffer (bigger+HACK)

void StartDmaTx(int channel, dma_channel_config *config, uint size) {
        constexpr uint sm2 = 2;

        // enable DMA
        channel_config_set_read_increment(config, true);
        channel_config_set_write_increment(config, false);
        channel_config_set_dreq(config,
                                pio_get_dreq(pio1, sm2, /*is_tx*/true));

        channel_config_set_transfer_data_size(config, DMA_SIZE_8);
        channel_config_set_irq_quiet(config, true);

        channel_config_set_enable(config, true);

        dma_channel_configure(
            channel,            // Channel to be configured
            config,            // The configuration we just created
            &pio1->txf[sm2],    // The initial write address
            dma_buffer - HACK,  // The initial read address ([HACK] with 1 waste at front)
            size + HACK,           // Number of transfers, [HACK] counting 1 waste at back.
            true                // Start immediately.      
        );
}

int main() {
  OutPin(16, 1); // direction
  OutPin(17, 1); // halt
  OutPin(18, 1); // slenb
  OutPin(19, 1); // spoon

  for (uint i = 0; i < 16; i++) InPin(i);
  OutPin(6, 1); // cart
  OutPin(7, 1); // nmi

  InPin(20); // reset
  InPin(21); // E clock
  InPin(22); // Write Control
  InPin(26); // Read Control
  InPin(27); // Write Data
  InPin(28); // Read Data
  OutPin(LED_PIN, 1);

  stdio_usb_init();
  printf("*** HELLO BONOBO\n");
  for (uint i =0; i < 256; i++) dma_buffer[i] = (byte)(i ^ 8);

  InitialBlinks();

  extern void spoonfeed();
  spoonfeed();

  StartPio1();

  int channel = dma_claim_unused_channel(/*required*/true);
  dma_channel_config config = dma_channel_get_default_config(channel);

  int lit = 0;
  int count = 0;
  uint num_bytes_to_mcp = 0;

  while (true) {
    ////////////// From MCP
    // Attempt to get a byte from the MCP.
    // If we get one, add it to the McpBuf.
    byte octet;
    if (getbyte(&octet)) {
        McpBuf.Put(octet);
    }

    ////////////// From COCO
    constexpr uint sm0 = 0, sm1 = 1, sm2 = 2, sm3 = 3;

    if (TryGetFromFifo(pio1, sm0, &octet)) {

        byte status_reply = 1;  // the default
        if (1 <= octet && octet <= 100) {
            // COCO READS n BYTES FROM MCP.
            uint n = octet;
            printf("[%d]  %d: coco reads %d bytes\n", count, octet, n);

            uint buffered = McpBuf.NumBytesBuffered();
            if (buffered < n) {
                printf("PANIC: wanted %d bytes from MCP, only %d buffered.\n",
                        n, buffered);
                Panic();
            }
            // GetBytesFromTether:
            for (uint i = 0; i < n; i++) {
                dma_buffer[i] = McpBuf.Take();
            }
            StartDmaTx(channel, &config, n);

        } else if (101 <= octet && octet <= 200) {
            // COCO WRITES n BYTES TO MCP.
            uint n = octet-100;
            printf("[%d]  %d: coco writes %d bytes for MCP\n", count, octet, n);

            if (num_bytes_to_mcp != 0) {
                printf("Got octet %d but num_bytes_to_mcp is %d and it should be zero.\n");
                Panic();
            }

            // enable DMA
            channel_config_set_read_increment(&config, false);
            channel_config_set_write_increment(&config, true);
            channel_config_set_dreq(&config,
                                    pio_get_dreq(pio1, sm3, /*is_tx*/false));

            channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
            channel_config_set_irq_quiet(&config, true);

            channel_config_set_enable(&config, true);
            dma_channel_configure(
                channel,           // Channel to be configured
                &config,           // The configuration we just created
                dma_buffer,           // The initial write address
                &pio1->rxf[sm2],   // The initial read address
                n,                 // Number of transfers
                true               // Start immediately.      
            );

            num_bytes_to_mcp = n;

        } else if (octet == 250) {
            // QUERY SIZE OF MCP IN BUFFER
            uint size = McpBuf.NumBytesBuffered();
            printf("[%d]  %d: querying size => %d from MCP buffered\n", count, octet, size);

            dma_buffer[0] = (byte)(size >> 8);
            dma_buffer[1] = (byte)(size >> 0);
            StartDmaTx(channel, &config, 2);

        } else if (octet == 251) {
            // PUSH num_bytes_to_mcp TO MCP.
            printf("[%d]  %d: Going to push %d bytes to MCP\n", count, octet, num_bytes_to_mcp);

            // Send an initial marker 129 to 138, telling how many bytes are coming.
            if (!(1 <= num_bytes_to_mcp && num_bytes_to_mcp <= 100)) {
                printf("Got octet %d (PUSH) but num_bytes_to_mcp is %d\n", octet, num_bytes_to_mcp);
                Panic();
            }
            putbyte(128 + num_bytes_to_mcp);

            // Now send that many data bytes.
            for (uint i = 0; i < num_bytes_to_mcp; i++) {
                putbyte(dma_buffer[i]);
            }
            num_bytes_to_mcp = 0;

        } else if (octet == 252) {
            // Bonobo Presence Query
            status_reply = 'b';  // A special answer.
        } else {
            printf("[%d] Panic: bad command byte %d\n", count, octet);
            Panic();
        }

        PUT(pio1, sm1, status_reply);  // Status is now 1: Ready.

        LED(lit);
        lit = !lit;
        count++;
    }
  }
}

#define G_CART   6
#define G_NMI    7
#define G_DIR   16
#define G_HALT  17
#define G_SLENB 18
#define G_SPOON 19
#define G_RESET 20
#define G_ECLK  21

void spoonfeed() {
  constexpr uint sm0 = 0, sm1 = 1, sm2 = 2, sm3 = 3;

  while (true) {

    bool reset;
    do {
        // wait for RESET to activate
        reset = gpio_get(G_RESET);
    } while (reset == true);
    printf(" Got reset. ");

    gpio_put(G_HALT, 0);   // Activate HALT
    gpio_put(G_SPOON, 0);  // Activate SPOON
    sleep_ms(100);
    printf(" Debounced. ");

    do {
        // wait for RESET to finish
        reset = gpio_get(G_RESET);
    } while (reset == false);
    printf(" Reset gone. ");

    sleep_ms(400);
    printf(" Slept \n");

    pio_clear_instruction_memory(pio0);
    pio_sm_restart (pio0, sm0);
    pio_sm_restart (pio0, sm1);
    pio_sm_restart (pio0, sm2);
    pio_sm_restart (pio0, sm3);

    ///////////////////////////////

    pio_sm_claim(pio0, sm0);
    const uint offset0 = pio_add_program(pio0, &onreset_pio_program);
    onreset_program_init(pio0, sm0, offset0);

    // FF=outputs $8888=reset_vector 00=inputs
    pio_sm_put(pio0, sm0, 0x008888FF);
    pio_sm_set_enabled(pio0, sm0, true);  // Run.
    sleep_ms(1);                         // long enough.
    pio_sm_set_enabled(pio0, sm0, false);  // Stop.
    pio_remove_program_and_unclaim_sm(&onreset_pio_program,
            pio0, sm0, offset0);

      ///////////////////////////////

    pio_clear_instruction_memory(pio0);
    pio_sm_restart (pio0, sm0);
    pio_sm_restart (pio0, sm1);
 
    pio_sm_claim(pio0, sm1);
    const uint offset1 = pio_add_program(pio0, &spoon_pio_program);
    spoon_program_init(pio0, sm1, offset1);

    const unsigned short* start1 = COCO3_INIT_DATA;
    const unsigned short* limit1 = (const unsigned short*)(
         (char*)COCO3_INIT_DATA + sizeof COCO3_INIT_DATA);

    for (const unsigned short* p = start1; p < limit1; p += 2) {
        uint _addr = p[0]; // embiggen unsigned short to uint, for sm1.
        uint _byte = p[1];
        uint w1 = ((_byte&0xFF) << 24) | 0xCCFF; // CC = LDD immediate
        uint w2 = ((_addr&0xFF) << 16) | (_addr & 0xFF00) | 0xF7; // F7 = STB extended
        pio_sm_put(pio0, sm1, w1);
        pio_sm_put(pio0, sm1, w2);

        pio_sm_exec(pio0, sm1, offset1);       // Jump to start of spoon_pio_program
        pio_sm_set_enabled(pio0, sm1, true);  // Run.
        sleep_us(20);                         // long enough.
        pio_sm_set_enabled(pio0, sm1, false);  // Stop.
    }

    const unsigned int* start2 = PairsOfWords;
    const unsigned int* limit2 = (const unsigned int*)(
         (char*)PairsOfWords + sizeof PairsOfWords);

    for (const unsigned int* p = start2; p < limit2; p += 2) {
        uint w1 = p[0];
        uint w2 = p[1];
        pio_sm_put(pio0, sm1, w1);
        pio_sm_put(pio0, sm1, w2);

        pio_sm_exec(pio0, sm1, offset1);       // Jump to start of spoon_pio_program
        pio_sm_set_enabled(pio0, sm1, true);  // Run.
        sleep_us(20);                         // long enough.
        pio_sm_set_enabled(pio0, sm1, false);  // Stop.
    }

    pio_remove_program_and_unclaim_sm(&spoon_pio_program,
        pio0, sm1, offset1);

    OutPin(19, 1); // spoon

    OutPin(16, 1); // direction
    OutPin(17, 1); // halt
    OutPin(18, 1); // slenb

 } // while
}  // spoonfeed
