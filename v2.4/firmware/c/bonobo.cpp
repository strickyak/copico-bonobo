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

// Bonobo 2.4 GPIO Pin Assignments
// outputs:
#define G_LED 25
#define G_CART 6  // only effective is SPOON=0
#define G_NMI 7   // only effective is SPOON=0

// data bus:
#define G_D_BEGIN 8
#define G_D_SIZE 8
#define G_D_LIMIT (G_D_BEGIN + G_D_SIZE)

// outputs:
#define G_DIR 16    // data bus direction. 0=toCoco 1=toPico
#define G_HALT 17   // only effective if SPOON=0
#define G_SLENB 18  // only effective if SPOON=0
#define G_SPOON 19  // 0=drives CART, NMI, HALT, and SLENB to coco

// inputs:
#define G_RESET 20
#define G_ECLK 21
#define G_WC 22
#define G_RC 26
#define G_WD 27
#define G_RD 28

// uint widebuf[100];

extern "C" {
extern int stdio_usb_in_chars(char* buf, int length);
}

#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define FORCE_INLINE inline __attribute__((always_inline))

typedef unsigned char byte;
typedef unsigned int word;

#include "_nekot1.decb.h"
#include "bootdata.h"
//
#include "spoon.pio.h"
#include "vector.pio.h"
//
#include "control.pio.h"
#include "read.pio.h"
#include "status.pio.h"
#include "write.pio.h"

// For USB Serial: PutByte & TryGetByte.

void PutByte(byte x) { putchar_raw(x); }

bool TryGetByte(byte* ptr) {
  int rc = stdio_usb_in_chars((char*)ptr, 1);
  return (rc != PICO_ERROR_NO_DATA);
}

// For GPIO in main core.

void InPin(int pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
}
void OutPin(int pin, uint value) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, value);
}

// LED for fun

void LED(bool x) { gpio_put(G_LED, x); }

uint FifoWaitAndGet(PIO pio, uint sm) {
  while (pio_sm_is_rx_fifo_empty(pio, sm)) continue;

  return pio_sm_get(pio, sm);
}

// FIFO Operations

bool TryFifoGet(PIO pio, uint sm, byte* octet_ptr) {
  if (pio_sm_is_rx_fifo_empty(pio, sm)) return false;

  *octet_ptr = pio_sm_get(pio, sm);
  return true;
}

void FifoPut(PIO pio, uint sm, uint x) { pio_sm_put(pio, sm, x); }

void FifoWaitAndPut(PIO pio, uint sm, uint x) {
  while (pio_sm_is_tx_fifo_full(pio, sm)) continue;
  FifoPut(pio, sm, x);
}

void InitialBlinks() {
  for (uint blink = 0; blink < 3; blink++) {
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
#if 0
    uint j = 8;
    for (uint i = nextOut; i != nextIn; i=(i+1)%N) {
        printf("/%x ", buf[i]);
        if (j-- == 0) break;
    }
#endif
    uint z;
    if (nextOut <= nextIn)
      z = nextIn - nextOut;
    else
      z = nextIn + N - nextOut;
    // printf(" -> %04x buf\n", z);
    return z;
  }

  byte Take() {
    byte z = buf[nextOut];
    ++nextOut;
    if (nextOut == N) nextOut = 0;
    return z;
  }

  void Put(byte x) {
    buf[nextIn] = x;
    ++nextIn;
    if (nextIn == N) nextIn = 0;
  }
};
CircBuf<20000> McpBuf;

void StopPortals(PIO pio) {
  constexpr uint smC = 0, smS = 1, smR = 2,
                 smW = 3;  // control, status, read, & write.

  pio_sm_set_enabled(pio, smC, false);
  pio_sm_set_enabled(pio, smS, false);
  pio_sm_set_enabled(pio, smR, false);
  pio_sm_set_enabled(pio, smW, false);

  pio_clear_instruction_memory(pio);

  pio_sm_restart(pio, smC);
  pio_sm_restart(pio, smS);
  pio_sm_restart(pio, smR);
  pio_sm_restart(pio, smW);
}

void StartPortals(PIO pio) {
  constexpr uint smC = 0, smS = 1, smR = 2,
                 smW = 3;  // control, status, read, & write.

  pio_sm_restart(pio, smC);
  pio_sm_restart(pio, smS);
  pio_sm_restart(pio, smR);
  pio_sm_restart(pio, smW);
  pio_clear_instruction_memory(pio);

  const uint offsetC = pio_add_program(pio, &control_pio_program);
  control_pio_init(pio, smC, offsetC);

  const uint offsetS = pio_add_program(pio, &status_pio_program);
  status_pio_init(pio, smS, offsetS);

  const uint offsetR = pio_add_program(pio, &read_pio_program);
  read_pio_init(pio, smR, offsetR);

  const uint offsetW = pio_add_program(pio, &write_pio_program);
  write_pio_init(pio, smW, offsetW);

  printf("Portal prog offsets: %d, %d, %d, %d\n", offsetC, offsetS, offsetR,
         offsetW);

  pio_sm_set_enabled(pio, smC, true);
  pio_sm_set_enabled(pio, smS, true);
  pio_sm_set_enabled(pio, smR, true);
  pio_sm_set_enabled(pio, smW, true);
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

constexpr uint HACK =
    0;  // Mark where our DMA ReadToCoco is Off by One and Hacked.
byte bigger[256 + HACK];
#define dma_buffer (bigger + HACK)

void StartDmaTx(PIO pio, int sm, int channel, dma_channel_config* config,
                uint size) {
  channel_config_set_read_increment(config, true);
  channel_config_set_write_increment(config, false);
  channel_config_set_dreq(config, pio_get_dreq(pio, sm, /*is_tx*/ true));

  channel_config_set_transfer_data_size(config, DMA_SIZE_8);
  channel_config_set_irq_quiet(config, true);

  channel_config_set_enable(config, true);

  auto txfifo = &pio->txf[sm];

  dma_channel_configure(
      channel,  // Channel to be configured
      config,   // The configuration we just created
      txfifo,   // The initial write address
      dma_buffer -
          HACK,     // The initial read address ([HACK] with 1 waste at front)
      size + HACK,  // Number of transfers, [HACK] counting 1 waste at back.
      true          // Start immediately.
  );
}

void StartDmaRx(PIO pio, int sm, int channel, dma_channel_config* config,
                uint size) {
        // enable DMA: coco writes become Rx inputs, get copied to dma_buffer.
        channel_config_set_read_increment(config, false);
        channel_config_set_write_increment(config, true);
        channel_config_set_dreq(config,
                                pio_get_dreq(pio, sm, /*is_tx*/ false));

        channel_config_set_transfer_data_size(config, DMA_SIZE_8);
        channel_config_set_irq_quiet(config, true);

        channel_config_set_enable(config, true);

        auto rxfifo = &pio->rxf[sm];

        dma_channel_configure(channel,     // Channel to be configured
                              config,      // The configuration we just created
                              dma_buffer,  // The initial write address
                              rxfifo,      // The initial read address
                              size,           // Number of transfers
                              true         // Start immediately.
        );
}

void OperatePortals(PIO pio, int channel, dma_channel_config* config) {
  constexpr uint smC = 0, smS = 1, smR = 2,
                 smW = 3;  // control, status, read, & write.
  // int lit = 0;
  int count = 0;
  uint num_bytes_to_mcp = 0;

  while (true) {

    bool reset = gpio_get(G_RESET);
    if (reset == false) {  // Active Low
      printf("\nOperate got RESET\n");
      break;
    }

    byte octet;
    ////////////// From MCP
    // Attempt to get a byte from the MCP.
    // If we get one, add it to the McpBuf.
    if (TryGetByte(&octet)) {
#if 1
      McpBuf.Put(octet);
#endif
    }
    ////////////// From COCO

    if (TryFifoGet(pio, smC, &octet)) {  // octet <- byte via Control Write
      byte status_reply = 1;              // the default reply, if not changed.

      if (1 <= octet && octet <= 100) {
        // COCO READS n BYTES FROM MCP.
        uint n = octet;
        // printf("[%d]  %d: coco reads %d bytes\n", count, octet, n);
        //printf("[%c,%c]\n", '0' + count, 'A' + octet);

        uint buffered = McpBuf.NumBytesBuffered();
        if (buffered < n) {
          printf("PANIC: wanted %d bytes from MCP, only %d buffered.\n", n,
                 buffered);
          Panic();
        }
        // GetBytesFromTether:
//      printf("TX#%d (buf=%d)\n", n, buffered);
        for (uint i = 0; i < n; i++) {
          dma_buffer[i] = McpBuf.Take();
//        if (i<8) printf(" [%d]g%d ", i, dma_buffer[i]);
        }
//      printf(" ok TX\n");

#if DMA_TX
        StartDmaTx(pio, smR, channel, config, n);
#else
        //for (uint i = 0; i < n; i++) {
            //FifoWaitAndPut(pio, smR, dma_buffer[i]);
        //}
        //printf("TXd %d\n", n);
#endif

      } else if (101 <= octet && octet <= 200) {
        // COCO WRITES n BYTES TO MCP.
        uint n = octet - 100;
        // printf("[%d]  %d: coco writes %d bytes for MCP\n", count, octet, n);
        //printf("[%c,%c]\n", '0' + count, 'a' + octet - 100);

        if (num_bytes_to_mcp != 0) {
          printf(
              "Got octet %d but num_bytes_to_mcp is %d and it should be "
              "zero.\n",
              octet, num_bytes_to_mcp);
          Panic();
        }

#if DMA_RX
        StartDmaRx(pio, smR, channel, config, n);
#endif

        num_bytes_to_mcp = n;

      } else if (octet == 250) {
        // QUERY SIZE OF MCP IN BUFFER
        uint size = McpBuf.NumBytesBuffered();
      //printf("[%d]  %d: querying size => %d from MCP buffered\n", count,
             //octet, size);

        dma_buffer[0] = (byte)(size >> 8);
        dma_buffer[1] = (byte)(size >> 0);
        constexpr uint n = 2;

#if DMA_TX
        StartDmaTx(pio, smR, channel, config, n);
#else
        //for (uint i = 0; i < n; i++) {
            //FifoWaitAndPut(pio, smR, dma_buffer[i]);
        //}
        //printf("TXd %d\n", n);
#endif

      } else if (octet == 251) {
        // PUSH num_bytes_to_mcp TO MCP.
        printf("[%d]  %d: Going to push %d bytes to MCP\n", count, octet,
               num_bytes_to_mcp);

        if (!(1 <= num_bytes_to_mcp && num_bytes_to_mcp <= 100)) {
          printf("Got octet %d (PUSH) but num_bytes_to_mcp is %d\n", octet,
                 num_bytes_to_mcp);
          Panic();
        }

        //for (uint i = 0; i < num_bytes_to_mcp; i++) {
          //printf(" %dp%d ", i, dma_buffer[i]);
        //}
        //printf("\n");

        // First byte of the group is 128 plus the size of the payload.
        // Followed by that many payload bytes.
        PutByte(128 + num_bytes_to_mcp);
        // Now send that many data bytes.
        for (uint i = 0; i < num_bytes_to_mcp; i++) {
          PutByte(dma_buffer[i]);
        }
        num_bytes_to_mcp = 0;

      } else if (octet == 252) {
        // Bonobo Presence Query
        printf("\n[%d] Probe 252->'b'\n", count);
        status_reply = 'b';  // A special answer.
      } else {
        printf("\n[%d] Panic: bad command byte %d\n", count, octet);
        Panic();
      } // what octet

      //FifoPut(pio, smS, 241);           // Try a junk first.
      FifoPut(pio, smS, status_reply);  // Status is now 1: Ready.
      //FifoPut(pio, smS, 244);           // Try a junk first.

      if (1 <= octet && octet <= 100) {
        const uint n = octet;
#if DMA_TX
        // StartDmaTx(pio, smR, channel, config, n);
#else
        for (uint i = 0; i < n; i++) {
            FifoWaitAndPut(pio, smR, dma_buffer[i]);
        }
        // printf("TXd %d\n", n);
#endif
      }

      if (octet == 250) {
        const uint n = 2;
#if DMA_TX
        // StartDmaTx(pio, smR, channel, config, n);
#else
        for (uint i = 0; i < n; i++) {
            FifoWaitAndPut(pio, smR, dma_buffer[i]);
        }
        // printf("TXd %d\n", n);
#endif
      }

#if DMA_RX
#else
      // NON-DMA-RX
      if (101 <= octet && octet <= 200) {
        // COCO WRITES n BYTES TO MCP.
        const uint n = octet - 100;

        for (uint i = 0; i < n; i++) {
            uint x = FifoWaitAndGet(pio, smW);
            dma_buffer[i] = (byte)x;
        }
        printf("RX done %d\n", n);
      }
#endif

      // printf("[%d] -> %d.\n", count, status_reply);

      // LED(lit);
      // lit = !lit;
      count++;
    } // if Try Fifo -> octet
  }

}

void SpoonFeed(PIO pio) {
  constexpr uint sm = 0;  // Only uses sm0.

  bool reset;
  bool led = false;
  do {
    led = !led;
    LED(led);      // toggle the LED
    sleep_ms(50);  // every 50 ms

    // wait for RESET to activate
    reset = gpio_get(G_RESET);
  } while (reset == true);
  printf("SpoonFeed: Got reset.\n");

  // void pio_sm_set_pins_with_mask (PIO pio, uint sm, uint32_t pin_values,
  // uint32_t pin_mask) Lower HALT and SPOON to 0.
  pio_sm_set_pins_with_mask(pio, sm, /*values*/ 0x5u << 16,
                            /*mask*/ 0xFu << 16);

  sleep_ms(100);
  printf(" Debounced. ");

  do {
    // wait for RESET to finish
    sleep_ms(50);
    reset = gpio_get(G_RESET);
  } while (reset == false);
  printf(" Reset gone. ");

  sleep_ms(400);  // Time for other hardware to reset.
  printf(" Slept.\n");

  ///////////////////////////////
  //
  //  Spoon feed Reset Vector.

  pio_clear_instruction_memory(pio);
  const uint offsetVectorProgram = pio_add_program(pio, &vector_pio_program);
  vector_program_init(pio, sm, offsetVectorProgram);

  // FF=outputs $8888=reset_vector 00=inputs
  pio_sm_put(pio, sm, 0x008888FF);
  pio_sm_set_enabled(pio, sm, true);   // Run.
  sleep_us(20);                        // long enough.
  pio_sm_set_enabled(pio, sm, false);  // Stop.

  ///////////////////////////////
  //
  //  smF  (for six-byte fetch sequences)

  pio_clear_instruction_memory(pio);
  const uint offsetSpoonProgram = pio_add_program(pio, &spoon_pio_program);
  spoon_program_init(pio, sm, offsetSpoonProgram);

  // TODO: fix the data to work on COCO1 & 2.
  const unsigned short* startF = COCO3_INIT_DATA;
  const unsigned short* limitF =
      (const unsigned short*)((char*)COCO3_INIT_DATA + sizeof COCO3_INIT_DATA);

  for (const unsigned short* p = startF; p < limitF; p += 2) {
    uint addr = p[0];
    uint value = p[1];  // one-byte value to put at addr.
    uint w1 = ((value & 0xFF) << 24) | 0xCCFF;  // CC = LDD immediate
    uint w2 =
        ((addr & 0xFF) << 16) | (addr & 0xFF00) | 0xF7;  // F7 = STB extended
    pio_sm_put(pio, sm, w1);
    pio_sm_put(pio, sm, w2);

    pio_sm_exec(pio, sm,
                offsetSpoonProgram);    // Jump to start of spoon_pio_program
    pio_sm_set_enabled(pio, sm, true);  // Run.
    sleep_us(20);  // long enough for unhalting & two instructions.
    pio_sm_set_enabled(pio, sm, false);  // Stop.
  }

  // The big table PairsOfWords contains pairs of words (representing two
  // opcodes each) premade ready for shoving into the FIFO of sm with
  // pio_sm_put.
  const unsigned int* start2 = PairsOfWords;
  const unsigned int* limit2 =
      (const unsigned int*)((char*)PairsOfWords + sizeof PairsOfWords);

  for (const unsigned int* p = start2; p < limit2; p += 2) {
    pio_sm_put(pio, sm, p[0]);
    pio_sm_put(pio, sm, p[1]);

    pio_sm_exec(pio, sm,
                offsetSpoonProgram);    // Jump to start of spoon_pio_program
    pio_sm_set_enabled(pio, sm, true);  // Run.
    sleep_us(20);  // long enough for unhalting & two instructions.
    pio_sm_set_enabled(pio, sm, false);  // Stop.
  }

  // void pio_sm_set_pins_with_mask (PIO pio, uint sm, uint32_t pin_values,
  // uint32_t pin_mask) Raise HALT and SPOON to 1.
  pio_sm_set_pins_with_mask(pio, sm, /*values*/ 0xFu << 16,
                            /*mask*/ 0xFu << 16);

}  // SpoonFeed

void InitializePins(PIO pio) {
  constexpr bool IN = false;  // pin directions
  constexpr bool OUT = true;
  uint sm = 0;
  pio_sm_restart(pio, sm);

  // Always inputs: GPIO can own them.
  InPin(G_RESET);  // reset
  InPin(G_ECLK);   // E clock
  InPin(G_WC);     // Write Control
  InPin(G_RC);     // Read Control == Status
  InPin(G_WD);     // Write Data
  InPin(G_RD);     // Read Data

  // GPIO Outputs.  Not used by PIO.
  OutPin(G_LED, 1);
  OutPin(G_CART, 1);
  OutPin(G_NMI, 1);

  // Owned by PIO: Data Bus.
  for (uint pin = 8; pin < 16; pin++) {
    pio_gpio_init(pio, pin);
  }
  pio_sm_set_consecutive_pindirs(pio, sm, /*begin*/ 8, /*size*/ 8, IN);

  // Owned by PIO: Side Sets (Spoon, Dir, Halt, Slenb)
  for (uint pin = 16; pin < 20; pin++) {
    pio_gpio_init(pio, pin);
  }

  pio_sm_set_consecutive_pindirs(pio, sm, /*begin*/ 16, /*size*/ 4, OUT);

  // Quickly HALT before Coco goes too crazy.
  // void pio_sm_set_pins_with_mask (PIO pio, uint sm, uint32_t pin_values,
  // uint32_t pin_mask)
  pio_sm_set_pins_with_mask(pio, sm, 0x5u << 16, 0xFu << 16);
}

int main() {
  const PIO pio = pio0;  // Only use pio0.

  InitializePins(pio);

  stdio_usb_init();
  printf("*** HELLO BONOBO\n");
  for (uint i = 0; i < 256; i++) dma_buffer[i] = (byte)(i ^ 8);

  // InitialBlinks(); // 3 second delay; 3 blinks.

  int channel = dma_claim_unused_channel(/*required*/ true);
  dma_channel_config config = dma_channel_get_default_config(channel);

  while (true) {
    SpoonFeed(pio);

#if 1
    StartPortals(pio);
    OperatePortals(pio, channel, &config);
    StopPortals(pio);
#endif
  }
}  // main

// Hints:
// pio_sm_set_pins_with_mask
// pio_sm_set_pins_with_mask
// pio_sm_set_consecutive_pindirs
// gpio_set_dir_all_bits
// gpio_put_masked
// gpio_put
// gpio_init
// gpio_init_mask
// gpio_pull_up
