// FOR M6809 -- life for Coco Semigraphics (VDG Text mode)
//
//   Runs in the first 4K of RAM (all under $1000).
//
//  COPIED AND MODIFIED FROM
//  ~/go/src/github.com/strickyak/spoonfeeder/micropy/life-semi.c
//

typedef unsigned char byte;
typedef unsigned int word;

#define N 1024
#define W 64
#define H 16

#if unix
#include <stdio.h>
byte A[N];
byte B[N];
byte S[W * H / 4];
#else
#define A ((volatile byte*)0x0800)
#define B ((volatile byte*)0x0C00)
#define S ((volatile byte*)0x0000)
#endif

#define AA(X, Y) A[((X) & (W - 1)) + W * ((Y) & (H - 1))]
#define BB(X, Y) B[((X) & (W - 1)) + W * ((Y) & (H - 1))]

#define STOP()  \
  {             \
    while (1) { \
      Poke(500, 1+Peek(500)); \
    }           \
  }

void Qmemset(volatile byte* p, byte c, word n) {
  while (n--) *p++ = c;
}
void Qmemcpy(volatile byte* d, byte* s, word n) {
  while (n--) *d++ = *s++;
}

#define Poke(ADDR,VALUE) ((*(volatile byte*)(ADDR)) = (VALUE));
void InitCoco3() {
    // Now PIA1 controling the VDG.
    Poke(0xFF21, 0);     // choose direction reg, A port.
    Poke(0xFF20, 0xFE);  // set direction
    Poke(0xFF23, 0);     // choose direction reg, B port.
    Poke(0xFF22, 0xF8);  // set direction

    Poke(0xFF21, 0x34);  // choose data reg, A port, and CA2 outputs 0.
    Poke(0xFF23, 0x34);  // choose data reg, B port, and CB2 outputs 0.

    Poke(0xFF20, 0x02);  // set data (the "2" bit is RS232 out.
    Poke(0xFF22, 0x08);  // set data.  $08: css=1 for Orange.

    // Now init the SAM.
    // Poke all the even addresses
    for (word i = 0xFFC0; i< 0xFFE0; i+=2) {
        Poke(i, 0);
    }
#if 1
    // Odd exceptions:
    Poke(0xFFDD, 1);  // Say we have 32/64K RAM
    Poke(0xFF90, 0x80); // Coco 1/2 Compatible, no MMU, no GIME F/IRQs.
    for (word i = 0xFF91; i < 0xFFA0; i++) {
        Poke(i, 0x00); // Nothing special.
    }
    // doesnt matter? // Poke(0xFF91, 0x40); // Memory type 256K
    Poke(0xFF9D, 0xE0); // $FF9[DE] <- $E000
    // AHA
    Poke(0xff98 , 0x03 );
    Poke(0xff99 , 0x20 );
    // NOTE bit 3 of 0xff98 sets 50Hz, PAL mode.
#endif

    // I don't understand this, but this fixed my semigraphics.
    Poke(0xFF9C, 0x0F);  // "Vertical scrolling enable." "7 scan lines to scroll up."

    ///////////////////

    // Now PIA1 controling the VDG.
    Poke(0xFF21, 0);     // choose direction reg, A port.
    Poke(0xFF20, 0xFE);  // set direction
    Poke(0xFF23, 0);     // choose direction reg, B port.
    Poke(0xFF22, 0xF8);  // set direction

    Poke(0xFF21, 0x34);  // choose data reg, A port, and CA2 outputs 0.
    Poke(0xFF23, 0x34);  // choose data reg, B port, and CB2 outputs 0.

    Poke(0xFF20, 0x02);  // set data (the "2" bit is RS232 out.
    Poke(0xFF22, 0x08);  // set data.  $08: css=1 for Orange.

    // And PIA0.
    Poke(0xFF01, 0);     // choose direction reg, A port.
    Poke(0xFF00, 0x00);  // set direction, all inputs.
    Poke(0xFF03, 0);     // choose direction reg, B port.
    Poke(0xFF02, 0xFF);  // set direction, all outputs.

    Poke(0xFF01, 0x34);  // choose data reg, A port, and CA2 outputs 0.
    Poke(0xFF03, 0x34);  // choose data reg, B port, and CB2 outputs 0.
    Poke(0xFF02, 0xFF);  // null output to keyboard on Port B.
}

void render() {
  for (word y = 0; y < H; y += 2) {
    for (word x = 0; x < W; x += 2) {
      byte z = 128;
      if (BB(x + 0, y + 0)) z |= 8;
      if (BB(x + 1, y + 0)) z |= 4;
      if (BB(x + 0, y + 1)) z |= 2;
      if (BB(x + 1, y + 1)) z |= 1;
      S[(y * (W / 4)) + (x >> 1)] = z;
    }
  }
  Poke(0xFF68, 252);
}

void generation() {
  for (word x = 0; x < W; x++) {
    for (word y = 0; y < H; y++) {
      byte count = 0;
      for (byte i = 0; i < 3; i++) {
        for (byte j = 0; j < 3; j++) {
          count += AA(x + i, y + j);
        }
      }
      if (AA(x + 1, y + 1)) {
        BB(x + 1, y + 1) = (count == 3 || count == 4);
      } else {
        BB(x + 1, y + 1) = (count == 3);
      }
    }
    Poke(0xFF68, 251);
  }
}

void AddPentomino(word x) {
  AA(x + 1, 3) = 1;
  AA(x + 2, 3) = 1;
  AA(x + 0, 4) = 1;
  AA(x + 1, 4) = 1;
  AA(x + 1, 5) = 1;
}

void pre_main() {
    asm volatile("\n"
        "  .globl ENTRY                     \n"
        "  .globl _main                     \n"
        "ENTRY:                             \n"
        "  lds #$07F0  ; Fits in 4K Coco 1  \n"
        "  jmp _main                        \n"
    );
}

void main() {
    Poke(0, (byte)(word)(&pre_main));  // Pin down pre_main.

    InitCoco3();
    Poke(0xFF9C, 0x0F); // JUST THIS

    Qmemset(S, '%', 512);
    Qmemset(A, 0, N);

    //for (word i = 0; i < 16; i++) {
        //Poke(i+i, 0x80 + i);
    //}

    while (1) {
      for (word x = 3; x < W; x += 7) {
        AddPentomino(x);
        {
          Qmemcpy(B, A, N);
          render();
        }
        for (word g = 0; g < 100; g++) {
          generation();
          render();
          Qmemcpy(A, B, N);
          
          Poke(0xFF68, g);
        }
      }
    }
}
