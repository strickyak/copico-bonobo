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
      ++(S[0]); \
    }           \
  }

void Qmemset(volatile byte* p, byte c, word n) {
  while (n--) *p++ = c;
}
void Qmemcpy(volatile byte* d, byte* s, word n) {
  while (n--) *d++ = *s++;
}

#if 0
io PutIOByte ff90 <-- 6c
io PutIOByte ff91 <-- 00
io PutIOByte ff92 <-- 00
io PutIOByte ff93 <-- 00
io PutIOByte ff94 <-- 09
io PutIOByte ff95 <-- 00
io PutIOByte ff96 <-- 00
io PutIOByte ff97 <-- 00
io PutIOByte ff98 <-- 03
io PutIOByte ff99 <-- 20
io PutIOByte ff9a <-- 00
io PutIOByte ff9b <-- 00
io PutIOByte ff9c <-- 00
io PutIOByte ff9d <-- 3c
io PutIOByte ff9e <-- 01
io PutIOByte ff9f <-- 00
io PutIOByte ff90 <-- 80
io PutIOByte ff90 <-- 0a
io PutIOByte ff90 <-- ce
io PutIOByte ff94 <-- ff
io PutIOByte ff95 <-- ff
io PutIOByte ff98 <-- 00
io PutIOByte ff99 <-- 00
io PutIOByte ff9a <-- 00
io PutIOByte ff9b <-- 00
;
io PutIOByte ff9c <-- 0f
;
io PutIOByte ff9d <-- e0
io PutIOByte ff9e <-- 00
io PutIOByte ff9f <-- 00
io PutIOByte ff90 <-- ca
io PutIOByte ff90 <-- c8
io PutIOByte ff90 <-- ca
io PutIOByte ff90 <-- ce

io PutIOByte ff90 <-- cc
io PutIOByte ff98 <-- 00
io PutIOByte ff99 <-- 00
io PutIOByte ff9a <-- 00
io PutIOByte ff9b <-- 00
io PutIOByte ff9c <-- 0f
io PutIOByte ff9d <-- e0
io PutIOByte ff9e <-- 00
io PutIOByte ff9f <-- 00

io PutIOByte ff90 <-- cc
io PutIOByte ff98 <-- 00
io PutIOByte ff99 <-- 00
io PutIOByte ff9a <-- 00
io PutIOByte ff9b <-- 00
io PutIOByte ff9c <-- 0f
io PutIOByte ff9d <-- e0
io PutIOByte ff9e <-- 00
io PutIOByte ff9f <-- 00
#endif

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

    ///////////
    // Now init the SAM.
    // Poke all the even addresses
    for (word i = 0xFFC0; i< 0xFFE0; i+=2) {
        Poke(i, 0);
    }
    // Odd exceptions:
    //// Poke(0xFFDB, 0);  // Say we have 16K RAM
    Poke(0xFFDD, 0);  // Say we have 32/64K RAM
    //// -- DONT -- Poke(0xFFC7, 0);  // move up to $0200 for Text Display
    ///////////

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

    // I don't understand this, but this fixed my semigraphics.
    Poke(0xFF9C, 0x0F);  // "Vertical scrolling enable." "7 scan lines to scroll up."


    // Now init the SAM.
    // // Poke all the even addresses
    // for (word i = 0xFFC0; i< 0xFFE0; i+=2) {
        // Poke(i, 0);
    // }
    // Odd exceptions:
    //// Poke(0xFFDB, 0);  // Say we have 16K RAM
    // Poke(0xFFDD, 0);  // Say we have 32/64K RAM
    //// -- DONT -- Poke(0xFFC7, 0);  // move up to $0200 for Text Display

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

#if 0
"=="+a02d N a02d:6f1d     {clr   -3,x             }  a=00 b=00 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #30 {{}}  ff1d:00
"=="+a02f N a02f:6f1f     {clr   -1,x             }  a=00 b=00 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #44 {{}}  ff1f:00
"=="+a031 N a031:6f1c     {clr   -4,x             }  a=00 b=00 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #58 {{}}  ff1c:00
"=="+a036 N a036:a71e     {sta   -2,x             }  a=ff b=34 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhINzvc dp=00 #74 {{}}  ff1e:ff
"=="+a038 N a038:e71d     {stb   -3,x             }  a=ff b=34 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInzvc dp=00 #84 {{}}  ff1d:34
"=="+a03a N a03a:e71f     {stb   -1,x             }  a=ff b=34 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInzvc dp=00 #94 {{}}  ff1f:34


"=="+a03c N a03c:6f01     {clr   1,x              }  a=ff b=34 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #108 {{}}  ff21:00
"=="+a03e N a03e:6f03     {clr   3,x              }  a=ff b=34 x=ff20:0000 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #122 {{}}  ff23:00
"=="+a041 N a041:a784     {sta   ,x               }  a=fe b=34 x=ff20:fe00 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhINzvc dp=00 #134 {{}}  ff20:fe
"=="+a045 N a045:a702     {sta   2,x              }  a=f8 b=34 x=ff20:fe00 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhINzvc dp=00 #148 {{}}  ff22:f8
"=="+a047 N a047:e701     {stb   1,x              }  a=f8 b=34 x=ff20:fe34 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInzvc dp=00 #158 {{}}  ff21:34
"=="+a049 N a049:e703     {stb   3,x              }  a=f8 b=34 x=ff20:fe34 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInzvc dp=00 #168 {{}}  ff23:34
"=="+a04b N a04b:6f02     {clr   2,x              }  a=f8 b=34 x=ff20:fe34 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #182 {{}}  ff22:00
"=="+a04f N a04f:e784     {stb   ,x               }  a=f8 b=02 x=ff20:0234 y=a00e:10ce u=0000:0000 s=8000:4558,8e80 cc=eFhInzvc dp=00 #194 {{}}  ff20:02
"=="+a064 N a064:e502     {bitb  2,x              }  a=f8 b=04 x=ff20:0234 y=a00e:10ce u=ffe0:0000 s=8000:4558,8e80 cc=eFhInZvc dp=00 #640 {{}}  ff22:00
"=="+a014 N a014:b7ff23   {sta   $ff23            }  a=37 b=04 x=ff20:0234 y=a00e:10ce u=ffe0:0000 s=03d7:0000,0000 cc=eFhInzvc dp=00 #692 {{}}  ff23:37
"=="+d578 N d578:d622     {ldb   $22              }  a=00 b=fd x=00ff:0000 y=0000:0000 u=7f17:0000 s=7f17:0000,0000 cc=eFhINzvc dp=ff #2259720 {{}}  ff22:fd
"=="+d57c N d57c:d722     {stb   $22              }  a=00 b=fd x=00ff:0000 y=0000:0000 u=7f17:0000 s=7f17:0000,0000 cc=eFhINzvc dp=ff #2259732 {{}}  ff22:fd
"=="+dbf6 N dbf6:d622     {ldb   $22              }  a=00 b=fd x=03e8:0000 y=0503:2020 u=7f17:0000 s=7f15:d552,0000 cc=eFHINzvc dp=ff #2727086 {{}}  ff22:fd
"=="+dbfa N dbfa:d722     {stb   $22              }  a=00 b=f5 x=03e8:0000 y=0503:2020 u=7f17:0000 s=7f15:d552,0000 cc=eFHINzvc dp=ff #2727098 {{}}  ff22:f5
#endif

void show(byte* p) {
#if unix
  static int gen;
  for (word y = 0; y < H; y++) {
    for (word x = 0; x < W; x++) {
      putchar(p[x + y * W] ? '#' : '.');
    }
    putchar('\n');
  }
  printf("---------------------------------- %d\n", ++gen);
#endif
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
  }
}

void AddPentomino(word x) {
  AA(x + 1, 3) = 1;
  AA(x + 2, 3) = 1;
  AA(x + 0, 4) = 1;
  AA(x + 1, 4) = 1;
  AA(x + 1, 5) = 1;
}

void main() {
    InitCoco3();
    Qmemset(S, '%', 512);
    Qmemset(A, 0, N);

    //-- for (word i=0; i < 512; i++) {
        //-- byte b = (i&1) ^ ((i>>5)&1);
        //-- Poke(S+i, 63 & (b ? '/' : '\\'));
    //-- }
    // Qmemset(512, '?', 256);

    for (word i = 0; i < 16; i++) {
        Poke(i+i, 0x80 + i);
    }

    // STOP();

    show(A);

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
          // STOP();
          show(B);
          Qmemcpy(A, B, N);
        }
      }
    }
}
