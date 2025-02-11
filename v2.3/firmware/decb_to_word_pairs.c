#include <stdio.h>

#define LDD_IMMEDIATE 0xCC
#define STD_EXTENDED  0xFD
#define JMP_EXTENDED  0x7E


typedef unsigned char byte;

byte q[5];  // 5 byte quintuple

int main() {
  printf("PairsOfWords = [\n");
  while (1) {
    if (q[0]) break; // not 00, so must be FF with JMP address.

    fread((char*)q, sizeof *q, sizeof q, stdin);
    unsigned n = ((unsigned)q[1]<<8) | q[2];
    unsigned base = ((unsigned)q[3]<<8) | q[4];

    for (unsigned i = 0; i<n; i++) {
      byte a = (byte) getchar();
      unsigned b1 = a;
      unsigned b2 = 0;  // just one byte at a time, the first

      unsigned a1 = 255&((base+i)>>8);
      unsigned a2 = 255&((base+i)>>0);

      printf("    ( 0x%02x%02x%02x%02x, 0x%02x%02x%02x%02x ), ## LDD #$%04x ; STD $%04x\n",
          b2, b1, LDD_IMMEDIATE, 0xFF,
          0, a2, a1, STD_EXTENDED,
	  (b1<<8)+b2,
	  (a1<<8)+a2);
    }
  }
  printf("    ( 0x%02x%02x%02x%02x, 0x%02x%02x%02x%02x ), ## JMP $%04x\n",
      0, 0, LDD_IMMEDIATE, 0xFF,
      0, q[4], q[3], JMP_EXTENDED,
      (q[3]<<8)+q[4]);
  printf("]\n");
}
