#include "loados.h"

#include <stdarg.h>

#define SLOW_CONSOLE 0
gword volatile slow_her_down;

static void AdvanceCursor() {
    ++Console.cursor;
    while (Console.cursor >= PANE_LIMIT) {
        // Scroll Pane upward
        for (gword p = PANE_BEGIN; p < PANE_LIMIT-32; p+=2) {
            gPoke2(p, gPeek2(p+32));
        }
        // Clear bottom Pane line
        for (gword p = PANE_LIMIT-32; p < PANE_LIMIT; p+=2) {
            gPoke2(p, 0x2020);
        }
        // Move cursor back into bottom Pane line.
        Console.cursor -= 32;
    }
    gPoke1(Console.cursor, 0xFF);
#if SLOW_CONSOLE
    for (gword i = 0; i < SLOW_CONSOLE; i++) {
        slow_her_down++;
    }
#endif
}

void PutRawByte(gbyte x) {
    gPoke1(Console.cursor, x);
    AdvanceCursor();
}
void PutChar(char c) {
    gPoke1(Console.cursor, 0x20);

    gbyte x = (gbyte)c; // Unsigned!
    if (x == '\n') {
            while (Console.cursor < PANE_LIMIT-1) {
                PutChar(' ');
            }
            PutChar(' ');
    } else if (x < 32) {
        // Ingore other control chars.
    } else if (x < 96) {
            PutRawByte(63&x);  // Upper case.
    } else if (x < 128) {
            PutRawByte(x-96); // Lower case.
    } else {
            PutRawByte(x);  // Semigraphics.
    }
}

void PutStr(const char* s) {
    for (; *s; s++) {
        PutChar(*s);
    }
}

#if 1
char HexAlphabet[] = "0123456789ABCDEF";

void PutHex(gword x) {
  if (x > 15u) {
    PutHex(x >> 4u);
  }
  PutChar(HexAlphabet[15u & x]);
}
#endif
gbyte DivMod10(gword x, gword* out_div) {  // returns mod
  gword div = 0;
  while (x >= 10000) x -= 10000, div += 1000;
  while (x >= 1000) x -= 1000, div += 100;
  while (x >= 100) x -= 100, div += 10;
  while (x >= 10) x -= 10, div++;
  *out_div = div;
  return (gbyte)x;
}
void PutDec(gword x) {
  gword div;
  if (x > 9u) {
    // eschew div // PutDec(x / 10u);
    DivMod10(x, &div);
    PutDec(div);
  }
  // eschew mod // PutChar('0' + (gbyte)(x % 10u));
  PutChar('0' + DivMod10(x, &div));
}
#if 1
void PutSigned(int x) {
    if (x<0) {
        x = -x;
        PutChar('-');
    }
    PutDec(x);
}
#endif
#if 1
void Printf(const char* format, ...) {
    gbyte cc_value = gIrqSaveAndDisable();

    va_list ap;
    va_start(ap, format);

    for (const char* s = format; *s; s++) {
        if (*s < ' ') {
            PutChar('\n');
        } else if (*s != '%') {
            PutChar(*s);
        } else {
            s++;
            switch (*s) {
            case 'd':
#if 0
                {
                    int x = va_arg(ap, int);
                    PutSigned(x);
                }
                break;
#endif
            case 'u':
                {
                    gword x = va_arg(ap, gword);
                    PutDec(x);
                    PutChar('.');
                }
                break;
#if 1
            case 'x':
                {
                    gword x = va_arg(ap, gword);
                    PutChar('$');
                    PutHex(x);
                }
                break;
#endif
            case 's':
                {
                    char* x = va_arg(ap, char*);
                    PutStr(x);
                }
                break;
            default:
                PutChar(*s);
            }; // end switch
       }  // end if
    }
    gIrqRestore(cc_value);
}
#endif

void Console_Init() {
#if 1
    Console.cursor = PANE_LIMIT - 32;
    PutStr(" \n\n");
    gPoke1(Console.cursor, 0xFF);
#endif

    // Draw a greenish bar across the top of the Console.
    for (gword p = CONSOLE_BEGIN; p < PANE_BEGIN; p+=2) {
        gPoke2(p, 0x9C9C);  // redish top bar
    }

#if 1
    // Fill the body of the screen with bangs.
    for (gword p = PANE_BEGIN; p < PANE_LIMIT; p+=2) {
        gPoke2(p, 0x2121);
    }
#endif

    // Draw a blueish bar across the bottom of the Console.
    for (gword p = PANE_LIMIT; p < CONSOLE_LIMIT; p+=2) {
        gPoke2(p, 0x9393);  // redish bottom bar
    }
    memcpy(CONSOLE_BEGIN+2, "LOADOS", 6);
    memcpy(CONSOLE_LIMIT-32+2, "LOADOS", 6);

    gPoke1(Console.cursor, 0xFF);
}
