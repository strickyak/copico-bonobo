#ifndef _FIRMWARE_LOAD_DECB_LOAD_H_
#define _FIRMWARE_LOAD_DECB_LOAD_H_

/////////////////////
//
//  Fundamental Types and Definitions.

typedef unsigned char gbool;
typedef unsigned char gbyte;
typedef unsigned char errnum;
typedef unsigned int gword;
typedef unsigned int size_t;

// We hope this union is both an efficient and an expressive word
// joiner/splitter.
typedef union gWordOrBytes {
  gword w;     // Access as a 2-Byte word, or
  gbyte b[2];  // access as two bytes (b[0] & b[1]).
  struct gHL {
    gbyte h, l;
  } hl;  // access as two bytes (hl.h & hl.l).
} gwob;

typedef void (*gfunc)(void);

#define gTRUE ((gbool)1)
#define gFALSE ((gbool)0)
#define gNULL ((void*)0)

#include <stdarg.h>  // You can write functions like `printf` with `...` args.

#define gPeek1(ADDR) (*(volatile gbyte*)(gword)(ADDR))
#define gPoke1(ADDR, VALUE) (*(volatile gbyte*)(gword)(ADDR) = (gbyte)(VALUE))

#define gPeek2(ADDR) (*(volatile gword*)(gword)(ADDR))
#define gPoke2(ADDR, VALUE) (*(volatile gword*)(gword)(ADDR) = (gword)(VALUE))

// These do a gPeek1, some bit manipulaton, and a gPoke1.
#define gPAND(ADDR, X) ((*(volatile gbyte*)(gword)(ADDR)) &= (gbyte)(X))
#define gPOR(ADDR, X) ((*(volatile gbyte*)(gword)(ADDR)) |= (gbyte)(X))
#define gPXOR(ADDR, X) ((*(volatile gbyte*)(gword)(ADDR)) ^= (gbyte)(X))

// If your ".bss" allocation of 128 bytes in Page 0 (the direct page)
// fills up, you can mark some of the global variable definitions with
// this attribute, to move those variables into a larger section.
#define gZEROED __attribute__((section(".data.more")))

#define gAssert(COND) \
  if (!(COND)) gFatal(__FILE__, __LINE__)

void gFatal(const char* s, gword value);

#define gDisableIrq() asm volatile("  orcc #$10")
#define gEnableIrq() asm volatile("  andcc #^$10")
gbyte gIrqSaveAndDisable();
void gIrqRestore(gbyte cc_value);

// ----------- bonobo -----------
gbyte BonoboRecvChunkTry(gbyte* buf, gword n);
void BonoboSend(const gbyte* addr, gword n);
void Bonobo_Init();

// ----------- network -----------
// Make noise on incoming packets.
#define NETWORK_CLICK 1

// struct quint is the 5-gbyte header of
// every packet embedded in the TCP stream
// to and from the MCP.  p varies with the
// command.  n is the number of bytes to
// immediately follow for the payload.
struct quint {
  gbyte cmd;
  gword n;
  gword p;
};

void gNetworkLog(const char* s);

void CheckReceived(void);
void WizSend(const gbyte* addr, gword size);
void xSendControlPacket(gword p, const gbyte* pay, gword size);

void HelloMCP();
void Network_Init(void);

// Cmd bytes used by Nekot.
#define CMD_HELLO_NEKOT 64
#define NEKOT_MEMCPY 65
#define NEKOT_POKE 66
#define NEKOT_CALL 67
#define NEKOT_LAUNCH 68

// Cmd bytes from coco to MCP.
#define NEKOT_KEYSCAN 69
#define NEKOT_CONTROL 70
#define NEKOT_GAMECAST 71

// Cmd bytes inherited from Lemma.
#define CMD_LOG 200
#define CMD_DATA 204
#define CMD_ECHO 217

#define NET_Send BonoboSend
#define NET_RecvChunkTry BonoboRecvChunkTry
#define NET_Init Bonobo_Init

#define NOTYET 1

// ----------------------

#define gPin(A) gPoke2(0, (A));

void RequestLoadOs();
void Loop();

#include "console.h"
#include "prelude.h"

#endif  // _FIRMWARE_LOAD_DECB_LOAD_H_
