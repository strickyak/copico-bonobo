#include "loados.h"

#define bCONTROL 0xFF68
#define bSTATUS 0xFF69
#define bDATA 0xFF6A

#define bOKAY 0
#define bNOTYET 1

// Low Level Hardware Interface

void bSendControl(gbyte c) {
  gbyte status;

  gPoke1(bCONTROL, c);

  for (gword i = 0; i < 55555; i++) {
    status = gPeek1(bCONTROL);

    // Good status is 'b' for bonobo.
    if (status == 'b') {
      return;
    }
  }
  gFatal("BONOBO", status);
}

void bSendData(const gbyte* p, gword n) {
  for (gword i = 0; i < n; i++) {
    gPoke1(bDATA, p[i]);
  }
}

void bReceiveData(gbyte* p, gword n) {
  for (gword i = 0; i < n; i++) {
    p[i] = gPeek1(bDATA);
  }
}

// Mid Level Operations

gword bQueryAvailableFromMcp() {
  gword x;
  bSendControl(250);
  bReceiveData((gbyte*)&x, sizeof x);
  return x;
}

void bReadFromMcp(gbyte* p, gword n) {
  gAssert(1 <= n);
  gAssert(n <= 100);
  bSendControl(n);
  bReceiveData(p, n);
}

void bWriteToMcp(const gbyte* p, gword n) {
  gAssert(1 <= n);
  gAssert(n <= 100);

  bSendControl(n + 100);
  bSendData(p, n);
  bSendControl(251);  // Push
}

// High Level Interface

errnum BonoboRecvChunkTry(gbyte* buf, gword n) {
  gbyte cc_value = gIrqSaveAndDisable();
  gAssert(1 <= n);
  gAssert(n <= 100);

  gword available = bQueryAvailableFromMcp();
  if (n > available) {
    gIrqRestore(cc_value);
    return bNOTYET;
  }
  bReadFromMcp(buf, n);
  gIrqRestore(cc_value);
  return bOKAY;
}

void BonoboSend(const gbyte* addr, gword n) {
  gbyte cc_value = gIrqSaveAndDisable();
  gAssert(1 <= n);
  gAssert(n <= 100);
  bWriteToMcp(addr, n);
  gIrqRestore(cc_value);
}

void Bonobo_Init() {
  bSendControl(252);  // Probe for Bonobo Hardware
  PutChar('B');
}
