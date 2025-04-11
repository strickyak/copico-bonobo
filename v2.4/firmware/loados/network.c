#include "loados.h"

// SendPacket sends a Quint Header and a payload
// of 0 to 64 bytes.
void SendPacket(gbyte cmd, gword p, const gbyte* pay, gbyte size) {
  size = (size > 64) ? 64 : size;

  gbyte qbuf[5];
  qbuf[0] = cmd;
  gPoke2(qbuf + 1, size);
  gPoke2(qbuf + 3, p);

  gbyte cc_value = gIrqSaveAndDisable();
  NET_Send(qbuf, 5);
  NET_Send(pay, size);
  gIrqRestore(cc_value);
}

// Games can log a message with the network.
// Dont spam it too badly!
void gNetworkLog(const char* s) {
  SendPacket(CMD_LOG, 0, (const gbyte*)s, strlen(s));
}

void Loop() {
  gbyte quint[5];

  gPoke1(0xFF22, 0x02);  // 1-bit click
  gbyte e1;
  do {
    e1 = NET_RecvChunkTry(quint, 5);
  } while (e1 == NOTYET);
  if (e1) gFatal("E-Q", e1);

  gbyte cmd = quint[0];
  gword n = gPeek2(quint + 1);
  gword p = gPeek2(quint + 3);
  gPoke1(0xFF22, 0x00);  // 1-bit click

  PutHex(cmd);
  PutChar('(');
  PutHex(n);
  PutChar(',');
  PutHex(p);
  PutChar(')');
  if (cmd == NEKOT_POKE) {  // 66
    ColdPrint("POKE");
    errnum e2;
    do {
      e2 = NET_RecvChunkTry((gbyte*)p, n);
    } while (e2 == NOTYET);

    if (e2) gFatal("E-P", e2);

  } else if (cmd == NEKOT_CALL) {  // 67
    ColdPrint("CALL");
    Delay(10000);
    // ColdPrint("....");
    // Delay(50000);
    gAssert(n == 0);
    gfunc fn = (gfunc)p;
    fn();

  } else {
    gFatal("XRC", cmd);
  }
}

const char TARGET_BUILD[] = "for-16k-bonobo";

void RequestLoadOs() {
  ColdPrint("LOADOS ...");
  ColdPrint(TARGET_BUILD);

  SendPacket(CMD_HELLO_NEKOT, /*p=*/16, TARGET_BUILD, strlen(TARGET_BUILD));
  ColdPrint("REQUEST SENT");
}

void Network_Init() {
  ColdPrint("NET...");
  NET_Init();
  ColdPrint("NET OK");
}
