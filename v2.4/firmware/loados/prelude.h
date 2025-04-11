#ifndef _KERNEL_PRELUDE_H_
#define _KERNEL_PRELUDE_H_

#define Cons ((gbyte*)0x0400)

#define Pia0PortA 0xFF00u
#define Pia0PortB 0xFF02u
#define Pia1PortA 0xFF20u
#define Pia1PortB 0xFF22u

#define KEYBOARD_SENSE Pia0PortA
#define KEYBOARD_PROBE Pia0PortB

#define IRQVEC_COCO12 0x010Cu
#define IRQVEC_COCO3 0xFEF7u

#define OPCODE_JMP_Extended (gbyte)0x7E

typedef void (*gfunc)(void);

void* memcpy(void* dest, const void* src, size_t count);
void* memset(void* dest, int value, size_t count);
int strlen(const char* s);

void memcpy_words(gword dest, gword src, gword num_words);
void memset_words(gword p, gword value, gword num_words);

void ColdPrint(const char* s);

#endif  // _KERNEL_PRELUDE_H_
