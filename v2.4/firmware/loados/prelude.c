#include "loados.h"

void memcpy_words(gword dest, gword src, gword num_words) {
    for (gword i = 0; i < num_words; i++) {
        gPoke2(dest, gPeek2(src));
        dest += 2;
        src += 2;
    }
}

void memset_words(gword p, gword value, gword num_words) {
    for (gword i = 0; i<num_words; i++) {
        gPoke2(p, value);
        p += 2;
    }
}

void* memset(void* dest, int value, size_t count) {
    gbyte* p = dest;
    gbyte v = (gbyte)value;
    for (gword i = 0; i < count; i++) {
        *p++ = v;
    }
    return dest;
}

void* memcpy(void* dest, const void* src, size_t count) {
    const gbyte* s = src;
    gbyte* p = dest;
    for (gword i = 0; i < count; i++) {
        *p++ = *s++;
    }
    return dest;
}

int strlen(const char* s) {
    const char* p = s;
    while (*p) ++p;
    return p-s;
}
