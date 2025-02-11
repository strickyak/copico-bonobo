    .area   .entry
    .globl _main
    .globl entry

entry:
    ldd #$07F0 
    tfr d,s 
    tfr d,u
    jmp _main
