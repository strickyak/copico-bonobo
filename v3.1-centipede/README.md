# copico-bonobo/v3.1-centipede

The Centipede is an experimental Coco Cartridge
that uses a RP2350B microcontroller
connected directly to the Coco Cartridge Expansion bus.

My experimental device uses this RP2350B board,
but any board that exposes nearly all 48 GPIOs
should work:

https://github.com/jvanderberg/RP2350B-Dev-Board

These are the current GPIO pin assigments:

| Coco | RP2350B GPIO |
|------|--------------|
| D0:7 | 0:7 |
| A0:15 | 32:47 |
| R/W  | 20 |
| E, Q | 21, 22 |
| /NMI | 26 |
| /RESET | 27 |
| /HALT | 28 |
| /SLENB | 29 |

