# Centipede 32d

TO DO -- document better.

This is pretty much a RP2350B microcontroller with direct connections
to the Radio Shack Color Computer's 40-pin expansion bus.

The RP2350B may be powered either by the USB-C or by the Coco.
The USB-C takes precedence by going through only one diode
to VBUS, whereas the Coco's +5V input goes through two diodes.

Jumper J6 must be configured in one of two ways:

*   Preferred: Connect pins 2 and 3, and connect pins 4 and 5.
    This runs both E and Q through the schmidt trigger inverter
    to clean them.   They arrive negated at GPIO 21 and 22.

*   Original:  Connect pins 1 and 2, and pins 5 and 6.
    This runs E and Q direct to GPIO 21 and 22, not negated.
    This has problems with bouncy detection.

## Maybe a BUG? VBUS powers the 74HC14.

## Bug: CTS

On a coco3, we must obey CTS, not just the address range $C000-$DFFF.

Temporarily I'm bodging it from J5 to J4, CTS (negative logic) to GPIO18.

## Clock Speed

My current PIO-less firmware works with the semi-fast poke ($FFD7)
but not with the fast poke ($FFD9).   That's with disk11.asm for
Disk Basic Cart ROM, and a floppy at $FF40/$FF4[89AB].

Testing on a 16K Coco2 (named "2C") and a Coco3 with a 6309
and 512K RAM (named "3B").
