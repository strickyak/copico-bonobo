* - lwasm --format=decb test95_ff69.asm -otest95_ff69.decb --list=test95_ff69.list
* cp test95_ff69.decb ~/coco-shelf/build-frobio/pizga/Internal/LEMMINGS/test95.lem

    ORG $0000
    LDX #$ff69
loop:
    LDD #$5678
    STD ,X
    LDD ,X
    BRA loop
