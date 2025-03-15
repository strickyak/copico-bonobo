* - lwasm --format=decb test95_redrum.asm -otest95_redrum.decb --list=test95_redrum.list
* cp test95_redrum.decb ~/coco-shelf/build-frobio/pizga/Internal/LEMMINGS/test95.lem

    ORG $0000
    LDU #$ff6A
loop:
    LDA #4      ; 4 is Read DMA request
    STA -1,U    ; command!

wait:
    LDB -1,U    ; get status
    BEQ wait    ;   until non-zero

    LDY #128    ; count 128
more:
    LDD ,U      ; read data from dual port
    LEAY -1,y   ; y gets y - 1
    BNE more

    BRA loop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;    ORG $0000
;    LDX #$ff69
;    LDU #$ff6A
;loop:
;    LDA #4    ; 4 is Read DMA request
;    STA ,X    ; command!
;
;wait:
;    LDB ,X    ; get status
;    CMPB #23  ; 23 is NOT YET
;    BEQ wait
;
;    LDA #0    ; count 256
;more:
;    LDB ,U
;    DECA
;    BNE more
;
;    BRA loop
