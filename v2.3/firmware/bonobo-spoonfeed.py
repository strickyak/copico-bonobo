import time, machine, rp2
from time import sleep
from machine import Pin

def OutputPin(pinNum, value):
    p = Pin(pinNum, Pin.OUT)
    p.value(value)
    return p
def InputPin(pinNum):
    return Pin(pinNum, Pin.IN)

DIRECTION_PIN = 16  # a.k.a. Direction
HALT_PIN = 17
SLENB_PIN = 18
SPOON_PIN = 19
Direction = OutputPin(DIRECTION_PIN , 1)
HaltBar = OutputPin(HALT_PIN, 1)
Slenb = OutputPin(SLENB_PIN, 0)
SpoonBar = OutputPin(SPOON_PIN, 1)
Led = OutputPin("LED", 1)

RESETN_PIN = 20
E_CLOCK_PIN = 21
ResetBar = InputPin(RESETN_PIN)
EClock = InputPin(E_CLOCK_PIN)
WriteControl = InputPin(22)
ReadControl = InputPin(26)
WriteData = InputPin(27)
ReadData = InputPin(28)

HaltBar.value(0)
for i in range(5):
    Led.value(0)
    SpoonBar.value(0)
    sleep(1)
    Led.value(1)
    SpoonBar.value(1)
    sleep(1)
HaltBar.value(1)
Slenb.value(0)
SpoonBar.value(1)

FIRST_DATA_PIN = 8
NUM_DATA_PINS = 8


########
OUT_HIGH, OUT_LOW, IN_HIGH = rp2.PIO.OUT_HIGH, rp2.PIO.OUT_LOW, rp2.PIO.IN_HIGH
@rp2.asm_pio(
    out_init=tuple(NUM_DATA_PINS * [IN_HIGH]),   # 0-7: D0-D7
    sideset_init=(OUT_HIGH, OUT_LOW, OUT_LOW, OUT_LOW),  # Dir, HaltBar, slenb, spoonBar
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # 8 bits at a time
    autopull=True,
    pull_thresh=32,
    )
def onreset_prog():
    E_CLOCK_PIN = 21 # Constants used in PIO ASM must be defined inside the program, not global
    SIDE_NORMAL = 0b1011   # Normal direction of D0 in from 6309E to RP2040.

    SIDE_DONT_DRIVE_D_BUS =   1
    SIDE_DO_DRIVE_D_BUS =     0

    SIDE_DONT_HALT =   2
    SIDE_DO_HALT =     0

    SIDE_DONT_FLOAT =   0
    SIDE_DO_FLOAT =     4

    SIDE_DONT_SPOON =   8
    SIDE_DO_SPOON =     0

    SIDE_NORMAL = SIDE_DONT_DRIVE_D_BUS + SIDE_DONT_HALT + SIDE_DONT_FLOAT + SIDE_DONT_SPOON
    SIDE_DRIVE = SIDE_DO_DRIVE_D_BUS + SIDE_DONT_HALT + SIDE_DO_FLOAT + SIDE_DO_SPOON
    SIDE_DRIVE_AND_HALT = SIDE_DO_DRIVE_D_BUS + SIDE_DO_HALT + SIDE_DO_FLOAT + SIDE_DO_SPOON
    SIDE_HALT = SIDE_DONT_DRIVE_D_BUS + SIDE_DO_HALT + SIDE_DONT_FLOAT + SIDE_DO_SPOON

    wait(0, gpio, E_CLOCK_PIN) # synchronize on a full pulse of E.
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo

    # Unhalt at the beginning of a Cycle, so we can count cycles.
    # "direction" is the direction of the bidirectional (low voltage cmos) buffer
    # between the Coco Data Bus (D0-D7) and the Pico's GPIO0-GPIO7.
    # Out means out to the coco.  In means in from the Coco.
    # The default is IN, unless we really mean to be writing to the Coco.
    set(x, 3)                   .side(SIDE_NORMAL) # 3 means loop 4x # direction=1=IN Slenb=no HaltBar=1 SpoonBar=0
    
    # Count four dead cycles (includes the one in which we unhalted).
    label("four_times")
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    jmp(x_dec, "four_times")
    
    # Output the HIGH then the LOW byte of the reset vector we want.
    out(pindirs, 8)             .side(SIDE_DRIVE) # direction=0=OUT Slenb=yes Halt=no
    out(pins, 8)                   # output HIGH byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    out(pins, 8)                   # output LOW byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN)  .side(SIDE_DRIVE_AND_HALT) # wait until E hi, Halt=yes (to halt before first instruction)
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    out(pindirs, 8)             .side(SIDE_HALT) # direction=1=IN, Slenb=no, Halt=yes

    # Get stuck here until the main routine re-inits this state machine.
    label("loop_forever")
    jmp("loop_forever")
    
@rp2.asm_pio(
    out_init=tuple(8 * [IN_HIGH]),   # 0-7: D0-D7
    sideset_init=(OUT_HIGH,
            OUT_HIGH, OUT_HIGH, OUT_HIGH,
            OUT_LOW, OUT_LOW, OUT_LOW),  # Dir, HaltBar, slenb, SpoonBar
    out_shiftdir=rp2.PIO.SHIFT_RIGHT, # 8 bits at a time
    autopull=True,
    pull_thresh=32,
    )
def ldd_immediate_std_extended_prog():
    """This PIO program is designed to unhalt, to output 6 bytes of opcodes
       in the first six cycles, then to allow 3 more cycles to pass,
       and then halt again.  That matches a sequence like this:
         LDD #$4845  (3 instruction bytes; 3 cycles)
	     STD $0400   (3 instruction bytes; 6 cycles)
    """
    ###############..............TODO...............
    E_CLOCK_PIN = 21 # Constants used in PIO ASM must be defined inside the program, not global
    SIDE_NORMAL = 0b1011   # Normal direction of D0 in from 6309E to RP2040.

    SIDE_DONT_DRIVE_D_BUS =   1
    SIDE_DO_DRIVE_D_BUS =     0

    SIDE_DONT_HALT =   2
    SIDE_DO_HALT =     0

    SIDE_DONT_SLENB =   0
    SIDE_DO_SLENB =     4

    SIDE_DONT_SPOON =   8
    SIDE_DO_SPOON =     0

    SIDE_NORMAL = SIDE_DONT_DRIVE_D_BUS + SIDE_DONT_HALT + SIDE_DONT_SLENB + SIDE_DONT_SPOON
    SIDE_DRIVE = SIDE_DO_DRIVE_D_BUS + SIDE_DONT_HALT + SIDE_DO_SLENB + SIDE_DO_SPOON
    SIDE_DRIVE_AND_HALT = SIDE_DO_DRIVE_D_BUS + SIDE_DO_HALT + SIDE_DO_SLENB + SIDE_DO_SPOON
    SIDE_HALT = SIDE_DONT_DRIVE_D_BUS + SIDE_DO_HALT + SIDE_DONT_SLENB + SIDE_DO_SPOON

    wait(0, gpio, E_CLOCK_PIN) # synchronize on a full pulse of E.
    wait(1, gpio, E_CLOCK_PIN) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo

    # Three dead cycles.  We release halt during these.
    set(x, 2)              # Loop three times, so count down with X=2.
    label("three_times")
    wait(1, gpio, E_CLOCK_PIN)  .side(0b001) # wait until E hi # trigger=0 direction=1=IN Slenb=no Halt=no # Unhalts the M6809
    wait(0, gpio, E_CLOCK_PIN) # wait until E lo
    jmp(x_dec, "three_times")

    set(x, 5)         # Loop six times, so count down with X=5.
    out(pindirs, 8)             .side(0b100) # trigger=0 direction=0=OUT Slenb=yes Halt=no
    label("six_times")
    out(pins, 8)                   # output HIGH byte of Reset Vector (8 bits from OSR)
    wait(1, gpio, E_CLOCK_PIN)  .side(0b100) # wait until E hi
    wait(0, gpio, E_CLOCK_PIN)  .side(0b100) # wait until E lo
    jmp(x_dec, "six_times")

    # Three more cycles will execute, but we don't have to wait for them.
    # The CPU will halt after those cycles, at the end of the instruction.
    out(pindirs, 8)   .side(0b011) # trigger=0 direction=1=IN Slenb=no Halt=yes

    # Get stuck here until the main routine re-inits this state machine.
    label("loop_forever")
    jmp("loop_forever")

while True:
    print("Step2: waiting for RESET.  ")
    while ResetBar.value()==1: pass  # wait for drop
    print("got RESET.  ")

    Led.value(0)
    Slenb.value(0)                  # halt while resetting
    HaltBar.value(0)                # halt while resetting
    SpoonBar.value(0)                  # halt while resetting
    time.sleep(0.50)    # was 0.1                 # debounce
    print("debounced.  ")

    while ResetBar.value()==0: pass  # wait for ResetBar to release
    print("RESET gone.  ")
    time.sleep(0.50)   # was 0.5              # debounce and wait to sync on Halt
    print("SLEPT half a second.  ")

    pio0 = rp2.PIO(0)
    pio0.add_program(onreset_prog)

    sm1 = pio0.state_machine(
        1,  # which state machine in pio0
        onreset_prog,
        freq=125_000_000,
        sideset_base=DIRECTION_PIN,
        out_base=FIRST_DATA_PIN,
    )
    # FF=outputs A027=reset_vector 00=inputs
    sm1.put(0x0027a0ff)
    # sm1.put(0x008888ff)

    Led.value(1)
    sm1.active(True)
    print("Activated onreset prog.  Deactivating.\n")
    sm1.active(False)
    pio0.remove_program(onreset_prog)
    Direction(1)
    Slenb(0)
    HaltBar(1)
    SpoonBar(1)
    print("=========OKAY=============");

pass
