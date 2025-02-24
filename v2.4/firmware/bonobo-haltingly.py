import time, machine, rp2
from time import sleep
from machine import Pin

def OutputPin(pinNum, value):
    p = Pin(pinNum, Pin.OUT)
    p.value(value)
    return p
def InputPin(pinNum):
    return Pin(pinNum, Pin.IN)

Cart = OutputPin(6, 1)
Nmi = OutputPin(7, 1)
for i in range(8, 16):
    InputPin(i)  # Data Bus

Direction = OutputPin(16, 1)
HaltBar = OutputPin(17, 1)
Slenb = OutputPin(18, 1)
SpoonBar = OutputPin(19, 1)
Led = OutputPin("LED", 1)

ResetBar = InputPin(20)
EClock = InputPin(21)
WriteControl = InputPin(22)
ReadControl = InputPin(26)
WriteData = InputPin(27)
ReadData = InputPin(28)

HaltBar.value(0)
while True:
    Led.value(0)
    SpoonBar.value(0)
    sleep(1)
    Led.value(1)
    SpoonBar.value(1)
    sleep(1)
"bonobo-haltingly.py"
