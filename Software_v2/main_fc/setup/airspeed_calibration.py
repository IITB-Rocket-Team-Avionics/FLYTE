import machine
import utime
import uos
from machine import SPI,Pin,I2C
from time import sleep,ticks_ms
from math import sqrt

class HX710B:
    def __init__(self, pd_sck, dout):
        self.pd_sck = machine.Pin(pd_sck, machine.Pin.OUT)
        self.dout = machine.Pin(dout, machine.Pin.IN)

    def read_raw_data(self):
        while self.dout.value() == 1:
            pass
        data = 0
        for i in range(24):
            self.pd_sck.value(1)
            data = (data << 1) | self.dout.value()
            self.pd_sck.value(0)

        self.pd_sck.value(1)
        self.pd_sck.value(0)
            
        if data & 0x800000:
            # negative number, take 2's complement
            data -= 1 << 24
        return data


# Connect HX710B to Raspberry Pi Pico
hx710b = HX710B(pd_sck=14, dout=15)  # Change GPIO pin numbers accordingly

offset = 0
n = 10

t = ticks_ms()

print('Calculating Offset... Please Wait')

for i in range(n):
    hx710b.read_raw_data()

for i in range(n):
    offset += hx710b.read_raw_data()/n


while True:
    reading = hx710b.read_raw_data()
    print((reading-offset))

