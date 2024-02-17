import machine
import utime
import sdcard
import uos
from machine import SPI,Pin,I2C
from time import sleep,ticks_ms
from bmp280 import *
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
hx710b = HX710B(pd_sck=17, dout=16)  # Change GPIO pin numbers accordingly

offset = 0
n = 10

# sd = sdcard.SDCard(SPI(1,
#                     sck= Pin(10),
#                     mosi=Pin(11),
#                     miso=Pin(12)), Pin(13))
# uos.mount(sd, "/sd")

runtime = 1000 * 1000

t = ticks_ms()

bmp = BMP280(I2C(1,sda=Pin(2),scl=Pin(3)))

with open('/air_data_1.txt','w') as file:

    for i in range(n):
        hx710b.read_raw_data()

    for i in range(n):
        offset += hx710b.read_raw_data()/n

    while True:
        reading = hx710b.read_raw_data()
        density = bmp.pressure/(287.08*(bmp.temperature+273.15))
        print(reading-offset)
        file.write(str(ticks_ms() - t) + ',' + str(reading) + ',' + str(density) + '\n')
        
        
#         k = 0.055
#         if reading >= offset:
#             print(sqrt(2*k*(reading - offset)/density))
        
file.close()
