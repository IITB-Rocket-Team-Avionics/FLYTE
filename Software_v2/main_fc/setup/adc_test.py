# Sample code for ESP8266 & ESP32, Micropython.org firmware
from machine import I2C, Pin, Timer
import ads1x15
from time import sleep_ms, ticks_ms, ticks_us
from array import array

addr = 72

i2c = I2C(1,scl=Pin(3), sda=Pin(2), freq=400000)

ads = ads1x15.ADS1115(i2c, addr)

while True:
    ads.set_conv(7, 3) # start the first conversion
    print(ads.raw_to_v(ads.read_rev()))
    ads.set_conv(7, 2) # start the first conversion
    print(ads.raw_to_v(ads.read_rev()))
