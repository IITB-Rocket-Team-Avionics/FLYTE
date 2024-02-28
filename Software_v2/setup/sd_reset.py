from machine import SPI, Pin
import sdcard, uos
import time

sd = sdcard.SDCard(SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12)), Pin(13))
uos.mount(sd, "/sd")
print(uos.listdir("/sd"))
