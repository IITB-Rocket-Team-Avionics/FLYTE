from machine import SPI, Pin
import sdcard, uos
import time

spi_bus = SPI(0, sck= Pin(2), mosi=Pin(3), miso=Pin(4))

SD_CS = Pin(5) # Chip Select for SD Card

sd = sdcard.SDCard(spi_bus, SD_CS)
uos.mount(sd, "/sd")

file = open('/sd/index.txt','w')
file.write('0' + '\n')
file.close()