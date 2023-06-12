from machine import SPI, Pin
import sdcard, winbond, uos
import time

spi_bus1 = SPI(1, sck= Pin(10), mosi=Pin(11), miso=Pin(12))

SD_CS = Pin(13) # Chip Select for SD Card
flash_CS = Pin(9)

sd = sdcard.SDCard(spi_bus1, SD_CS)
uos.mount(sd, "/sd")
#uos.umount('/sd')

flash = winbond.W25QFlash(spi_bus1, flash_CS)
uos.mount(flash, '/win')

# print("unmounting sd card")
# uos.umount("/sd")
# time.sleep(20)
# print("mounting sd card")
# sd = sdcard.SDCard(spi_bus1, SD_CS)
# uos.mount(sd, "/sd")
# print(uos.listdir('/'))