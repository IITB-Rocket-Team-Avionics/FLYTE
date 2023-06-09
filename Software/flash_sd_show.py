from machine import SPI, Pin
import sdcard, winbond, os

spi_bus1 = SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12))

SD_CS = Pin(13) # Chip Select for SD Card
flash_CS = Pin(9)

sd = sdcard.SDCard(spi_bus1, SD_CS)
os.mount(sd, "/sd")

flash = winbond.W25QFlash(spi_bus1, flash_CS)
os.mount(flash, '/win')
print(os.listdir('/sd'))