from machine import SPI, Pin, UART
import sdcard, winbond, uos
import time
import uasyncio as asyncio
import as_GPS

spi_bus1 = SPI(1, sck= Pin(10), mosi=Pin(11), miso=Pin(12))

SD_CS = Pin(13) # Chip Select for SD Card
flash_CS = Pin(9)

sd = sdcard.SDCard(spi_bus1, SD_CS)
uos.mount(sd, "/sd")
#uos.umount('/sd')

flash = winbond.W25QFlash(spi_bus1, flash_CS)
# flash.format()
# uos.VfsFat.mkfs(flash)
uos.mount(flash, '/win')
# print('format done!')


uart = UART(0, 9600, rx = Pin(1), tx = Pin(0), timeout=1000, timeout_char=1000)
sreader = asyncio.StreamReader(uart)
gps = as_GPS.AS_GPS(sreader)

# Measure ps coordinates
async def test():
    print('waiting for GPS data')
    await gps.data_received(position=True, altitude=True)
    for _ in range(10):
        print(gps.latitude(), gps.longitude(), gps.altitude)
        await asyncio.sleep(2)

#asyncio.run(test())
