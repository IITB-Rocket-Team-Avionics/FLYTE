from machine import SPI, Pin
import uos
import sdcard
from flash_spi import FLASH
from winbond import W25QFlash
import time

# led_1 = Pin(18,Pin.OUT)
# led_2 = Pin(19,Pin.OUT)
# led_1.value(1)
# led_2.value(1)

cspins = [Pin(7)] # has to be a list for this library to work
sd_cs = Pin(13)
spi = SPI(1, sck= Pin(10), mosi=Pin(11), miso=Pin(12))
flash_mount_point = '/flash'

# DO NOT QUESTION THIS
flash = W25QFlash(spi, Pin(7))
try:
    uos.mount(flash, flash_mount_point)
    uos.umount(flash_mount_point)
    pass
except Exception as e:
    print(e)
    print("huh, expected")
    pass

flash = FLASH(spi, cspins, cmd5 = False) # 4byte commands, 32Mbit = 4MB = 4096 KiB
print("Instantiated Flash")

format_flash = input("Format flash?[y/n]: ")

try:
    # Format the flash and create a littleFS filesystem
    if (format_flash == "y"):
        print("formatting the flash")
        flash.erase()
        uos.VfsLfs2.mkfs(flash)
        print("format done and littleFS created")
        
    uos.mount(flash, flash_mount_point)
    print(f"flash mounted on {flash_mount_point}")
    print(uos.statvfs(flash_mount_point))
    contents = uos.listdir(flash_mount_point)
    
    if 'index.txt' not in contents:
        print("creating index file")
        with open(flash_mount_point + "/index.txt", 'w') as index_file:
            index_file.write("0")
        print("index file created")
        with open(flash_mount_point + "/info.txt", 'w') as test_file:
            for i in uos.uname():
                test_file.write(i + "\n")
            test_file.write("Last formatted on: " + "-".join(list(map(str,time.gmtime()[:6]))))
        print("info file created")
        print(f"Contents: {uos.listdir(flash_mount_point)}")
    else:
        print(f"Contents: {contents}")
        
except Exception as e:
    print(e)
    print("mounting failed lol")
    if e.errno == 19:
        pass

# Set the RTC of the rpi pico


# Attempt to transfer things to sd card
# sdcard_mount_point = "/sd"
# try:
#     sd = 