from machine import SPI, Pin
import uos
import winbond

# the used SPI and CS pin is setup specific, change accordingly
# check the boot.py file of this repo for further boards
flash = winbond.W25QFlash(spi=SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12)), cs=Pin(7))

flash_mount_point = '/win'

try:
    print("formatting...")
    flash.format()
    uos.VfsFat.mkfs(flash)
    uos.mount(flash, flash_mount_point)
#     flash.format()
#     os.VfsFat.mkfs(flash)
except Exception as e:
    if e.errno == 19:
        # [Errno 19] ENODEV aka "No such device"
        # create the filesystem, this takes some seconds (approx. 10 sec)
        print('Creating filesystem for external flash ...')
        print('This might take up to 10 seconds')
        uos.VfsFat.mkfs(flash)
    else:
        # takes some seconds/minutes (approx. 40 sec for 128MBit/16MB)
        print('Formatting external flash ...')
        print('This might take up to 60 seconds')
        # !!! only required on the very first start (will remove everything)
        flash.format()

        # create the filesystem, this takes some seconds (approx. 10 sec)
        print('Creating filesystem for external flash ...')
        print('This might take up to 10 seconds')
        # !!! only required on first setup and after formatting
        uos.VfsFat.mkfs(flash)

    print('Filesystem for external flash created')

    # finally mount the external flash
    uos.mount(flash, flash_mount_point)