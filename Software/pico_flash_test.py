#import rp2
import ustruct
import time
import winbond
import uos
from machine import SPI, Pin

spi_bus1 = SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12))
flash_CS = Pin(9)
flash = winbond.W25QFlash(spi_bus1, flash_CS)
#flash.format()
uos.VfsFat.mkfs(flash)
uos.mount(flash, '/win')

temp_mult = 10
accel_mult = 10000
gyro_mult = 10
alt_mult = 10000
packing_str = '!bibiiihhhii'

state = 5
calib_temp = 30.6
temp = 34.5
t_log = 100768
accel_x = -9.844957
accel_y = 0.2681506
accel_z = 0.2489977
gyro_x = -0.07180387
gyro_y = 0.002317485
gyro_z = 0.02536964
alt = 299.4403

data_file = open('/win/test.bin', 'wb')
data_array = bytearray()
data_fast = bytes(32)

t_prev = 0
counter = 0
block = 0x01
print("start logging")
for i in range(16*10):
    counter += 1
    t_log = time.ticks_ms()
    data_fast = ustruct.pack(packing_str,
                              state,
                              t_log - t_prev,
                              (int)((temp - calib_temp)*temp_mult),
                              (int)(accel_x*accel_mult),
                              (int)(accel_y*accel_mult),
                              (int)(accel_z*accel_mult),
                              (int)(gyro_x*gyro_mult),
                              (int)(gyro_y*gyro_mult),
                              (int)(gyro_z*gyro_mult),
                              (int)(alt*alt_mult),
                              45) # how do we get speed here?
    #print(type(data_fast))
    data_array+=data_fast
    if(counter==16):
        #print(data_array)
        data_file.write(data_array)
        data_array = bytearray()
        block += 1
        counter = 0
    t_prev = t_log
    time.sleep_ms(25 - (t_log - t_prev))
    
data_file.close()

read_array = bytes(32)    
read_file = open('/win/test.bin', 'rb')
for i in range(16*10):
    read_array = read_file.read(32)
    print(ustruct.unpack(packing_str, read_array))
read_file.close()
    


    
    


