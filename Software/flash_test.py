import os, winbond
import time
from bmp280 import BMP280
from imu import MPU6050
from machine import Pin, SPI, I2C
import ustruct

i2c = I2C(1, scl = Pin(3), sda = Pin(2), freq = 400000)
spi = SPI(1, sck = Pin(10), mosi = Pin(11), miso = Pin(12))
flash = winbond.W25QFlash(spi, machine.Pin(9))
bmp = BMP280(i2c_bus = i2c)
mpu = MPU6050(side_str = i2c)
flash.format()         # !!! only required on the very first start (will remove everything); takes some seconds/minutes!
os.VfsFat.mkfs(flash)  # !!! only required on first setup and after formatting

# start_block = 1
# current_block = start_block
# end_block = -1
# 
# os.mount(flash, '/win')  # after every reboot of the ESP
# print("mounting done!")
# print(f"Number of blocks: {flash.count()}")
# #print(f"Number of bytes: {flash.get_size()}")
# print("starting write")
# t_prev = 0
# buff = bytes()
# for j in range(100):
#     for i in range(8):
#         t = time.ticks_ms()
#         p = bmp.pressure
#         temp = bmp.temperature
#         a = mpu.accel
#         g = mpu.gyro
#         buff += ustruct.pack('qdddddqibbbb',t,p,temp,a.x,a.y,a.z,t-t_prev,2,2,2,10,13) #10, 13 are LFCR or \n\r
#         t_prev = t
#     flash.writeblocks(current_block, buff)
#     current_block = current_block + 1
#     
#     
#         
# test_file.close()
# print('writing done')
# 
# read_file = open('/win/flash_bmp.txt','r')
# a = read_file.read(82)
# print(a)
# read_file.close()
#     
    

