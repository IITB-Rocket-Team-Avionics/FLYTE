from machine import Pin,I2C,SPI
from time import ticks_ms,sleep
from bmp280 import *
from bno055 import *
import sdcard

i2c_bus = I2C(1,sda=Pin(2),scl=Pin(3))

print(i2c_bus.scan())

imu = BNO055(i2c_bus)

imu.mode(12)

while True:
    if not imu.calibrated():
        print(imu.cal_status())
        sleep(0.05)
    else:
        #print(imu.sensor_offsets())
        print(imu.accel())
        sleep(0.05)