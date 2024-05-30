from machine import Pin,I2C,freq
from bmp085 import BMP180
from imu import MPU6050
from time import sleep_ms,ticks_ms,sleep
from math import sqrt
from ulab import numpy as np

freq(270000000)
    
pyro = Pin(20,Pin.OUT)
pyro.value(0)

index_file = open('/index.txt','r')
index = int(index_file.read())
index_file.close()

index_file = open('/index.txt','w')
index_file.write(str(index + 1))
index_file.close()
 
i2c_bus = I2C(0,sda=Pin(4),scl=Pin(5))

bmp = BMP180(i2c_bus)
mpu = MPU6050(i2c_bus)

buf_len = 30
alt = np.zeros(buf_len)
vel = np.zeros(buf_len)
acc = np.zeros(buf_len)

hist_len = 150
t_hist = np.zeros(hist_len)
alt_hist = np.zeros(hist_len)
temp_hist = np.zeros(hist_len)
ax_hist = np.zeros(hist_len)
ay_hist = np.zeros(hist_len)
az_hist = np.zeros(hist_len)

state = 0
t_liftoff = 0
last_t_log = ticks_ms()

liftoff_alt = 2
liftoff_accel = 0
min_apogee = 5
main_alt = 1
touchdown_alt = 0.3

t1 = ticks_ms()

for i in range(3):
    p = 100 * bmp.pressure
    calib_alt = 4947.19 * (8.9611 - pow(p,0.190255))
    print(calib_alt)
    sleep(0.05)

bmp_working = True
mpu_working = True

max_alt = 0
t_start = ticks_ms()

data = open('/data_' + str(index) + '.csv','a')

while True:
    
    pyro.value(0)
    
    # Get data
    try:
        t_log = ticks_ms() - t_start
        
        try:
            if not bmp_working:
                bmp = BMP180(i2c_bus)
            p = 100 * bmp.pressure
            T = bmp.temperature
            y = 4947.19 * (8.9611 - pow(p,0.190255)) - calib_alt
            
            bmp_working = True
        except Exception as e:
            bmp1_working = False
            print(e)
            data.write(str(ticks_ms() - t_start) + ',' + 'BMP not working, ' + str(e) + '\n')
    
        try:
            if not mpu_working:
                mpu = MPU6050(i2c_bus)
            
            ax = mpu.accel.x
            ay = mpu.accel.y
            az = mpu.accel.z
            
            a = sqrt(ax**2 + ay**2 + az**2)
            
            mpu_working = True
        except Exception as e:
            bmp1_working = False
            print(e)
            data.write(str(ticks_ms() - t_start) + ',' + 'MPU not working, ' + str(e) + '\n')
        
    except Exception as e:
        print(e)
        data.write(str(ticks_ms() - t_start) + ',' + str(e) + '\n')
    
    # STATE MACHINE
    alt = np.roll(alt,-1)
    vel = np.roll(vel,-1)
    acc = np.roll(acc,-1)
    
    # Basic error detection between 2 BMPs for altitude
    alt[-1] = y
    acc[-1] = a
    
    max_alt = max(max_alt,alt[-1])
    
#     if ticks_ms() - t1 > 10 * 1000:
#         print(y)
#         t1 = ticks_ms()
    
    # Velocity
    if t_log != last_t_log:
        vel[-1] = (alt[-1] - alt[-2])/(t_log - last_t_log) * 1000

    # State Machine
    if state == 0:
        if np.all(alt > liftoff_alt) and np.all(acc > liftoff_accel):
            state = 1
            t_liftoff = t_log
            print('liftoff')
        else:
            t_hist = np.roll(t_hist,-1)
            alt_hist = np.roll(alt_hist,-1)
            temp_hist = np.roll(temp_hist,-1)
            ax_hist = np.roll(ax_hist,-1)
            ay_hist = np.roll(ay_hist,-1)
            az_hist = np.roll(az_hist,-1)
            
            t_hist[-1] = t_log
            alt_hist[-1] = y
            temp_hist[-1] = T
            ax_hist[-1] = ax
            ay_hist[-1] = ay
            az_hist[-1] = az
            
    elif state == 1 and np.mean(alt) < main_alt and max_alt > min_apogee:
        state = 2
        print('main')
        for i in range(3):
            pyro.value(1)
            sleep(0.1)
            pyro.value(0)
            sleep(0.3)
            
    elif state == 0 and np.mean(alt) < touchdown_alt and abs(np.mean(vel)) < 0.1:
        state = 3
        print('touchdown')
        break

    if state == 1 or state == 2:
        data.write(str(t_log) + ',' + str(state) + ',' + str(y) + ',' + str(T) + ',' + str(ax) + ',' + str(ay) + ',' + str(az) + '\n')
    else:
        pass
    
    last_t_log = t_log
    
    print(y)
    
    sleep_ms(13)

for i in range(len(alt_hist)):
    data.write(str(t_hist[i]) + ',' + str(0) + ',' + str(alt_hist[i]) + ',' + str(temp_hist[i]) + ',' + str(ax_hist[i]) + ',' + str(ay_hist[i]) + ',' + str(az_hist[i]) + '\n')

data.close()

