# Standard Library
from machine import Pin, I2C,PWM,freq
import time
import _thread
import ustruct
import uasyncio as asyncio 
from time import sleep,ticks_ms
from math import pi,acos,sqrt,sin,cos
import uos
# from ulab import numpy as np

freq(270000000)

# Local
from i2c_responder import I2CResponder

RESPONDER_I2C_DEVICE_ID = 0
RESPONDER_ADDRESS = 0x41
GPIO_RESPONDER_SDA = 4
GPIO_RESPONDER_SCL = 5

led_2 = Pin(3,Pin.OUT)
led_1 = Pin(2,Pin.OUT)

servo = PWM(Pin(7))
servo.freq(50)

buzzer = PWM(Pin(10))
buzzer.freq(1000)
buzzer.duty_u16(0)

servo_min = 1500
servo_max = 8000

t_step = 0.05
g = 9.80665

m = 8.001
kd_max = 0.5*pi*pow(0.05,2)*0.7207/m
kd_min = 0.5*pi*pow(0.05,2)*0.4468/m

def main():
    i2c_responder = I2CResponder(
        RESPONDER_I2C_DEVICE_ID, sda_gpio=GPIO_RESPONDER_SDA, scl_gpio=GPIO_RESPONDER_SCL, responder_address=RESPONDER_ADDRESS
    )
    
    print("Starting")
    
    for i in range(4):
        buzzer.duty_u16(60000)
        sleep(0.05)
        buzzer.duty_u16(0)
        sleep(0.05)

    storage = uos.statvfs("/")
    free_bytes = storage[0]*storage[3]
    
    idx = open('/index.txt','r')
    index = idx.read()
    idx.close()
    new_idx = int(index) + 1
    idx_file = open('/index.txt','w')
    idx_file.write(str(new_idx))
    idx_file.close()
    
    t = ticks_ms()
    
    calib_1 = bytearray(16)
    calib_2 = bytearray(10)

    fast_1 = bytearray(16)
    fast_2 = bytearray(14)
    fast_3 = bytearray(16)
    fast_4 = bytearray(4)

    slow_1 = bytearray(16)
    slow_2 = bytearray(13)
    slow_3 = bytearray(8)

    kf_1 = bytearray(16)
    kf_2 = bytearray(16)
    kf_3 = bytearray(4)

    got_slow_data = False
    kf_2_updated = False
    
    calib_alt = 0
    calib_temp = 30
    
    g1 = g*t_step
    k1_max = kd_max*t_step
    k1_min = kd_min*t_step
    
    last_rx_time = ticks_ms()
    size = 0
    
    t_burnout = 0
    target = 1000.01
    alpha = 1
    set_target = False
    
    last_beep_time = 0
    
    while True:

        if ticks_ms() - last_rx_time > 500:
            buzzer.duty_u16(60000)
            sleep(0.75)
            buzzer.duty_u16(0)
            sleep(0.75)
        
        
        if i2c_responder.write_data_is_available():
                 
            last_rx_time = ticks_ms()
                 
            data = i2c_responder.get_write_data(max_size=20)

            T = ticks_ms()
            
            if len(data) == 17:
                if data[0] == 65:
                    calib_1 = bytearray(data[1:])
                    calib_data = ustruct.unpack('!ffff',calib_1)
                    
                    calib_alt = calib_data[0]
                    calib_temp = calib_data[2]
                elif data[0] == 67:
                    fast_1 = bytearray(data[1:])
                    fast_1_data = ustruct.unpack('!sibhff',fast_1)
                    t_log = fast_1_data[1]
                    state = fast_1_data[2]
                elif data[0] == 69:
                    fast_3 = bytearray(data[1:])
                elif data[0] == 74:
                    kf_1 = bytearray(data[1:])
                elif data[0] == 75:
                    kf_2 = bytearray(data[1:])
                    kf_2_updated = True
            elif len(data) == 16:
                if data[0] == 71:
                    slow_1 = bytearray(data[1:])
                    got_slow_data = True
                elif data[0] == 72:
                    slow_2 = bytearray(data[1:])
                    got_slow_data = True
            elif len(data) == 15:
                if data[0] == 68:
                    fast_2 = bytearray(data[1:])
            elif len(data) == 11:
                if data[0] == 66:
                    calib_2 = bytearray(data[1:])
            elif len(data) == 9:
                if data[0] == 73:
                    slow_3 = bytearray(data[1:])
                    got_slow_data = True
            elif len(data) == 5:
                if data[0] == 70:
                    fast_4 = bytearray(data[1:])
                elif data[0] == 77:
                    m = ustruct.unpack('!f',bytearray(data[1:]))[0]
                    print(m)
                    data_file = open('/data_' + index + '.bin','ab')
                    data_file.write(b'M' + bytearray(data[1:]))
                    size += 5
                    data_file.close()
                elif data[0] == 78:
                    target = ustruct.unpack('!f',bytearray(data[1:]))[0]
                    print(target)
                    data_file = open('/data_' + index + '.bin','ab')
                    data_file.write(b'N' + bytearray(data[1:]))
                    size += 5
                    data_file.close()
                elif data[0] == 76:
                    kf_3 = bytearray(data[1:])

                    fast = fast_1 + fast_2 + fast_3 + fast_4
                    kf = b'K' + kf_1 + kf_2 + kf_3
                    
                    if free_bytes - size > 1000 and state != 0:
                        data_file = open('/data_' + index + '.bin','ab')
                        data_file.write(fast)
                        data_file.write(kf)
                        try:
                            data_file.write(b'T' + ustruct.pack('!fff',y_low,y_high,alpha))
                            size += 99
                        except:
                            size += 86
                            pass
                        data_file.close()
                        
                                        
                    if got_slow_data and free_bytes - size > 1000 and state != 0:
                        slow = slow_1 + slow_2 + slow_3
                        data_file = open('/data_' + index + '.bin','ab')
                        data_file.write(slow)
                        data_file.close()
                        size += 38
                        got_slow_data = False
                        
            if kf_2_updated:
                
                kf_1_data = ustruct.unpack('!ffff',kf_1)
                kf_2_data = ustruct.unpack('!ffff',kf_2)
                
                if kf_2_data[2] > 0 and kf_2_data[3] != 0:
                    
                    y_high = kf_2_data[1]
                    vy_high = kf_2_data[2]
                    v_high = kf_2_data[3]
                    pitch_high = acos(kf_1_data[2])
                    
                    y_low = kf_2_data[1]
                    vy_low = kf_2_data[2]
                    v_low = kf_2_data[3]
                    pitch_low = acos(kf_1_data[2])

                    while vy_high > 0:
                        density = pow(8.9611 - (y_high + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y_high + calib_alt))
                        pitch_high += g1*sin(pitch_high)/v_high
                        v_high += -(g1*cos(pitch_high) + k1_min*density*v_high*v_high)
                        vy_high += -(g1 + k1_min*density*v_high*v_high*cos(pitch_high))
                        y_high += vy_high*t_step
                        
                    while vy_low > 0:
                        density = pow(8.9611 - (y_low + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y_low + calib_alt))
                        pitch_low += g1*sin(pitch_low)/v_low
                        v_low += -(g1*cos(pitch_low) + k1_max*density*v_low*v_low)
                        vy_low += -(g1 + k1_max*density*v_low*v_low*cos(pitch_low))
                        y_low += vy_low*t_step
                                                                        
                    kf_2_updated = False
                    
                    if state == 0 and ticks_ms() - last_beep_time > 5000:
                        buzzer.duty_u16(60000)
                        sleep(0.1)
                        buzzer.duty_u16(0)
                        last_beep_time = ticks_ms()
                    if state == 1:
                        t_burnout = t_log
                        servo.duty_u16(int(servo_min))
                    elif state == 2:
                        if t_log - t_burnout < 5000:                   
                            servo.duty_u16(int(servo_max))
                        else:
                            servo.duty_u16(int((servo_min + servo_max)/2))
                        
                    else:
                        servo.duty_u16(int(servo_min))
                          
    
    try:
        data_file.close()
    except:
        pass
            

if __name__ == "__main__":
    main()