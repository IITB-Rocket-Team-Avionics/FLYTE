from machine import Pin,PWM,I2C
from time import sleep,sleep_ms
import ads1x15

main = Pin(20,Pin.OUT)
drogue = Pin(28,Pin.OUT)

buzzer = PWM(Pin(6))
buzzer.freq(2000)
buzzer.duty_u16(0)

led_red = Pin(19,Pin.OUT)
led_white = Pin(21,Pin.OUT)

led_white.value(0)
led_red.value(0)

main.value(0)
drogue.value(0)

# sleep(20)

led_white.value(1)
led_red.value(1)

addr = 72

i2c = I2C(1,scl=Pin(3), sda=Pin(2), freq=400000)

ads = ads1x15.ADS1115(i2c, addr)

for i in range(2,50):
    buzzer.freq(1000 + 10*i)
    buzzer.duty_u16(60000)
    led_white.value(0)
    sleep(1/i)
    buzzer.duty_u16(0)
    led_white.value(1)
    sleep(1/i)
    
buzzer.duty_u16(60000)
led_red.value(0)

sleep(1)

buzzer.duty_u16(0)
led_red.value(1)

for i in range(3):
    main.value(1)
    sleep(0.3)
    
    ads.set_conv(7, 0) # vcc
    a = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 1) # main
    b = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 2) # drogue
    c = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 3) # 3v3
    d = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    
    print(a,b,c,d)
    
    main.value(0)
    sleep(0.1)
    
sleep(1)

for i in range(2,50):
    buzzer.freq(1000 + 10*i)
    buzzer.duty_u16(60000)
    led_white.value(0)
    sleep(1/i)
    buzzer.duty_u16(0)
    led_white.value(1)
    sleep(1/i)
    
buzzer.duty_u16(60000)
led_red.value(0)

sleep(1)

buzzer.duty_u16(0)
led_red.value(1)

for i in range(3):
    drogue.value(1)
    sleep(0.3)
    
    ads.set_conv(7, 0) # vcc
    a = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 1) # main
    b = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 2) # drogue
    c = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    ads.set_conv(7, 3) # 3v3
    d = ads.raw_to_v(ads.read_rev())
    sleep_ms(10)
    
    print(a,b,c,d)
    
    drogue.value(0)
    sleep(0.1)
    