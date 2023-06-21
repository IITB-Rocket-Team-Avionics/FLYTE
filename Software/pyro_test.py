from machine import Pin, PWM
import time

drogue = Pin(15, Pin.OUT)
main = Pin(20, Pin.OUT)
buzzer = PWM(Pin(6))
drogue.value(0)
main.value(1)

buzzer.freq(2000)
buzzer.duty_u16(30000)

for i in range(2): # Signal drogue about to start
    buzzer.duty_u16(30000)
    time.sleep_ms(500)
    buzzer.duty_u16(0)
    time.sleep_ms(500)
    
for i in range(10):
    buzzer.duty_u16(30000)
    time.sleep_ms(100)
    buzzer.duty_u16(0)
    time.sleep_ms(900)
    
drogue.value(1)
time.sleep_ms(150)
drogue.value(0)

for i in range(4): # Signal main about to start
    buzzer.duty_u16(30000)
    time.sleep_ms(500)
    buzzer.duty_u16(0)
    time.sleep_ms(500)
    
for i in range(10):
    buzzer.duty_u16(30000)
    time.sleep_ms(100)
    buzzer.duty_u16(0)
    time.sleep_ms(900)

main.value(1)
time.sleep_ms(150)
main.value(0)
