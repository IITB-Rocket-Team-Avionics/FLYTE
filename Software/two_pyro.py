import time
from machine import Pin, PWM

buzzer = PWM(Pin(6))
buzzer.freq(2000)
buzzer.duty_u16(0)
drogue = Pin(15)
main = Pin(20)

drogue.value(0)
main.value(0)

for i in range(10):
    buzzer.duty_u16(30000)
    time.sleep(0.5)
    buzzer.duty_u16(0)
    time.sleep(0.5)
    
buzzer.duty_u16(30000)    
drogue.value(1)
time.sleep(1)
drogue.value(0)
buzzer.duty_u16(0)

for i in range(10):
    buzzer.duty_u16(30000)
    time.sleep(0.5)
    buzzer.duty_u16(0)
    time.sleep(0.5)
    
buzzer.duty_u16(30000)    
main.value(1)
time.sleep(1)
main.value(0)
buzzer.duty_u16(0)


