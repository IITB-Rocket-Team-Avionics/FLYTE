from machine import Pin, PWM
from time import sleep

buzzer = PWM(Pin(6))

buzzer.duty_u16(30000)

l = 0.5

for i in range(2):

    buzzer.duty_u16(30000)

    buzzer.freq(349)
    sleep(l)
    buzzer.freq(349)
    sleep(l/2)

    sleep(l/16)

    buzzer.freq(392)
    sleep(l/2)
    buzzer.freq(392)
    sleep(l)

    sleep(l/16)

    buzzer.freq(262)
    sleep(l)

    buzzer.freq(392)
    sleep(l/2)
    buzzer.freq(392)
    sleep(l)

    sleep(l/16)

    buzzer.freq(440)
    sleep(l/2)
    buzzer.freq(440)
    sleep(l)

    sleep(l/16)

    buzzer.freq(523)
    sleep(l/4)

    sleep(l/16)

    buzzer.freq(466)
    sleep(l/4)

    sleep(l/16)

    buzzer.freq(440)
    sleep(l/2)

    sleep(l/16)

    buzzer.freq(349)
    sleep(l*3/2)

    sleep(l/16)

    buzzer.freq(392)
    sleep(l*3/2)

    sleep(l/16)

    buzzer.freq(262)
    sleep(l*5/2)

    sleep(l*17/16)
    
    buzzer.freq(262)
    sleep(l/4)
    
    sleep(l/16)
    
    buzzer.freq(262)
    sleep(l/4)
    
    sleep(l/16)
    
    buzzer.freq(294)
    sleep(l/4)
    
    sleep(l/16)
    
    buzzer.freq(330)
    sleep(l/2)
    
    sleep(l/16)
    
    buzzer.freq(349)
    sleep(l/4)
    
    sleep(l/16)
    
    buzzer.duty_u16(0)

j = 1

buzzer.freq(2000)

while j<50:
    
    buzzer.duty_u16(30000)
    sleep(2/j)
    buzzer.duty_u16(0)
    sleep(2/j)
    
    j+=1
