from machine import Pin,PWM
from time import sleep

servo = PWM(Pin(6))
servo.freq(50)

angle = 0

if angle > 260:
    angle = 260
elif angle < 0:
    angle = 0
    
pwm = int(1500 + (0/270)*(8000 - 1500))
servo.duty_u16(pwm)
sleep(2)
pwm = int(1500 + (260/270)*(8000 - 1500))
servo.duty_u16(pwm)
sleep(2)
pwm = int(1500 + (0/270)*(8000 - 1500))
servo.duty_u16(pwm)
