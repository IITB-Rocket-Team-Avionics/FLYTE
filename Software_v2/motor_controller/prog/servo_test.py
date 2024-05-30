from machine import Pin,PWM
from time import sleep

servo = PWM(Pin(7))
servo.freq(50)

angle = 0

if angle > 260:
    angle = 260
elif angle < 0:
    angle = 0

pwm = int(1500 + (angle/270)*(8000 - 1500))

servo.duty_u16(pwm)
