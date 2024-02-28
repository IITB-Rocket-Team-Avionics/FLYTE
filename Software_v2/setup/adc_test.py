# Sample code for ESP8266 & ESP32, Micropython.org firmware
from machine import I2C, Pin, Timer
import ads1x15
from time import sleep_ms, ticks_ms, ticks_us
from array import array

addr = 72
gain = 1
_BUFFERSIZE = const(512)

data = array("h", (0 for _ in range(_BUFFERSIZE)))
timestamp = array("L", (0 for _ in range(_BUFFERSIZE)))
i2c = I2C(1,scl=Pin(3), sda=Pin(2), freq=400000)
# for the Pycom branch, use:
# i2c = I2C()
ads = ads1x15.ADS1115(i2c, addr, gain)

#
# Interrupt service routine for data acquisition
# called by a timer interrupt
#
def sample(x, adc = ads.read_rev, data=data, timestamp = timestamp):
    global index_put, irq_busy
    if irq_busy:
        return
    irq_busy = True
    if index_put < _BUFFERSIZE:
        timestamp[index_put] = ticks_us()
        data[index_put] = adc()
        print(ads.raw_to_v(adc()))
        index_put += 1
    irq_busy = False

irq_busy = False

index_put = 0
ADC_RATE = 500

# set the conversion rate to 860 SPS = 1.16 ms; that leaves about
# 3 ms time for processing the data with a 5 ms timer
ads.set_conv(7, 3) # start the first conversion
ads.read_rev()
sleep_ms(ADC_RATE)
tim = Timer(-1)
tim.init(period=ADC_RATE, mode=Timer.PERIODIC, callback=sample)

while index_put < _BUFFERSIZE:
    pass

tim.deinit()

# at that point data contains the sampled values, and
# timestamp the timer ticks which correlate to the conversion time
#