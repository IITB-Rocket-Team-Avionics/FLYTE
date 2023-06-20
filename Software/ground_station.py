from time import sleep
import ustruct
import _thread
from machine import Pin
from sx1262 import SX1262

#push_button = Pin(16, Pin.IN)
#manual ejection detection. Dont use

packing_str_ascent = '!bibiiihhhif'
packing_str_touchdown = '!biiibiiibfhi'
packing_str_descent = '!biiibiiibfhibiiihhhif'

pdata = [(0,)]
data = [(10,10,10,10,10)]
a = 0
def main_func():
    def cb(events):
        if events & SX1262.RX_DONE:
            msg, err = sx.recv()
            error = SX1262.STATUS[err]
            sendPrint = True
            if (len(msg) == 20):
                # Calibration
                message = ustruct.unpack('iiiif', msg)
            elif (len(msg) == 32):
                # Ascent
                message = ustruct.unpack(packing_str_ascent,msg)
            elif (len(msg) == 64):
                # Descent
                message = ustruct.unpack(packing_str_descent, msg)
            elif (len(msg) == 37):
                # Touchdown IDLE
                message = ustruct.unpack(packing_str_touchdown, msg)
            else:
                sendPrint = False
            if(sendPrint):
                print(pdata[0] + message)

    sx = SX1262(spi_bus=1, clk=14, mosi=15, miso=12, cs=9, irq=2, rst=6, gpio=7)

    sx.begin(freq=909.4, bw=500.0, sf=8, cr=8, syncWord=0x12,
        power=22, currentLimit=140.0, preambleLength=8,
        implicit=False, implicitLen=0xFF,
        crcOn=True, txIq=False, rxIq=False,
        tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

    sx.setBlockingCallback(False, cb)

def pyro():
    for i in range(20):
        a = int(input())
        sleep(1)

#_thread.start_new_thread(pyro , ())
main_func()




