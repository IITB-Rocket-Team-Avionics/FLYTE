# Standard Library
from machine import Pin, I2C
import time
import _thread
import ustruct
import uasyncio as asyncio 

# Local
from i2c_responder import I2CResponder

I2C_FREQUENCY = 100000

RESPONDER_I2C_DEVICE_ID = 0
RESPONDER_ADDRESS = 0x41
GPIO_RESPONDER_SDA = 4
GPIO_RESPONDER_SCL = 5

READBUFFER = bytearray(16)

LED = Pin(25)

def main():
    i2c_responder = I2CResponder(
        RESPONDER_I2C_DEVICE_ID, sda_gpio=GPIO_RESPONDER_SDA, scl_gpio=GPIO_RESPONDER_SCL, responder_address=RESPONDER_ADDRESS
    )
    
    print("Starting")
    
    while True:
        if i2c_responder.write_data_is_available():
            READBUFFER = i2c_responder.get_write_data(max_size=16)
            print('   Motor Controller: Received I2C WRITE data: ' + str(READBUFFER))
            print('   Motor Controller: Formatted data: ' + str(ustruct.unpack('ffff',READBUFFER)))
            print()

if __name__ == "__main__":
    main()