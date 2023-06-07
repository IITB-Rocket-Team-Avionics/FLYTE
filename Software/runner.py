from helper import Flyte
import _thread
from machine import Pin
from time import sleep
import micropython
#micropython.alloc_emergency_exception_buf(100) # for debugging ISR

flyte = Flyte(deltaT_log = 0.025, deltaT_trans = 0.08)
    
def interrupt_handler_main(prog):
    global flyte
    for i in range(100000): # For debounce. For some reason sleep does not work
        pass
    v = prog.value()
    if (v and flyte.run_state == 0 and not flyte.except_occr):
        flyte.run_state = 1
        flyte.logging_done = False
    elif (not(v) and flyte.run_state == 2 and not flyte.except_occr):
        flyte.logging_done = True
        flyte.run_state = 0

flyte.init_board()
flyte.prog.irq(trigger = Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = interrupt_handler_main, hard = True)

while True:
    if (flyte.run_state == 1):
        flyte.run_state = 2
        print("Attempted start log")
        _thread.start_new_thread(flyte.dataAndState_Launch1,())
        flyte.sd_LoRa()
        for i in range(3): #Means logging has ended
                    flyte.buzzer.duty_u16(30000)
                    sleep(0.5)
                    flyte.buzzer.duty_u16(0)
                    sleep(0.2)
        print("End log")
        flyte.run_state = 0
    
