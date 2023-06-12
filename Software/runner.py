from helper import Flyte
import _thread
from machine import Pin
#import micropython
#micropython.alloc_emergency_exception_buf(100) # for debugging ISR

flyte = Flyte()
    
def interrupt_handler_main(prog):
    global flyte
    for i in range(100000): # For debounce. For some reason sleep does not work
        pass
    v = prog.value()
    if (v and flyte.run_state == 0 and not flyte.except_occr):
        flyte.run_state = 1
        flyte.logging_done = False
    elif (not(v) and flyte.run_state == 2 and not flyte.except_occr):
        flyte.shutdown = True
        flyte.run_state = 0

flyte.init_board()
flyte.prog.irq(trigger = Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = interrupt_handler_main, hard = True)

while True:
    if (flyte.run_state == 1):
        flyte.run_state = 2
        print("Attempted start log")
        _thread.start_new_thread(flyte.fast_core_test_launch,())
        flyte.slow_core_test_launch()
        print("End log")
        flyte.run_state = 0
        break
    
