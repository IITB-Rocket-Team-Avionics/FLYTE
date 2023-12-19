from helper_sac import Flyte
import _thread
from machine import Pin
import time
#import micropython
#micropython.alloc_emergency_exception_buf(100) # for debugging ISR

flyte = Flyte()
    
# def interrupt_handler_main(prog):
#     global flyte
#     for i in range(100000): # For debounce. For some reason sleep does not work
#         pass
#     v = prog.value()
#     if (v and flyte.run_state == 0 and not flyte.except_occr):
#         flyte.run_state = 1
#         flyte.logging_done = False
#     elif (not(v) and flyte.run_state == 2 and not flyte.except_occr):
#         flyte.shutdown = True
#         flyte.run_state = 0

# flyte.buzzer.duty_u16(30000)
# 
# flyte.buzzer.freq(1319)
# time.sleep(0.166)
# flyte.buzzer.freq(1175)
# time.sleep(0.166)
# flyte.buzzer.freq(740)
# time.sleep(0.334)
# flyte.buzzer.freq(831)
# time.sleep(0.334)
# flyte.buzzer.freq(1109)
# time.sleep(0.166)
# flyte.buzzer.freq(998)
# time.sleep(0.166)
# flyte.buzzer.freq(587)
# time.sleep(0.334)
# flyte.buzzer.freq(659)
# time.sleep(0.334)
# flyte.buzzer.freq(988)
# time.sleep(0.166)
# flyte.buzzer.freq(880)
# time.sleep(0.166)
# flyte.buzzer.freq(554)
# time.sleep(0.334)
# flyte.buzzer.freq(659)
# time.sleep(0.334)
# flyte.buzzer.freq(880)
# time.sleep(0.666)
# 
# flyte.buzzer.duty_u16(0)

flyte.init_board()
time.sleep(10)

print("Attempted start log")
_thread.stack_size(8*1024) # To allow large calls for second tread
_thread.start_new_thread(flyte.fast_core_test_launch,())
flyte.slow_core_test_launch()
print("End log")

#flyte.prog.irq(trigger = Pin.IRQ_RISING|Pin.IRQ_FALLING, handler = interrupt_handler_main, hard = True)

# while True:
#     if (flyte.run_state == 1):
#         flyte.run_state = 2
#         print("Attempted start log")
#         _thread.stack_size(8*1024) # To allow large calls for second tread
#         _thread.start_new_thread(flyte.fast_core_test_launch,())
#         flyte.slow_core_test_launch()
#         print("End log")
#         flyte.run_state = 0
#         break
    
# At this point press reset to fix everything
    
