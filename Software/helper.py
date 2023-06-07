# import micropython
import gc
# micropython.alloc_emergency_exception_buf(100)

from ulab import numpy as np
import uasyncio as asyncio
import as_GPS
from machine import I2C, SPI, Pin, PWM, UART
from bmp280 import BMP280
from imu import MPU6050
import winbond
import sdcard
import uos
import time
import _thread
import ustruct
gc.collect() # THIS IS A HACK. NEED TO CHANGE
from sx1262 import SX1262
gc.collect()

# BUZZER guide:
# EVENT         | NO. OF BEEPS
# INIT DONE     | 1
# INIT FAILED   | 10
# INIT CALIB    | 2
# MORE CALIB    | 3
# Note that the buzzer is only controlled by the log data thread. We kinda won't know if the state machine fucked up

class Flyte:

    def __init__(self, deltaT_log = 0.05, deltaT_trans = 0.1):
        self.deltaT = deltaT_log
        self.deltaT_trans = deltaT_trans
        self.init_done = False
        self.data = [0,0,0,0,0,0,0,0,0,0,0]
        self.kalman_data = [0,0,0]
        self.t_events = [0,0,0,0,0] # Liftoff, Burnout, Apogee, Chute, Touchdown
        self.logging_done = False
        self.state = 0
        self.i2c_bus, self.spi_bus1, self.uart = None, None, None
        self.SD_CS, self.flash_CS = None, None
        self.bmp, self.mpu, self.sd, self.sx, self.gps, self.flash = None, None, None, None, None, None
        self.led = Pin(25, Pin.OUT)
        self.led.value(0)
        self.buzzer = PWM(Pin(6))
        self.buzzer.freq(2000)
        self.buzzer.duty_u16(0)
        self.x = None
        self.lock = _thread.allocate_lock()
        self.run_state = 0    # 0: All threads off
                              # 1: Signal the threads to start
                              # 2: Threads are running
        self.prog = Pin(14, Pin.IN, Pin.PULL_DOWN)
        self.except_occr = False
        self.data_array = bytes()
        self.data_fast = bytes()
        self.data_slow = bytes()
        self.manual_pyro = False
        self.pyro_fired = False
        self.msg_recv = []
        self.t_msg = []
        self.sx_busy = False
        self.calib_time, self.calib_alt, self.calib_temp = 0,0,0
            
    def cb(self, events):
        self.sx_busy = True
        if events & SX1262.TX_DONE:
            print('TX done.')
        
        if events & SX1262.RX_DONE:
            print('RX done.')
            msg,err = self.sx.recv()
            self.msg_recv.append(msg)
            self.t_msg.append(self.data[0])
            if (msg == b'pyro'):
                self.manual_pyro = True
        self.sx_busy = False
        
    def callback_gps(gps, *_):  # Runs for each valid fix
        print(gps.latitude(), gps.longitude(), gps.altitude)
        
    def calib_bmp(self, n = 100):
        self.calib_time = time.ticks_ms()
        self.calib_temp = self.bmp.temperature
        avg_alt = 0
        for i in range(n):
            avg_alt += self.bmp.getAlti()
        self.calib_alt = avg_alt/n
    
    def init_board(self): #Initiate BMP, MPU, SX1262, SD card, GPS, and flash
        print("starting init")
        
        # i2c bus
        self.i2c_bus = I2C(1,
                scl = Pin(3),
                sda = Pin(2),
                freq = 400000)

        # spi buses
#         self.spi_bus0 = SPI(0,
#                     sck= Pin(18),
#                     mosi=Pin(19),
#                     miso=Pin(16))
        
        self.spi_bus1 = SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12))
        
        self.SD_CS = Pin(13) # Chip Select for SD Card
        self.flash_CS = Pin(9)
        
        #uart bus
        self.uart = UART(0, 9600, rx = Pin(1), tx = Pin(0), timeout=5000, timeout_char=5000)
        

        try:
            self.bmp = BMP280(i2c_bus = self.i2c_bus)
        except Exception as e:
            print('Failed to initialize BMP280')
            print(e)
            self.except_occr = True
            #Logger.log('ERROR init BMP280: ', e)

        try:
            self.mpu = MPU6050(self.i2c_bus)
        except Exception as e:
            print('Failed to initialize MPU6500')
            print(e)
            self.except_occr = True
            #Logger.log('ERROR init MPU6500: ', e)

        try:
            self.sd = sdcard.SDCard(self.spi_bus1, self.SD_CS)
            vfs = uos.VfsFat(self.sd)
            uos.mount(vfs, "/sd")
        except Exception as e: #OSError
            print('Failed to initialize SD Card Reader')
            print(e)
            self.except_occr = True
            
        try:
            self.flash = winbond.W25QFlash(self.spi_bus1, self.flash_CS)
            uos.mount(self.flash, '/win')
        except Exception as e: #OSError
            print('Failed to initialize Flash')
            print(e)
            self.except_occr = True
            
        try:
            sreader = asyncio.StreamReader(self.uart)  # Create a StreamReader
            self.gps = as_GPS.AS_GPS(sreader)  # Instantiate GPS
        except Exception as e:
            print('Failed to initialize GPS')
            print(e)
            self.except_occr = True
            
        try:
            self.sx = SX1262(spi_bus=0, clk=18, mosi=19, miso=16, cs=17, irq=22, rst=26, gpio=27)
            self.sx.begin(freq=909.4, bw=500.0, sf=8, cr=5, syncWord=0x12,
                     power=22, currentLimit=140.0, preambleLength=8,
                     implicit=False, implicitLen=0xFF,
                     crcOn=True, txIq=False, rxIq=False,
                     tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
            #self.sx.setBlockingCallback(False, self.cb)
        except Exception as e:
            print('Failed to initialize transmitter')
            print(e)
            self.except_occr = True
        
        if (self.except_occr):
            for i in range(10): # Initialization failed
                self.buzzer.duty_u16(30000)
                time.sleep(0.3)
                self.buzzer.duty_u16(0)
                time.sleep(0.2)
        else:    
            print('Init done')
            self.buzzer.duty_u16(30000) #Bzz start
            time.sleep(0.5)
            self.buzzer.duty_u16(0) #Bzz off
            time.sleep(0.5)
            
    def fast_core_test_launch(self): # Has data acquisition, kalman filter, and state machine
    
        # Kalman filter definitions-----------------------------------------------------------------------------------------
        self.x = np.array([0,0,0]) # Estimate matrix. Init to zero for testing
        P = np.array([[1.2526,0.7362,0], [0.7362,0.8872,0.0001], [0,0.0001,1.0012]]) # Covariance matrix. No idea how to init

        # Control inputs. Currently just thrust from thrust curve
        #control_input = False
        #B = np.array([[0.5*deltaT*deltaT], [deltaT], [1]])

        # The measurement matrix to change the state space from model to sensor frame
        H = np.array([[1, 0, 0], [0, 0, 1]])

        # Process noise matrix, which remains constant
        Q = np.array([[0.1, 0.1, 0.1], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]])

        # Sensor noise matrix (From BMP and MPU)
        bmp_variance = 0.004096 # from datasheet
        mpu_variance = 0.000676 # fro mzero test
        R = np.array([[bmp_variance, 0], [0, mpu_variance]])
        
        #--------------------------------------------------------------------------------------------------------------------
        
        #State machine definitions-----------------------------------------------------------------------------------------
        
        drogue = Pin(15, Pin.OUT)
        main = Pin(20, Pin.OUT)
        drogue.value(0)
        main.value(0)
        
        max_alt = 0
        calib_gap = 60000 # Gap in milliseconds between subsequent calibrations
        
        liftoff_accel = -30 # in m/s^2, minimum sustained acceleration for liftoff
        liftoff_alt = 15 # in m, minimum altitude to be cleared for liftoff
        liftoff_to_cutoff = 2500 # In milliseconds, force cutoff after this amount of time from detected liftoff
        apogee_alt_diff = 5 # in m, the minimum difference between highest recorded altitude and current altitude for apogee detection
        liftoff_to_drogue = 10000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        apogee_to_drogue = 100 # In milliseconds, drogue deployment after this amount of time from detected apogee
        drogue_lockout = 8000 # In milliseconds, lockout for drogue deployment from detected liftoff
        touchdown_alt = 15 # in m, If altitude is less than this in descent, declare touchdown
        touchdown_to_idle = 10000 # In milliseconds, declare idle after this amount of time from detected touchdown
        
        buf_len = 20
        alt_buf = [0]*buf_len
        accel_buf = [0]*buf_len
        index = 0
        
        temp_mult = 10
        accel_mult = 10000
        gyro_mult = 10
        alt_mult = 10000
        packing_str = '!bibiiihhhii'
        
        #--------------------------------------------------------------------------------------------------------------------
        
        print('First Calibration...')
        for i in range(2): # Means initial calibration is happening
            self.buzzer.duty_u16(30000)
            time.sleep(0.2)
            self.buzzer.duty_u16(0)
            time.sleep(0.1)
        
        self.calib_bmp()
        
        #runtime = 300000 # in millisecs, changed to 300s so that transmission occurs for a long time
        self.init_done = True # Start lora and sd card thread
        t_prev = 0
        
        while True:
            t = time.ticks_ms()
            t_log = t - self.calib_time
            t_delta = t_log - t_prev
            
            # Get sensor readings
            accel = self.mpu.accel
            gyro = self.mpu.gyro
            temp = self.bmp.temperature
            pres = self.bmp.pressure
            alt = self.bmp.getAlti()
            
            alt_buf[index] = alt
            accel_buf[index] = accel.y
            index = (index+1)%(buf_len)
            #self.data = [t_log, self.state, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp, pres, alt]
            
            self.data_fast = ustruct.pack(packing_str,
                                          self.state,
                                          t_log,
                                          (int)((temp - self.calib_temp)*temp_mult),
                                          (int)(accel.x*accel_mult),
                                          (int)(accel.y*accel_mult),
                                          (int)(accel.z*accel_mult),
                                          (int)(gyro.x*gyro_mult),
                                          (int)(gyro.y*gyro_mult),
                                          (int)(gyro.z*gyro_mult),
                                          (int)(alt*alt_mult),
                                          45) # how do we get speed here?
            # Update data array
            if (len(self.data_array) < 512):  
                self.data_array += self.data_fast
            else:
                # self.data_array = bytes()
                while not(len(self.data_array) < 512):
                    continue
                self.data_array += self.data_fast
                
            # Calibrate regularly
            if (self.state == 0):
                if (t - self.calib_time > 60000):
                    print('Calibrating...')
                    for i in range(3): # Means subsequent calibration is happening
                        self.buzzer.duty_u16(30000)
                        time.sleep(0.2)
                        self.buzzer.duty_u16(0)
                        time.sleep(0.1)
                    self.calib_bmp()
                    alt_buf = [0]*buf_len
                    accel_buf = [0]*buf_len
                    self.data_array = []
                continue
            
            # Predict, updates x and P
            A = np.array([[1, t_delta, 0.5*t_delta*t_delta], [0, 1, t_delta], [0, 0, 1]]) # Primary transform matrix
            self.x = np.dot(A,self.x)    # np.dot(A,x) + np.dot(B,u) for the control matrix, when we have the thrust curve
            P = np.dot(np.dot(A,P), A.T)
            accel_m = (-1*accel.y - 9.8) #acceleration at rest is now 0 instead of g. Acceleration x
            alt_m = alt
            mu = np.array([alt_m, accel_m])
                    
            # Writing the Kalman Gain, step by step
            K1 = np.dot(np.dot(H, P), H.T)
            K2 = np.linalg.inv(K1+R)
            Kal = np.dot(P, np.dot(H.T, K2)) # The final Kalman Gain
                    
            # Merging the Prediction Model with the Sensor Data
            self.x = self.x + np.dot(Kal, (mu - np.dot(H,self.x)))
            P = P - np.dot(Kal, np.dot(H, P))
            
#             if(self.state==0 or self.state==1):
#                 detection_array[counter] = self.x[2] # Ay
#             elif(self.state==2 or self.state==4):
#                 detection_array[counter] = self.x[1] # Vy
#             
#             counter = (counter+1)%n
            
            # Idle is state 0
            
#             if (self.state==0 and sum(detection_array) > n*1.5*9.8): #Liftoff is state 1
#                 self.state = 1
#                 self.t_events[0] = t_log
#                 print("Liftoff")
#                 
#             elif (self.state==1 and sum(detection_array) < n*1.0): #Burnout is state 2
#                 self.state = 2
#                 self.t_events[1] = t_log
#                 print("Burnout")
#                                                 # This | condition might cause an issue since its the same for burnout detection
#             elif (self.state==2 and sum(detection_array) < n*1.0 and x[0] > min_apogee): #Apogee is state 3
#                 self.state = 3
#                 self.t_events[2] = t_log
#                 print("Apogee")
#             
#             elif (self.state==3 and t_log - self.t_events[2] > chute_delay): #Chute is state 4
#                 self.state = 4
#                 self.t_events[3] = t_log
#                 print("Chute")
#                 #for i in range(3):
#                 #    pyro.value(1)
#                 #    time.sleep(1)
#                 #    pyro.value(0)
#                 #    time.sleep(0.1)
#             
#             elif (self.state==4 and (sum(detection_array) < n*0.5 or t_log - self.t_events[3] > chute_time)): #Touchdown is state 5
#                 self.state = 5 
#                 self.t_events[4] = t_log
#                 self.logging_done = True
#                 print("Touchdown")
#             
# #             if (self.state >= 1 and t_log - self.t_events[0] > force_pyro_liftoff): # Force ejection of parachute after 9 seconds from liftoff
# #                 self.t_events[3] = time.ticks_ms
# #                 self.state = 4
# #                 for i in range(3):
# #                     pyro.value(1)
# #                     time.sleep(1)
# #                     pyro.value(0)
# #                     time.sleep(0.1)
# #                 self.pyro_fired = True
# 
#             if ((t_log > force_pyro_power or self.manual_pyro) and not self.pyro_fired):
#                 self.state = 4
#                 self.t_events[3] = t_log
#                 self.buzzer.duty_u16(0)
#                 for i in range(3):
#                     pyro.value(1)
#                     self.buzzer.duty_u16(30000)
#                     print("MANUAL EJECTION SUCCESSFUL")
#                     time.sleep(1)
#                     pyro.value(0)
#                     self.buzzer.duty_u16(0)
#                     time.sleep(0.3)
#                 self.pyro_fired = True
#                 
#             if (t_log > runtime or self.logging_done == True): # Stop data logging after run_time or if touchdown is detected
#                 self.logging_done == True
#                 self.data_array = []
#                 break
#             
#             # force each iteration to take deltaT time
#             time.sleep(self.deltaT - (time.ticks_ms() - t)/1000)
#             t_prev = t_log

        return

flyte = Flyte()
flyte.init_board()
flyte.fast_core_test_launch()