import micropython
import gc
# micropython.alloc_emergency_exception_buf(100)

#### WE HAVE TO CHANGE CODE ACCORDING TO MAIN FLIGHT
#### ADD EXTRA STATE: MAIN OUT

#from ulab import numpy as np
import uasyncio as asyncio
import as_GPS
from machine import I2C, SPI, Pin, PWM, UART, Timer
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

    def __init__(self, deltaT_log = 25, deltaT_trans = 2000):
        self.deltaT = deltaT_log
        self.deltaT_trans = deltaT_trans
        self.sensor_init, self.comm_init = False, False
        self.t_events = [0,0,0,0,0,0,0] # Liftoff, Burnout, Apogee, Drogue, Main, Touchdown, Data shutdown
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
        self.lock = _thread.allocate_lock()
        self.run_state = 0    # 0: All threads off
                              # 1: Signal the threads to start
                              # 2: Threads are running
        self.prog = Pin(14, Pin.IN, Pin.PULL_DOWN)
        self.except_occr = False
        self.data_array = bytearray()
        self.data_fast = bytes(32)
        self.data_slow = bytes(32)
        self.speed = 0
        self.latitude = (0,0,0,'N')
        self.longitude = (0,0,0,'S')
        self.manual_pyro = False
        self.pyro_fired = False
        #self.msg_recv = []
        self.calib_time, self.calib_alt, self.calib_temp = 0,0,0
        self.shutdown = False
        self.send_calib = False
        self.calib_data = bytes(20)
        self.drogue = Pin(15, Pin.OUT)
        self.main = Pin(20, Pin.OUT)
        self.drogue.value(0)
        self.main.value(0)
#         self.tim = Timer()
            
    def cb(self, events):
        if events & SX1262.TX_DONE:
            print('TX done.')
            pass
        
        if events & SX1262.RX_DONE:
            print('RX done.')
            msg,err = self.sx.recv()
#             self.msg_recv.append(msg)
#             if (msg == b'pyro'):
#                 self.manual_pyro = True
                
    def callback_gps(self,gps, *_):  # Runs for each valid fix
        print("gps callback")
        #print(gps.latitude(), gps.longitude(), gps.altitude)
        self.latitude = gps.latitude(coord_format=as_GPS.DMS)
        self.longitude = gps.longitude(coord_format=as_GPS.DMS)
        self.speed = gps.speed(unit=as_GPS.KPH)
        
    def calib_bmp(self, n = 100):
        self.calib_time = time.ticks_ms()
        self.calib_temp = self.bmp.temperature
        avg_alt = 0
        for i in range(n):
            avg_alt += self.bmp.getAlti()
        self.calib_alt = avg_alt/n
        
#     def drogue_callback(self,t):
#         self.drogue.value(0)
#         print('drogue done')
#         self.tim.deinit()
#     
#     def main_callback(self,t):
#         self.main.value(0)
#         print('main done')
#         self.tim.deinit()
    
    def init_board(self): #Initiate BMP, MPU, SD card, and flash
        print("starting init")
        
        # i2c bus
        self.i2c_bus = I2C(1,
                scl = Pin(3),
                sda = Pin(2),
                freq = 400000)
        # spi buses
        self.spi_bus1 = SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12))
        
        self.SD_CS = Pin(13) # Chip Select for SD Card
        self.flash_CS = Pin(9)
        
        #uart bus
        self.uart = UART(0, 9600, rx = Pin(1), tx = Pin(0), timeout=1000, timeout_char=1000) # decide on timeouts
        
        try:
            self.bmp = BMP280(i2c_bus = self.i2c_bus)
            print('bmp done')
        except Exception as e:
            print('Failed to initialize BMP280')
            print(e)
            self.except_occr = True
            #Logger.log('ERROR init BMP280: ', e)

        try:
            self.mpu = MPU6050(self.i2c_bus)
            print('imu done')
            #self.mpu.accel_range(3) # Set to 16g
        except Exception as e:
            print('Failed to initialize MPU6500')
            print(e)
            self.except_occr = True
            #Logger.log('ERROR init MPU6500: ', e)

        try:
            self.sd = sdcard.SDCard(self.spi_bus1, self.SD_CS)
            uos.mount(self.sd, "/sd")
            print('sd done')
        except Exception as e: #OSError
            print('Failed to initialize SD Card Reader')
            print(e)
            self.except_occr = True
            
        try:
            self.flash = winbond.W25QFlash(self.spi_bus1, self.flash_CS)
            uos.mount(self.flash, '/win')
            print('flash done')
        except Exception as e: #OSError
            print('Failed to initialize Flash')
            print(e)
            self.except_occr = True
        
        try:
            self.sx = SX1262(spi_bus=0, clk=18, mosi=19, miso=16, cs=17, irq=22, rst=26, gpio=27)
            self.sx.begin(freq=909.4, bw=500.0, sf=8, cr=8, syncWord=0x12,
                     power=22, currentLimit=140.0, preambleLength=8,
                     implicit=False, implicitLen=0xFF,
                     crcOn=True, txIq=False, rxIq=False,
                     tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
            #self.sx.setBlockingCallback(False, self.cb)
            print('sx done')
        except Exception as e:
            print('Failed to initialize transmitter')
            print(e)
            self.except_occr = True
        
        try:
            sreader = asyncio.StreamReader(self.uart)  # Create a StreamReader
            #self.gps = as_GPS.AS_GPS(sreader, fix_cb = self.callback_gps, fix_cb_args = (self))  # Instantiate GPS
            self.gps = as_GPS.AS_GPS(sreader)
            print('gps done')
#             print(self.gps.latitude(coord_format=as_GPS.DMS))
#             print(self.gps.longitude(coord_format=as_GPS.DMS))
#             print(self.gps.speed(units=as_GPS.KPH))
        except Exception as e:
            print('Failed to initialize GPS')
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
    

        #State machine definitions-----------------------------------------------------------------------------------------
        
        max_alt = 0
        calib_gap = 300000 # Gap in milliseconds between subsequent calibrations
        calib_count = 0 # number of calibrations performed till now
        calib_max = 5 # max number of calibrations
        
        liftoff_accel = -3 # in g, minimum sustained acceleration for liftoff
        liftoff_alt = 15 # in m, minimum altitude to be cleared for liftoff
        liftoff_to_cutoff = 4500 # In milliseconds, force cutoff after this amount of time from detected liftoff
        apogee_alt_diff = 5 # in m, the minimum difference between highest recorded altitude and current altitude for apogee detection
        liftoff_to_drogue = 28000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        apogee_to_drogue = 100 # In milliseconds, drogue deployment after this amount of time from detected apogee
        drogue_lockout = 18000 # In milliseconds, lockout for drogue deployment from detected liftoff
        touchdown_alt = 15 # in m, If altitude is less than this in descent, declare touchdown
        touchdown_to_idle = 10000 # In milliseconds, declare idle after this amount of time from detected touchdown
        
        buf_len = 20 # assuming the loop runs at 40Hz
        alt_buf = [0]*buf_len
        accel_buf = [0]*buf_len
        index = 0
        inc_num = 0 # stores how many readings are greater than the previous reading chronologically, resets to 0 data isn't increasing
        zero_crossing = False # stores whether a zero crossing has occured

        
        temp_mult = 10
        accel_mult = 100000
        gyro_mult = 10
        alt_mult = 100000
        packing_str = '!bibiiihhhif'
        packing_str_slow = '!iiibiiibfh'
        
        # Initialise data file
        index_file = open('/sd/index.txt', 'r')
        idx = index_file.readline()
        new_idx = str(int(idx) + 1)
        index_file.close()

        index_file = open('/sd/index.txt', 'w')
        index_file.write(new_idx)
        index_file.close()
        
        #unmount SD card as it will probably lose contact
        uos.umount("/sd")
        
        #--------------------------------------------------------------------------------------------------------------------
        
        print('First Calibration...')
        for i in range(2): # Means initial calibration is happening
            self.buzzer.duty_u16(30000)
            time.sleep(0.2)
            self.buzzer.duty_u16(0)
            time.sleep(0.1)
        
        self.calib_bmp()
        self.calib_data = ustruct.pack('iiiif',
                                              alt_mult,
                                              accel_mult,
                                              gyro_mult,
                                              temp_mult,
                                              self.calib_temp)
        self.send_calib = True
        
         #data_file = open('/sd/data_' + new_idx +'.txt', 'w')
        data_file = open('/win/data_' + new_idx + '.bin', 'wb')
        
        runtime = 80000 # in millisecs, changed to 300s so that transmission occurs for a long time
        #self.state = 3 #####################################BRUH
        buzz_counter = 0
        self.deltaT_trans = 2000 # Start logging very slow
        self.sensor_init = True # Start lora and sd card thread
        
        while True:
            t = time.ticks_ms()
            t_log = t - self.calib_time

            # Get sensor readings
            accel = self.mpu.accel
            gyro = self.mpu.gyro
            temp = self.bmp.temperature
            alt = self.bmp.getAlti() - self.calib_alt
            
            alt_buf[index] = alt # alt_buf stores calibrated values
            accel_buf[index] = accel.y

            if alt > max_alt:
                max_alt = alt

            if self.state == 0: # to check whether altitude is MI
                if(alt_buf[index] > alt_buf[(index+buf_len-1)%buf_len]):
                    inc_num += 1
                else:
                    inc_num = 0
            elif self.state == 1:
                zero_crossing = (accel_buf[index]*accel_buf[(index+buf_len-1)%buf_len]) < 0


            index = (index+1)%buf_len

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
                                          (int)((alt)*alt_mult),
                                          self.speed) # how do we get speed here?
            
            if (self.state == 0): # If in IDLE state, do not log, just keep appending the data array
                if (len(self.data_array) < 512):  
                    self.data_array += self.data_fast
                else:
                    self.data_array = self.data_array[32:] + self.data_fast
            elif (self.state <3): # store ascent data without GPS
                # Update data array
                if (len(self.data_array) < 512):  
                    self.data_array += self.data_fast
                else:
                    data_file.write(self.data_array)
                    self.data_array = bytearray()
                    self.data_array += self.data_fast
            else: # Store descent data with GPS
                slow = self.data_slow # So that the information from the other thread is not accessed multiple times
                # Update data array
                if (len(self.data_array) < 512):  
                    self.data_array += self.data_fast + slow
                else:
                    data_file.write(self.data_array)
                    self.data_array = bytearray()
                    self.data_array += self.data_fast + slow
            # We do not need to account for TOUCHDOWN IDLE as the state machine will force the logging loop to close in that state
                
            # Calibrate and beep regularly
            if (self.state == 0):
                if (t_log > calib_gap and calib_count < calib_max):
                    self.buzzer.duty_u16(0)
                    print('Calibrating...')
                    for i in range(3): # Means subsequent calibration is happening
                        self.buzzer.duty_u16(30000)
                        time.sleep_ms(200)
                        self.buzzer.duty_u16(0)
                        time.sleep_ms(100)
                    self.calib_bmp()
                    alt_buf = [0]*buf_len
                    accel_buf = [0]*buf_len
                    self.calib_data = ustruct.pack('iiiif',
                                              alt_mult,
                                              accel_mult,
                                              gyro_mult,
                                              temp_mult,
                                              self.calib_temp)
                    self.send_calib = True
                    self.data_array = bytearray()
                    calib_count += 1
                    continue
                else:
                    if(buzz_counter == 7):
                        self.buzzer.duty_u16(0)
                    elif(buzz_counter == 79):
                        self.buzzer.duty_u16(30000)
                    else:
                        pass
                    buzz_counter = (buzz_counter + 1)%80
           #------------------------------------------------------------------------

           #State: 0,1,2,3,4,5,6,7
           #Idle, Powered Ascent, Coasting, Descent, Drogue Out, Main out, Touchdown, TD Idle
            
            if (self.state != 0 and self.state != 7):
                self.deltaT_trans = 250
            
            if (t_log> runtime or self.logging_done): # Stop data logging after run_time or if touchdown is detected
                self.data_array = bytearray()
                break

            if (self.state==0 and abs(sum(accel_buf))>abs(buf_len*liftoff_accel) and alt_buf[index]>liftoff_alt and inc_num>=buf_len): #Liftoff
                self.state = 1
                self.t_events[0] = t_log
                print("Liftoff")

            elif (self.state==1 and (zero_crossing or t_log - self.t_events[0] > liftoff_to_cutoff)): #Burnout
                self.state = 2
                self.t_events[1] = t_log
                print("Burnout")

            elif (self.state==2 and max_alt-alt_buf[index]>apogee_alt_diff and max_alt-alt_buf[((index-1)%buf_len)]>apogee_alt_diff): #Apogee
                self.state = 3
                self.t_events[2] = t_log
                print("Apogee")

            elif ((self.state==3 and t_log-self.t_events[2]>apogee_to_drogue and t_log-self.t_events[0]>drogue_lockout)
                  or (self.state==2 and t_log-self.t_events[0]>liftoff_to_drogue)): #Chute, need to add condition for signals sent by ground station
                self.state = 4
                self.t_events[3] = t_log
                print("drogue")
                for i in range(3):
                    self.drogue.value(1)
                    time.sleep_ms(150)
                    self.drogue.value(0)
                    time.sleep_ms(50)
                    
            elif (self.state == 4 and alt < 420):
                self.state = 5
                self.t_events[4] = t_log
                print("main")
                for i in range(3):
                    self.main.value(1)
                    time.sleep_ms(150)
                    self.main.value(0)
                    time.sleep_ms(50)
                 
            elif (self.state==5 and sum(alt_buf) < buf_len*touchdown_alt): #Touchdown, is g vector sum required?
                self.state = 6
                self.t_events[5] = t_log
                print("Touchdown")

            elif (self.state==6 and t_log-self.t_events[5]>touchdown_to_idle): #Touchdown idle
                self.state = 7
                self.t_events[6] = t_log
                self.logging_done = True
                self.deltaT_trans = 2000
                print("Touchdown idle")
                
            else:
                pass
            
            ## FOR TESTING. FORCES LIFTOFF AT A CERTAIN TIME. COMMENT OUT BEFORE FLIGHT
            
            if (self.state == 0 and t_log> 25000):
                self.state = 1
                self.t_events[0] = t_log
                print('force change')
                
            ############################################################################
            
            # force each iteration to take deltaT time
            time.sleep_ms(self.deltaT - (time.ticks_ms() - t) - 1)
        
        print("Logging finished. Transferring to SD card now")
        data_file.close()
        
        transfer_to_SD = True
        try: # try to re-mount the SD card as it has probably lost contact with the reader upon launch
            self.sd = sdcard.SDCard(self.spi_bus1, self.SD_CS)
            uos.mount(self.sd,"/sd")
            sd_file = open('/sd/data_' + new_idx + '.txt', 'w')
        except:
            transfer_to_SD = False
            print("Failed to mount sd card. Saving config in flash memory")
            config_file = open('/win/config_' + new_idx + '.txt','w')
            config_file.write("(state, time, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, altitude, 45 lmao)\n")
            config_file.write(f"Calibration details- Time: {self.calib_time} Alt: {self.calib_alt} Temp: {self.calib_temp}\n")
            config_file.write("Events tracked: Liftoff, Burnout, Apogee, Chute release, Touchdown, Data shutdown\n")
            config_file.write(f"Event timestamps: {self.t_events}\n")
            config_file.write(f"Maximum altitude reached: {max_alt}\n")
            config_file.close()
            
            
        if(transfer_to_SD == True):
            
            for i in range(4): # Means storing to SD card now
                self.buzzer.duty_u16(30000)
                time.sleep(0.2)
                self.buzzer.duty_u16(0)
                time.sleep(0.1)
            
            sd_file.write("(state, time, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, altitude, 45 lmao)\n")
            read_file = open('/win/data_' + new_idx + '.bin', 'rb')
            read_file.seek(0,2) # Move to end of file
            file_size = read_file.tell() # Get number of bytes
            read_file.seek(0,0) # Move to start of file
            while(file_size - read_file.tell()>0):
                
                flash_data = ustruct.unpack(packing_str, read_file.read(32))
                
                sd_file.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(flash_data[0],
                                                                       flash_data[1],
                                                                       (flash_data[2]/temp_mult) + self.calib_temp,
                                                                       flash_data[3]/accel_mult,
                                                                       flash_data[4]/accel_mult,
                                                                       flash_data[5]/accel_mult,
                                                                       flash_data[6]/gyro_mult,
                                                                       flash_data[7]/gyro_mult,
                                                                       flash_data[8]/gyro_mult,
                                                                       (flash_data[9]/alt_mult),
                                                                       flash_data[10]))
                
                if (flash_data[0] >= 3):
                    flash_data = ustruct.unpack(packing_str_slow, read_file.read(32))
                    sd_file.write("Lat: {},{},{},{} Long: {},{},{},{} Speed: {} Check: {}\n".format(flash_data[0],
                                                                                                    flash_data[1],
                                                                                                    flash_data[2],
                                                                                                    chr(flash_data[3]),
                                                                                                    flash_data[4],
                                                                                                    flash_data[5],
                                                                                                    flash_data[6],
                                                                                                    chr(flash_data[7]),
                                                                                                    flash_data[8],
                                                                                                    flash_data[9]))
                    
            sd_file.write(f"Calibration details- Time: {self.calib_time} Alt: {self.calib_alt} Temp: {self.calib_temp}\n")
            sd_file.write("Events tracked: Liftoff, Burnout, Apogee, Drogue release, Main release, Touchdown, Data shutdown\n")
            sd_file.write(f"Event timestamps: {self.t_events}\n")
            sd_file.write(f"Maximum altitude reached: {max_alt}\n")
            sd_file.close()
            
        print("End fast core")
        
        # Indicates end of logging
        while(not(self.shutdown)):
            self.buzzer.duty_u16(30000)
            time.sleep_ms(1000)
            self.buzzer.duty_u16(0)
            time.sleep_ms(1000)
        pass
    
    def slow_core_test_launch(self):
        
        self.sx.setBlockingCallback(False, self.cb)
        
        while(not(self.sensor_init)): # Wait for sensor initialisation to complete
            continue
        
        print('start slow core')
        packing_str_slow = '!iiibiiibfh'

        while(not(self.shutdown)):
            t  = time.ticks_ms()
            
            if(self.state == 7 and self.prog.value(0)): # switch off everything after finding the rocket. Should we use prog switch?
                self.shutdown = True
            
            self.latitude = self.gps.latitude(coord_format=as_GPS.DMS)
            self.longitude = self.gps.longitude(coord_format=as_GPS.DMS)
            self.speed = self.gps.speed(units = as_GPS.KPH)
            
            if(self.state >= 3):
                while not self.lock.acquire(0):
                    continue
                self.data_slow = ustruct.pack(packing_str_slow, 
                                            self.latitude[0],
                                            self.latitude[1],
                                            self.latitude[2],
                                            ord(self.latitude[3]),
                                            self.longitude[0],
                                            self.longitude[1],
                                            self.longitude[2],
                                            ord(self.longitude[3]),
                                            self.speed,
                                            69)
                self.lock.release()
                
            buf = bytes()
            
            #print('hi from slow core')
            if (self.send_calib == True): # Calibration
                while not self.lock.acquire(0):
                    continue
                buf = self.calib_data
                self.send_calib = False
                self.lock.release()
            elif(self.state <3): # Ascent 
                while not self.lock.acquire(0):
                    continue
                buf = self.data_fast
                self.lock.release()
            elif(not(self.state ==7)): # Descent
                while not self.lock.acquire(0):
                    continue
                buf = ustruct.pack('b',self.state) + self.data_slow + self.data_fast[1:]
                self.lock.release()
            else: # Touchdown IDLE
                buf = ustruct.pack('b',self.state) + self.data_slow + ustruct.pack('i',t - self.calib_time)
                
            self.sx.send(buf)
            time.sleep_ms(self.deltaT_trans - (time.ticks_ms() - t))
        pass
