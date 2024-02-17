
# ------------------------------------------ as_GPS HAS BEEN MODIFIED --------------------------------------------------

from machine import I2C,Pin,UART,SPI,freq,PWM
from math import sqrt,pi,asin,atan,tan,cos,sin,atan2,acos
import time
from time import ticks_ms,sleep,sleep_us,sleep_ms
from ulab import numpy as np
from bno055 import *
from bmp280 import *
import as_GPS
import uasyncio as asyncio
import ustruct
import winbond
import uos
import sdcard
import gc
import micropython
micropython.alloc_emergency_exception_buf(100)
gc.collect()


freq(270000000)

class async_test:
    
    def altitude(self):
        return 4947.19 * (8.9611 - pow(self.bmp.pressure,0.190255))
    
    def rho(self,y):
        return pow(8.9611 - (y)/4947.19,5.2479)/(78410.439 + 287.06*self.temp - 1.86589*(y))
    
    def delta_p(self,y1,y2):
        return pow(8.9611 - (y1 + self.calib_altitude)/4947.19,5.2479) - pow(8.9611 - (y2 + self.calib_altitude)/4947.19,5.2479)
    
    async def beep(self,*args):
             
        n = 1
        f = 2000
        t = 100

        if len(args) == 1:
            n = args[0]
        elif len(args) == 2:
            n = args[0]
            t = args[1]
        elif len(args) == 3:
            n = args[0]
            t = args[1]
            f = args[2]
        
        self.buzzer.freq(f)

        for i in range(n):
            self.buzzer.duty_u16(0)
            await asyncio.sleep_ms(t)
            self.buzzer.duty_u16(30000)
            await asyncio.sleep_ms(t)
            
    async def beep_regular(self, t=100):
        self.buzzer.freq(2000)
        while True:
            self.buzzer.duty_u16(0)
            await asyncio.sleep_ms(t)
            self.buzzer.duty_u16(30000)
            await asyncio.sleep_ms(900)
            
    def beep_blocking(self,n,t):
        self.buzzer.freq(2000)
        for i in range(n):
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
            self.buzzer.duty_u16(0)
            sleep_ms(t)
    
    # INITIALISE BOARD    
    def init(self):
        self.async_loop = None
        self.tel_delay = 1000
        self.sensor_init = False
        self.t_events = [0.,0.,0.,0.,0.]
        self.logging_done = False
        self.state = 0
        self.bmp, self.bno, self.xbee, self.gps, self.flash = None, None, None, None, None
        self.led = Pin(25, Pin.OUT)
        self.led.value(0)
        self.send_calib = False
        self.buzzer = PWM(Pin(6))
        self.drogue_pin = Pin(28)
        self.main_pin = Pin(20)
        self.beep(1)
        self.r = 6371000                  # RADIUS OF EARTH
        self.g = 9.81                     # ACCELERATION DUE TO GRAVITY         
        self.except_occr = False
        self.t_log = ticks_ms()
        self.last_t_log = ticks_ms()
        self.data_slow = bytes(64)
        self.speed = 0
        self.latitude = (0,0,0.0,'N')
        self.longitude = (0,0,0.0,'S')
        self.course = 0
        self.hdop = 0
        self.vdop = 0
        self.gps_altitude = 0
        self._fix_time = 0
        self.time_since_fix = 0
        self.gps_counter = 0
        self.last_gps_counter = 0
        self.sentence_type = b'NUL'
        
        # RAW READINGS FROM FAST SENSORS
        self.bno_accel = [0,0,0]
        self.gyro = [0,0,0]
        self.mag = [0,0,0]
        self.orientation = [0,0,0]
        self.temp = 0
        self.alt = 0
        self.ram_diff = 0               # RESTRICTED TO 10 HZ. DATASHEET EXPLAINS HOW TO INCREASE TO 40 HZ BUT I CAN'T GET IT TO WORK
        
        self.bno_working = True
        self.bmp_working = True
        
        self.calib_time, self.calib_altitude, self.calib_temp ,self.ram_offset = 0,0,0,0
        self.shutdown = False
        self.calib_data = bytes(24)
        
        self.runtime = 20 * 1000
        
        self.temp_mult = 100
        self.mag_mult = 100
        self.orientation_mult = 10
        self.packing_str = '!sibhfffffffffffhhhhhh'
        self.packing_str_slow = '!sfffifffiffff3sfff'
        
        self.max_alt = 0
        self.calib_gap = 10000 # Gap in milliseconds between subsequent calibrations
        self.calib_count = 0 # number of calibrations performed till now
        self.calib_max = 3 # max number of calibrations
        
        self.liftoff_accel = 0 # in g, minimum sustained acceleration for liftoff
        self.min_liftoff_alt = 10 # in m, minimum altitude to be cleared for liftoff
        self.force_burnout_time = 10000 # In milliseconds, force burnout after this amount of time from detected liftoff
        self.force_drogue_time = 30000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        self.lockout_drogue_time = 12000 # In milliseconds, lockout for drogue deployment from detected liftoff
        self.touchdown_alt = 100 # in m, If altitude is less than this in descent, declare touchdown
        self.touchdown_vel_limit = 0.01 # in m/s, if velocity is less than this in descent, declare touchdown
        self.main_alt = 400 # in m, deploy main parachute when descent comes under this height
        self.force_liftoff_alt = 100 # in m, force liftoff detection if height increases above this
        
        self.drogue_descent_rate = 30
        self.main_descent_rate = 8
        
        self.buf_len = 5
        self.alt_buf = np.zeros((self.buf_len))
        self.acc_buf = np.zeros((self.buf_len))
        self.vel_buf = np.zeros((self.buf_len))
        self.apogee = np.zeros((self.buf_len))
        self.density = 1.15
        self.heading_init = 0
        
        self.data_array = bytearray(512)
        self.buffer = bytearray(64)
        self.log_count = 0

    def callback_gps(self,gps, *_):  # RUNS FOR EACH VALID GPS FIX
        self.gps_counter += 1

        self.latitude = gps.latitude(coord_format=as_GPS.DMS)
        self.longitude = gps.longitude(coord_format=as_GPS.DMS)
        self.speed = 0.5144*gps.speed()                           # CONVERTING KNOTS TO M/S
        self.course = gps.course
        self.sentence_type = gps.sentence_type
        self.hdop = gps.hdop
        self.vdop = gps.vdop
        self.gps_altitude = gps.altitude - self.calib_altitude
        self._fix_time = gps._fix_time
        self.time_since_fix = gps.time_since_fix()
        
    
    # RAW DATA FROM RAM BAROMETER (MULTIPLY BY ~0.055 (FINER CALIBRATION NEEDED) TO GET DIFFERENTIAL PRESSURE)
    def ram_raw_data(self):
        data = 0
        for i in range(24):
            self.ram_sck.value(1)
            data = (data << 1) | self.ram_dout.value()
            self.ram_sck.value(0)

        self.ram_sck.value(1)
        self.ram_sck.value(0)
        
        if data & 0x800000:
            # negative number, take 2's complement
            data -= 1 << 24
        return data
    
    # CALIBRATE INITIAL ALTITUDE  
    def calib_bmp(self, n = 10):
        self.calib_temp = self.bmp.temperature
        avg_alt = 0
        for i in range(n):
            avg_alt += self.altitude()
            time.sleep_ms(10)
        self.calib_altitude = avg_alt/n
        
    # CALIBRATE RAM BAROMETER OFFSET
    def calib_ram(self,n = 10):
        self.ram_offset = 0
        count = 0
        while count < n:
            if self.ram_dout.value() == 0:
                self.ram_offset += self.ram_raw_data()/n
                count += 1
                
    
    # INITIALISE BOARD
    def init_board(self): #Initiate BMP, MPU, SD card, and flash
        print("starting init")

        # UART BUS OF GPS
        self.uart = UART(1, 9600, rx = Pin(9), tx = Pin(8))
        
        # UART BUS OF XBEE
        self.xbee = UART(0, 57600, rx = Pin(1), tx = Pin(0))
        
        # RAM BAROMETER PINS        
        self.ram_sck = Pin(17,Pin.OUT)
        self.ram_dout = Pin(16,Pin.IN)
        
        # INITIALISE STATIC BAROMTER
        try:
            self.bmp = BMP280(I2C(1,
                scl = Pin(3),
                sda = Pin(2)))
            print('bmp done')
        except Exception as e:
            print('Failed to initialize BMP280')
            print(e)
            self.except_occr = True
        
        # INITIALISE RAM BAROMETER        
        if self.ram_dout.value() == 0:           # IF DATA FROM RAM BAROMETER IS READY
            if self.ram_raw_data() != 0:         # AND IT IS NOT 0 (CONNECTION IS GOOD)
                print('ram done')
            else:
                print('Failed to initialize Ram Barometer')
                self.except_occr = True
        else:
            print('Failed to initialize Ram Barometer')
            self.except_occr = True


        # INITIALISE FLASH
        try:
            self.flash = winbond.W25QFlash(SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12)), Pin(7))
            uos.mount(self.flash, '/win')
            print('flash done')
        except Exception as e: #OSError
            print('Failed to initialize Flash')
            print(e)
            self.except_occr = True
        
        # INITIALISE GPS (THIS ONLY DEFINES THE STREAMREADER AND DOESN'T ACTUALLY CHECK FOR THE GPS CHIP. MUST BE CHANGED)
        try:
            sreader = asyncio.StreamReader(self.uart)  # Create a StreamReader
            
            # fix_cb DEFINES THE FUNCTION CALLED WHEN A VALID FIX IS ACCQUIRED
            # cb_mask DEFINES THE NMEA SENTENCES WHICH WILL TRIGGER THE CALLBACK (RMC,VTG,GLL,GGA)
            self.gps = as_GPS.AS_GPS(sreader,fix_cb = self.callback_gps,cb_mask= as_GPS.RMC | as_GPS.VTG | as_GPS.GLL | as_GPS.GGA)
            
            print('gps done')
        except Exception as e:
            print('Failed to initialize GPS')
            print(e)
            self.except_occr = True 
         
        # INITIIALISE IMU 
        try:
            self.bno = BNO055(I2C(1,
                scl = Pin(3),
                sda = Pin(2)))
            
            # REGISTER ADDRESSES FOR ACCELEROMTER OFFSETS
            ACCEL_OFFSET_X_LSB_ADDR = const(0x55)
            ACCEL_OFFSET_X_MSB_ADDR = const(0x56)
            ACCEL_OFFSET_Y_LSB_ADDR = const(0x57)
            ACCEL_OFFSET_Y_MSB_ADDR = const(0x58)
            ACCEL_OFFSET_Z_LSB_ADDR = const(0x59)
            ACCEL_OFFSET_Z_MSB_ADDR = const(0x5A)
            ACCEL_RADIUS_LSB_ADDR = const(0x67)
            ACCEL_RADIUS_MSB_ADDR = const(0x68)
            
            # SET IMU TO CONFIG MODE (TO SET ACCELEROMTER OFFSETS)
            
            self.bno.mode(0)
            
            # BYTEARRAY OF IMU OFFSETS (FOUND FROM PRIOR CALIBRATION PROCESS)            
            
            bno_offsets = bytearray(b'\xf7\xff\xd4\xff\xf1\xff\xbe\xffq\xfc\xc3\xef\xff\xff\xfe\xff\x01\x00\xe8\x03L\x02')

            # WRITE ACCELEROMETER OFFSETS TO THEIR REGISTERS (NOT GYRO AND MAGNETOMETERS, THEY SHOULD BE CALIBRATED ON EACH STARTUP)

            self.bno._write(ACCEL_OFFSET_X_LSB_ADDR, bno_offsets[0])
            self.bno._write(ACCEL_OFFSET_X_MSB_ADDR, bno_offsets[1])
            self.bno._write(ACCEL_OFFSET_Y_LSB_ADDR, bno_offsets[2])
            self.bno._write(ACCEL_OFFSET_Y_MSB_ADDR, bno_offsets[3])
            self.bno._write(ACCEL_OFFSET_Z_LSB_ADDR, bno_offsets[4])
            self.bno._write(ACCEL_OFFSET_Z_MSB_ADDR, bno_offsets[5])
            self.bno._write(ACCEL_RADIUS_LSB_ADDR, bno_offsets[18])
            self.bno._write(ACCEL_RADIUS_MSB_ADDR, bno_offsets[19])
                    
            # SWITCH OUT OF CONFIG MODE
                    
            self.bno.mode(12)
            
            # START CALIBRATION OF GYRO AND MAGNETOMTER
            
            while self.bno.calibrated():
                print(self.bno.cal_status())         # WILL SHOW SOMETHING LIKE b'\x00\x00\x00\x00'
                                                     #
                                                     # x00/x01/x02 - UNCALIBRATED, x03 - CALIBRATED
                                                     #
                                                     # FIRST BIT SHOWS CALIBRATION OF BNO'S INTERNAL ALGORITHMS
                                                     # SECOND BIT SHOWS CALIBRATION OF GYRO (KEEP IMU AT REST)
                                                     # THIRD BIT SHOWS CALIBRATION OF ACCELEROMTER (SHOULD SHOW x03 IF OFFSETS HAVE BEEN WRITTEN, IF IT SHOWS x00 OR x01, RESTART)
                                                     # FOURTH BIT SHOWS CALIBRATION OF MAGNETOMETER (WAVE IMU AROUND IN FIGURE 8)
                sleep(0.05)
                
                
            # CALIBRATION OF g
            print('Place IMU at rest')
            
            sleep(3)
            
            self.g = 0
            
            for i in range(10):
                bno_accel = self.bno.accel()
                self.g += sqrt(pow(bno_accel[0],2) + pow(bno_accel[1],2) + pow(bno_accel[2],2))/10
                
            print('g = ' + str(self.g))           # PRINT g (SHOULD BE AROUND 9.8 OR 9.9)
            
            print('bno done')
        except Exception as e:
            print('Failed to initialize BNO055')
            print(e)
            self.except_occr = True
        
#       # FLASH LED AND PRINT ERROR
        if (self.except_occr):
            print("error in init")
            self.async_loop.run_until_complete(self.beep(10))
            self.async_loop.stop()
            pass
        else:    
            print('Init done')
    
    async def calibrate(self):

        self.calib_count += 1
        
        self.beep(1,1)
        
        self.calib_time = ticks_ms()
        
        print('Calibration Number ' + str(self.calib_count))
        
        self.calib_bmp()
        self.calib_ram()
        
        X_A = np.eye(3)
        C1 = np.eye(3)

        self.bno_accel = self.bno.accel()
        self.mag = self.bno.mag()
    
        ax = self.bno_accel[2]
        ay = -self.bno_accel[1]
        az = self.bno_accel[0]
         
        mx = self.mag[2]
        my = -self.mag[1]
        mz = self.mag[0]
        
        X_A[0][0] = atan2(ay,az)    
        X_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))

        C1[0][0] = cos(X_A[1][0])
        C1[0][1] = 0
        C1[0][2] = sin(X_A[1][0])
        
        C1[1][0] = sin(X_A[0][0])*sin(X_A[1][0])
        C1[1][1] = cos(X_A[0][0])
        C1[1][2] = -sin(X_A[0][0])*cos(X_A[1][0])
 
        C1[2][0] = -sin(X_A[1][0])*cos(X_A[0][0])
        C1[2][1] = sin(X_A[0][0])
        C1[2][2] = cos(X_A[0][0])*cos(X_A[1][0])
        
        b = np.dot(C1,np.array([[mx],[my],[mz]]))
          
        if atan2(-b[1][0],b[0][0]) > 0:
            self.heading_init = 2*pi - atan2(-b[1][0],b[0][0])     
        else:
            self.heading_init = - atan2(-b[1][0],b[0][0])

        
        if self.calib_count == self.calib_max:
            self.bno.mode(7)
            self.bno.config(ACC,(16,62))
            self.bno.config(GYRO,(2000,32))
            self.bno.config(MAG,(30,))
        
        # CALIBRATION DATA PACKED TO BYTES TO WRITE IN FLASH
        self.calib_data = ustruct.pack('ffiiif',
                                              self.calib_altitude,
                                              self.ram_offset,
                                              self.mag_mult,
                                              self.orientation_mult,
                                              self.temp_mult,
                                              self.calib_temp)
        self.send_calib = True
        
        self.async_loop.create_task(self.beep(4))
        
        await asyncio.sleep(0)


    async def get_data(self):
        
        while True:

            self.t_log = ticks_ms() - self.calib_time
            self.last_loop_t = ticks_ms()

            # Get sensor readings
            try:
                self.bno_accel = self.bno.accel()
                self.gyro = self.bno.gyro()
                self.mag = self.bno.mag()
                self.bno_working = True
            except:
                self.bno_working = False

            try:
                self.temp = self.bmp.temperature
                self.alt = self.altitude() - self.calib_altitude
                self.density = self.rho(self.alt)

                self.bmp_working = True
            except:
                self.bmp_working = False
                    
            # READ RAM BAROMETER ONLY IF DATA IS READY
            if self.ram_dout.value() == 0:
                self.ram_diff = self.ram_raw_data() - self.ram_offset
                self.new_ram_data = True
            else:
                self.new_ram_data = False
                
            # Check if calibration is needed    
            if (self.state == 0):
                    if (self.t_log > self.calib_gap and self.calib_count < self.calib_max):
                        self.async_loop.create_task(self.calibrate()) # RUN CALIBRATION ONCE

            await asyncio.sleep(0)
        
        

    async def state_machine(self):
        
        while True:
        
            
            t = ticks_ms()
            
    #         for index in range(1,self.buf_len):
    #             self.alt_buf[index - 1] = self.alt_buf[index]
    #             self.acc_buf[index - 1] = self.acc_buf[index]
    #         for index in range(1,self.buf_len - 1):
    #             self.vel_buf[index - 1] = self.vel_buf[index]

            if self.bmp_working:
                #self.alt_buf[self.buf_len - 1] = self.alt
                self.alt_buf = np.concatenate((self.alt_buf[:-1], np.array([self.alt])))
                #self.vel_buf[self.buf_len - 2] = 1000 * (self.alt_buf[self.buf_len - 1] - self.alt_buf[self.buf_len - 2])/(self.t_log - self.last_t_log)
                self.vel_buf = np.concatenate((self.vel_buf[:-1], np.array([1000 * (self.alt_buf[-1] - self.alt_buf[-2])/(self.t_log - self.last_t_log + 0.00001)]))) # avoid divide by zero errors
    #         else:
    #             if self.state == 5:
    #                 self.alt_buf[self.buf_len - 1] = self.alt_buf[self.buf_len - 1]
    #                 self.vel_buf[self.buf_len - 2] = 0
    #             elif self.state == 1:
    #                 self.alt_buf[self.buf_len - 1] += (self.vel_buf[self.buf_len - 2] - 0.5*(self.acc_buf[self.buf_len - 1] + self.g)*(self.t_log - self.last_t_log))*(self.t_log - self.last_t_log)
    #                 self.vel_buf[self.buf_len - 2] += -(self.acc_buf[self.buf_len - 1] + self.g)
    #             elif self.state == 2:
    #                 self.alt_buf[self.buf_len - 1] += (self.vel_buf[self.buf_len - 2] - 0.5*self.g*(self.t_log - self.last_t_log))*(self.t_log - self.last_t_log)
    #                 self.vel_buf[self.buf_len - 2] += -self.g*(self.t_log - self.last_t_log)
    #             elif self.state == 3:
    #                 self.alt_buf[self.buf_len - 1] += -self.drogue_descent_rate*(self.t_log - self.last_t_log)
    #                 self.vel_buf[self.buf_len - 2] = -self.drogue_descent_rate
    #             elif self.state == 4:
    #                 self.alt_buf[self.buf_len - 1] += -self.main_descent_rate*(self.t_log - self.last_t_log)
    #                 self.vel_buf[self.buf_len - 2] = -self.main_descent_rate

            #self.acc_buf[self.buf_len - 1] = self.bno_accel[0]
            self.acc_buf = np.concatenate((self.acc_buf[:-1], np.array([self.bno_accel[0]])))
              
            if self.alt > self.max_alt:
                self.max_alt = self.alt

            if (self.state != 0 and self.state != 5):
                self.tel_delay = 110
            
            if (self.t_log > self.runtime or self.logging_done): # Stop data logging after run_time or if touchdown is detected
                self.data_array = bytearray()
                self.async_loop.stop() # EXIT ASYNC LOOP
                
            if (self.state==0 and ((np.all(self.alt_buf > self.min_liftoff_alt) and np.all(self.acc_buf > self.liftoff_accel*self.g)) or np.all(self.alt_buf > self.force_liftoff_alt))): #Liftoff
                self.state = 1
                self.t_events[0] = self.t_log
                self.beep()
                print("liftoff")

            elif (self.state==1 and (np.all(self.acc_buf < 0.05*self.g) or self.t_log - self.t_events[0] > self.force_burnout_time)):
                self.state = 2
                self.t_events[1] = self.t_log
                self.beep(2)
                print("burnout")

            elif ((self.state==2 and self.t_log - self.t_events[0] >self.lockout_drogue_time) and (np.all(self.alt_buf < self.max_alt) or self.t_log - self.t_events[0] > self.force_drogue_time)):
                self.state = 3
                self.t_events[2] = self.t_log
                self.beep(3)
                
                for i in range(3):
                    self.drogue_pin.value(1)
                    sleep_ms(300)
                    self.drogue_pin.value(0)
                    sleep_ms(100)
                
                print("drogue")

            elif (self.state==3 and np.all(self.alt_buf < self.main_alt)):
                self.state = 4
                self.t_events[3] = self.t_log
                self.beep(4)
                
                for i in range(3):
                    self.main_pin.value(1)
                    sleep_ms(300)
                    self.main_pin.value(0)
                    sleep_ms(100)
                
                print("main")
                    
            elif (self.state == 4 and (np.all(self.alt_buf < self.touchdown_alt) and np.mean(self.vel_buf) < self.touchdown_vel_limit)):
                self.state = 5
                self.t_events[4] = self.t_log
                self.logging_done = True
                self.tel_delay = 1000
                self.beep(5)
                
                print("touchdown")

            else:
                pass

            self.last_t_log = self.t_log

            await asyncio.sleep(0) 


    async def log_data(self):
        
        while True:
           # print("log data")
        
            t = ticks_ms()

    #         gc.collect()
            
            # PACK FAST READINGS TO BYTES
            ustruct.pack_into(self.packing_str,
                                          self.buffer,
                                          0,                      # offset 0
                                          b'F',                   # F INDICATES THAT DATA IS OF FAST READINGS
                                          self.t_log,
                                          self.state,
                                          (int)((self.temp - self.calib_temp)*self.temp_mult),
                                          (float)(self.alt),
                                          (float)(self.ram_diff),
                                          (float)(self.bno_accel[0]),
                                          (float)(self.bno_accel[1]),
                                          (float)(self.bno_accel[2]),
    #                                           (float)(high_g_accel[0]),
    #                                           (float)(high_g_accel[1]),
    #                                           (float)(high_g_accel[2]),
                                          3.14,
                                          2.71,
                                          9.81,
                                          (float)(self.gyro[0]),
                                          (float)(self.gyro[1]),
                                          (float)(self.gyro[2]),
                                          (int)(self.mag[0]*self.mag_mult),
                                          (int)(self.mag[1]*self.mag_mult),
                                          (int)(self.mag[2]*self.mag_mult),
                                          (int)(self.orientation[0]*self.orientation_mult),
                                          (int)(self.orientation[1]*self.orientation_mult),
                                          (int)(self.orientation[2]*self.orientation_mult)
                                          )
            
    #         gc.collect()
            
            # PACK SLOW READINGS TO BYTES
            # ONLY IF NEW DATA HAS BEEN RECEIVED AND POSITION FIX HAS OBTAINED ATLEAST ONCE
            if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                ustruct.pack_into( self.packing_str_slow,
                                               self.data_slow,
                                               0,
                                               b'S',                     # 1 INDICATES THAT DATA IS OF SLOW READINGS
                                               self.latitude[0],
                                               self.latitude[1],
                                               self.latitude[2],
                                               ord(self.latitude[3]),
                                               self.longitude[0],
                                               self.longitude[1],
                                               self.longitude[2],
                                               ord(self.longitude[3]),
                                               self.speed,
                                               self.course,
                                               self.hdop,
                                               self.vdop,
                                               self.sentence_type,
                                               self.gps_altitude,
                                               self._fix_time - self.calib_time,
                                               self.time_since_fix
                                               )
            
            
            # DATA MUST BE WRITTEN TO FLASH IN PACKETS OF 512 BYTES. IF SIZE OF PACKET IS LESS THAN 512, APPEND DATA TO THE PACKET.
            # ELSE WRITE PACKET TO FLASH AND RESET THE PACKET.
            
    #         gc.collect()
                    
    #         print(self.log_count,self.data_array[-64:])
                    
            if (self.log_count <= 448):
                self.data_array[self.log_count:self.log_count+64] = self.buffer
                self.log_count += 64
            if (self.log_count == 512):
                self.log_count = 0
                if (self.state != 0 and self.state!= 5): # only log when liftoff happens
                    sleep_ms(10)
                    self.data_file.write(self.data_array)
                    sleep_ms(20)
                #await asyncio.sleep_ms(20)
    #             for i in range(len(self.data_array)):
    #                 self.data_array[i:i+1] = b'/x00'
                
                #self.data_array[self.log_count:self.log_count+64] += self.buffer
    #             print(self.data_array == bytearray(512))
                
    #         gc.collect()
                
            # WRITE SLOW DATA TO FLASH ONLY IF NEW FIX IS ACQUIRED      
            if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                if (self.log_count <= 448):
                    self.data_array[self.log_count:self.log_count+64] = self.data_slow
                    self.log_count += 64
                if (self.log_count == 512):
                    self.log_count = 0
                    if (self.state != 0 and self.state!= 5):
                        sleep_ms(10)
                        self.data_file.write(self.data_array)
                        sleep_ms(20)
                    #await asyncio.sleep_ms(20)
                    
                    #self.data_array[self.log_count:self.log_count+64] += self.data_slow

    #         gc.collect()

            await asyncio.sleep_ms(25)        
     
             
    # COMMUNICATION BETWEEN ROCKET AND GROUND STATION   
    async def comms(self):
        
        while True:
        
            #print("comms------------------------------------------------------")
            
            t = ticks_ms()
            
            if self.send_calib:
                self.xbee.write(self.calib_data)
                self.send_calib = False
                
            if self.state != 5:
                self.xbee.write(self.buffer)
            
            if self.gps_counter > self.last_gps_counter:
                sleep(0.01)
                self.xbee.write(self.data_slow)

            await asyncio.sleep_ms(self.tel_delay)
        
    def fast_core_init(self):
        
        print('fast core init')
        index = 0
        
        if ("/win/index.txt" in uos.listdir("/win")):
            with open('/win/index.txt','r') as index_file:
                index_lines = index_file.readlines()
                if (index_lines is not([])):
                    index = int(index_lines[0])
                    
        with open('/win/index.txt','w') as index_file:
            index_file.write(str(index + 1))
                
        self.data_file = open('/win/data_' + str(index) + '.bin','wb')
        
        print("starting event loop")
        
        
        self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.beep(2,800))
        self.async_loop.create_task(self.calibrate())        # GET CALIBRATION DATA
        self.async_loop.create_task(self.beep_regular())     # BEEP BEEP BITCH
        self.async_loop.create_task(self.get_data())         # GET SENSOR DATA
        self.async_loop.create_task(self.log_data())         # LOG THE DATA
        self.async_loop.create_task(self.comms())            # TRANSMIT TO GROUND STATION
        self.async_loop.create_task(self.state_machine())    # RUN THE STATE MACHINE
        self.async_loop.create_task(self.nav())              # RUN THE NAVIGATION FILTER
        self.async_loop.run_forever()                        # REPEAT
        
        self.data_file.close()
        
        print("lmao fail")
        
        with open('/win/config_' + str(index) + '.bin','wb') as config_file:
            config_file.write(self.calib_data)
            
        print("logging done, now sending only GPS")
#         self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.beep_regular(1000))
        self.async_loop.create_task(self.comms())
        self.async_loop.run_forever()
 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ----------------------------------------------------

#     async def after_touchdown(self):
#         asyncio.create_task(self.beep_regular(1000))
#         asyncio.create_task(self.comms())
#         await asyncio.sleep(0)
    
    async def nav(self):
        
        counter = 0
        
        X_A = np.full((3,1),0.)
        W_A = np.full((3,1),0.)  
        F_A = np.eye(3)
        P_A = np.full((3,3),0.)
        Q_A = np.full((3,3),0.)
        R_A = np.eye(3)
        Z_A = np.full((3,1),0.)
        C = C1 = np.full((3,3),0.)
        U_A = np.full((3,1),0.)

        X = np.full((6,1),0.)
        W = np.full((6,1),0.)  
        F = np.eye(6)
        P = np.full((6,6),0.)
        Q = np.full((6,6),0.)
        R = np.eye(6)
        Z = np.full((6,1),0.)
        U = np.full((6,1),0.)
        
        last_valid_X = X
        last_valid_P = P

        dt = 1/30
        nx = ny = 1
        init = False
        declination = -0.111*pi/180
        
        i2c_mc = I2C(0,scl=Pin(5),sda=Pin(4))
                
        while True:
            #print("nav")
            #break
            if counter > 0:
                
                t = ticks_ms()

                ax = self.bno_accel[2]
                ay = -self.bno_accel[1]
                az = self.bno_accel[0]
                        
                a = sqrt(ax*ax + ay*ay + az*az)

                gx = self.gyro[2]*pi/180
                gy = -self.gyro[1]*pi/180
                gz = -self.gyro[0]*pi/180

                k = 1/700
                
                if k*(self.ram_diff) + self.delta_p(X[2][0],X[2][0] + X[3][0]*dt) >= 0:
                    airspeed = sqrt(2*(k*(self.ram_diff) + self.delta_p(X[2][0],X[2][0] + X[3][0]*dt))/self.density)
                
            # ATTITUDE KF --------------------------------------------      

                e = abs(a-self.g)/self.g

                cutoff = 0.05

                if e < cutoff:
                    X_A[0][0] = atan2(ay,az)    
                    X_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))
                else:
                    X_A[0][0] += nx*gx*dt
                    X_A[1][0] += ny*gy*dt


                if self.t_events[0]:
                    X_A[2][0] += gz*dt
                elif counter > 0: 																			
                    X_A[2][0] = self.heading_init

                if abs(X_A[0][0] + nx*gx*dt) >= pi/2:
                    nx *= -1
                if abs(X_A[1][0] + ny*gy*dt) >= pi/2:
                    ny *= -1

                C1[0][0] = cos(X_A[1][0])
                C1[0][1] = 0
                C1[0][2] = sin(X_A[1][0])
                
                C1[1][0] = sin(X_A[0][0])*sin(X_A[1][0])
                C1[1][1] = cos(X_A[0][0])
                C1[1][2] = -sin(X_A[0][0])*cos(X_A[1][0])
         
                C1[2][0] = -sin(X_A[1][0])*cos(X_A[0][0])
                C1[2][1] = sin(X_A[0][0])
                if e < cutoff and Z[2][0] < 300:
                    C1[2][2] = cos(X_A[0][0])*cos(X_A[1][0])
                elif X[3][0] > 0:
                    C1[2][2] = X[3][0]/sqrt(pow(X[3][0],2)+pow(X[4][0],2))
                
        # -------------------------------------------------------------------------------------------------------------------
                
                last_alt = Z[2][0]
                
                gravity = np.dot(C1,np.array([[0],[0],[-self.g]]))

                lin_acc_x = - ax + gravity[0][0]
                lin_acc_y = - ay + gravity[1][0]
                lin_acc_z = az + gravity[2][0]

                a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))

                dv = sqrt(pow(a_r[0][0],2) + pow(a_r[1][0],2))*dt
                alpha = atan2(-a_r[0][0],a_r[1][0]) + X_A[2][0] + declination
                
                if self.latitude[0] != 0 and self.longitude[0] != 0 and init == False:
                    X[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                    X[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                    init = True                         
                 
                F[0][4] = cos(Z[5][0])*dt/self.r
                
                if abs(Z[0][0]*180/pi - 90) > 10:
                    F[1][4] = sin(Z[5][0])*dt/(self.r*cos(Z[0][0]))
                else:
                    F[1][4] = 0
                    
                F[2][3] = dt
                
                U[2][0] = a_r[2][0]*dt*dt/2
                U[3][0] = a_r[2][0]*dt
                U[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha)) - Z[4][0]
                U[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha))) - Z[5][0]
                 
                for j in (0,1):
                    Q[j][j] = 0.01/(self.hdop + 0.001)   
                for j in (2,3,4,5):
                    Q[j][j] = 0.1
                
                Xp = np.dot(F,X) + U

                Pp = np.dot(np.dot(F,P),F.T) + Q

                if self.bmp_working:
                    Z[2][0] = self.alt
                elif self.bno_working:
                    Z[2][0] += (Z[3][0] + a_r[2][0]*dt/2)*dt

                if self.bmp_working:
                    Z[3][0] = (Z[2][0] - last_alt)/dt
                elif self.bno_working or self.high_g_working:
                    Z[3][0] += a_r[2][0]*dt


                if self.gps_counter > self.last_gps_counter:
                    if 'GGA' in self.sentence_type or 'GLL' in self.sentence_type:
                        if abs((self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600) - 90) > 10:
                            Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        else:
                            Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                            
                        Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                        
                        if self.t_events[0]:
                            if self.new_ram_data:
                                if airspeed > abs(Xp[3][0]):
                                    Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                                            

                        Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                       
                       
                    elif 'VTG' in self.sentence_type:
                        if self.t_events[0]:
                            if self.new_ram_data:
                                if airspeed > abs(Xp[3][0]):
                                    Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = self.speed
                            
                        Z[5][0] = self.course*pi/180
                        
                        if abs(Z[0][0]*180/pi - 90) > 10:
                            Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                            
                        Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                    
                    elif 'VNC' in self.sentence_type:
                        if self.t_events[0]:
                            if self.new_ram_data:
                                if airspeed > abs(Xp[3][0]):
                                    Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = self.speed
                            
                        Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                        
                        if abs(Z[0][0]*180/pi - 90) > 10:
                            Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                            
                        Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                        
                    elif 'RMC' in self.sentence_type:
                        if abs((self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600) - 90) > 10:
                            Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        else:
                            Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                            
                        Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                        
                        if self.t_events[0]:
                            if self.new_ram_data:
                                if airspeed > abs(Xp[3][0]):
                                    Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = self.speed
                            
                        Z[5][0] = self.course*pi/180
                   
                    elif 'ROV' in self.sentence_type:
                        if abs((self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600) - 90) > 10:
                            Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        else:
                            Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                            
                        Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                        
                        if self.t_events[0]:
                            if self.new_ram_data:
                                if airspeed > abs(Xp[3][0]):
                                    Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = self.speed
                        
                        Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                        
                else:
                    if self.t_events[0]:
                        if self.new_ram_data:
                            if 0 <= pow(airspeed,2) - pow(Xp[3][0],2) < pow(110*(self.t_log - self.t_events[0] + 0.5) + 30,2):
                                Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        else:
                            Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                    else:
                        Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                        
                    Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                    
                    if abs(Z[0][0]*180/pi - 10) > 10:
                        Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                    Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r

                if -180 < Z[5][0]*180/pi < 0:
                    Z[5][0] += 2*pi

                K = np.dot(Pp,np.linalg.inv(Pp + R))

                X = Xp + np.dot(K,Z - Xp)
                
                P = np.dot(np.eye(len(K)) - K,Pp)
  
                try:
                    mc_data = ustruct.pack('ffff',X[2][0],X[3][0],sqrt(pow(X[3][0],2)),acos(C1[2][2]))
                    i2c_mc.writeto(0x41,mc_data)
                except:
                    pass

                dt = (ticks_ms() - self.last_loop_t)/1000
                
                self.last_gps_counter = self.gps_counter
                      
            counter += 1
         
            await asyncio.sleep(0)
# 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ---------------------------------------------------


if __name__ == "__main__":
    test_instance = async_test()                    # START INSTANCE OF CLASS
    test_instance.init()                            # INITIALISE SOFTWARE
    test_instance.init_board()                      # INITIALISE SENSORS
    test_instance.fast_core_init()                  # START THE FAST CORE ASYNC LOOP
    #asyncio.run(test_instance.after_touchdown())
    
    #asyncio.run(test_instance.fast_readings())      # START FAST CORE


