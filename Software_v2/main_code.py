
# ------------------------------------------ as_GPS HAS BEEN MODIFIED --------------------------------------------------

from machine import I2C,Pin,UART,SPI,freq,PWM
from math import sqrt,pi,asin,atan,tan,cos,sin,atan2,acos
from time import ticks_ms,sleep,sleep_us,sleep_ms, gmtime
from ulab import numpy as np
from bno055 import *
from bmp280 import *
import as_GPS
import uasyncio as asyncio
import ustruct
from flash_spi import FLASH
import uos
import sdcard
import gc
import micropython
micropython.alloc_emergency_exception_buf(100)
gc.collect()


freq(270000000) #overclock?!!???

class async_test:
    
    # ALTITUDE FROM STATIC AIR PRESSURE
    def altitude(self):
        return 4947.19 * (8.9611 - pow(self.bmp.pressure,0.190255))

    # DIFFERENCE IN AIR PRESSURE BETWEEN TWO ALTITUDES
    def delta_p(self,y1,y2):
        return pow(8.9611 - (y1 + self.calib_altitude)/4947.19,5.2479) - pow(8.9611 - (y2 + self.calib_altitude)/4947.19,5.2479)
    
    # DENSITY
    def rho(self,y):
        return pow(8.9611 - (y + self.calib_altitude)/4947.19,5.2479)/(78410.439 + 287.06*self.calib_temp - 1.86589*(y + self.calib_altitude))
    
    # toggle led asynchronously 
    async def blink(self, t):
        while True:
            self.led_1.value(0)
            self.led_2.value(0)
            await asyncio.sleep_ms(t)
            self.led_1.value(1)
            self.led_2.value(1)
            await asyncio.sleep_ms(t)
    
    # LED + BEEPS ON POWER UP
    def board_init(self):
        for i in range(2):
            self.led_1.value(1)
            self.led_2.value(0)
            self.beep()
            self.led_1.value(0)
            self.led_2.value(1)
            self.beep()

        self.led_1.value(1)
        self.led_2.value(1)
    
    # LED + BEEPS ON SENSOR INIT
    def sensor_init(self):
        for i in range(2):
            self.led_1.value(0)
            self.led_2.value(0)
            self.beep()
            self.led_1.value(1)
            self.led_2.value(1)
            sleep(0.2)
        
    # LED + BEEPS IF MOTOR CONTROLLER BOARD IS NOT FOUND
    def mc_missing(self):
        return # REMOVE LATER
        if self.state == 0:
            while True:
                self.led_1.value(0)
                self.led_2.value(0)
                self.beep()
                self.led_1.value(1)
                self.led_2.value(1)
                sleep(0.1)
        else:
            pass

    # LED + BEEPS IF NAV FILTER CRASHES
    def nav_failure(self):
        if self.state == 0 and not KeyboardInterrupt:
            while True:
                self.led_1.value(0)
                self.led_2.value(1)
                self.beep(1,500)
                self.led_1.value(1)
                self.led_2.value(0)
                sleep(1)
        else:
            pass
            
    # LED + BEEPS FOR OTHER GENERAL FAILURES
    def failure(self):
        if self.state == 0 and not KeyboardInterrupt:
            self.buzzer.duty_u16(0)
            self.led_1.value(0)
            self.led_2.value(0)
        else:
            pass

    # BUZZZER BEEP
    def beep(self,*args):
             
        n = 1
        f = 2000
        t = 50

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
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
    
    # INITIALISE BOARD    
    def init(self):
        self.async_loop = None
        self.tel_delay = 1000
        self.t_events = [0.,0.,0.,0.,0.]
        self.logging_done = False
        self.state = 0
        self.bmp, self.bno, self.xbee, self.gps, self.flash = None, None, None, None, None
        self.send_calib = False
        self.buzzer = PWM(Pin(6))
        self.led_1 = Pin(18,Pin.OUT)
        self.led_2 = Pin(19,Pin.OUT)
        
        self.drogue_pin = Pin(28)
        self.main_pin = Pin(20)
        self.r = 6371000                  # RADIUS OF EARTH
        self.g = 9.81                     # ACCELERATION DUE TO GRAVITY         
        self.except_occr = False
        self.t_log = ticks_ms()
        self.last_t_log = ticks_ms()
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
        self.density = 1.15
        
        self.bno_working = True
        self.bmp_working = True
        
        self.calib_time, self.calib_altitude, self.calib_temp ,self.ram_offset = 0,0,0,0
        self.shutdown = False
        self.calib_data = bytes(26)
        
        self.runtime = 20 * 1000
        
        self.temp_mult = 100
        self.acc_mult = 100
        self.volt_mult = 10
        self.gps_mult = 100
        self.packing_str = '!sibhffhhhffffffbbbb'
        self.packing_str_slow = '!sbbfbbbfbHHHH3sfff'
        
        self.max_alt = 0
        self.calib_gap = 10000 # Gap in milliseconds between subsequent calibrations
        self.calib_count = 0 # number of calibrations performed till now
        self.calib_max = 2 # max number of calibrations
        
        self.liftoff_accel = 3 # in g, minimum sustained acceleration for liftoff
        self.min_liftoff_alt = 10 # in m, minimum altitude to be cleared for liftoff
        self.force_burnout_time = 10000 # In milliseconds, force burnout after this amount of time from detected liftoff
        self.force_drogue_time = 20000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        self.lockout_drogue_time = 12000 # In milliseconds, lockout for drogue deployment from detected liftoff
        self.touchdown_alt = 100 # in m, If altitude is less than this in descent, declare touchdown
        self.touchdown_vel_limit = 0.01 # in m/s, if velocity is less than this in descent, declare touchdown
        self.main_alt = 400 # in m, deploy main parachute when descent comes under this height
        self.force_liftoff_alt = 100 # in m, force liftoff detection if height increases above this

        self.buf_len = 5
        self.alt_buf = np.zeros((self.buf_len))
        self.acc_buf = np.zeros((self.buf_len))
        self.vel_buf = np.zeros((self.buf_len))
        
        # NOT SURE IF SIZES NEED TO BE FACTORS OF 512 FOR PICO'S ONBOARD FLASH. IT'S PRETTY FAST ALREADY.
        self.data_fast = bytearray(50)
        self.data_slow = bytearray(38)
        
        self.size = 0 # SIZE OF DATA FILE IN BYTES
        
        # CALCULATE FREE SPACE
        storage = uos.statvfs("/")
        self.free_bytes = storage[0]*storage[3]
        
        self.board_init()
    

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
        
    
    # RAW DATA FROM RAM BAROMETER (MULTIPLY BY 1/700 TO GET DIFFERENTIAL PRESSURE)
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
            sleep_ms(10)
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
            cspins = [Pin(7)] # has to be a list for this library to work
            spi = SPI(1, sck= Pin(10), mosi=Pin(11), miso=Pin(12))
            self.flash = FLASH(spi, cspins, cmd5 = False)
            uos.mount(self.flash, '/flash')
            print('flash done')
        except Exception as e: #OSError
            print('Failed to initialize Flash')
            print(e)
            self.except_occr = True
        
        # INITIALISE GPS (THIS ONLY DEFINES THE STREAMREADER AND DOESN'T ACTUALLY CHECK FOR THE GPS CHIP)
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
            
            # BYTEARRAY OF IMU OFFSETS (CALIBRATE AGAIN BEFORE LAUNCH)            
            
            bno_offsets = bytearray(b'\xcc\xff\xb7\xff\xe2\xff\xb2\xffJ\xfc\xb7\xef\xff\xff\x00\x00\xff\xff\xe8\x03\x17\x04')
            
            # WRITE ACCELEROMETER OFFSETS TO THEIR REGISTERS (GYRO WILL CALIBRATE WHEN LEFT AT REST)
            #                                                (MAGNETOMETER IS PROBLEMATIC. EVEN TILTING IT A LITTLE BIT IS ENOUGH TO CALIBRATE.)
            #                                                (SO MIGHT HAVE TO TILT THE ROCKET BACK AND FORTH ON THE RAIL A LITTLE BIT. BUT IT'S NOT ESSENTIAL SO IT'S FINE)

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
                
            # CALIBRATION OF g
            print('Place IMU at rest')
            
            self.beep(2)
            self.led_1.value(0)
            sleep(3)
            self.led_1.value(1)
            self.g = 0
            
            for i in range(10):
                bno_accel = self.bno.accel()
                self.g += sqrt(pow(bno_accel[0],2) + pow(bno_accel[1],2) + pow(bno_accel[2],2))/10
                
            print('g = ' + str(self.g))           # PRINT g (SHOULD BE AROUND 9.8 OR 9.9)
            
            print('Wave around IMU')
            
            # CALIBRATE MAGNETOMETER
            
            self.beep(2)
            self.led_2.value(0)
            sleep(3)
            self.led_2.value(1)
            
            print('bno done')
        except Exception as e:
            print('Failed to initialize BNO055')
            print(e)
            self.except_occr = True

        if (self.except_occr):
            pass
        else:    
            self.sensor_init()
    
    # CALIBRATION
    async def calibrate(self):

        self.calib_count += 1
        
        self.calib_time = ticks_ms()
        
        print('Calibration Number ' + str(self.calib_count))
        
        self.calib_bmp()
        self.calib_ram()
        
        # CALCULATING HEADING
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
        
        # TILT COMPENSATION
        b = np.dot(C1,np.array([[mx],[my],[mz]]))
          
        if atan2(-b[1][0],b[0][0]) > 0:
            self.heading_init = 2*pi - atan2(-b[1][0],b[0][0])     
        else:
            self.heading_init = - atan2(-b[1][0],b[0][0])

        print(self.heading_init*180/pi)

        lat = self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600
        lon = self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600

        # MAGNETOMETER ONLY WORKS PROPERLY WHEN BNO IS IN FUSION MODE BUT FUSION MODE IS LIMITED TO +- 4G.
        # AFTER FINAL CALIB, SWITCH TO +- 16G AND 2000 DPS MODE. MAGNETOMETER VALUES ARE BASICALLY USELESS NOW.
        if self.calib_count == self.calib_max:
            self.bno.mode(7)
            self.bno.config(ACC,(16,62))
            self.bno.config(GYRO,(2000,32))
            self.bno.config(MAG,(30,))
        
        # CALIBRATION DATA PACKED TO BYTES
        self.calib_data = ustruct.pack('!fffffbfb',
                                              self.calib_altitude,
                                              self.ram_offset,
                                              self.calib_temp,
                                              self.heading_init,
                                              lat,
                                              (int)(ord(self.latitude[3])),
                                              lon,
                                              (int)(ord(self.longitude[3]))
                                              )
        self.send_calib = True
        
        self.led_1.value(1)
        self.led_2.value(1)
        sleep(0.2)
        self.led_1.value(0)
        self.led_2.value(0)
        self.beep()
        sleep(0.2)
        self.led_1.value(1)
        self.led_2.value(1)

        await asyncio.sleep(0)

    # DATA ACQUISITION
    async def get_data(self):
        
        while True:

            self.t_log = ticks_ms() - self.calib_time
            self.last_loop_t = ticks_ms()

            # TRY BNO
            try:
                self.bno_accel = self.bno.accel()
                self.gyro = self.bno.gyro()
                self.mag = self.bno.mag()
                self.bno_working = True
            except:
                self.failure()
                self.bno_working = False
            
            # TRY BMP
            try:
                self.temp = self.bmp.temperature
                self.alt = self.altitude() - self.calib_altitude
                self.density = self.rho(self.alt)
                self.bmp_working = True
            except:
                self.failure()
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
        
        
    # STATE MACHINE
    async def state_machine(self):
        
        while True:
            
            # TURN ON GREEN LED WHILE IDLING ON PAD. TURN OFF LEDs AFTER LIFTOFF
            if self.state == 0:
                self.led_1.value(0)
            else:
                self.led_1.value(1)
                self.led_2.value(1)
            
            t = ticks_ms()

            self.alt_buf = np.roll(self.alt_buf,-1)
            self.vel_buf = np.roll(self.vel_buf,-1)
            self.acc_buf = np.roll(self.acc_buf,-1)
                
            self.alt_buf[-1] = self.alt
            self.vel_buf[-1] = 1000 * (self.alt_buf[-1] - self.alt_buf[-2])/(self.t_log - self.last_t_log)
            self.acc_buf[-1] = self.bno_accel[0]
            
            if self.alt > self.max_alt:
                self.max_alt = self.alt

            if (self.state != 0 and self.state != 5):
                self.tel_delay = 200
            
            # LOG DATA UNTIL RUNTIME ENDS OR TOUCHDOWN OR SPACE RUNS OUT (1kB SAFETY MARGIN)
            if (self.t_log > self.runtime or self.logging_done):# or self.free_bytes - self.size < 1000): 
                self.data_array = bytearray() # why?
                self.state = 5 # to go into recovery mode
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

            elif ((self.state==2 and self.t_log - self.t_events[0] > self.lockout_drogue_time) and (np.all(self.vel_buf < 0) or self.t_log - self.t_events[0] > self.force_drogue_time)):
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

    # LOG DATA
    async def log_data(self):
        
        while True:
        
            t = ticks_ms()
            
            # PACK FAST READINGS TO BYTES
            ustruct.pack_into(self.packing_str,
                              self.data_fast,
                              0,                      # offset 0
                              b'F',                   # F INDICATES THAT DATA IS OF FAST READINGS
                              (int)(self.t_log),
                              (int)(self.state),
                              (int)((self.temp - self.calib_temp)*self.temp_mult),
                              (float)(self.alt),
                              (float)(self.ram_diff),
                              (int)(self.bno_accel[0]*self.acc_mult),
                              (int)(self.bno_accel[1]*self.acc_mult),
                              (int)(self.bno_accel[2]*self.acc_mult),
                              (float)(3.14),
                              (float)(2.71),
                              (float)(9.81),
                              (float)(self.gyro[0]),
                              (float)(self.gyro[1]),
                              (float)(self.gyro[2]),
                              (int)(5),
                              (int)(3),
                              (int)(0),
                              (int)(0)
                              )
            
                                              
            # PACK SLOW READINGS TO BYTES
            # ONLY IF NEW DATA HAS BEEN RECEIVED AND POSITION FIX HAS OBTAINED ATLEAST ONCE
            if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                                                                
                ustruct.pack_into( self.packing_str_slow,
                                   self.data_slow,
                                   0,
                                   b'S',                     # S INDICATES THAT DATA IS OF SLOW READINGS
                                   (int)(self.latitude[0]),
                                   (int)(self.latitude[1]),
                                   (float)(self.latitude[2]),
                                   (int)(ord(self.latitude[3])),
                                   (int)(self.longitude[0]),
                                   (int)(self.longitude[1]),
                                   (float)(self.longitude[2]),
                                   (int)(ord(self.longitude[3])),
                                   (int)(self.speed*self.gps_mult),
                                   (int)(self.course*self.gps_mult),
                                   (int)(self.hdop*self.gps_mult),
                                   (int)(self.vdop*self.gps_mult),
                                   self.sentence_type,
                                   (float)(self.gps_altitude),
                                   (float)(self._fix_time - self.calib_time),
                                   (float)(self.time_since_fix)
                                   )
                
            if self.state != 1 and self.state != 5: #change before flight
                self.data_file.write(self.data_fast)
                self.size += len(self.data_fast)
                #print("written fast data")
                        
                # WRITE SLOW DATA TO FLASH ONLY IF NEW FIX IS ACQUIRED      
                if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                    self.data_file.write(self.data_slow)
                    self.size += len(self.data_slow)

            await asyncio.sleep_ms(50)        
     
             
    # COMMUNICATION BETWEEN ROCKET AND GROUND STATION   
    async def comms(self):
        
        while True:
                    
            t = ticks_ms()
            
            if self.send_calib:
                self.xbee.write(self.calib_data)
                self.send_calib = False
                
            if self.state != 5:
                self.xbee.write(self.data_fast)
            
            if self.gps_counter > self.last_gps_counter:
                sleep_ms(100)
                self.xbee.write(self.data_slow)
            
            if self.state == 5:
                self.xbee.write(self.data_slow)
                print("recovery")

            await asyncio.sleep_ms(self.tel_delay)

    # NAV FILTER
    async def nav(self):
        
        counter = 0
        
        X_A = np.full((3,1),0.)
        C1 = np.full((3,3),0.)

        X = np.full((6,1),0.)
        W = np.full((6,1),0.)  
        F = np.eye(6)
        P = np.full((6,6),0.)
        Q = np.full((6,6),0.)
        R = np.eye(6)
        Z = np.full((6,1),0.)
        U = np.full((6,1),0.)
        
        dt = 1/30
        nx = ny = 1
        init = False
        declination = -0.111*pi/180
        
        i2c_mc = I2C(0,scl=Pin(5),sda=Pin(4))
        mc_delay = 1
                
        while True:
            
            # EVEYTHING INSIDE TRY EXCEPT IN CASE IT DECIEDS TO CRASH MID-FLIGHT
            try:
                if counter > 0:
                    
                    ax = self.bno_accel[2]
                    ay = -self.bno_accel[1]
                    az = self.bno_accel[0]
                            
                    a = sqrt(ax*ax + ay*ay + az*az)

                    gx = self.gyro[2]*pi/180
                    gy = -self.gyro[1]*pi/180
                    gz = -self.gyro[0]*pi/180

                    k = 1/700
                    
                    # CALCULATE AIRSPEED
                    if k*self.ram_diff + self.delta_p(X[2][0],X[2][0] + X[3][0]*dt) >= 0:
                        airspeed = sqrt(2*(k*self.ram_diff + self.delta_p(X[2][0],X[2][0] + X[3][0]*dt))/self.density)

                    # ATTITUDE KF --------------------------------------------      

                    e = abs(a-self.g)/self.g

                    cutoff = 0.05
                    
                    # USE ACCELEROMETER TO CALCULATE PITCH AND ROLL IF ACCELERATION IS LOW AND LIFTOFF IS NOT DETECTED. ELSE GYRO.
                    if e < cutoff and not self.t_events[0]:
                        X_A[0][0] = atan2(ay,az)    
                        X_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))
                    else:
                        X_A[0][0] += nx*gx*dt
                        X_A[1][0] += ny*gy*dt

                    # IF LIFTOFF IS DETECTED THEN USE GYRO FOR YAW (SECOND CONDITION FOR INSTANT DETECTION ON IGNITION). ELSE NO YAW.
                    if self.t_events[0] or e > cutoff:
                        X_A[2][0] += gz*dt
                    else: 
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
                    if np.all(self.vel_buf < 10):
                        C1[2][2] = cos(X_A[0][0])*cos(X_A[1][0])
                    elif X[3][0] > 0:
                        C1[2][2] = X[3][0]/sqrt(pow(X[3][0],2)+pow(X[4][0],2))  # 0 AOA IN FLIGHT
                    
            # -------------------------------------------------------------------------------------------------------------------
                    
                    last_alt = Z[2][0]
                    
                    # CALCULATE GRAVITY VECTOR
                    gravity = np.dot(C1,np.array([[0],[0],[-self.g]]))

                    # CALCULATE ACCELERATION WRT GROUND
                    lin_acc_x = - ax + gravity[0][0]
                    lin_acc_y = - ay + gravity[1][0]
                    lin_acc_z = az + gravity[2][0]
                    
                    # LATITUDE AND LONGITUDE UNCERTAINITIES DEPEND ON QUALITY OF FIX (HDOP). REST FIXED FOR NOW.
                    for j in (0,1):
                        Q[j][j] = 0.01/(self.hdop + 0.001)   
                    for j in (2,3,4):
                        Q[j][j] = 0.1
                    
                    # ACCELERATIONS IN ROTATED FRAME(Z AXIS VERTICAL)
                    if np.linalg.det(C1) > pow(10,-6):
                        a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
                    else:
                        a_r = np.array([[0.],[0.],[0.]])
                    
                    dv = sqrt(pow(a_r[0][0],2) + pow(a_r[1][0],2))*dt
                    # DIRECTION OF HORIZONTAL ACCELERATION
                    alpha = atan2(-a_r[0][0],a_r[1][0]) + X_A[2][0] + declination
                    
                    if self.latitude[0] != 0 and self.longitude[0] != 0 and init == False:
                        X[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        X[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                        init = True                         
                     
                    # STM UPDATE
                    F[0][4] = cos(Z[5][0])*dt/self.r
                    
                    if abs(Z[0][0]*180/pi - 90) > 10:
                        F[1][4] = sin(Z[5][0])*dt/(self.r*cos(Z[0][0]))
                    else:
                        F[1][4] = 0
                        
                    F[2][3] = dt
                    
                    # CONTROL MATRIX UPDATE
                    U[2][0] = a_r[2][0]*dt*dt/2
                    U[3][0] = a_r[2][0]*dt
                    U[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha)) - Z[4][0]
                    U[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha))) - Z[5][0]
                    
                    Xp = np.dot(F,X) + U

                    Pp = np.dot(np.dot(F,P),F.T) + Q
                    
                    # ALTITUDE AND VERTICAL VELOCTY READINGS
                    if self.bmp_working:
                        Z[2][0] = self.alt
                        Z[3][0] = (Z[2][0] - last_alt)/dt
                    elif self.bno_working:
                        Z[2][0] += (Z[3][0] + a_r[2][0]*dt/2)*dt
                        Z[3][0] += a_r[2][0]*dt

                    # IF GPS FIX
                    if self.gps_counter > self.last_gps_counter:
                        
                        if 'GGA' in self.sentence_type or 'GLL' in self.sentence_type:
                            # UPDATE COORDINATES FROM GPS
                            if abs((self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600) - 90) > 10:
                                Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                            else:
                                Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                                
                            Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                            
                            # IF LIFTOFF
                            if self.t_events[0]:
                                # IF NEW RAM BAROMETER DATA
                                if self.new_ram_data:
                                    # IF AIRSPEED > VERTICAL VELOCITY USE IT TO CALULATE HORIZONTAL SPEED. ALL ELSE USE IMU.
                                    if airspeed > abs(Xp[3][0]):
                                        Z[4][0] = sqrt(pow(airspeed,2) - pow(Xp[3][0],2))
                                    else:
                                        Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                                else:
                                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            else:
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                                                
                            # GGA AND GLL DON'T HAVE TRACK DATA SO USE IMU
                            Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                           
                        # NOT USING GPS SPEEDS FOR FLIGHT BECAUSE NOT SURE IF THEY WILL UPDATE PROPERLY.
                        # STILL USING TRACK READINGS BECAUSE GYROS WILL PROBABLY DRIFT LIKE CRAZY ANYWAY.
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
                        
                        # NOT A REAL SENTENCE TYPE. JUST CREATED IT FOR WHEN VTG SIGNAL IS RECEIVED WITH NOT TRACK DATA(GENERALLY AT LOW SPEED).
                        # AS_GPS HAS BEEN MODIFIED TO RETURN THIS.
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
                       
                       # WHEN RMC DOESN'T HAVE TRACK READINGS
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
                    
                    # IF NO GPS FIX, USE IMU.        
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
                        
                        if abs(Z[0][0]*180/pi - 90) > 10:
                            Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                        Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r

                    if -180 < Z[5][0]*180/pi < 0:
                        Z[5][0] += 2*pi

                    K = np.dot(Pp,np.linalg.inv(Pp + R))

                    X = Xp + np.dot(K,Z - Xp)
                    
                    P = np.dot(np.eye(len(K)) - K,Pp)
                    
                    # TRACK VALUES ARE NOT NOISY ANYWAY BUT BIG JUMPS CAN MESS UP THE FILTER WHEN GPS FIX IS ACCQUIRED. SO NO NEED TO FILTER THEM.
                    X[5][0] = Z[5][0]
                    
                    # WRITE DATA TO MOTOR CONTROLLER. DOESN'T LIKE RECEIVING MORE THEN 18-19 BYTES AT A TIME SO NEED TO BREAK DOWN DATA INTO PACKETS.
                    # RAW DATA IS BACKED UP ON MOTOR CONTROLLER (MUCH SLOWER RATE THOUGH BECAUSE IT HAS TO DO AIRBRAKE CALCULATIONS).
                    try:
                        if self.send_calib:
                            i2c_mc.writeto(0x41,b'A' + self.calib_data[:16])
                            sleep_ms(mc_delay)
                            i2c_mc.writeto(0x41,b'B' + self.calib_data[16:])
                            sleep_ms(mc_delay)
                            
                        i2c_mc.writeto(0x41,b'C' + self.data_fast[:16])
                        sleep_ms(mc_delay)
                        i2c_mc.writeto(0x41,b'D' + self.data_fast[16:30])
                        sleep_ms(mc_delay)
                        i2c_mc.writeto(0x41,b'E' + self.data_fast[30:46])
                        sleep_ms(mc_delay)
                        i2c_mc.writeto(0x41,b'F' + self.data_fast[46:])
                        sleep_ms(mc_delay)
                        
                        if self.gps_counter > self.last_gps_counter:
                                                        
                            i2c_mc.writeto(0x41,b'G' + self.data_slow[:15])
                            sleep_ms(mc_delay)
                            i2c_mc.writeto(0x41,b'H' + self.data_slow[15:30])
                            sleep_ms(mc_delay)
                            i2c_mc.writeto(0x41,b'I' + self.data_slow[30:38])
                            sleep_ms(mc_delay)
                            
                        i2c_mc.writeto(0x41,b'J' + ustruct.pack('!ffff',X_A[0][0],X_A[1][0],X_A[2][0],X[0][0]))
                        sleep_ms(mc_delay)
                        i2c_mc.writeto(0x41,b'K' + ustruct.pack('!ffff',X[1][0],X[2][0],X[3][0],X[4][0]))
                        sleep_ms(mc_delay)
                        i2c_mc.writeto(0x41,b'L' + ustruct.pack('!f',X[5][0]))
                        sleep_ms(mc_delay)
                                                
                    except:
                        self.mc_missing()
                        
            except:
                self.nav_failure()
            
                dt = (ticks_ms() - self.last_loop_t)/1000
            
            counter += 1
                
                
            self.last_gps_counter = self.gps_counter        
         
            await asyncio.sleep(0)
            
#     def endgame(self, loop, context):
#         print("Loop stopped with the following error: ")
#         print(context)
#         self.async_loop.stop()
#         self.async_loop.close()
#         print("we have ended successfully")
            
    async def fast_core(self):
        
        print('fast core init')
        
        index_file = open('/flash/index.txt','r')
        index = int(index_file.read())
        index_file.close()
        
        index_file = open('/flash/index.txt','w')
        index_file.write(str(index + 1))
        index_file.close()
        
        timestamp = "_".join(list(map(str,gmtime()[:6])))
        
        self.data_file = open('/flash/data_' + timestamp + "_" + str(index) + '.bin','wb')
        
        print("starting event loop")
        
        self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.calibrate())        # GET CALIBRATION DATA
        self.async_loop.create_task(self.get_data())         # GET SENSOR DATA
        self.async_loop.create_task(self.state_machine())    # RUN THE STATE MACHINE
        self.async_loop.create_task(self.nav())              # RUN THE NAVIGATION FILTER
        self.async_loop.create_task(self.comms())            # TRANSMIT TO GROUND STATION
        self.async_loop.create_task(self.log_data())         # LOG THE DATA
        self.async_loop.create_task(self.blink(500))
        self.async_loop.run_forever()                        # REPEAT
        
        self.data_file.write(b'C' + self.calib_data)
        self.data_file.close()
        
        print("Unmounting flash")
        uos.umount("/flash")
        
        print("logging done, now sending only GPS")
        
        self.async_loop = asyncio.new_event_loop()
        self.async_loop.create_task(self.comms())
        self.async_loop.create_task(self.blink(1000))
        #self.async_loop.set_exception_handler(self.endgame)
        self.async_loop.run_forever()
        
        # BEEP AFTER TOUCHDOWN
#         while True:
#             self.beep()

if __name__ == "__main__":
    test_instance = async_test()                    # START INSTANCE OF CLASS
    test_instance.init()                            # INITIALISE SOFTWARE
    test_instance.init_board()                      # INITIALISE SENSORS
    try:
        asyncio.run(test_instance.fast_core())      # START THE FAST CORE ASYNC LOOP
    except KeyboardInterrupt as e:
        print("User has ended loop, closing asyncio")
        test_instance.async_loop.stop()
        test_instance.async_loop.close()
    finally:
        print("done")



