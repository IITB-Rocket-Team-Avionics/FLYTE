
# ------------------------------------------ as_GPS HAS BEEN MODIFIED --------------------------------------------------

from machine import I2C,Pin,UART,SPI,freq,PWM
from math import sqrt,pi,asin,atan,tan,cos,sin,atan2,acos
import time
from time import ticks_ms,sleep,sleep_us
from ulab import numpy as np
from bno055 import *
from bmp280 import *
import as_GPS
import uasyncio as asyncio
import ustruct
import winbond
import uos

freq(270000000)

class async_test:
    
    def altitude(self):
        return 4947.19 * (8.9611 - pow(self.bmp.pressure,0.190255))
    
    def rho(self,y):
        return pow(8.9611 - y/4947.19,5.2479)/(78410.439 + 287.06*self.temp - 1.86589*y)
    
    def beep(self,*args):
             
        n = 1
        f = 2000
        t = 0.1

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
            sleep(t)
            self.buzzer.duty_u16(30000)
            sleep(t)
    
    def state_is_valid(self,S):
        if self.t_events[0]:
            if self.t_log - self.t_events[0] + 0.5 <= 2:
                if abs(S[0][0]*180/pi - 90) > 10 and -5.71*pow((self.t_log - self.t_events[0]+0.5),2) + 142.09*(self.t_log - self.t_events[0]+0.5) - 200 < S[2][0] < -6.17*pow(self.t_log - self.t_events[0]+0.5,2) + 213.71*(self.t_log - self.t_events[0]+0.5) + 300 and 100*(self.t_log - self.t_events[0] + 0.5) - 80 < S[3][0] < 110*(self.t_log - self.t_events[0] + 0.5) + 100 and 0 <= S[4][0] < pow(110*(self.t_log - self.t_events[0] + 0.5) + 30,2):
                    return True
                else:
                    return False
            else:
                if abs(S[0][0]*180/pi - 90) > 10 and -5.71*pow((self.t_log - self.t_events[0]+0.5),2) + 142.09*(self.t_log - self.t_events[0]+0.5) - 200 < S[2][0] < -6.17*pow(self.t_log - self.t_events[0]+0.5,2) + 213.71*(self.t_log - self.t_events[0]+0.5) + 300 and 150 - 15*(self.t_log - self.t_events[0] + 0.5) < S[3][0] < 350 - 15*(self.t_log - self.t_events[0] + 0.5) and 0 <= S[4][0] < pow(270 - 10*(self.t_log - self.t_events[0] + 0.5),2):
                    return True
                else:
                    return False
        else:
            return True
    
    # INITIALISE BOARD    
    def init(self):
        self.freq = 40
        self.tel_delay = 2000
        self.sensor_init = False
        self.t_events = [0.,0.,0.]
        self.logging_done = False
        self.state = 0
        self.bmp, self.bno, self.xbee, self.gps, self.flash = None, None, None, None, None
        self.led = Pin(25, Pin.OUT)
        self.led.value(0)
        self.send_calib = False
        self.buzzer = PWM(Pin(6))
        self.buzzer.freq(2000)
        
        self.beep(1,0.5)
        
        self.r = 6371000                  # RADIUS OF EARTH
        self.g = 9.81                     # ACCELERATION DUE TO GRAVITY         
        self.except_occr = False
        self.t_log = ticks_ms()
        self.last_t_log = ticks_ms()
        self.last_loop_t = ticks_ms()
        self.data_array = bytearray()
        self.data_fast = bytes(64)
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
        
#         self.last_latitude = self.latitude
#         self.last_longitude = self.longitude
#         self.last_speed = self.speed
#         self.last_course = self.course
#         self.last_gps_altitude = self.gps_altitude
#         self.last_fix_time = self._fix_time
        
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
        
        self.runtime = 300 * 1000   
        self.temp_mult = 100
        self.mag_mult = 100
        self.orientation_mult = 10
        self.packing_str = '!sibhfffffffffffhhhhhh'
        self.packing_str_slow = '!sfffifffiffff3sfff'
        self.new_idx = 0
        
        self.max_alt = 0
        self.calib_gap = 1000 # Gap in milliseconds between subsequent calibrations
        self.calib_count = 0 # number of calibrations performed till now
        self.calib_max = 1 # max number of calibrations
        
        self.min_liftoff_alt = 5 # in m, minimum altitude to be cleared for liftoff
        self.touchdown_alt = 1 # in m, If altitude is less than this in descent, declare touchdown
        self.touchdown_vel_limit = 0.01
        
        self.buf_len = 25
        self.alt_buf = np.zeros((1,self.buf_len))[0]
        self.acc_buf = np.zeros((1,self.buf_len))[0]
        self.vel_buf = np.zeros((1,self.buf_len - 1))[0]
        self.apogee = np.zeros((1,self.buf_len - 1))[0]
        self.density = 1.225
        
        self.rec_size = 5
        self.rec = bytearray(self.rec_size)
        
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
            
            MAG_OFFSET_X_LSB_ADDR = const(0x5B)
            MAG_OFFSET_X_MSB_ADDR = const(0x5C)
            MAG_OFFSET_Y_LSB_ADDR = const(0x5D)
            MAG_OFFSET_Y_MSB_ADDR = const(0x5E)
            MAG_OFFSET_Z_LSB_ADDR = const(0x5F)
            MAG_OFFSET_Z_MSB_ADDR = const(0x60)
            MAG_RADIUS_LSB_ADDR = const(0x69)
            MAG_RADIUS_MSB_ADDR = const(0x6A)

            GYRO_OFFSET_X_LSB_ADDR = const(0x61)
            GYRO_OFFSET_X_MSB_ADDR = const(0x62)
            GYRO_OFFSET_Y_LSB_ADDR = const(0x63)
            GYRO_OFFSET_Y_MSB_ADDR = const(0x64)
            GYRO_OFFSET_Z_LSB_ADDR = const(0x65)
            GYRO_OFFSET_Z_MSB_ADDR = const(0x66)
            
            # SET IMU TO CONFIG MODE (TO SET ACCELEROMTER OFFSETS)
            
            self.bno.mode(0x00)

            # BYTEARRAY OF IMU OFFSETS (FOUND FROM PRIOR CALIBRATION PROCESS)            
            
            bno_offsets = bytearray(b'\xd5\xff\xdc\xff\xde\xff7\x009\xfe\x06\xee\xff\xff\xfd\xff\x00\x00\xe8\x03\x19\x02')
            
            # WRITE ACCELEROMETER OFFSETS TO THEIR REGISTERS (NOT GYRO AND MAGNETOMETERS, THEY SHOULD BE CALIBRATED ON EACH STARTUP)

            self.bno._write(ACCEL_OFFSET_X_LSB_ADDR, bno_offsets[0])
            self.bno._write(ACCEL_OFFSET_X_MSB_ADDR, bno_offsets[1])
            self.bno._write(ACCEL_OFFSET_Y_LSB_ADDR, bno_offsets[2])
            self.bno._write(ACCEL_OFFSET_Y_MSB_ADDR, bno_offsets[3])
            self.bno._write(ACCEL_OFFSET_Z_LSB_ADDR, bno_offsets[4])
            self.bno._write(ACCEL_OFFSET_Z_MSB_ADDR, bno_offsets[5])
            self.bno._write(ACCEL_RADIUS_LSB_ADDR, bno_offsets[18])
            self.bno._write(ACCEL_RADIUS_MSB_ADDR, bno_offsets[19])
            
            self.bno._write(GYRO_OFFSET_X_LSB_ADDR, bno_offsets[12])
            self.bno._write(GYRO_OFFSET_X_MSB_ADDR, bno_offsets[13])
            self.bno._write(GYRO_OFFSET_Y_LSB_ADDR, bno_offsets[14])
            self.bno._write(GYRO_OFFSET_Y_MSB_ADDR, bno_offsets[15])
            self.bno._write(GYRO_OFFSET_Z_LSB_ADDR, bno_offsets[16])
            self.bno._write(GYRO_OFFSET_Z_MSB_ADDR, bno_offsets[17])
            
            self.bno._write(MAG_OFFSET_X_LSB_ADDR, bno_offsets[6])
            self.bno._write(MAG_OFFSET_X_MSB_ADDR, bno_offsets[7])
            self.bno._write(MAG_OFFSET_Y_LSB_ADDR, bno_offsets[8])
            self.bno._write(MAG_OFFSET_Y_MSB_ADDR, bno_offsets[9])
            self.bno._write(MAG_OFFSET_Z_LSB_ADDR, bno_offsets[10])
            self.bno._write(MAG_OFFSET_Z_MSB_ADDR, bno_offsets[11])
            self.bno._write(MAG_RADIUS_LSB_ADDR, bno_offsets[20])
            self.bno._write(MAG_RADIUS_MSB_ADDR, bno_offsets[21])
                  
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
            
            self.beep(4)
            
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
            self.beep(1,5)
        else:    
            print('Init done')

    
    def fast_core_init(self):

#  -----------------------------------------------------------------------------------------------------------------------------------------------
        
        print('fast core init')
        
        file_counter = 0

        while True:
            try:
                self.data_file = open('/win/data_' + str(file_counter) + '.bin','rb')
                self.data_file.close()
            except:
                self.data_file = open('/win/data_' + str(file_counter) + '.bin','ab')
                break
            
            file_counter += 1
    
        
    
    async def calibrate(self):

        self.calib_count += 1
        
        self.beep(1,1)
        
        self.calib_time = ticks_ms()
        
        print('Calibration Number ' + str(self.calib_count))
        
        self.calib_bmp()
        self.calib_ram()
        
        # CALIBRATION DATA PACKED TO BYTES TO WRITE IN FLASH
        self.calib_data = ustruct.pack('ffiiif',
                                              self.calib_altitude,
                                              self.ram_offset,
                                              self.mag_mult,
                                              self.orientation_mult,
                                              self.temp_mult,
                                              self.calib_temp)
        self.send_calib = True

        await asyncio.sleep(0)


    async def get_data(self):   

        self.t_log = ticks_ms() - self.calib_time

        # Get sensor readings
        try:
            self.bno_accel = self.bno.accel()
            self.gyro = self.bno.gyro()
            self.mag = self.bno.mag()
            self.orientation = self.bno.euler()
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

        await asyncio.sleep(0)
        
        

    async def state_machine(self):
        
        t = ticks_ms()
        
        for index in range(1,self.buf_len):
            self.alt_buf[index - 1] = self.alt_buf[index]
            self.acc_buf[index - 1] = self.acc_buf[index]
        for index in range(1,self.buf_len - 1):
            self.vel_buf[index - 1] = self.vel_buf[index]

        if self.bmp_working:
            self.alt_buf[self.buf_len - 1] = self.alt
            self.vel_buf[self.buf_len - 2] = 1000 * (self.alt_buf[self.buf_len - 1] - self.alt_buf[self.buf_len - 2])/(self.t_log - self.last_t_log)
        else:
            if self.state == 0 or self.state == 2:
                self.alt_buf[self.buf_len - 1] = self.alt_buf[self.buf_len - 1]
                self.vel_buf[self.buf_len - 2] = 0
            elif self.state == 1:
                self.alt_buf[self.buf_len - 1] += self.vel_buf[self.buf_len - 2]*(self.t_log - self.last_t_log)
                self.vel_buf[self.buf_len - 2] += -(self.acc_buf[self.buf_len - 1] + self.g)*(self.t_log - self.last_t_log)

        self.acc_buf[self.buf_len - 1] = self.bno_accel[2]
        
        if self.alt > self.max_alt:
            self.max_alt = self.alt

        if (self.state != 0 and self.state != 2):
            self.tel_delay = 110
        
        if (self.t_log > self.runtime or self.logging_done): # Stop data logging after run_time or if touchdown is detected
            self.data_array = bytearray()
                        
        if (self.state==0 and (np.all(self.alt_buf > self.min_liftoff_alt))): #Liftoff
            self.state = 1
            self.t_events[0] = self.t_log
            print("liftoff")
            self.beep(1,0.2)

        elif (self.state==1 and np.sum(self.vel_buf)/(self.buf_len - 1)) < -3:
            self.state = 2
            self.t_events[1] = self.t_log
            print("dropped")
            self.beep(2,0.2)
                
        elif (self.state == 2 and (np.all(self.alt_buf < self.touchdown_alt) and np.sum(self.vel_buf)/(self.buf_len - 1) < self.touchdown_vel_limit)):
            self.state = 3
            self.t_events[2] = self.t_log
            self.logging_done = True
            self.tel_delay = 2000
            print("touchdown")
            self.beep(1,0.2)

        else:
            pass

        self.last_t_log = self.t_log

        await asyncio.sleep(0) 


    async def log_data(self):
    
        # PACK FAST READINGS TO BYTES
        self.data_fast = ustruct.pack(self.packing_str,
                                      b'F',                   # 0 INDICATES THAT DATA IS OF FAST READINGS
                                      self.t_log,
                                      self.state,
                                      (int)((self.temp - self.calib_temp)*self.temp_mult),
                                      (float)(self.alt),
                                      (float)(self.ram_diff),
                                      (float)(self.bno_accel[0]),
                                      (float)(self.bno_accel[1]),
                                      (float)(self.bno_accel[2]),
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
                
        # PACK SLOW READINGS TO BYTES
        # ONLY IF NEW DATA HAS BEEN RECEIVED AND POSITION FIX HAS OBTAINED ATLEAST ONCE
        if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
            self.data_slow = ustruct.pack( self.packing_str_slow,
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
        

        if (len(self.data_array) < 512):
            self.data_array += self.data_fast
        else:
            self.data_file.write(self.data_array)
            self.data_array = bytearray()
            self.data_array += self.data_fast
                        
        # WRITE SLOW DATA TO FLASH ONLY IF NEW FIX IS ACQUIRED      
        if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
            if (len(self.data_array) < 512):
                self.data_array += self.data_slow
            else:
                self.data_file.write(self.data_array)
                self.data_array = bytearray()
                self.data_array += self.data_slow


        await asyncio.sleep(0)        


        
    # COMMUNICATION BETWEEN ROCKET AND GROUND STATION   
    async def comms(self):
        
        t = ticks_ms()
        
        if self.xbee.any() >= self.rec_size:
            uart.readinto(self.rec)
            data = str(self.rec)
            print(data)
        else:     
            if self.send_calib:
                self.xbee.write(self.calib_data)
                self.send_calib = False
                
            if self.state != 5:
                self.xbee.write(self.data_fast)
            
            if self.gps_counter > self.last_gps_counter:
                sleep(0.01)
                self.xbee.write(self.data_slow)

        await asyncio.sleep_ms(0)
        

    # START FAST CORE
    async def fast_readings(self):
        
        ran_once = False
        
        while True:
            
            self.last_loop_t = ticks_ms()
            
            loop = asyncio.get_event_loop()
            
            if (self.state == 0):
                if (self.t_log > self.calib_gap and self.calib_count < self.calib_max):
                    loop.create_task(self.calibrate())
            
            loop.create_task(self.get_data())
            loop.create_task(self.log_data())
            loop.create_task(self.comms())

            if ran_once == False:
                ran_once = True    
            elif (self.t_log > self.runtime or self.logging_done) and self.calib_count >= self.calib_max:
                self.data_file.close()
                while True:
                    self.beep()
            
            loop.run_until_complete(self.state_machine())
            
            print(ticks_ms() - self.last_loop_t)
 

test_instance = async_test()                    # START INSTANCE OF CLASS
test_instance.init()                            # INITIALISE SOFTWARE
test_instance.init_board()                      # INITIALISE SENSORS
test_instance.fast_core_init()
asyncio.run(test_instance.fast_readings())      # START FAST CORE


