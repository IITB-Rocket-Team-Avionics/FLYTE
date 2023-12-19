
# ------------------------------------------ as_GPS HAS BEEN MODIFIED --------------------------------------------------

from machine import I2C,Pin,UART,SPI
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
import sdcard

class async_test:
    
    # ALTITUDE FROM PRESSURE
    def altitude(self):
        return 4947.19 * (pow(101325,0.190255) - pow(self.bmp.pressure,0.190255))
    
    # HORIZONTAL DISTANCE B/W TWO GPS COORDINATES
    def distance(self,lat_i,lon_i,lat_f,lon_f):
        
        lat_1 = lat_i[0] + lat_i[1]/60 + lat_i[2]/3600
        lon_1 = lon_i[0] + lon_i[1]/60 + lon_i[2]/3600
        lat_2 = lat_f[0] + lat_f[1]/60 + lat_f[2]/3600
        lon_2 = lon_f[0] + lon_f[1]/60 + lon_f[2]/3600
        
        return sqrt(pow((self.r*(lat_2 - lat_1)*pi/180),2) + pow(self.r*(lon_2 - lon_1)*pi/180*cos(lat_1*pi/180),2))
    
    # COURSE B/W TWO GPS COORDINATES    
    def track(self,lat_i,lon_i,lat_f,lon_f):
        
        lat_1 = lat_i[0] + lat_i[1]/60 + lat_i[2]/3600
        lon_1 = lon_i[0] + lon_i[1]/60 + lon_i[2]/3600
        lat_2 = lat_f[0] + lat_f[1]/60 + lat_f[2]/3600
        lon_2 = lon_f[0] + lon_f[1]/60 + lon_f[2]/3600
        
        if lat_i != lat_f:
            return atan(cos(lat_1)*(lon_2 - lon_1)/(lat_2 - lat_1))
        else:
            return pi/2
    
    # INITIALISE BOARD    
    def init(self,logging_frequency = 10000):
        self.freq = logging_frequency
        self.sensor_init = False
        self.t_events = []
        self.logging_done = False
        self.state = 0
        self.i2c_bmp, self.i2c_bno, self.uart ,self.spi_flash = None, None, None, None
        self.SD_CS, self.flash_CS = None, None
        self.bmp, self.mpu, self.sd, self.sx, self.gps, self.flash = None, None, None, None, None, None
        self.led = Pin(25, Pin.OUT)
        self.led.value(0)
#         self.buzzer = PWM(Pin(6))
#         self.buzzer.freq(2000)
#         self.buzzer.duty_u16(0)]
        self.r = 6371000                  # RADIUS OF EARTH
        self.g = 9.81                     # ACCELERATION DUE TO GRAVITY         
        self.except_occr = False
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
        self.sentence_type = 'NUL'
        
        self.last_latitude = self.latitude
        self.last_longitude = self.longitude
        self.last_speed = self.speed
        self.last_course = self.course
        self.last_gps_altitude = self.gps_altitude
        self.last_fix_time = self._fix_time
        
        # RAW READINGS FROM FAST SENSORS
        self.bno_accel = [0,0,0]
        self.high_accel = [0,0,0]
        self.gyro = [0,0,0]
        self.mag = [0,0,0]
        self.orientation = [0,0,0]
        self.temp = 0
        self.alt = 0
        self.ram_diff = 0               # RESTRICTED TO 10 HZ. DATASHEET EXPLAINS HOW TO INCREASE TO 40 HZ BUT I CAN'T GET IT TO WORK
        
        self.calib_time, self.calib_alt, self.calib_temp ,self.ram_offset = 0,0,0,0
        self.shutdown = False
        self.calib_data = bytes(24)
        
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

        self.last_latitude = self.latitude
        self.last_longitude = self.longitude
        self.last_speed = self.speed
        self.last_course = self.course
        self.last_gps_altitude = self.gps_altitude
        self.last_fix_time = self._fix_time
    
    
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
        self.calib_time = time.ticks_ms()
        self.calib_temp = self.bmp.temperature
        avg_alt = 0
        for i in range(n):
            avg_alt += self.altitude()
            time.sleep_ms(10)
        self.calib_altitude = avg_alt/n
        
    # CALIBRATE RAM BAROMETER OFFSET
    def calib_ram(self,n = 5):
        self.ram_offset = 0
        count = 0
        while count < n:
            if self.ram_dout.value() == 0:
                self.ram_offset += self.ram_raw_data()/n
                count += 1
    
    # INITIALISE BOARD
    def init_board(self): #Initiate BMP, MPU, SD card, and flash
        print("starting init")
        
        # I2C BUS OF STATIC BAROMETER
        self.i2c_bmp = I2C(0,
                scl = Pin(17),
                sda = Pin(16),
                freq = 400000)
        
        # I2C BUS OF IMU
        self.i2c_bno = I2C(1,
                scl = Pin(15),
                sda = Pin(14),
                freq = 400000)
        
        # SPI BUS OF FLASH
        self.spi_flash = SPI(1,
                    sck= Pin(10),
                    mosi=Pin(11),
                    miso=Pin(12))
        
        # SPI BUS OF SD CARD        
        self.spi_sd = SPI(0,
            sck= Pin(2),
            mosi=Pin(3),
            miso=Pin(4))
        
        
        self.SD_CS = Pin(5)          # CHIP SELECT PIN OF SD CARD
        self.flash_CS = Pin(13)      # CHIP SELECT PIN OF FLASH
        
        # UART BUS OF GPS
        self.uart = UART(0, 9600, rx = Pin(1), tx = Pin(0), timeout=1000, timeout_char=1000) # decide on timeouts
        
        # RAM BAROMETER PINS        
        self.ram_sck = Pin(6,Pin.OUT)
        self.ram_dout = Pin(7,Pin.IN)
        
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
        
        # INITIALISE STATIC BAROMTER
        try:
            self.bmp = BMP280(i2c_bus = self.i2c_bmp)
            print('bmp done')
        except Exception as e:
            print('Failed to initialize BMP280')
            print(e)
            self.except_occr = True
            #Logger.log('ERROR init BMP280: ', e)
         
        # INITIIALISE IMU 
        try:
            self.bno = BNO055(self.i2c_bno)
            
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
            
            bno_offsets = bytearray(b'\xd9\xff\x13\x00\xfb\xff\x83\x05\x88\x08.\x16\xfe\xff\x01\x00\x02\x00\xe8\x03;\x03')         

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
            
            while not self.bno.calibrated():
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

        # INITIALISE FLASH
        try:
            self.flash = winbond.W25QFlash(self.spi_flash, self.flash_CS)
            uos.mount(self.flash, '/win')
            print('flash done')
        except Exception as e: #OSError
            print('Failed to initialize Flash')
            print(e)
            self.except_occr = True

        # INITIALISE SD CARD
        try:
            self.sd = sdcard.SDCard(self.spi_sd, self.SD_CS)
            uos.mount(self.sd, "/sd")
            print('sd done')
        except Exception as e: #OSError
            print('Failed to initialize SD Card Reader')
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
        
#       # FLASH LED AND PRINT ERROR
        if (self.except_occr):
            for i in range(10): # Initialization failed
                self.led.on()
                time.sleep(0.1)
                self.led.off()
                time.sleep(0.1)
        else:    
            print('Init done')
#             for i in range(3):
#                 self.led.on()
#                 time.sleep(0.5)
#                 self.led.off()
#                 time.sleep(0.5)
    
    # START FAST CORE
    async def fast_readings(self):
        
        #State machine definitions-----------------------------------------------------------------------------------------
        
         
        max_alt = 0
        calib_gap = 30000 # Gap in milliseconds between subsequent calibrations
        calib_count = 0 # number of calibrations performed till now
        calib_max = 0 # max number of calibrations
        
        liftoff_accel = 3 # in g, minimum sustained acceleration for liftoff
        liftoff_alt = 15 # in m, minimum altitude to be cleared for liftoff
        liftoff_to_cutoff = 4500 # In milliseconds, force cutoff after this amount of time from detected liftoff
        apogee_alt_diff = 5 # in m, the minimum difference between highest recorded altitude and current altitude for apogee detection
        liftoff_to_drogue = 27000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        apogee_to_drogue = 100 # In milliseconds, drogue deployment after this amount of time from detected apogee
        drogue_lockout = 20000 # In milliseconds, lockout for drogue deployment from detected liftoff
        touchdown_alt = 15 # in m, If altitude is less than this in descent, declare touchdown
        touchdown_to_idle = 10000 # In milliseconds, declare idle after this amount of time from detected touchdown
        
        buf_len = 20 # assuming the loop runs at 40Hz
        alt_buf = [0]*buf_len
        accel_buf = [0]*buf_len
        index = 0
        inc_num = 0 # stores how many readings are greater than the previous reading chronologically, resets to 0 data isn't increasing
        zero_crossing = False # stores whether a zero crossing has occured

#  -----------------------------------------------------------------------------------------------------------------------------------------------
        
        temp_mult = 100
        mag_mult = 100
        orientation_mult = 10
        packing_str = '!bibhfffffffffffhhhhhh'
        packing_str_slow = '!bfffifffiffff3sfff'
        
        ram_diff = 0
        
        # INITIALISE DATA FILE IN SD CARD
        index_file = open('/sd/index.txt', 'r')
        idx = index_file.readline()
        new_idx = str(int(idx) + 1)
        index_file.close()

        index_file = open('/sd/index.txt', 'w')
        index_file.write(new_idx)
        index_file.close()
        
        # UNMOUNT SD CARD AS IT WILL LOSE CONTACT IN FLIGHT
        
        uos.umount("/sd")
        
        #--------------------------------------------------------------------------------------------------------------------
        
#         START NAV FILTER IN PARALLEL (TURNED OFF CURRENTLY)
#         asyncio.create_task(self.nav())
        
        print('First Calibration...')
        for i in range(2): # Means initial calibration is happening
            self.led.on()
            time.sleep(0.2)
            self.led.off()
            time.sleep(0.2)
        
        self.calib_bmp()
        self.calib_ram()
        
        # CALIBRATION DATA PACKED TO BYTES TO WRITE IN FLASH
        self.calib_data = ustruct.pack('ffiiif',
                                              self.calib_altitude,
                                              self.ram_offset,
                                              mag_mult,
                                              orientation_mult,
                                              temp_mult,
                                              self.calib_temp)
#         self.send_calib = True
        
        # INITIALISE DATA FILE IN FLASH
        data_file = open('/win/data_' + new_idx + '.bin', 'wb')
        
        runtime = 0.1 * 60 * 1000 # in millisecs, changed to 5h so that transmission occurs for a long time
        buzz_counter = 0
#         self.deltaT_trans = 2000 # Start logging very slow
#         self.sensor_init = True # Start lora and sd card thread
        
        
        while True:            
            t = time.ticks_ms()
            t_log = t - self.calib_time
    
            # Get sensor readings
            self.bno_accel = self.bno.accel()
#           READ HIGH-G ACCELEROMETER HERE
            self.gyro = self.bno.gyro()
            self.mag = self.bno.mag()
            self.orientation = self.bno.euler()
            self.temp = self.bmp.temperature
            self.alt = self.altitude() - self.calib_altitude
            # READ RAM BAROMETER ONLY IF DATA IS READY
            if self.ram_dout.value() == 0:
                self.ram_diff = self.ram_raw_data() - self.ram_offset

#           STATE MACHINE STUFF

            alt_buf[index] = self.alt # alt_buf stores calibrated values
            accel_buf[index] = self.bno_accel[2]

            if self.alt > max_alt:
                max_alt = self.alt

            if self.state == 0: # to check whether altitude is MI
                if(alt_buf[index] > alt_buf[(index+buf_len-1)%buf_len]):
                    inc_num += 1
                else:
                    inc_num = 0
            elif self.state == 1:
                zero_crossing = (accel_buf[index]*accel_buf[(index+buf_len-1)%buf_len]) < 0


            index = (index+1)%buf_len
                    
                    
            # PACK FAST READINGS TO BYTES
            self.data_fast = ustruct.pack(packing_str,
                                          0,                   # 0 INDICATES THAT DATA IS OF FAST READINGS
                                          t_log,
                                          self.state,
                                          (int)((self.temp - self.calib_temp)*temp_mult),
                                          (float)(self.alt),
                                          (float)(self.ram_diff),
                                          (float)(self.bno_accel[0]),
                                          (float)(self.bno_accel[1]),
                                          (float)(self.bno_accel[2]),
#                                           (float)(high_accel[0]),
#                                           (float)(high_accel[1]),
#                                           (float)(high_accel[2]),
                                          3.14,
                                          2.71,
                                          9.81,
                                          (float)(self.gyro[0]),
                                          (float)(self.gyro[1]),
                                          (float)(self.gyro[2]),
                                          (int)(self.mag[0]*mag_mult),
                                          (int)(self.mag[1]*mag_mult),
                                          (int)(self.mag[2]*mag_mult),
                                          (int)(self.orientation[0]*orientation_mult),
                                          (int)(self.orientation[1]*orientation_mult),
                                          (int)(self.orientation[2]*orientation_mult)
                                          )
            
            # PACK SLOW READINGS TO BYTES
            # ONLY IF NEW DATA HAS BEEN RECEIVED AND POSITION FIX HAS OBTAINED ATLEAST ONCE
            if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                self.data_slow = ustruct.pack( packing_str_slow,
                                               1,                     # 1 INDICATES THAT DATA IS OF SLOW READINGS
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
            
            
            # DATA MUST BE WRITTEN TO FLASH IN PACKETS OF 512 BYTES. IF SIZE OF PACKET IS LESS THA 512, APPEND DATA TO THE PACKET.
            # ELSE WRITE PACKET TO FLASH AND RESET THE PACKET.
            
            if (len(self.data_array) < 512):  
                self.data_array += self.data_fast
            else:
                data_file.write(self.data_array)
                self.data_array = bytearray()
                self.data_array += self.data_fast
                
            # WRITE SLOW DATA TO FLASH ONLY IF NEW FIX IS ACQUIRED TO AVOID DUPLICATE DATA AND SAVE SPACE      
            if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
                if (len(self.data_array) < 512):
                    self.data_array += self.data_slow
                else:
                    data_file.write(self.data_array)
                    self.data_array = bytearray()
                    self.data_array += self.data_slow
            
# --------------------------------------------------------------------------------------------------------------------------------------------            

               
            # CALIBRATE REGULARLY
            if (self.state == 0):
                if (t_log > calib_gap and calib_count < calib_max):
                    self.led.off()
                    print('Calibrating...')
                    for i in range(2): # Means subsequent calibration is happening
                        self.led.on()
                        time.sleep(0.2)
                        self.led.off()
                        time.sleep(0.2)
                    self.calib_bmp()
                    self.calib_ram()
#                     alt_buf = [0]*buf_len
#                     accel_buf = [0]*buf_len
                    self.calib_data = ustruct.pack('ffiiif',
                                              self.calib_altitude,
                                              self.ram_offset,
                                              mag_mult,
                                              orientation_mult,
                                              temp_mult,
                                              self.calib_temp)
#                     self.send_calib = True
                    self.data_array = bytearray()
                    calib_count += 1
                    continue
#                 else:
#                     if(buzz_counter == 4):
#                         self.buzzer.duty_u16(0)
#                     elif(buzz_counter == 79):
#                         self.buzzer.duty_u16(30000)
#                     else:
#                         pass
#                     buzz_counter = (buzz_counter + 1)%80
           #------------------------------------------------------------------------

           #State: 0,1,2,3,4,5,6,7
           #Idle, Powered Ascent, Coasting, Descent, Drogue Out, Main out, Touchdown, TD Idle
            
#             if (self.state != 0 and self.state != 7):
#                 self.deltaT_trans = 250
            
            
            # DO THIS UNTIL RUNTIME ENDS
            if (t_log > runtime or self.logging_done): # Stop data logging after run_time or if touchdown is detected
                self.data_array = bytearray()
                break

#             if (self.state==0 and abs(sum(accel_buf))>abs(buf_len*liftoff_accel) and alt_buf[index]>liftoff_alt and inc_num>=buf_len): #Liftoff
#                 self.state = 1
#                 self.t_events[0] = t_log
#                 print("Liftoff")
# 
#             elif (self.state==1 and (zero_crossing or t_log - self.t_events[0] > liftoff_to_cutoff)): #Burnout
#                 self.state = 2
#                 self.t_events[1] = t_log
#                 print("Burnout")
# 
#             elif (self.state==2 and max_alt-alt_buf[index]>apogee_alt_diff and max_alt-alt_buf[((index-1)%buf_len)]>apogee_alt_diff): #Apogee
#                 self.state = 3
#                 self.t_events[2] = t_log
#                 print("Apogee")
# 
#             elif ((self.state==3 and t_log-self.t_events[2]>apogee_to_drogue and t_log-self.t_events[0]>drogue_lockout)
#                   or (self.state==2 and t_log-self.t_events[0]>liftoff_to_drogue)): #Chute, need to add condition for signals sent by ground station
#                 self.state = 4
#                 self.t_events[3] = t_log
#                 print("drogue")
#                 for i in range(3):
#                     self.drogue.value(1)
#                     time.sleep_ms(150)
#                     self.drogue.value(0)
#                     time.sleep_ms(50)
#                     
#             elif (self.state == 4 and alt < 420):
#                 self.state = 5
#                 self.t_events[4] = t_log
#                 print("main")
#                 for i in range(3):
#                     self.main.value(1)
#                     time.sleep_ms(150)
#                     self.main.value(0)
#                     time.sleep_ms(50)
#                  
#             elif (self.state==5 and sum(alt_buf) < buf_len*touchdown_alt): #Touchdown, is g vector sum required?
#                 self.state = 6
#                 self.t_events[5] = t_log
#                 print("Touchdown")
# 
#             elif (self.state==6 and t_log-self.t_events[5]>touchdown_to_idle): #Touchdown idle
#                 self.state = 7
#                 self.t_events[6] = t_log
#                 self.logging_done = True
#                 self.deltaT_trans = 2000
#                 print("Touchdown idle")
#                 
#             else:
#                 pass
            
            
            self.last_gps_counter = self.gps_counter
            
            # KEEP THIS TO MAKE SURE OTHER ASYNC FUNCTIONS GET A TURN
            await asyncio.sleep_ms(0)
            
            
            
        print("Logging finished. Transferring to SD card now")
        data_file.close()
        
        
        
        transfer_to_SD = True
        try: # try to re-mount the SD card as it has probably lost contact with the reader upon launch
            self.sd = sdcard.SDCard(self.spi_sd, self.SD_CS)
            uos.mount(self.sd,"/sd")
            sd_file = open('/sd/data_' + new_idx + '.txt', 'w')
        except:
            transfer_to_SD = False
            print("Failed to mount sd card. Saving config in flash memory")
            config_file = open('/win/config_' + new_idx + '.txt','w')
#             config_file.write("(state, time, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, altitude, 45 lmao)\n")
            config_file.write(f"Calibration details- Time: {self.calib_time} Alt: {self.calib_altitude} Temp: {self.calib_temp} Ram: {self.ram_offset}\n")
#             config_file.write("Events tracked: Liftoff, Burnout, Apogee, Chute release, Touchdown, Data shutdown\n")
#             config_file.write(f"Event timestamps: {self.t_events}\n")
#             config_file.write(f"Maximum altitude reached: {max_alt}\n")
            config_file.close()
            
            
        if(transfer_to_SD == True):
            
#             for i in range(4): # Means storing to SD card now
#                 self.buzzer.duty_u16(30000)
#                 time.sleep(0.2)
#                 self.buzzer.duty_u16(0)
#                 time.sleep(0.1)
            
            sd_file.write("(data_type, time, state, temperature, alt, ram_diff, bno_x, bno_y, bno_z, high_x, high_y, high_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, heading, roll, pitch)\n")
            read_file = open('/win/data_' + new_idx + '.bin', 'rb')
            read_file.seek(0,2) # Move to end of file
            file_size = read_file.tell() # Get number of bytes
            read_file.seek(0,0) # Move to start of file
            while(file_size - read_file.tell()>0): # IF FILE IS NOT EMPTY
                
                # READ FIRST BYTE (0 - FAST READINGS, 1 - SLOW READINGS)
                
                data_type = read_file.read(1)
                
                if data_type == b'\x00':
                    flash_data = ustruct.unpack(packing_str, data_type + read_file.read(63))
                    sd_file.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(flash_data[0],
                                                                           flash_data[1],
                                                                           flash_data[2],
                                                                           (flash_data[3]/temp_mult) + self.calib_temp,
                                                                           flash_data[4],
                                                                           flash_data[5],
                                                                           flash_data[6],
                                                                           flash_data[7],
                                                                           flash_data[8],
                                                                           flash_data[9],
                                                                           flash_data[10],
                                                                           flash_data[11],
                                                                           flash_data[12],
                                                                           flash_data[13],
                                                                           flash_data[14],
                                                                           flash_data[15]/mag_mult,
                                                                           flash_data[16]/mag_mult,
                                                                           flash_data[17]/mag_mult,
                                                                           flash_data[18]/orientation_mult,
                                                                           flash_data[19]/orientation_mult,
                                                                           flash_data[20]/orientation_mult
                                                                           ))
                
                elif data_type == b'\x01':
                    flash_data = ustruct.unpack(packing_str_slow, data_type + read_file.read(63))
                    sd_file.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(flash_data[0],
                                                                                                    flash_data[1],
                                                                                                    flash_data[2],
                                                                                                    flash_data[3],
                                                                                                    chr(flash_data[4]),
                                                                                                    flash_data[5],
                                                                                                    flash_data[6],
                                                                                                    flash_data[7],
                                                                                                    chr(flash_data[8]),
                                                                                                    flash_data[9],
                                                                                                    flash_data[10],
                                                                                                    flash_data[11],
                                                                                                    flash_data[12],
                                                                                                    flash_data[13],
                                                                                                    flash_data[14],
                                                                                                    flash_data[15],
                                                                                                    flash_data[16]
                                                                                                    ))
                    
                else:
                    print('FORMATTING ERROR' + '\n' + '!!! SUFFER !!!')
                    
            sd_file.write(f"Calibration details- Time: {self.calib_time} Alt: {self.calib_altitude} Temp: {self.calib_temp} Ram: {self.ram_offset}\n")
#             sd_file.write("Events tracked: Liftoff, Burnout, Apogee, Drogue release, Main release, Touchdown, Data shutdown\n")
#             sd_file.write(f"Event timestamps: {self.t_events}\n")
#             sd_file.write(f"Maximum altitude reached: {max_alt}\n")
            sd_file.close()
            
        print("End fast core")
        
#         # Indicates end of logging
#         while(not(self.shutdown)):
#             self.led.on()
#             time.sleep_ms(1000)
#             self.led.off()
#             time.sleep_ms(1000)
#         pass
 
# -----------------------------------------------------------------------------------------------------------------------
 
 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ----------------------------------------------------
# 
#     def nav(self):
#         
#         X_A = np.full((3,1),0.)
#         W_A = np.full((3,1),0.)  
#         F_A = np.full((3,3),0.)
#         P_A = np.full((3,3),0.)
#         Q_A = np.full((3,3),0.)
#         R_A = np.full((3,3),0.)
#         Z_A = np.full((3,1),0.)
#         C = C1 = np.full((3,3),0.)
#         U_A = np.full((3,1),0.)
# 
#         R_A = 0.007*np.eye(len(R_A))
#         Q_A = 0.0005*np.eye(len(Q_A))
# 
#         X = np.full((6,1),0.)
#         W = np.full((6,1),0.)  
#         F = np.full((6,6),0.)
#         P = np.full((6,6),0.)
#         Q = np.full((6,6),0.)
#         R = np.full((6,6),0.)
#         Z = np.full((6,1),0.)
#         U = np.full((6,1),0.)
# 
#         R = 0.007*np.eye(len(R))
#         Q = 0.005*np.eye(len(Q))
#         
#         vx = vy = vz = 0
# 
#         v_r = a_r = np.array([[0],[0],[0]])
#         
#         dt = 1/self.freq
#         n = 1
#         init = False
#         declination = -0.2*pi/180
#         
#         while True:
#     
#             t = ticks_ms()
#         
#             last_heading = Z_A[2][0]
# 
#             ax = -self.bno_accel[0]
#             ay = -self.bno_accel[1]
#             az = -self.bno_accel[2]
#             
#             a = sqrt(ax*ax + ay*ay + az*az)
# 
#             gx = self.gyro[0]*pi/180
#             gy = self.gyro[1]*pi/180
#             gz = self.gyro[2]*pi/180
#             
#             mx = self.mag[0]
#             my = self.mag[1]
#             mz = self.mag[2]
#             
#         # ATTITUDE KF --------------------------------------------      
#             
#             F_A = np.eye(len(F_A))
# 
#             U_A[0][0] = gx*dt    
#             U_A[1][0] = n*gy*dt
#             
#             e = abs(a-self.g)/self.g
#             
#             if e < 0.05:
#                 Z_A[0][0] = atan2(ay,az)    
#                 Z_A[1][0] = atan2(-ax,sqrt(ay*ay + az*az))
#                 Q_A = 0.0005*np.eye(len(Q_A))
#             else:
#                 Q_A = 0.001*np.eye(len(Q_A))
# 
#             C[0][0] = cos(Z_A[1][0])
#             C[0][1] = 0
#             C[0][2] = sin(Z_A[1][0])
#             
#             C[1][0] = sin(Z_A[0][0])*sin(Z_A[1][0])
#             C[1][1] = cos(Z_A[0][0])
#             C[1][2] = -sin(Z_A[0][0])*cos(Z_A[1][0])
#             
#             C[2][0] = -sin(Z_A[1][0])*cos(Z_A[0][0])
#             C[2][1] = sin(Z_A[0][0])
#             C[2][2] = cos(Z_A[0][0])*cos(Z_A[1][0])
#             
#             b = np.dot(C,np.array([[mx],[my],[mz]]))
#             
#             if atan2(b[0][0],b[1][0]) >= 0:
#                 Z_A[2][0] = atan2(b[0][0],b[1][0])
#             else:
#                 Z_A[2][0] = 2*pi + atan2(b[0][0],b[1][0])
#             
#             if Z_A[2][0] - last_heading > 2*pi - 0.5:
#                 U_A[2][0] = gz*dt + 2*pi
#             if Z_A[2][0] - last_heading < -2*pi + 0.5:
#                 U_A[2][0] = gz*dt - 2*pi
#             if abs(Z_A[2][0] - last_heading) < 2*pi - 0.5:
#                 U_A[2][0] = gz*dt
#             
#             if not init:
#                 X_A[0][0] = Z_A[0][0]
#                 X_A[1][0] = Z_A[1][0]
#                 X_A[2][0] = Z_A[2][0]
#                 init = True
#             
#             Xp_A = np.dot(F_A,X_A) + U_A + W_A
#             
#             Pp_A = np.dot(np.dot(F_A,P_A),F_A.T) + Q_A
#             
#             K_A = np.dot(Pp_A,np.linalg.inv(Pp_A + R_A))
#             
#             if e >= 0.05:
#                 K_A[0][0] = K_A[1][1] = 0
#             
#             X_A = Xp_A + np.dot(K_A,Z_A - Xp_A)
#             
#             P_A = np.dot(np.eye(len(K_A)) - K_A,Pp_A)
#             
#             if abs(X_A[1][0] + n*gy*dt) >= pi/2:
#                 n *= -1
#                
#             inc_raw = atan(sqrt(tan(Z_A[0][0])*tan(Z_A[0][0]) + tan(Z_A[1][0])*tan(Z_A[1][0])))
#             inc_kalman = atan(sqrt(tan(X_A[0][0])*tan(X_A[0][0]) + tan(X_A[1][0])*tan(X_A[1][0])))
# #             inc_bno = atan(sqrt(tan(roll)*tan(roll) + tan(pitch)*tan(pitch)))
# 
#             C1[0][0] = cos(X_A[1][0])
#             C1[0][1] = 0
#             C1[0][2] = sin(X_A[1][0])
#             
#             C1[1][0] = sin(X_A[0][0])*sin(X_A[1][0])
#             C1[1][1] = cos(X_A[0][0])
#             C1[1][2] = -sin(X_A[0][0])*cos(X_A[1][0])
#             
#             C1[2][0] = -sin(X_A[1][0])*cos(X_A[0][0])
#             C1[2][1] = sin(X_A[0][0])
#             C1[2][2] = cos(X_A[0][0])*cos(X_A[1][0])
# 
# # ----------------------------------------------------------------------------------------------------------------------
# 
# # POSITION KF ---------------------------------------------------------------------------------------------------------
#             
#             last_alt = Z[2][0]
#             
#             gravity = np.dot(C1,np.array([[0],[0],[-self.g]]))
# 
#             lin_acc_x = - ax + gravity[0][0]
#             lin_acc_y = - ay + gravity[1][0]
#             lin_acc_z = az + gravity[2][0]
#             
#             vx += lin_acc_x*dt
#             vy += lin_acc_y*dt
#             vz += lin_acc_z*dt
#             
#             v = sqrt(vx*vx + vy*vy + vz*vz)
#             
#             v_r_last = v_r
#             
#             v_r = np.dot(np.linalg.inv(C1),np.array([[vx],[vy],[vz]]))
#             a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
#             
#             dv = sqrt(a_r[0][0]*a_r[0][0] + a_r[1][0]*a_r[1][0])*dt
#             alpha = atan2(-a_r[0][0],a_r[1][0]) + X_A[2][0] + declination
#             
#             if self.latitude != 0 and self.last_latitude == 0:
#                 X[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
#                 X[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
#             
#             F = np.eye(len(F))
#             
#             F[0][4] = cos(X[5][0])*dt/self.r
#             F[1][4] = sin(X[5][0])*dt/(self.r*cos(X[0][0]))
#             F[2][3] = dt
#             
#             U[3][0] = a_r[2][0]*dt
#             U[4][0] = sqrt(X[4][0]*X[4][0] + dv*dv + 2*X[4][0]*dv*cos(X[5][0] - alpha)) - X[4][0]
#             U[5][0] = atan((X[4][0]*sin(X[5][0]) + dv*sin(alpha))/((X[4][0]*cos(X[5][0]) + dv*cos(alpha)))) - X[5][0]
#             
#             Xp = np.dot(F,X) + U + W
#             
#             Pp = np.dot(np.dot(F,P),F.T) + Q
#             
# #             print(self.gps_counter,self.last_gps_counter)
#             
#             if self.gps_counter > self.last_gps_counter:
#                 if self.sentence_type == 'GGA' or self.sentence_type == 'GLL':
#                     Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
#                     Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
#                     Z[4][0] = sqrt(X[4][0]*X[4][0] + dv*dv + 2*X[4][0]*dv*cos(X[5][0] - alpha))
#                     Z[5][0] = atan((X[4][0]*sin(X[5][0]) + dv*sin(alpha))/((X[4][0]*cos(X[5][0]) + dv*cos(alpha))))
#                     
#                 elif self.sentence_type == 'VTG':
#                     Z[4][0] = self.speed
#                     Z[5][0] = self.course*pi/180
#                     Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
#                     Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
#                 
#                 elif self.sentence_type == 'VNC':
#                     Z[4][0] = self.speed
#                     Z[5][0] = atan((X[4][0]*sin(X[5][0]) + dv*sin(alpha))/((X[4][0]*cos(X[5][0]) + dv*cos(alpha))))
#                     Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
#                     Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
#                     
#                 elif self.sentence_type == 'RMC':
#                     Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
#                     Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
#                     Z[4][0] = self.speed
#                     Z[5][0] = self.course*pi/180
#                 
#                 elif self.sentence_type == 'ROV':
#                     Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
#                     Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
#                     Z[4][0] = self.speed
#                     Z[5][0] =  atan((X[4][0]*sin(X[5][0]) + dv*sin(alpha))/((X[4][0]*cos(X[5][0]) + dv*cos(alpha))))
#                     
#             else:
#                 
#                 Z[4][0] = sqrt(X[4][0]*X[4][0] + dv*dv + 2*X[4][0]*dv*cos(X[5][0] - alpha))
#                 Z[5][0] = atan((X[4][0]*sin(X[5][0]) + dv*sin(alpha))/((X[4][0]*cos(X[5][0]) + dv*cos(alpha))))
#                 
#                 Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
#                 Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
#                 
#             Z[2][0] = self.alt
#             Z[3][0] = (Z[2][0] - last_alt)/dt   
#                 
#             
#             K = np.dot(Pp,np.linalg.inv(Pp + R))
#             
#             X = Xp + np.dot(K,Z - Xp)
#             
#             P = np.dot(np.eye(len(K)) - K,Pp)
#             
# #             print(((atan2(-v_r[0][0],v_r[1][0]) + X_A[2][0] + declination)*180/pi)%360,X[4][0])
# #             print(X[4][0],self.speed)
# 
#             dt = (ticks_ms() - t)/1000
#         
#             await asyncio.sleep_ms(0)
# 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ---------------------------------------------------


test_instance = async_test()                    # START INSTANCE OF CLASS
test_instance.init()                            # INITIALISE SOFTWARE
test_instance.init_board()                      # INITIALISE SENSORS
asyncio.run(test_instance.fast_readings())      # START FAST CORE

 