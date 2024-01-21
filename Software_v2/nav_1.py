
# ------------------------------------------ as_GPS HAS BEEN MODIFIED --------------------------------------------------

from machine import I2C,Pin,UART,SPI,freq
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

freq(270000000)

class async_test:
    
    def altitude(self):
        return 4947.19 * (pow(101325,0.190255) - pow(self.bmp.pressure,0.190255))
    
    def pressure_at_altitude(self,y):
        return pow(pow(101325,0.190255) - y/4947.19,1/0.190255)
    
    # INITIALISE BOARD    
    def init(self):
        self.freq = 40
        self.tel_delay = 10
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
        self.t_log = ticks_ms()
        self.last_t_log = ticks_ms()
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
        
        self.calib_time, self.calib_altitude, self.calib_temp ,self.ram_offset = 0,0,0,0
        self.shutdown = False
        self.calib_data = bytes(24)
        
        self.runtime = 0.5 * 60 * 1000   
        self.temp_mult = 100
        self.mag_mult = 100
        self.orientation_mult = 10
        self.packing_str = '!sibhfffffffffffhhhhhh'
        self.packing_str_slow = '!sfffifffiffff3sfff'
        self.new_idx = 0
        
        self.max_alt = 0
        self.calib_gap = 1000 # Gap in milliseconds between subsequent calibrations
        self.calib_count = 0 # number of calibrations performed till now
        self.calib_max = 3 # max number of calibrations
        
        self.liftoff_accel = 3 # in g, minimum sustained acceleration for liftoff
        self.min_liftoff_alt = 10 # in m, minimum altitude to be cleared for liftoff
        self.liftoff_to_cutoff = 4500 # In milliseconds, force cutoff after this amount of time from detected liftoff
        self.apogee_alt_diff = 5 # in m, the minimum difference between highest recorded altitude and current altitude for apogee detection
        self.liftoff_to_drogue = 27000 # In milliseconds, force drogue deployment after this amount of time from detected liftoff
        self.apogee_to_drogue = 100 # In milliseconds, drogue deployment after this amount of time from detected apogee
        self.drogue_lockout = 20000 # In milliseconds, lockout for drogue deployment from detected liftoff
        self.touchdown_alt = 15 # in m, If altitude is less than this in descent, declare touchdown
        self.touchdown_vel_limit = 3
        self.main_alt = 400
        
        self.buf_len = 5 # assuming the loop runs at 40Hz
        self.alt_buf = np.zeros((1,self.buf_len))[0]
        self.acc_buf = np.zeros((1,self.buf_len))[0]
        self.vel_buf = np.zeros((1,self.buf_len - 1))[0]
        self.apogee = np.zeros((1,self.buf_len - 1))[0]
        
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
        self.uart = UART(0, 9600, rx = Pin(1), tx = Pin(0))
        
        # UART BUS OF XBEE
        self.xbee = UART(1, 57600, rx = Pin(5), tx = Pin(4))

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
            for i in range(10): # Initialization failed
                self.led.on()
                time.sleep(0.1)
                self.led.off()
                time.sleep(0.1)
        else:    
            print('Init done')



    
    def fast_core_init(self):

#  -----------------------------------------------------------------------------------------------------------------------------------------------
        
        print('fast core init')

        # INITIALISE DATA FILE IN SD CARD
#         index_file = open('/sd/index.txt', 'r')
#         idx = index_file.readline()
#         self.new_idx = str(int(idx) + 1)
#         index_file.close()
# 
#         index_file = open('/sd/index.txt', 'w')
#         index_file.write(self.new_idx)
#         index_file.close()
        
        # UNMOUNT SD CARD AS IT WILL LOSE CONTACT IN FLIGHT
        
#         uos.umount("/sd")
        
        # INITIALISE DATA FILE IN FLASH
        self.data_file = open('/win/data_' + '0' + '.bin', 'wb')
    
        
    
    async def calibrate(self):

        self.calib_count += 1
        
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
        self.bno_accel = self.bno.accel()
#           READ HIGH-G ACCELEROMETER HERE
        self.gyro = self.bno.gyro()
        self.mag = self.bno.mag()
        self.orientation = self.bno.euler()
        self.temp = self.bmp.temperature
        self.alt = self.altitude() - self.calib_altitude
        
        self.density = self.pressure_at_altitude(self.alt + self.calib_altitude)/((self.bmp.temperature + 273.15 - 0.0065*self.alt)*287.086)
        
        # READ RAM BAROMETER ONLY IF DATA IS READY
        if self.ram_dout.value() == 0:
            self.ram_diff = self.ram_raw_data() - self.ram_offset

        await asyncio.sleep(0)
        
        

    async def state_machine(self):
        
        t = ticks_ms()
        
        for index in range(1,self.buf_len):
            self.alt_buf[index - 1] = self.alt_buf[index]
            self.acc_buf[index - 1] = self.acc_buf[index]
        for index in range(1,self.buf_len - 1):
            self.vel_buf[index - 1] = self.vel_buf[index]

        self.alt_buf[self.buf_len - 1] = self.alt
        self.vel_buf[self.buf_len - 2] = 1000 * (self.alt_buf[self.buf_len - 1] - self.alt_buf[self.buf_len - 2])/(self.t_log - self.last_t_log)
        self.acc_buf[self.buf_len - 1] = self.bno_accel[2]
 
        if self.alt > self.max_alt:
            self.max_alt = self.alt

        if (self.state != 0 and self.state != 7):
            self.tel_delay = 10
        
        if (self.t_log > self.runtime or self.logging_done): # Stop data logging after run_time or if touchdown is detected
            self.data_array = bytearray()
            
        if (self.state==0 and (np.all(self.alt_buf > self.min_liftoff_alt) and np.all(self.acc_buf < - self.liftoff_accel*self.g))): #Liftoff
            self.state = 1
            self.t_events[0] = self.t_log
            print("liftoff")

        elif (self.state==1 and np.all(self.acc_buf > 0)):
            self.state = 2
            self.t_events[1] = self.t_log
            print("burnout")

        elif (self.state==2 and np.all(self.vel_buf <= 0)): #Apogee
            self.state = 3
            self.t_events[2] = self.t_log
            print("drogue")

        elif (self.state==3 and np.all(self.alt_buf < self.main_alt)): #Chute, need to add condition for signals sent by ground station
            self.state = 4
            self.t_events[3] = self.t_log
            print("main")
                
        elif (self.state == 4 and (np.all(self.alt_buf < self.touchdown_alt) and np.sum(self.vel_buf)/(self.buf_len - 1) < self.touchdown_vel_limit)):
            self.state = 5
            self.t_events[4] = self.t_log
            self.logging_done = True
            self.tel_delay = 2000
            print("touchdown")

        else:
            pass

        self.last_t_log = self.t_log

        await asyncio.sleep(0) 


    async def log_data(self):
    
        t = ticks_ms()

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
#                                           (float)(high_accel[0]),
#                                           (float)(high_accel[1]),
#                                           (float)(high_accel[2]),
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
            
        # WRITE SLOW DATA TO FLASH ONLY IF NEW FIX IS ACQUIRED TO AVOID DUPLICATE DATA AND SAVE SPACE      
        if self.gps_counter > self.last_gps_counter and (self.latitude[0] != 0 and self.longitude[0] != 0):
            if (len(self.data_array) < 512):
                self.data_array += self.data_slow
            else:
                self.data_file.write(self.data_array)
                self.data_array = bytearray()
                self.data_array += self.data_slow

        await asyncio.sleep(0)        
     
     
     
    async def transfer_data(self):
                
        print("Logging finished. Transferring to SD card now")
        self.data_file.close()
        
        transfer_to_SD = True
        try: # try to re-mount the SD card as it has probably lost contact with the reader upon launch
            self.sd = sdcard.SDCard(self.spi_sd, self.SD_CS)
            uos.mount(self.sd,"/sd")
            sd_file = open('/sd/data_' + self.new_idx + '.txt', 'w')
        except:
            transfer_to_SD = False
            print("Failed to mount sd card. Saving config in flash memory")
            config_file = open('/win/config_' + self.new_idx + '.txt','w')
#             config_file.write("(state, time, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, altitude, 45 lmao)\n")
            config_file.write(f"Calibration details- Time: {self.calib_time} Alt: {self.calib_altitude} Temp: {self.calib_temp} Ram: {self.ram_offset}\n")
#             config_file.write("Events tracked: Liftoff, Burnout, Apogee, Chute release, Touchdown, Data shutdown\n")
#             config_file.write(f"Event timestamps: {self.t_events}\n")
#             config_file.write(f"Maximum altitude reached: {max_alt}\n")
            config_file.close()
            
            
        if(transfer_to_SD == True):
            
#             for i in range(4): # Means storing to SD card now
            
            sd_file.write("(data_type, time, state, temperature, alt, ram_diff, bno_x, bno_y, bno_z, high_x, high_y, high_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, heading, roll, pitch)\n")
            read_file = open('/win/data_' + self.new_idx + '.bin', 'rb')
            read_file.seek(0,2) # Move to end of file
            file_size = read_file.tell() # Get number of bytes
            read_file.seek(0,0) # Move to start of file
            while(file_size - read_file.tell()>0): # IF FILE IS NOT EMPTY
                
                # READ FIRST BYTE (0 - FAST READINGS, 1 - SLOW READINGS
                
                data_type = read_file.read(1)
                
                if data_type == b'\x00':
                    flash_data = ustruct.unpack(self.packing_str, data_type + read_file.read(63))
                    sd_file.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(flash_data[0],
                                                                           flash_data[1],
                                                                           flash_data[2],
                                                                           (flash_data[3]/self.temp_mult) + self.calib_temp,
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
                                                                           flash_data[15]/self.mag_mult,
                                                                           flash_data[16]/self.mag_mult,
                                                                           flash_data[17]/self.mag_mult,
                                                                           flash_data[18]/self.orientation_mult,
                                                                           flash_data[19]/self.orientation_mult,
                                                                           flash_data[20]/self.orientation_mult
                                                                           ))
                
                elif data_type == b'\x01':
                    flash_data = ustruct.unpack(self.packing_str_slow, data_type + read_file.read(63))
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
        
        
    # COMMUNICATION BETWEEN ROCKET AND GROUND STATION   
    async def comms(self):
        
        t = ticks_ms()
        
        if self.send_calib:
            self.xbee.write(self.calib_data)
            self.send_calib = False
            
        if self.xbee.any() >= 2:
            print('RX')
            
        self.xbee.write(self.data_fast)
        
        if self.gps_counter > self.last_gps_counter:
            self.xbee.write(self.data_slow)
        

    # START FAST CORE
    async def fast_readings(self):
        
        ran_once = False
        
        asyncio.create_task(self.nav())
        
        while True:
            
            loop = asyncio.get_event_loop()
            
            if (self.state == 0):
                if (self.t_log > self.calib_gap and self.calib_count < self.calib_max):
                    loop.create_task(self.calibrate())
                    
            loop.create_task(self.get_data())
            loop.create_task(self.log_data())
            loop.create_task(self.comms())

            if ran_once == False:
                ran_once = True
            elif (self.t_log > self.runtime or self.logging_done):
                asyncio.run(self.transfer_data())
                break
            
            loop.run_until_complete(self.state_machine())

 
 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ----------------------------------------------------

    async def nav(self):
        
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

        a_r = np.array([[0],[0],[0]])
        a0_true = a1_true = a2_true = 0
        ax_bias = ay_bias = az_bias = 0
        last_vtg_fix_time = last_vtg_speed = last_vtg_course = last_bias_alt = last_bias_time = 0
        time_array = track_array = a0_array = a1_array = a2_array = ax_array = ay_array = az_array = np.array([])
        
        dt = 1/58.8
        nx = ny = 1
        init = False
        declination = -0.2*pi/180
        
        while True:
    
            t = ticks_ms()
        
            last_heading = Z_A[2][0]

            ax = -self.bno_accel[0]
            ay = -self.bno_accel[1]
            az = -self.bno_accel[2]
            
            a = sqrt(ax*ax + ay*ay + az*az)

            gx = self.gyro[0]*pi/180
            gy = self.gyro[1]*pi/180
            gz = self.gyro[2]*pi/180
            
            mx = self.mag[0]
            my = self.mag[1]
            mz = self.mag[2]
            
        # ATTITUDE KF --------------------------------------------      

            U_A[0][0] = nx*gx*dt    
            U_A[1][0] = ny*gy*dt

            e = abs(a-self.g)/self.g

            cutoff = 0.05

            if e < cutoff:
                Z_A[0][0] = atan2(ay,az)    
                Z_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))
                Q_A = 0.0001*np.eye(len(Q_A))
            else:
                Q_A = 0.001*np.eye(len(Q_A))

            C[0][0] = cos(Z_A[1][0])
            C[0][1] = 0
            C[0][2] = sin(Z_A[1][0])

            C[1][0] = sin(Z_A[0][0])*sin(Z_A[1][0])
            C[1][1] = cos(Z_A[0][0])
            C[1][2] = -sin(Z_A[0][0])*cos(Z_A[1][0])

            C[2][0] = -sin(Z_A[1][0])*cos(Z_A[0][0])
            C[2][1] = sin(Z_A[0][0])
            C[2][2] = cos(Z_A[0][0])*cos(Z_A[1][0])

            b = np.dot(C,np.array([[mx],[my],[mz]]))

            if atan2(b[0][0],b[1][0]) >= 0:
                Z_A[2][0] = atan2(b[0][0],b[1][0])
            else:
                Z_A[2][0] = 2*pi + atan2(b[0][0],b[1][0])

            if Z_A[2][0] - last_heading > 2*pi - 0.5:
                U_A[2][0] = gz*dt + 2*pi
            if Z_A[2][0] - last_heading < -2*pi + 0.5:
                U_A[2][0] = gz*dt - 2*pi
            if abs(Z_A[2][0] - last_heading) <= 2*pi - 0.5:
                U_A[2][0] = gz*dt

            Xp_A = np.dot(F_A,X_A) + U_A
            
            Pp_A = np.dot(np.dot(F_A,P_A),F_A.T) + Q_A

            K_A = np.dot(Pp_A,np.linalg.inv(Pp_A + R_A))

            if e >= cutoff:
                K_A[0][0] = K_A[1][1] = 0

            X_A = Xp_A + np.dot(K_A,Z_A - Xp_A)

            P_A = np.dot(np.eye(len(K_A)) - K_A,Pp_A)

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
            C1[2][2] = cos(X_A[0][0])*cos(X_A[1][0])
            
    # -------------------------------------------------------------------------------------------------------------------
            
            last_alt = Z[2][0]
            
            gravity = np.dot(C1,np.array([[0],[0],[-self.g]]))

            lin_acc_x = - ax + gravity[0][0]
            lin_acc_y = - ay + gravity[1][0]
            lin_acc_z = az + gravity[2][0]

            a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
            a_r_true = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x + ax_bias],[lin_acc_y + ay_bias],[lin_acc_z - az_bias]]))

            if self.gps_counter > self.last_gps_counter and (self.sentence_type == "b'VTG'" or self.sentence_type == "b'RMC'") and len(a0_array) != 0:
                delta_vx = self.speed*sin(self.course*pi/180) - last_vtg_speed*sin(last_vtg_course*pi/180)
                delta_vy = self.speed*cos(self.course*pi/180) - last_vtg_speed*cos(last_vtg_course*pi/180)
                
                IC = IS = 0
                
                avg_a0 = a0_array[0]/len(a0_array)
                avg_a1 = a1_array[0]/len(a1_array)
                avg_a2 = a2_array[0]/len(a2_array)
                avg_ax = ax_array[0]/len(ax_array)
                avg_ay = ay_array[0]/len(ay_array)
                avg_az = az_array[0]/len(az_array)

                for count in range(1,len(track_array)):
                    IC += cos(track_array[count])*(time_array[count] - time_array[count - 1])
                    IS += sin(track_array[count])*(time_array[count] - time_array[count - 1])
                    avg_a0 += a0_array[count]/len(a0_array)
                    avg_a1 += a1_array[count]/len(a1_array)
                    avg_a2 += a2_array[count]/len(a2_array)
                    avg_ax += ax_array[count]/len(ax_array)
                    avg_ay += ay_array[count]/len(ay_array)
                    avg_az += az_array[count]/len(az_array)
                    
                k = delta_vx/delta_vy
               
                theta = atan((k*IC - IS)/(IC + k*IS))
               
                a1_true = delta_vx/(cos(theta)*(IS + tan(theta)*IC))*sqrt(pow(IC + k*IS,2)/(1 + k*k))
                a0_true = a1_true*(IS - k*IC)/(IC + k*IS)
                a2_true = 2 * 0 * (self.alt - last_bias_alt)/pow((t - last_bias_time)/1000,2)
               
                B = np.dot(C1,np.array([[a0_true],[a1_true],[a2_true + self.g]]))
               
                ax_bias = avg_ax + B[0][0]
                ay_bias = avg_ay + B[1][0]
                az_bias = avg_az - B[2][0]
               
                time_array = track_array = a0_array = a1_array = a2_array = ax_array = ay_array = az_array = np.array([])
                
                last_vtg_fix_time = self.fix_time
                last_vtg_speed = self.speed
                last_vtg_course = self.course
                last_bias_alt = self.alt
                last_bias_time = t
               
            time_array += t/1000
            track_array += X_A[2][0] + declination
            a0_array += a_r[0][0]
            a1_array += a_r[1][0]
            a2_array += a_r[2][0]
            ax_array += ax
            ay_array += ay
            az_array += az

            dv = sqrt(pow(a_r_true[0][0],2) + pow(a_r_true[1][0],2))*dt
            alpha = atan2(-a_r_true[0][0],a_r_true[1][0]) + X_A[2][0] + declination
           
            if self.latitude[0] != 0 and self.longitude[0] != 0 and init == False:
                X[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                X[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                init = True
                         
             
            F[0][4] = cos(Z[5][0])*dt/self.r
            F[1][4] = sin(Z[5][0])*dt/(self.r*cos(Z[0][0]))
            F[2][3] = dt
            
            U[3][0] = a_r[2][0]*dt
            U[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha)) - Z[4][0]
            U[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha))) - Z[5][0]
             
            Q = 0.0001/(self.hdop + 0.00001)*np.eye(len(Q))
            
            Xp = np.dot(F,X) + U

            Pp = np.dot(np.dot(F,P),F.T) + Q

            if self.gps_counter > self.last_gps_counter:
                if 'GGA' in self.sentence_type or 'GLL' in self.sentence_type:
                    Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                    Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                    Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                    Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                   
                elif 'VTG' in self.sentence_type:
                    Z[4][0] = self.speed
                    Z[5][0] = self.course*pi/180
                    Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                    Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                
                elif 'VNC' in self.sentence_type:
                    Z[4][0] = self.speed
                    Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                    Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                    Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                    
                elif 'RMC' in self.sentence_type:
                    Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                    Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                    Z[4][0] = self.speed
                    Z[5][0] = self.course*pi/180
               
                elif 'ROV' in self.sentence_type:
                    Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                    Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                    Z[4][0] = self.speed
                    Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                    
            else:
               
                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
               
                Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(self.r*cos(Z[0][0]))
                Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/self.r
                
            Z[2][0] = self.alt
            Z[3][0] = (Z[2][0] - last_alt)/dt

            K = np.dot(Pp,np.linalg.inv(Pp + R))

            X = Xp + np.dot(K,Z - Xp)
            
            P = np.dot(np.eye(len(K)) - K,Pp)

            dt = (ticks_ms() - t)/1000
            
            self.last_gps_counter = self.gps_counter
         
            await asyncio.sleep(0)
# 
# ------------------------------------- NAV FILTER UNDER DEVELOPMENT ---------------------------------------------------


test_instance = async_test()                    # START INSTANCE OF CLASS
test_instance.init()                            # INITIALISE SOFTWARE
test_instance.init_board()                      # INITIALISE SENSORS
test_instance.fast_core_init()
asyncio.run(test_instance.fast_readings())      # START FAST CORE

 