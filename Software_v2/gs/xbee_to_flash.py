import uos
from machine import SPI,Pin,UART,freq
import ustruct
from time import ticks_ms
import uasyncio as asyncio
from math import sqrt,atan,pi
from time import sleep

freq(270000000)

class ground_station:

    def init(self):
        
        #xbee
        self.xbee = UART(0, baudrate=57600, tx=Pin(0), rx=Pin(1))

        index_file = open('/index.txt','r')
        index = int(index_file.read())
        index_file.close()
        
        index_file = open('/index.txt','w')
        index_file.write(str(index + 1))
        index_file.close()

        #packing        
        self.packing_str = '!sibhffhhhffffffbbbb'
        self.packing_str_slow = '!sbbfbbbfbHHHH3sfff'
        self.packing_str_calib = '!fffffbfb'
        self.temp_mult = 100
        self.acc_mult = 100
        self.volt_mult = 10
        self.gps_mult = 100
        
        self.r = 6371000

    async def comms(self):
        fast_len = 50
        slow_len = 38
        calib_len = 26
        t_start = ticks_ms()
        t = ticks_ms()
        fast_rec = bytearray(fast_len)
        slow_rec = bytearray(slow_len)
        calib_rec = bytearray(calib_len)
        runtime = 10000 * 1000
        
        rocket_alt = 0
        rocket_lat = 0
        rocket_lon = 0
        rocket_vy = 0
        current_t = 0
        last_t = -1
        last_alt = 0
        rocket_state = 0
        
        max_acc = 0
        max_speed = 0
        apogee = 0
                 
        last_print = ticks_ms()
                 
        time_of_fix = 0
                 
        while True:

            if self.xbee.any():
                data = self.xbee.read()
                if len(data) == 50:                    
                    fast_data = ustruct.unpack(self.packing_str,data)
                    
                    self.data_file = open('/data_' + str(index) + '.bin','ab')
                    self.data_file.write(fast_rec)
                    self.data_file.close()
                    
                    if fast_data[2] == 0:
                        rocket_state = 'On Pad'
                    elif fast_data[2] == 1:
                        rocket_state = 'Boost'
                    elif fast_data[2] == 2:
                        rocket_state = 'Coasting'
                    elif fast_data[2] == 3:
                        rocket_state = 'Drogue'
                    elif fast_data[2] == 4:
                        rocket_state = 'Main'
                        
                    current_t = fast_data[1]
                    rocket_alt = fast_data[4]
                    rocket_vy = 1000 * (rocket_alt - last_alt)/( current_t - last_t)
                    rocket_acc = fast_data[6]/self.acc_mult
                    if fast_data[5] > 0:
                        rocket_v = sqrt(2*fast_data[5]/700)
                    else:
                        rocket_v = 0
                                   
                    apogee = max(apogee,rocket_alt)
                    max_speed = max(max_speed,rocket_v)
                    max_acc = max(max_acc,rocket_acc)
                                   
                    last_t = current_t
                    last_alt = rocket_alt
                    
                    t = ticks_ms()
                        
                elif len(data) == 38:
                    
                    self.data_file = open('/data_' + str(index) + '.bin','ab')
                    self.data_file.write(slow_rec)
                    self.data_file.close()
                    
                    slow_data = ustruct.unpack(self.packing_str_slow,data)
                    
                    rocket_lat = slow_data[1] + slow_data[2]/60 + slow_data[3]/3600
                    rocket_lon = slow_data[5] + slow_data[6]/60 + slow_data[7]/3600
                    time_of_fix = slow_data[15]
                                            
                elif len(data) == 26:
                    self.data_file = open('/data_' + str(index) + '.bin','ab')
                    self.data_file.write(calib_rec)
                    self.data_file.close()
                                    
                else:
                    self.data_file = open('/data_' + str(index) + '.bin','ab')
                    self.data_file.write(data)
                    print(data,len(data))
                    self.data_file.close()
    
                if ticks_ms() - last_print > 1000:
                    print('LIVE DATA')
                    print('---------' + '\n')
                    
                    print('State : ' + rocket_state + '\n')
                    
                    print('Altitude : ' + str(round(rocket_alt,1)) + ' m' + '\n')
                    
                    print('Speed (Baro) : ' + str(round(rocket_vy,1)) + ' m/s' + '\n')
                    
                    print('Speed (Ram) : ' + str(round(rocket_v,1)) + ' m/s' + '\n')
                    
                    print('Acceleration : ' + str(round(rocket_acc/9.81,1)) + ' G' + '\n')
                    
                    
                    try:
                        if rocket_lan != 0 and rocket_lon != 0:
                            print('GPS Coordinates : ' + str(rocket_lat) + ',' + str(rocket_lon) + '          ' + 'Time since last GPS fix : ' + str(int((current_t - time_of_fix)/1000)) + 'sec' + '\n')
                            
                        else:
                            print('GPS Coordinates : ' + 'No GPS Fix' + '\n')
                            
                    except:
                        print('GPS Coordinates : ' + 'No GPS Fix' + '\n')
                        
                        
                    print()
                    print()
                    print()
                    
                    print('FLIGHTS STATS')
                    print('-------------' + '\n')
                    
                    
                    print('Apogee : ' + str(round(apogee,1)) + ' m' + '\n')
                    print('Maximum Speed : ' + str(round(max_speed,1)) + ' m/s' + ' = ' 'Mach ' + str(round(max_speed/355,2)) + '\n')
#                     
                    print('Max Acceleration : ' + str(round(max_acc/9.81,1)) + ' G' + '\n')
                    
                    last_print = ticks_ms()

            await asyncio.sleep(0)

        
gs = ground_station()
gs.init()
asyncio.run(gs.comms())