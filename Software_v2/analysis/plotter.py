import numpy as np
import csv
from math import sqrt,pi,asin,atan,tan,cos,sin,atan2,acos
import matplotlib.pyplot as plt

X_A = np.full((3,1),0.)
W_A = np.full((3,1),0.)  
F_A = np.full((3,3),0.)
P_A = np.full((3,3),0.)
Q_A = np.full((3,3),0.)
R_A = np.full((3,3),0.)
Z_A = np.full((3,1),0.)
C = C1 = np.full((3,3),0.)
U_A = np.full((3,1),0.)

R_A = 1*np.eye(len(R_A))

X = np.full((6,1),0.)
W = np.full((6,1),0.)  
F = np.full((6,6),0.)
P = np.full((6,6),0.)
Q = np.full((6,6),0.)
R = np.full((6,6),0.)
Z = np.full((6,1),0.)
U = np.full((6,1),0.)

R = 1*np.eye(len(R))

nx = ny = 1

init = False
g = 0

vx = vy = vz = 0

declination = -0.2*pi/180

counter = 0

raw = kalman = bno = np.array([])

start = 0
stop = 37000

a0_bias = a1_bias = 0

q = q_w = np.array([[1.,0.,0.,0.]])
w = np.full((4,4),0.)
J = np.full((4,6),0.)
f = np.full((6,1),0.)

beta = 0
r = 0

latitude = last_latitude = 0
longitude = last_longitude = 0
speed = last_speed = 0
course = last_course = 0
fix_time = last_fix_time = 0
sentence_type = b'NUL'
hdop = vdop = 0

last_vtg_fix_time = last_vtg_speed = last_vtg_course = 0

g = 9.81
r = 6371000
new_gps_data = False

time_array = track_array = a0_array = a1_array = np.array([])

gps_counter = 0

# def distance(lat_1,lon_1,lat_2,lon_2):
#         return sqrt(pow((r*(lat_2 - lat_1)*pi/180),2) + pow(r*(lon_2 - lon_1)*pi/180*cos(lat_1*pi/180),2))    
# 
# def track(lat_1,lon_1,lat_2,lon_2):
#     if lat_1 != lat_2:
#         return atan(cos(lat_1*pi/180)*(lon_2 - lon_1)/(lat_2 - lat_1))
#     else:
#         return pi/2*(course/abs(course))

with open('data_4.txt','r') as file:
    with open('earth.csv','w') as kml:       
        spamreader = csv.reader(file, delimiter=',')
        for data in spamreader:
            
            # IGNORE THIS
            if counter == 0:
                    calib_time = 0
                    calib_alt = 0
                    calib_temp = 0
                    
            if counter == 3:
                g = sqrt(pow(float(data[6]),2) + pow(float(data[7]),2) + pow(float(data[8]),2))
                    
            if start <= counter <= stop:
                
                if counter > 0:
                    
                    last_heading = Z_A[2][0]

            # RAW DATA ------------------------------------------------------------------------------------------------------------
                    if data[0] == str(1):
                        sentence_type = data[13]
                        if sentence_type == "b'GGA'" or "b'GLL'":
                            latitude = float(data[1]) + float(data[2])/60 + float(data[3])/3600
                            longitude = float(data[5]) + float(data[6])/60 + float(data[7])/3600
                            fix_time = float(data[15])
                            gps_alt = float(data[14]) + calib_alt
                        if sentence_type == "b'RMC'":
                            latitude = float(data[1]) + float(data[2])/60 + float(data[3])/3600
                            longitude = float(data[5]) + float(data[6])/60 + float(data[7])/3600
                            fix_time = float(data[15])
                            speed = float(data[9])
                            course = float(data[10])
                            gps_alt = float(data[14]) + calib_alt
                        if sentence_type == "b'ROV'":
                            latitude = float(data[1]) + float(data[2])/60 + float(data[3])/3600
                            longitude = float(data[5]) + float(data[6])/60 + float(data[7])/3600
                            fix_time = float(data[15])
                            speed = float(data[9])
                            gps_alt = float(data[14]) + calib_alt
                        if sentence_type == "b'VTG'":     
                            speed = float(data[9])
                            course = float(data[10])
                            fix_time = float(data[15])
                            gps_alt = float(data[14]) + calib_alt
                        if sentence_type == "b'VNC'":
                            speed = float(data[9])
                            fix_time = float(data[15])
                            gps_alt = float(data[14]) + calib_alt

                        last_latitude = latitude
                        last_longitude = longitude
                        last_speed = speed
                        last_course = course
                        
                        hdop = float(data[11])
                        vdop = float(data[12])
                        
                        gps_counter += 1
                        new_gps_data = True
                         
                    elif data[0] == str(0):
                        ax = -float(data[6])
                        ay = -float(data[7])
                        az = -float(data[8])
                        
                        a = sqrt(ax*ax + ay*ay + az*az)

#                         if counter == 1:
#                             g = a

                        gx = float(data[12])*pi/180
                        gy = float(data[13])*pi/180
                        gz = float(data[14])*pi/180
                        
                        mx = float(data[15])
                        my = float(data[16])
                        mz = float(data[17])
                        
                        m = sqrt(mx*mx + my*my + mz*mz)
                        
                # ---------------------------------------------------------------------------------------------------------------------            
                        
                        heading = float(data[18])*pi/180
                        roll = float(data[19])*pi/180
                        pitch = float(data[20])*pi/180
                        
                        if counter > start + 1:
                            dt = (float(data[1]) - float(last_data[1]))/1000
                        else:
                            dt = 0.05
          
    #         # ATTITUDE KF ---------------------------------------------------------------------------------------------------------         
                    
                        F_A = np.eye(len(F_A))

                        U_A[0][0] = nx*gx*dt    
                        U_A[1][0] = ny*gy*dt

                        e = abs(a-g)/g

                        cutoff = 0.05

                        if e < cutoff:
                            Z_A[0][0] = atan2(ay,az)    
                            Z_A[1][0] = atan2(-ax,sqrt(ay*ay + az*az))
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

                        Xp_A = np.dot(F_A,X_A) + U_A + W_A

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

    #      
    #          # ----------------------------------------------------------------------------------------------------------------------
    # 
    #         # POSITION KF ---------------------------------------------------------------------------------------------------------
    #                  
                        last_alt = Z[2][0]
                        
                        alt = float(data[4]) + calib_alt
                        
                        gravity = np.dot(C1,np.array([[0],[0],[-g]]))

                        lin_acc_x = - ax + gravity[0][0]
                        lin_acc_y = - ay + gravity[1][0]
                        lin_acc_z = az + gravity[2][0]

                        a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
                                  
                        dv = sqrt(pow(a_r[0][0] - a0_bias,2) + pow(a_r[1][0] - a1_bias,2))*dt
                        alpha = atan2(-(a_r[0][0] - a0_bias),(a_r[1][0] - a1_bias)) + X_A[2][0] + declination
   
                        if new_gps_data and sentence_type == "b'VTG'" and len(a0_array) != 0:
                            delta_vx = speed*sin(course*pi/180) - last_vtg_speed*sin(last_vtg_course*pi/180)
                            delta_vy = speed*cos(course*pi/180) - last_vtg_speed*cos(last_vtg_course*pi/180)
                            
                            IC = IS = 0
                            
                            avg_a0 = a0_array[0]/len(a0_array)
                            avg_a1 = a1_array[0]/len(a1_array)
                            
                            for count in range(1,len(track_array)):
                                IC += cos(track_array[count])*(time_array[count] - time_array[count - 1])
                                IS += sin(track_array[count])*(time_array[count] - time_array[count - 1])
                                avg_a0 += a0_array[count]/len(a0_array)
                                avg_a1 += a1_array[count]/len(a1_array)
                                
                            k = delta_vx/delta_vy
                            
                            theta = atan((k*IC - IS)/(IC + k*IS))
                            
                            a1_bias = avg_a1 - delta_vx/(cos(theta)*(IS + tan(theta)*IC))*sqrt(pow(IC + k*IS,2)/(1 + k*k))
                            a0_bias = avg_a0 - (avg_a1 - a1_bias)*(IS - k*IC)/(IC + k*IS)
                                                       
                            time_array = track_array = a0_array = a1_array = np.array([])
                            
                            last_vtg_fix_time = fix_time
                            last_vtg_speed = speed
                            last_vtg_course = course
                        
                        
                        time_array = np.append(time_array,float(data[1])/1000)
                        track_array = np.append(track_array,X_A[2][0] + declination)
                        a0_array = np.append(a0_array,a_r[0][0])
                        a1_array = np.append(a1_array,a_r[1][0])
                    
                        if latitude != 0 and longitude != 0 and init == False:
                            X[0][0] = latitude*pi/180
                            X[1][0] = longitude*pi/180
                            init = True
                        
                        F = np.eye(len(F))
                        
                        F[0][4] = cos(Z[5][0])*dt/r
                        F[1][4] = sin(Z[5][0])*dt/(r*cos(Z[0][0]))
                        F[2][3] = dt
                        
                        U[3][0] = a_r[2][0]*dt
                        U[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha)) - Z[4][0]
                        U[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha))) - Z[5][0]
                        
                        Q = 0.0001/(hdop + 0.00001)*np.eye(len(Q))
                        
                        Xp = np.dot(F,X) + U + W
                        
                        Pp = np.dot(np.dot(F,P),F.T) + Q

                        if new_gps_data and gps_counter%1 == 0:
                            if 'GGA' in sentence_type or 'GLL' in sentence_type:
                                Z[0][0] = latitude*pi/180
                                Z[1][0] = longitude*pi/180
                                Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                                Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                                
                            elif 'VTG' in sentence_type:
                                Z[4][0] = speed
                                Z[5][0] = course*pi/180
                                Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(r*cos(Z[0][0]))
                                Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/r
                            
                            elif 'VNC' in sentence_type:
                                Z[4][0] = speed
                                Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                                Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(r*cos(Z[0][0]))
                                Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/r
                                
                            elif 'RMC' in sentence_type:
                                Z[0][0] = latitude*pi/180
                                Z[1][0] = longitude*pi/180
                                Z[4][0] = speed
                                Z[5][0] = course*pi/180
                            
                            elif 'ROV' in sentence_type:
                                Z[0][0] = latitude*pi/180
                                Z[1][0] = longitude*pi/180
                                Z[4][0] = speed
                                Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))

                                
                        else:
                            
                            Z[4][0] = sqrt(Z[4][0]*Z[4][0] + dv*dv + 2*Z[4][0]*dv*cos(Z[5][0] - alpha))
                            Z[5][0] = atan2((Z[4][0]*sin(Z[5][0]) + dv*sin(alpha)),(Z[4][0]*cos(Z[5][0]) + dv*cos(alpha)))
                            
                            Z[1][0] += Z[4][0]*dt*sin(Z[5][0])/(r*cos(Z[0][0]))
                            Z[0][0] += Z[4][0]*dt*cos(Z[5][0])/r
                            
                        Z[2][0] = alt
                        Z[3][0] = (Z[2][0] - last_alt)/dt
                        
                        K = np.dot(Pp,np.linalg.inv(Pp + R))
                        
                        X = Xp + np.dot(K,Z - Xp)
                        
                        P = np.dot(np.eye(len(K)) - K,Pp)
                              
                        new_gps_data = False
            
                        if X[1][0]*180/pi > 73.70 and X[0][0]*180/pi > 18.60:
                            kml.write(str(X[1][0]*180/pi) + ','  + str(X[0][0]*180/pi) + '\n')
                        
#                         if counter > 23500:
#                             print(counter/10000,Z[5][0]*180/pi,Z[4][0])
                        
                        last_data = data

            counter += 1
                
            if counter%1000 == 0 and counter > start:
                print(str(int((counter-start)/(stop-start)*100)) + '%')
            
            pass
    
kml.close()
