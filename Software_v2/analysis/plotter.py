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

R_A = 0.007*np.eye(len(R_A))
Q_A = 0.0005*np.eye(len(Q_A))

X = np.full((6,1),0.)
W = np.full((6,1),0.)  
F = np.full((6,6),0.)
P = np.full((6,6),0.)
Q = np.full((6,6),0.)
R = np.full((6,6),0.)
Z = np.full((6,1),0.)
U = np.full((6,1),0.)

R = 1*np.eye(len(R))
Q = 0.05*np.eye(len(Q))

n = 1

init = False
g = 0

vx = vy = vz = 0

declination = -0.2*pi/180

counter = 0

raw = kalman = bno = np.array([])

start = 0
stop = 37000

fast_counter = 0

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


g = 9.9379
r = 6371000
new_gps_data = False

gps_counter = 0

def distance(lat_1,lon_1,lat_2,lon_2):
        return sqrt(pow((r*(lat_2 - lat_1)*pi/180),2) + pow(r*(lon_2 - lon_1)*pi/180*cos(lat_1*pi/180),2))    

def track(lat_1,lon_1,lat_2,lon_2):
    if lat_1 != lat_2:
        return atan(cos(lat_1*pi/180)*(lon_2 - lon_1)/(lat_2 - lat_1))
    else:
        return pi/2*(course/abs(course))

with open('data_2.txt','r') as file:
    with open('earth.csv','w') as kml:       
        spamreader = csv.reader(file, delimiter=',')
        for data in spamreader:
            
            if counter == 0:
                    calib_time = float(data[0])
                    calib_alt = float(data[1])
                    calib_temp = float(data[2])
                    ram_offset = float(data[3])
            
            if counter == start:
                t_init = float(data[1])/1000
            
            if counter == stop:
                t_final = float(data[1])/1000
                    
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
    #                 
    #         # ---------------------------------------------------------------------------------------------------
    #
    #                     MADGWICK FILTER
    #
    #                     wx = gx
    #                     wy = gy
    #                     wz = gz
    #                     
    #                     w[0][1] = -wx
    #                     w[0][2] = -wy
    #                     w[0][3] = -wz
    #                     
    #                     w[1][0] = wx
    #                     w[1][2] = wz
    #                     w[1][3] = -wy
    #                     
    #                     w[2][0] = wy
    #                     w[2][1] = -wz
    #                     w[2][3] = wx
    #                     
    #                     w[3][0] = wz
    #                     w[3][1] = wy
    #                     w[3][2] = -wx
    #                     
    #                     q_w_dot = 0.5*np.dot(q,w) 
    #                     
    #                     Bh = 38.285
    #                     Bv = 20.9
    # 
    #                     f[0][0] = 2*(q[0][1]*q[0][3] - q[0][0]*q[0][2]) - ax/a
    #                     f[1][0] = 2*(q[0][0]*q[0][1] + q[0][2]*q[0][3]) - ay/a
    #                     f[2][0] = 2*(0.5 - q[0][1]*q[0][1] - q[0][2]*q[0][2]) - az/a
    #                     f[3][0] = 2*Bh*(0.5 - q[0][2]*q[0][2] - q[0][3]*q[0][3]) + 2*Bv*(q[0][1]*q[0][3] - q[0][0]*q[0][2]) - mx/m
    #                     f[4][0] = 2*Bh*(q[0][1]*q[0][2] - q[0][0]*q[0][3]) + 2*Bv*(q[0][0]*q[0][1] + q[0][2]*q[0][3]) - my/m
    #                     f[5][0] = 2*Bh*(q[0][0]*q[0][2] + q[0][1]*q[0][3]) + 2*Bv*(0.5 - q[0][1]*q[0][1] - q[0][2]*q[0][2]) - mz/m
    #                     
    #                     J[0][0] = -2*q[0][2]
    #                     J[1][0] = 2*q[0][3]
    #                     J[2][0] = -2*q[0][0]
    #                     J[3][0] = 2*q[0][1]
    #                     
    #                     J[0][1] = 2*q[0][1]
    #                     J[1][1] = 2*q[0][0]
    #                     J[2][1] = 2*q[0][3]
    #                     J[3][1] = 2*q[0][2]
    #                     
    #                     J[0][2] = 0
    #                     J[1][2] = -4*q[0][1]
    #                     J[2][2] = -4*q[0][2]
    #                     J[3][2] = 0
    #                     
    #                     J[0][3] = -2*Bv*q[0][2]
    #                     J[1][3] = 2*Bv*q[0][3]
    #                     J[2][3] = -4*Bh*q[0][2] - 2*Bv*q[0][0]
    #                     J[3][3] = -4*Bh*q[0][1] + 2*Bv*q[0][1]
    #                     
    #                     J[0][4] = -2*Bh*q[0][3] + 2*Bv*q[0][1]
    #                     J[1][4] = 2*Bh*q[0][2] + 2*Bv*q[0][0]
    #                     J[2][4] = 2*Bh*q[0][1] + 2*Bv*q[0][3]
    #                     J[3][4] = -2*Bh*q[0][0] + 2*Bv*q[0][2]
    #                     
    #                     J[0][5] = 2*Bh*q[0][2]
    #                     J[1][5] = 2*Bh*q[0][3] - 4*Bv*q[0][1]
    #                     J[2][5] = 2*Bh*q[0][0] - 4*Bv*q[0][2]
    #                     J[3][5] = 2*Bh*q[0][1]
    #                     
    #                     x = np.dot(J,f).T
    #                     
    #                     x /= sqrt(x[0][0]*x[0][0] + x[0][1]*x[0][1] + x[0][2]*x[0][2] + x[0][3]*x[0][3])
    #                     
    #                     q += (q_w_dot - beta*x)*dt
    #                     
    #                     q /= sqrt(q[0][0]*q[0][0] + q[0][1]*q[0][1] + q[0][2]*q[0][2] + q[0][3]*q[0][3])
    # 
    #                     e1 = atan2(2*(q[0][0]*q[0][1] + q[0][2]*q[0][3]), 1 - 2*(q[0][1]*q[0][1] + q[0][2]*q[0][2]))
    #                     e2 = asin(2*(q[0][0]*q[0][2] - q[0][1]*q[0][3]))
    #                     e3 = atan2(2*(q[0][0]*q[0][3] + q[0][1]*q[0][2]), 1 - 2*(q[0][2]*q[0][2] + q[0][3]*q[0][3]))
    #                 
    #         # ATTITUDE KF ---------------------------------------------------------------------------------------------------------         
                    
                        F_A = np.eye(len(F_A))

                        U_A[0][0] = gx*dt    
                        U_A[1][0] = n*gy*dt
                        
                        e = abs(a-g)/g
                        
                        cutoff = 0.05
                        
                        if e < cutoff and (abs(gx) < 50*pi/180 and abs(gy) < 50*pi/180 and abs(gz) < 50*pi/180):
                            Z_A[0][0] = atan2(ay,az)    
                            Z_A[1][0] = atan2(-ax,sqrt(ay*ay + az*az))
                            Q_A = 0.000001*np.eye(len(Q_A))
                        else:
                            Q_A = 0.00001*np.eye(len(Q_A))

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
                        if abs(Z_A[2][0] - last_heading) < 2*pi - 0.5:
                            U_A[2][0] = gz*dt
                        
                        Xp_A = np.dot(F_A,X_A) + U_A + W_A
                        
                        Pp_A = np.dot(np.dot(F_A,P_A),F_A.T) + Q_A
                        
                        K_A = np.dot(Pp_A,np.linalg.inv(Pp_A + R_A))
                        
                        if e >= cutoff:
                            K_A[0][0] = K_A[1][1] = 0
                        
                        X_A = Xp_A + np.dot(K_A,Z_A - Xp_A)
                        
                        P_A = np.dot(np.eye(len(K_A)) - K_A,Pp_A)
                        
                        if abs(X_A[1][0] + n*gy*dt) >= pi/2:
                            n *= -1

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
                        
                        vx += lin_acc_x*dt
                        vy += lin_acc_y*dt
                        vz += lin_acc_z*dt
                        
                        v = sqrt(vx*vx + vy*vy + vz*vz)
          
                        v_r = np.dot(np.linalg.inv(C1),np.array([[vx],[vy],[vz]]))
                        a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
                        
                        dv = sqrt(a_r[0][0]*a_r[0][0] + a_r[1][0]*a_r[1][0])*dt
                        alpha = atan2(-a_r[0][0],a_r[1][0]) + X_A[2][0] + declination
                        
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
            
                        if X[1][0]*180/pi > 73.70 and X[0][0]*180/pi > 18.66:
                            kml.write(str(X[1][0]*180/pi) + ','  + str(X[0][0]*180/pi) + '\n')
                        
                        last_data = data

            counter += 1
                
            if counter%1000 == 0 and counter > start:
                print(str(int((counter-start)/(stop-start)*100)) + '%')
            
            pass
    
kml.close()
#     
# t = np.linspace(0, t_final - t_init,fast_counter)
# 
# plt.plot(t,raw,marker = 'o',markersize=0)
# plt.plot(t,bno,marker = 'o',markersize=0)
# plt.plot(t,kalman,marker = 'o',markersize=0)
# 
# plt.show()