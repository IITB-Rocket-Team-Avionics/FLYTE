import csv
import numpy as np
from math import sqrt,cos,sin,acos,pi,atan2,tan,atan
import matplotlib.pyplot as plt

q0 = 0.05

q = q0

true = np.loadtxt('true.csv',delimiter=',')

while q <= q0:
    
    error = 0
    min_error = 10000000
    
    counter = 0

    calib_temp = 49.39
    calib_alt = 678.07

    last_t = 743.265
    last_airspeed_t = 743.165
    last_y = 21
    last_airspeed = 20

    r = 6371000

    pitch = np.array([])
    time = np.array([])
    acc = np.array([])
    ias = np.array([])
    alt = np.array([])
    vertical_rate = np.array([])
    t_speed = np.array([])
    t_motor = np.array([])
    dec_drag = np.array([])
    low_array = np.array([])
    high_array = np.array([])
    
    t_sim = np.array([])
    y_sim = np.array([])
    vx_sim = np.array([])
    vy_sim = np.array([])
    v_sim = np.array([])
    pitch_sim = np.array([])
    az_sim = np.array([])

    X_A = np.full((3,1),0.)        # ORIENTATION MATRIX
    C1 = np.full((3,3),0.)         # ROTATION MATRIX TO CONVERT TO 'Z-AXIS VERTICAL' ORIENTATION

    X = np.full((5,1),0.)          # STATE MATRIX
    W = np.full((5,1),0.)          # IDK WHAT YOU CALL THIS
    F = np.eye(5)                  # STATE TRANSITION MATRIX
    P = np.full((5,5),0.)          # UNCERTAINITY MATRIX
    Q = np.full((5,5),0.)          # UNCERTAINITY TRANSITION MATRIX
    R = np.eye(5)                  # SENSOR NOISE MATRIX OR SOMETHING
    Z = np.full((5,1),0.)          # MEASUREMENT MATRIX
    U = np.full((5,1),0.)          # CONTROL MATRIX

    dt = 1/30
    nx = ny = 1
    declination = -0.111*pi/180    # MAGNETIC DECLINATION OF LAUNCH SITE

    new_ram_data = False
    last_ram_diff = 0
    
    X[3][0] = 38
    Z[4][0] = 26.18
    X[4][0] = 26.18

    state = 0

    t_step = 0.01
    g1 = t_step*9.80665
    m = 8
    kd_max = 0.5*pi*pow(0.05,2)*0.7032/m
    kd_min = 0.5*pi*pow(0.05,2)*0.4468/m
    k1_max = kd_max*t_step
    k1_min = kd_min*t_step
    g = 9.80665

    target = 0
    motor_cutoff = 0
    set_target = False

    gyro1 = 0
    gyro2 = 0

    j = 0
    avg = 0
    speed = np.array([])
    abc = np.array([])
    gyro_pitch = speed_pitch = np.array([])

    az_min = 10
    
    y_low = y_high = 0
    
    vx = 0
    max_dec = min_dec = t_airbrakes = acc_airbrakes = np.array([])

    true_pitch = np.array([])
    p = 0
    reset = False

    def true_vy(T):
        if T < 744.856:
            return 17.5488424766*T*T - 26065.3463851055*T + 9678804.451577477
        elif T < 758.393:
            return 0.1073653644*T*T - 171.4810433614*T + 68294.3314416435
        else:
            return 0
            

    with open('final.csv') as csvfile:
        
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        
        n = 1
        
        for data in spamreader:
            
            if counter != 0 and counter%n == 0 and 0 < counter < 450:
                                            
                t = float(data[1])/1000
                
                dt = t - last_t
                
                state = int(data[2])
                temp = float(data[3]) + calib_temp
                y = float(data[4])
                ram_diff = float(data[5])
                accel = np.array([[float(data[6]),float(data[7]),float(data[8])]])
                gyro = np.array([[float(data[9]),float(data[10]),float(data[11])]])

                vy = (y - last_y)/(t - last_t)
                
                if ram_diff != last_ram_diff:
                    new_ram_data = True
                else:
                    new_ram_data = False
                    
                last_ram_diff = ram_diff          
                
                ax = accel[0][2]
                ay = -accel[0][1]
                az = accel[0][0]
                        
                a = sqrt(ax*ax + ay*ay + az*az)
                
                gx = gyro[0][2]*pi/180
                gy = gyro[0][1]*pi/180
                gz = gyro[0][0]*pi/180
                                
                course = 0

                if az <= -5.18 and motor_cutoff == 0:
                    motor_cutoff = t
                
                # RAM BAROMETER SCALE(RAW READING TO DIFFERENTIAL PRESSURE)
                k = 1/700
                
                dp = k*ram_diff
                p = pow(8.9611 - (y + calib_alt)/4947.19,5.2479)
                density = pow(8.9611 - (y + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y + calib_alt))
                if dp >= 0:
                    airspeed = sqrt(7*p/density*(pow(1 + dp/p,0.2857) - 1))
                    if new_ram_data:
                        t_speed = np.append(t_speed,t)
                        speed = np.append(speed,airspeed)
                else:
                    airspeed = 0


                # ATTITUDE KF ----------------------------------------------------------------------------------------------      
                # WAS SUPPOSED TO BE A KF BUT BNO DECIDED TO BE A LITTLE BITCH (WON'T LET MAGNETOMETER WORK IN HIGH-G MODE)                    
                # SO WE JUST INTEGRATE THE GYROS NOW

                e = abs(a-9.81)/9.81

                cutoff = 0.05
                
                # USE ACCELEROMETER TO CALCULATE PITCH AND ROLL IF ACCELERATION IS LOW AND LIFTOFF IS NOT DETECTED. ELSE GYRO.
                if e < cutoff:
                    X_A[0][0] = atan2(ay,az)    
                    X_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))
                else:
                    X_A[0][0] += nx*gx*dt
                    X_A[1][0] += ny*gy*dt

                # IF LIFTOFF IS DETECTED THEN USE GYRO FOR YAW (SECOND CONDITION FOR INSTANT DETECTION ON IGNITION). ELSE NO YAW.
                if e > cutoff:
                    X_A[2][0] += gz*dt
                else: 
                    X_A[2][0] = 0
                    
                if abs(X_A[0][0] + nx*gx*dt) >= pi/2:
                    nx *= -1
                if abs(X_A[1][0] + ny*gy*dt) >= pi/2:
                    ny *= -1
                
                # UPDATE ROTATION MATRIX
                C1[0][0] = cos(X_A[1][0])
                C1[0][1] = 0
                C1[0][2] = sin(X_A[1][0])
                
                C1[1][0] = sin(X_A[0][0])*sin(X_A[1][0])
                C1[1][1] = cos(X_A[0][0])
                C1[1][2] = -sin(X_A[0][0])*cos(X_A[1][0])
         
                C1[2][0] = -sin(X_A[1][0])*cos(X_A[0][0])
                C1[2][1] = sin(X_A[0][0])

                gyro1 += gx*dt
                gyro2 += gy*dt
                
                gyro_pitch = np.append(gyro_pitch,atan(sqrt(tan(gyro1)**2 + tan(gyro2)**2)) + pi/18)
                    
                if not motor_cutoff:
                    C1[2][2] = cos(atan(sqrt(tan(gyro1)**2 + tan(gyro2)**2)) + pi/18)
                else:
                    if abs(X[3][0]) < abs(X[4][0]):
                        C1[2][2] = X[3][0]/X[4][0]
                    else:
                        C1[2][2] = cos(atan(sqrt(tan(gyro1)**2 + tan(gyro2)**2)) + pi/18)

                                
        # -------------------------------------------------------------------------------------------------------------------
                
                last_alt = Z[2][0]
                
                # CALCULATE GRAVITY VECTOR
                gravity = np.dot(C1,np.array([[0],[0],[-9.80665]]))

                # CALCULATE ACCELERATION WRT GROUND
                lin_acc_x = - ax + gravity[0][0]
                lin_acc_y = - ay + gravity[1][0]
                lin_acc_z = az + gravity[2][0]
                
                # LATITUDE AND LONGITUDE UNCERTAINITIES DEPEND ON QUALITY OF FIX (HDOP). REST FIXED FOR NOW.
                Q[0][0] = 10
                Q[1][1] = 10
                
                Q[2][2] = 0.1
                Q[4][4] = 1
                
                if not motor_cutoff:
                    Q[3][3] = 0.01
                    Q[3][4] = 3
                    
                elif Z[3][0] > 70:
                    Q[3][3] = 0.0005
                    Q[3][4] = 5.5
                    
                else:
                    Q[3][3] = 0.001
                    Q[3][4] = -5
                    
                # ACCELERATIONS IN ROTATED FRAME (Z AXIS VERTICAL)
                if np.linalg.det(C1) > pow(10,-10) and counter > 1:
                    a_r = np.dot(np.linalg.inv(C1),np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
                else:
                    a_r = np.array([[0.],[0.],[0.]])
                
                dv = sqrt(pow(a_r[0][0],2) + pow(a_r[1][0],2))*dt
                
                # STATE TRANSITION MATRIX UPDATE
#                 F[0][4] = cos(course)*dt/r
#                 
#                 if abs(Z[0][0]*180/pi - 90) > 10:
#                     F[1][4] = sin(course)*dt/(r*cos(Z[0][0]))
#                 else:
#                     F[1][4] = 0
                    
                F[2][3] = dt
                
                # CONTROL MATRIX UPDATE
                U[2][0] = a_r[2][0]*dt*dt/2
                U[3][0] = a_r[2][0]*dt
                U[4][0] = lin_acc_z*dt
                
                Xp = np.dot(F,X) + U

                Pp = np.dot(np.dot(F,P),F.T) + Q

                # ALTITUDE AND VERTICAL VELOCTY READINGS
                Z[2][0] = y
                if counter != 1:
                    Z[3][0] = (Z[2][0] - last_alt)/(t - last_t)
                else:
                    Z[3][0] = 40
                
                # IF NO GPS FIX, USE IMU.        
                if new_ram_data and abs(airspeed) > abs(Xp[3][0]):
                    Z[4][0] = airspeed
                else:
                    Z[4][0] += lin_acc_z*dt

                if abs(Z[0][0]*180/pi - 90) > 10:
                    Z[1][0] += vx*dt*sin(course)/(r*cos(Z[0][0]))
                Z[0][0] += vx*dt*cos(course)/r
                
                last_X4 = X[4][0]
                
                K = np.dot(Pp,np.linalg.inv(Pp + R))

                X = Xp + np.dot(K,Z - Xp)
                
                P = np.dot(np.eye(len(K)) - K,Pp)
                    
                if abs(X[4][0]) > abs(X[3][0]):
                    vx = sqrt(X[4][0]**2 - X[3][0]**2)
                else:
                    vx += dv  
                
                last_y = y
                last_t = t
                
                az_min = min(az,az_min)
                
                time = np.append(time,t)
                alt = np.append(alt,X[3][0])
                vertical_rate = np.append(vertical_rate,Z[3][0])
                acc = np.append(acc,X[4][0])
                ias = np.append(ias,airspeed)
                
                t_sim = np.append(t_sim,true[counter][0])
                y_sim = np.append(y_sim,true[counter][3])
                vx_sim = np.append(vx_sim,true[counter][11])
                vy_sim = np.append(vy_sim,true[counter][12])
                v_sim = np.append(v_sim,true[counter][13])
                pitch_sim = np.append(pitch_sim,true[counter][14])
                az_sim = np.append(az_sim,true[counter][5])
                
                try:
                    pitch = np.append(pitch,acos(C1[2][2]))
                except:
                    if len(pitch) != 0:
                        pitch = np.append(pitch,pitch[-1])
                    else:
                        pitch = np.append(pitch,0)
                
                if state == 2 and motor_cutoff:
                    
                    y_high = y_low = X[2][0]
                    v_high = v_low = X[4][0]
                    pitch_high = pitch_low = acos(C1[2][2])
                    vy_high = vy_low = v_high*cos(pitch_high)
            
#                     for p in range(int(pitch[-1]*180/pi-20),int(pitch[-1]*180/pi+20)):
#                         
#                         y_high = y_low = X[2][0]
#                         v_high = v_low = X[4][0]
#                         pitch_high = pitch_low = p*pi/180
#                         vy_high = vy_low = v_high*cos(pitch_high)
#                         
#                         while vy_high > 0:
#                             density = pow(8.9611 - (y_high + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y_high + calib_alt))
#                             pitch_high += g1*sin(pitch_high)/v_high
#                             v_high += -(g1*cos(pitch_high) + k1_min*density*v_high*v_high)
#                             vy_high += -(g1 + k1_min*density*v_high*v_high*cos(pitch_high))
#                             y_high += vy_high*t_step
#                             
#                         if abs(y_high - 1025.71) < 5:
#                             true_pitch = np.append(true_pitch,p)
#                             break
                    
                    while vy_high > 0:
                        density = pow(8.9611 - (y_high + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y_high + calib_alt))
                        pitch_high += g1*sin(pitch_high)/v_high
                        v_high += -(g1*cos(pitch_high) + k1_min*density*v_high*v_high)
                        vy_high += -(g1 + k1_min*density*v_high*v_high*cos(pitch_high))
                        y_high += vy_high*t_step
                        
                    while vy_low > 0:
                        density = pow(8.9611 - (y_low + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y_low + calib_alt))
                        pitch_low += g1*sin(pitch_low)/v_low
                        v_low += -(g1*cos(pitch_low) + k1_max*density*v_low*v_low)
                        vy_low += -(g1 + k1_max*density*v_low*v_low*cos(pitch_low))
                        y_low += vy_low*t_step
                                    
                    density = pow(8.9611 - (y + calib_alt)/4947.19,5.2479)/(78410.439 + 287.06*calib_temp - 1.86589*(y + calib_alt))
                            
                    if t - motor_cutoff > 2 and not set_target:
                        target = (y_high + y_low)/2
#                         print(t,y_high,y_low)
#                         print('Target : ' + str(target))
                        set_target = True
                    if y_high != y_low:
                        alpha = (target - y_low)/(y_high - y_low)
                    else:
                        alpha = 1
                    
                    if alpha > 1:
                        alpha = 1
                    elif alpha < 0:
                        alpha = 0

                    t_airbrakes = np.append(t_airbrakes,t)
                    acc_airbrakes = np.append(acc_airbrakes,-az)
                    min_dec = np.append(min_dec,0.5*density*pi*pow(0.05,2)*0.4468*pow(airspeed,2)/m)
                    max_dec = np.append(max_dec,0.5*density*pi*pow(0.05,2)*0.7032*pow(airspeed,2)/m)
                
                else:
                    true_pitch = np.append(true_pitch,0)
                
                low_array = np.append(low_array,y_low)
                high_array = np.append(high_array,y_high)
                
            else:
                pass
            
            counter += 1
            
    if error < min_error:
        print(q,error)
        min_error = error
            
    q += 0.001

plt.plot(time,gyro_pitch*180/pi)
plt.plot(time,vertical_rate)
plt.plot(time,alt)
plt.plot(time,ias)
plt.plot(time,acc)
plt.plot(time,pitch*180/pi)

plt.plot(time,low_array)
plt.plot(time,high_array)

plt.plot(t_sim,vy_sim)
plt.plot(t_sim,pitch_sim*180/pi)
# plt.plot(time,np.append(np.array([]),true_pitch))

# plt.ylim(-20,200)

plt.axhline(y = 1025.71, color = 'r', linestyle = '-')
plt.axvline(x = motor_cutoff, color = 'b', label = 'axvline - full height')

plt.show()