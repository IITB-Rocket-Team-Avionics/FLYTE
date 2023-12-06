import Statemachine
#import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

#csvfile = pd.read_csv("log.csv")
data = np.loadtxt('data.csv',delimiter=',')

FlightArray = Statemachine.DataTab()
StM = Statemachine.StateMachine(FlightArray)

t = []
a = []
l = []

for i in range(len(data) - 1):
    FlightArray.time =  data[i][0]
    t.append(data[i][0])
    FlightArray.a_x = data[i][2]
    a.append(data[i][2])
#    FlightArray.a_y = data[i][3]
#    FlightArray.a_z = data[i][5]
    FlightArray.alt = data[i][5]
    l.append(data[i][5])
    FlightArray.vel = data[i][6]

    StM.update()

plt.plot(t, a, label = "accel")
plt.plot(t, l, label = "alt")
plt.legend()
#plt.xlim(55000, 70000)

plt.axvline(x = StM.liftoff)
plt.axvline(x = StM.cutoff)
plt.axvline(x = StM.apogee)
plt.axvline(x = StM.drogueOut)
plt.axvline(x = StM.mainsOut)
#plt.axvline(x = StM.touchdown)
print(StM.liftoff)
print(StM.cutoff)
print(StM.apogee)
print(StM.drogueOut)
print(StM.mainsOut)
print(StM.touchdown)

plt.show()