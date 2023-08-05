import math

class StateMachine:
    def __init__(self, table):
        self.table = table
        self.state = 0
        self.lastUpdate = -10000
        self.alt_buf = 0
        self.liftoff = None
        self.cutoff = None
        self.apogee = None
        self.drogueOut = None
        self.mainsOut = None
        self.touchdown = None
        
    def stateA(self):
        match self.state:
            case 0:
                return "Idle"
            case 1:
                return "Boost"
            case 2:
                return "Coasting"
            case 3:
                return "Descent"
            case 4:
                return "Descent with drogue"
            case 5:
                return "Descent with mains"
            case 6:
                return "Touchdown"
    
    def update(self):
        match self.state:
            case 0:
                if((self.table.a_x >= 30 and self.table.time - self.lastUpdate > 0.5)and self.alt_buf < self.table.alt and self.table.alt >= 15):
                    self.state = 0
                    self.liftoff = self.table.time
                elif(self.table.a_x < 3):
                    self.lastUpdate = self.table.time
                self.alt_buf = self.table.alt

            case 1:
                if(self.table.a_x < -9.8 or self.table.time - self.liftoff > 4):
                    self.state = 2
                    self.cutoff = self.table.time

            case 2:
                if(abs(self.table.a_x) < 3 and self.alt_buf - self.table.alt > 5):
                    self.state = 3
                    self.apogee = self.table.time
                elif(self.alt_buf < self.table.alt):
                    self.alt_buf = self.table.alt

            case 3:
                if((self.table.time - self.apogee > 0.5 and self.table.time - self.liftoff > 18)or(self.table.time - self.liftoff > 27)):
                    self.state = 4
                    self.drogueOut = self.table.time
                    self.mainsOut = self.drogueOut + ((self.table.alt - 420) / 25)

            case 4:
                if(self.table.time > self.mainsOut or self.table.alt < 420):
                    self.state = 5
                    self.mainsOut = self.table.time
                    self.lastUpdate = self.mainsOut
                    self.alt_buf = self.table.alt

            case 5:
                if(abs(math.sqrt(self.table.a_x ^ 2 + self.table.a_y ^ 2 + self.table.a_z ^ 2)) < 1 or abs((self.alt_buf - self.table.alt) / (self.lastUpdate - self.table.time)) < 1):
                    self.state = 6
                else:
                    self.lastUpdate = self.table.time
                    self.alt_buf = self.table.alt
            case 6:
                return
            
    def updateA(self):
        self.update()
        return self.stateA()

class DataTab:
    def __init__(self):
        self.time = 0
        self.alt = 0
        self.a_x = 0
        self.a_y = 0
        self.a_z = 0
        self.vel = 0
        self.bmp = bmp()

class bmp :
    def __init__(self):    
        self.pres = 0
        self.temp = 0

