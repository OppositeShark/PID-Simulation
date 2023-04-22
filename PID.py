# -*- coding: utf-8 -*-
"""
Created on Tue Jan 18 14:49:27 2022

@author: degog
"""
import time
import math
import random
import tkinter as tk

window = tk.Tk()
window.title("PID Car")
width, height = (1000, 600)

class stats():
    divisions = 1
    timeScale = 2
    targetRange = 5
    pixelsPerUnit = height/2/(targetRange+1)

class car():
    maxVolts = 1
    noise = 0
    mass = 10
    friction = 1
    xspeed = 200
    def __init__(self, y):
        self.x = 0
        self.y = y
        self.vy = 0
        self.target = 0
        self.pid = None
        
    def setPID(self, pid):
        self.pid = pid
    
    def run(self, dt):
        self.x += 100*dt
        self.y += self.vy*dt
        self.vy += random.uniform(-car.noise, car.noise)*dt
        
        if self.isAtTarget():
            self.newTarget()
            self.pid.reset()
            
        result = self.pid.calc(self.getDistanceToTarget(), dt)
        volts = result
        if(volts > car.maxVolts):
            volts = car.maxVolts
        elif(volts < -car.maxVolts):
            volts = -car.maxVolts
        self.vy += volts
     
    def isAtTarget(self):
        if self.pid == None:
            return False
        return self.pid.isAtTarget()
    
    
    def newTarget(self):
        self.target = random.randint(-stats.targetRange,stats.targetRange)
        
    def getTargetY(self):
        return self.target
    
    def getDistanceToTarget(self):
        return self.getTargetY()-self.y
    
    turnScale = 100
    def turn(self, ang, dt):
        self.ang += ang*dt*(self.v/car.turnScale)
        
    def getScreenXY(self):
        return (self.x, self.y*stats.pixelsPerUnit)
    
    def draw(self, canvas, offset):
        width = 40
        height = 20
        x = self.x+offset[0]
        y = self.y*stats.pixelsPerUnit+offset[1]
        canvas.create_rectangle(x-width/2, 
                                y-height/2, 
                                x+width/2, 
                                y+height/2, 
                                fill = 'black')
        canvas.create_text(x, y+30, text = "Position: %.2f" %(self.y))
    
    def drawTarget(self, canvas, offset):
        y = self.getTargetY()*stats.pixelsPerUnit + offset[1]
        canvas.create_line(0,
                           y,
                           width,
                           y,
                           width = 2,
                           dash = 10, 
                           fill = 'blue')
        canvas.create_text(width/2, y+30, text = "Target: %.2f" %(self.getTargetY()))
        if self.pid != None:
            errorPixels = self.pid.errorThreshold*stats.pixelsPerUnit
            canvas.create_line(0,
                               y-errorPixels,
                               width,
                               y-errorPixels,
                               width = 1,
                               dash = 10, 
                               fill = 'light green')
            canvas.create_line(0,
                               y+errorPixels,
                               width,
                               y+errorPixels,
                               width = 1,
                               dash = 10, 
                               fill = 'light green')
        
    def reset(self):
        self.x = 0
        self.y = 0
        self.vy = 0
        self.newTarget()
        
class trails():
    trails = []
    def addTrail(xy):
        trails.trails += list(xy)
        if len(trails.trails) > 500:
            trails.trails.pop(0)
            trails.trails.pop(0)
    
    def draw(canvas, offset):
        if(len(trails.trails) < 4):
            return
        offsetPoints = [i+offset[n%2] for n, i in enumerate(trails.trails)]
        canvas.create_line(*offsetPoints)
        
class PID():
    lengthSpeedData = 1000
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.speedThreshold = 0.1
        self.errorThreshold = 0.1
        self.lastError = 0
        self.sumError = 0
        self.atTarget = True
        
        self.timeElapsed = 0
        self.startError = None
        self.avgSpeeds = []
      
    def calc(self, error, dt):
        if self.startError == None:
            self.startError = error
            self.lastError = error
        self.sumError += error*dt
        p = error*self.kp
        i = self.sumError*self.ki
        d = (error-self.lastError)/dt*self.kd
        self.atTarget = abs(error - self.lastError)/dt < self.speedThreshold and abs(error) < self.errorThreshold
        self.lastError = error
        self.timeElapsed += dt
        return p+i+d
    
    def isAtTarget(self):
        return self.atTarget
    
    def reset(self):
        #Add to list of speeds
        if self.startError not in (0, None) and self.timeElapsed != 0:
            self.avgSpeeds.append(abs(self.startError/self.timeElapsed))
        while len(self.avgSpeeds) > PID.lengthSpeedData:
            self.avgSpeeds.pop(0)
        if len(self.avgSpeeds) > 0:
            timeLabel.set("Average Speed To Target: "+str(sum(self.avgSpeeds)/len(self.avgSpeeds)))
        #Reset stats
        self.startError = None
        self.sumError = 0
        self.timeElapsed = 0
        
        #Updating display
        for display in varDisplay.displays:
            display.updateSelf()

class tuner():
    def __init__(self, pid):
        self.pid = pid
        self.realpid = [pid.kp, pid.ki, pid.kd]
        self.stepScaleScale = 15
        self.batches = 50
        self.timeGiven = 10
        self.dp = 0;
        self.di = 0;
        self.dd = 0;
        self.stepScale = 1;
        self.batch = 0
    
    def periodic(self):
        if self.pid.timeElapsed > self.timeGiven:
            self.timeElapsed += 2*self.pid.lastError-self.lastError
            self.reset()
    
    def autoTune(self):
        newP = self.realpid[0] - (self.stepScale*self.dp/PID.batches)
        newI = self.realpid[1] - (self.stepScale*self.di/PID.batches)
        newD = self.realpid[2] - (self.stepScale*self.dd/PID.batches)
        print(self.dp/PID.batches, self.di/PID.batches, self.dd/PID.batches)
        self.stepScale = abs(sum([self.dp, self.di, self.dd])/PID.stepScaleScale/PID.batches)
        self.dp = 0
        self.di = 0
        self.dd = 0
        self.realpid = [newP, newI, newD]
        print(self.realpid)
        print(self.stepScale)
        
    def reset(self):
        if (self.tune) and (self.startError not in (0, None)):
            self.dp += (self.kp - self.realpid[0])*self.timeElapsed/self.startError
            self.di += (self.ki - self.realpid[1])*self.timeElapsed/self.startError
            self.dd += (self.kd - self.realpid[2])*self.timeElapsed/self.startError
            
            self.kp, self.ki, self.kd = self.realpid
            self.kp += random.uniform(-self.stepScale, self.stepScale)
            self.ki += random.uniform(-self.stepScale, self.stepScale)
            self.kd += random.uniform(-self.stepScale, self.stepScale)
                
            self.batch += 1
            if self.batch >= PID.batches:
                self.autoTune()
                self.batch = 0
    
    def setToRealValues(self):
        self.kp, self.ki, self.kd = self.realpid
        
    def setRealValues(self):
        self.realpid = [self.kp, self.ki, self.kd]
        
    def toggleTuning():
        if pid1.tune:
            pid1.tune = False
            pid1.setToRealValues()
            stats.divisions = 2
            stats.timeScale = 1
        else:
            pid1.tune = True
            pid1.setRealValues()
            stats.divisions = 40
            stats.timeScale = 40
        
car1 = car(0)
pid1 = PID(0,0,0)
car1.setPID(pid1)
tuner1 = tuner(pid1)
lastTime = time.time()

def reset():
    car1.reset()
    trails.trails.clear()

def main():
    #Get change in time between frames
    global lastTime
    dt = time.time() - lastTime
    lastTime = time.time()
    #Running code
    for i in range(int(stats.divisions)):
        if stats.timeScale != 0:
            car1.run(dt*stats.timeScale/stats.divisions)
        else:
            car1.run(dt/stats.divisions)
    #Drawing
    trails.addTrail(car1.getScreenXY())
    #Offsets to center the car
    offsetx = width/4*3 - car1.getScreenXY()[0]
    offsety = height/2
    offset = (offsetx, offsety)
    #Drawing
    canvas.delete('all')
    canvas.create_rectangle(0,0,width,height, fill = 'white')
    car1.draw(canvas, offset)
    car1.drawTarget(canvas, offset)
    trails.draw(canvas, offset)
    try:
        window.after(20, main)
    except:
        return

class varDisplay(tk.Frame):
    displays = []
    def __init__(self, master, obj, varName):
        tk.Frame.__init__(self, master)
        self.obj = obj
        self.varName = varName
        tk.Label(self, text = varName).pack(side = 'left')
        self.textVar = tk.StringVar()
        self.textVar.set(str(getattr(self.obj, self.varName)))
        entry = tk.Entry(self, textvariable = self.textVar)
        entry.bind("<KeyRelease>",self.updateVar)
        entry.pack(side = 'left')
        varDisplay.displays.append(self)
    
    def updateVar(self, *args):
        try:
            setattr(self.obj, self.varName, float(self.textVar.get()))
            print("Set " + self.varName + " to " + self.textVar.get())
        except:
            pass
        
    def updateSelf(self):
        self.textVar.set(str(getattr(self.obj, self.varName)))

canvas = tk.Canvas(window, width=width, height=height)
canvas.pack(side = 'left')

stuff = tk.Frame(window)
stuff.pack(side = 'left')
varDisplay(stuff, car, 'maxVolts').pack()
varDisplay(stuff, pid1, 'kp').pack()
varDisplay(stuff, pid1, 'ki').pack()
varDisplay(stuff, pid1, 'kd').pack()
varDisplay(stuff, car, 'noise').pack()
varDisplay(stuff, pid1, 'speedThreshold').pack()
varDisplay(stuff, pid1, 'errorThreshold').pack()
varDisplay(stuff, stats, 'timeScale').pack()
varDisplay(stuff, stats, 'divisions').pack()


tk.Button(stuff, text = 'Auto Tune', command = tuner1.toggleTuning).pack()
tk.Button(stuff, text = "reset", command = reset).pack()
varDisplay(stuff, PID, 'lengthSpeedData').pack()
timeLabel = tk.StringVar()
timeLabel.set("Seconds per 10 Pixels:")
tk.Label(stuff, textvariable = timeLabel).pack()

window.after(10, main)
window.mainloop()
window.quit()