# -*- coding: utf-8 -*-
"""
Created on Wed Jan  4 19:07:50 2023

@author: degog
"""

import random
import tkinter as tk

window = tk.Tk()
window.title("PID Car")
width, height = (1000, 600)
canvas = tk.Canvas(window, width=width, height=height)
canvas.pack(side = 'left')
labels = tk.Frame(window)
labels.pack(side = 'left')

class stats():
    simDivisions = 1
    simSpeed = 1
    targetRange = 5
    pixelsPerUnit = height/2/(targetRange+1)
    gravity = 9.8
    carPos = width/4*3

class car():
    def __init__(self):
        self.y = 0
        self.vy = 0
        self.power = 0
        #https://www.motioncontroltips.com/faq-whats-relationship-voltage-dc-motor-output-speed/
        self.maxPower = 10
        self.mass = 100
        self.kFrict = 0.1
        self.wheelRadius = 0.5
        self.backEmfKE = 1.0
        self.torqueConstant = 7.0
        self.resistance = 0.01
        
    def run(self, dt):
        backEmf = self.backEmfKE*(self.vy/self.wheelRadius)
        I = (self.power - backEmf) / self.resistance
        torque = I * self.torqueConstant
        force = torque * self.wheelRadius
        self.vy += (force / self.mass) * dt
        friction = self.kFrict * self.mass * dt
        if self.vy > friction:
            self.vy -= friction
        elif self.vy < -friction:
            self.vy += friction
        else:
            self.vy = 0
        self.y += self.vy*dt
        
    def setPower(self, power):
        if power > self.maxPower:
            self.power = self.maxPower
        elif power < -self.maxPower:
            self.power = -self.maxPower
        else:
            self.power = power
    
    def draw(self, canvas, x, yOffset):
        width = 40
        height = 20
        y = self.y*stats.pixelsPerUnit + yOffset
        canvas.create_rectangle(x-width/2, 
                                y-height/2, 
                                x+width/2, 
                                y+height/2, 
                                fill = 'black')
        canvas.create_text(x, y+30, text = "Position: %.2f" %(self.y))
    
    def reset(self):
        self.vy = 0
        self.y = 0
        self.power = 0
    
    def getPos(self):
        return self.y

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.startError = None
        self.lastError = 0
        self.sumError = 0
        self.timeElapsed = 0
        
        self.speedThreshold = 0.01
        self.errorThreshold = 0.05
        
        self.target = 0
    
    def getError(self, position):
        return self.target-position
    
    def calc(self, pos, dt):
        return self.calcUsingError(self.target-pos, dt)
    
    def calcUsingError(self, error, dt):
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
    
    def newTarget(self, centered = True):
        self.reset()
        oldTarget = self.target
        if centered:
            while self.target != 0:
                self.target = random.randint(-stats.targetRange,stats.targetRange)
        else:
            while self.target == oldTarget:
                self.target = random.randint(-stats.targetRange,stats.targetRange)
        
    def setTarget(self, target):
        self.target = target
        
    def drawTarget(self, canvas, yOffset, color = "blue"):
        y = self.target*stats.pixelsPerUnit + yOffset
        canvas.create_line(0,
                           y,
                           width,
                           y,
                           width = 2,
                           dash = 10, 
                           fill = color)
        canvas.create_text(width/2, y+20, text = "Target: %.2f" %(self.target))
        errorPixels = self.errorThreshold*stats.pixelsPerUnit
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
        #Reset stats
        self.startError = None
        self.sumError = 0
        self.timeElapsed = 0
        
    def getPID(self):
        return self.kp, self.ki, self.kd
        
class trails():
    trails = []
    def addTrail(xy):
        trails.trails += list(xy)
        if len(trails.trails) > 800:
            trails.trails.pop(0)
            trails.trails.pop(0)
    
    def draw(canvas, offset):
        if(len(trails.trails) < 4):
            return
        offsetPoints = [i+offset[n%2] for n, i in enumerate(trails.trails)]
        canvas.create_line(*offsetPoints)
        
class autoTunerBruteForce():
    def __init__(self, pid):
        self.pid = pid
        
        self.error = 0
        self.errorVals = []
        self.timeLimit = 5
        
        self.divisions = 3
        self.batches = 10
        self.divisionNum = 0
        self.batchNum = 0
        self.iterations = 0
        
        self.fixPID = [False, False, False] #Choose not to change in tuning
        self.pRange = [0, 10]
        self.iRange = [0, 10]
        self.dRange = [0, 10]
    
    def run(self, dt):
        error = self.pid.lastError
        if error * self.pid.startError < 0:
            self.error += abs(error) * 2
        else:
            self.error += abs(error)
        #self.error += abs(self.pid.lastError)
        #self.error += dt
        if self.pid.timeElapsed > self.timeLimit:
            self.reset()
            c.reset()
            self.pid.newTarget()
        
    def reset(self):
        if len(self.errorVals) <= self.divisionNum:
            self.errorVals += [0]*(self.divisionNum-len(self.errorVals)+1)
        if self.pid.startError == 0:
            return
        #self.errorVals[self.divisionNum] += self.error/abs(self.pid.startError)
        self.errorVals[self.divisionNum] += self.error*self.pid.timeElapsed
        self.error = 0
        self.distance = 0
        self.batchNum += 1
        if self.batchNum >= self.batches:
            self.divisionNum += 1
            self.setDivision()
            pow3 = self.divisions*self.divisions*self.divisions
            if self.divisionNum >= pow3:
                self.setNextRange()
                self.divisionNum = 0
                self.iterations += 1
            self.batchNum = 0
                
    def setDivision(self):
        pIndex = self.divisionNum % self.divisions
        pSize = self.pRange[1] - self.pRange[0]
        self.pid.kp = ((pIndex + 0.5)/ self.divisions * pSize) + self.pRange[0]
        iIndex = (self.divisionNum // self.divisions) % self.divisions
        iSize = self.iRange[1] - self.iRange[0]
        self.pid.ki = ((iIndex + 0.5)/ self.divisions * iSize) + self.iRange[0]
        dIndex = (self.divisionNum // (self.divisions*self.divisions))% self.divisions
        dSize = self.dRange[1] - self.dRange[0]
        self.pid.kd = ((dIndex + 0.5)/ self.divisions * dSize) + self.dRange[0]
        
    def setNextRange(self):
        minIndex = 0
        minError = self.errorVals[0]
        pow3 = self.divisions*self.divisions*self.divisions
        for i in range(pow3):
            if self.errorVals[i] < minError:
                minIndex = i
                minError = self.errorVals[i]
        pIndex = minIndex % self.divisions
        pSize = self.pRange[1] - self.pRange[0]
        self.pRange[0] += pIndex/self.divisions*pSize
        self.pRange[1] = self.pRange[0] + pSize/self.divisions
        iIndex = (minIndex // self.divisions) % self.divisions
        iSize = self.iRange[1] - self.iRange[0]
        self.iRange[0] += iIndex/self.divisions*iSize
        self.iRange[1] = self.iRange[0] + iSize/self.divisions
        dIndex = (minIndex // (self.divisions*self.divisions))% self.divisions
        dSize = self.dRange[1] - self.dRange[0]
        self.dRange[0] += dIndex/self.divisions*dSize
        self.dRange[1] = self.dRange[0] + dSize/self.divisions
                    
    def fullReset(self):
        self.divisionNum = 0
        self.batchNum = 0
        self.iterations = 0
        
c = car()
cPID = PID(10,0,3)
#cPID2 = PID(1,0,0)
brute = autoTunerBruteForce(cPID)

class varDisplay(tk.Frame):
    displays = []
    def __init__(self, master, obj, varName, edit = True, varType = float, rang = "(-inf,inf)", isVariable = True):
        tk.Frame.__init__(self, master)
        self.obj = obj
        self.edit = edit
        self.varType = varType
        self.varName = varName
        self.textVar = tk.StringVar()
        #Range
        split = rang.split(",")
        self.min = float(split[0][1:])
        self.max = float(split[1][:-1])
        #Name of variable
        tk.Label(self, text = "%s"%(varName)).pack(side = 'left', padx = 10)
        if isVariable:
            #Value
            if edit:
                entry = tk.Entry(self, textvariable = self.textVar)
                entry.bind("<KeyRelease>",self.updateVar)
                entry.pack(side = 'right')
                self.textVar.set(str(getattr(self.obj, self.varName)))
            if not edit:
                label = tk.Label(self, textvariable = self.textVar)
                label.pack(side = 'right')
                label.config({"background" : "Light Gray", "width" : "15"})
                varDisplay.displays.append(self)
                self.textVar.set(str(getattr(self.obj, self.varName)))
        else:
            label = tk.Label(self, textvariable = self.textVar)
            label.pack(side = 'right')
            label.config({"background" : "Light Gray", "width" : "15"})
    
    def updateVar(self, *args):
        if self.edit:
            try:
                val = self.varType(self.textVar.get())
                if self.varType in (float, int):
                    if val <= self.min or val > self.max:
                        return
                setattr(self.obj, self.varName, val)
                print("Set " + self.varName + " to " + self.textVar.get())
            except:
                pass
        
    def updateSelf(self):
        if self.varType == float:
            self.textVar.set("%.4f" %(getattr(self.obj, self.varName)))
        else:
            self.textVar.set(str(getattr(self.obj, self.varName)))
    
    def updateSelfWithValue(self, string):
        self.textVar.set(string)    
    
    def updateAll():
        for display in varDisplay.displays:
            display.updateSelf()
            
    def updateVarDisplayList(l):
        for display in l:
            display.updateSelf()

def reset():
    c.reset()
    cPID.reset()
    brute.fullReset()
    #cPID2.reset()

tuning = False
def toggleTuning():
    global tuning
    tuning = not tuning
    if tuning:
        autoButton.config({"background" : "light green"})
        brute.setDivision()
        reset()
    else:
        autoButton.config({"background" : "white"})
        brute.fullReset()
    print("Tuning is %s" %(tuning))

varDisplay(labels, stats, 'simSpeed', rang = "(0,inf)").pack()
varDisplay(labels, stats, 'simDivisions', rang = "(0,inf)", varType = int).pack()

varDisplay(labels, c, 'maxPower', rang = "(0,inf)").pack()

varDisplay(labels, c, 'mass', rang = "(0,inf)").pack()
varDisplay(labels, c, 'kFrict', rang = "(0,inf)").pack()
varDisplay(labels, c, 'wheelRadius', rang = "(0,inf)").pack()
varDisplay(labels, c, 'backEmfKE', rang = "(0,inf)").pack()
varDisplay(labels, c, 'torqueConstant', rang = "(0,inf)").pack()
varDisplay(labels, c, 'resistance', rang = "(0,inf)").pack()  

pidDisplays = [ 
    varDisplay(labels, cPID, 'kp'),
    varDisplay(labels, cPID, 'ki'),
    varDisplay(labels, cPID, 'kd')
]
for display in pidDisplays:
    display.pack()
#varDisplay(labels, cPID2, 'kp').pack()
#varDisplay(labels, cPID2, 'ki').pack()
#varDisplay(labels, cPID2, 'kd').pack()
varDisplay(labels, cPID, 'errorThreshold').pack()
varDisplay(labels, cPID, 'speedThreshold').pack()

varDisplay(labels, c, 'vy', edit = False).pack()
outputDisplay = varDisplay(labels, None, 'pid output', isVariable=False)
outputDisplay.pack()
      
tk.Button(labels, text = "reset", command = reset).pack()

def boolList(s):
    l = eval(s)
    assert isinstance(l, list)
    assert all([isinstance(i, bool) for i in l])
    return l

def rangeList(s):
    l = eval(s)
    assert isinstance(l, list)
    assert len(l) == 2
    return [float(i) for i in l]

bruteAutoUpdateDisplays = [
    varDisplay(labels, brute, 'fixPID', varType = boolList),
    varDisplay(labels, brute, 'pRange', varType = rangeList),
    varDisplay(labels, brute, 'iRange', varType = rangeList),
    varDisplay(labels, brute, 'dRange', varType = rangeList)
]
for display in bruteAutoUpdateDisplays:
    display.pack()
varDisplay(labels, brute, 'divisions', varType = int, rang = "(0,inf)").pack()
varDisplay(labels, brute, 'batches', varType = int, rang = "(0,inf)").pack()
varDisplay(labels, brute, 'divisionNum', edit = False, varType = int).pack()
varDisplay(labels, brute, 'batchNum', edit = False, varType = int).pack()
varDisplay(labels, brute, 'iterations', edit = False, varType = int).pack()

autoButton = tk.Button(labels, text = 'Auto Tune', command = toggleTuning)
autoButton.pack()

time = 0
output = 0
def calc():
    global time
    global output
    dt = 0.02
    time += dt
    #PID calc
    pos = c.getPos()
    output = cPID.calc(pos, dt)
    #output2 = cPID2.calcUsingError(output, dt)
    c.setPower(output)
    if tuning:
        brute.run(dt)
    if cPID.atTarget:
        if tuning:
            brute.reset()
            varDisplay.updateVarDisplayList(pidDisplays)
            varDisplay.updateVarDisplayList(bruteAutoUpdateDisplays)
        cPID.newTarget(False)
    #Run simulation
    for i in range(stats.simDivisions):
        c.run(dt/stats.simDivisions)
    #Scrolling
    x = time*stats.pixelsPerUnit*2.5
    #Add trail point
    trails.addTrail([stats.carPos + x, pos*stats.pixelsPerUnit])

def main():
    calc()
    #Extra calc if runs faster, also time between frames
    ms = int(20/stats.simSpeed)
    if ms < 1:
        for i in range(int(stats.simSpeed/20) - 1):
            calc()
        ms = 1
    #Scrolling
    x = time*stats.pixelsPerUnit*2.5
    #Variable Display
    varDisplay.updateAll()
    outputDisplay.updateSelfWithValue("%.2f" %(output))
    #Drawing
    canvas.delete('all')
    canvas.create_rectangle(0,0,width,height, fill = 'white')
    #Offsets to center the car
    trails.draw(canvas, [-x, height/2])
    c.draw(canvas, stats.carPos, height/2)
    cPID.drawTarget(canvas, height/2)
    #cPID2.setTarget(pos + output)
    #cPID2.drawTarget(canvas, height/2, color = "dark blue")
    window.after(ms, main)

window.after(10, main)
window.mainloop()
window.quit()