# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 22:38:54 2022

@author: degog
"""

import time
import math
import random
import tkinter as tk
import sys
import os

def rotatePoint(x,y,a):
    nx =  x*math.cos(a) + y*math.sin(a)
    ny = -x*math.sin(a) + y*math.cos(a)
    return [nx, ny]

scale = 50/1
class Target():
    x = 7
    speedThreshold = 0.1
    distanceThreshold = 0.1
    
    def check():
        distance = abs(Richard.x - Target.x)
        speed = abs(Richard.vx)
        return ((distance < Target.distanceThreshold) and 
                (speed < Target.speedThreshold))
    
    drawids = []
    def new():
        oldx = Target.x
        newx = random.uniform(2, CANVASWIDTH/scale - 2)
        while abs(newx - Richard.x) < 2:
            newx = random.uniform(2, CANVASWIDTH/scale - 2)
        diff = newx-oldx
        for i in Target.drawids:
            canvas.move(i, diff*scale, 0)
        Target.x = newx
    
    def draw():
        x = Target.x*scale
        if len(Target.drawids) == 0:
            Target.drawids += [
                canvas.create_rectangle(x-3,
                                        FLOORHEIGHT - 50,
                                        x+3,
                                        FLOORHEIGHT)
                ]
        
class Richard():
    x  = 1
    vx = 0
    mass = 10
    power = 0
    enabled = False
    friction = 1.6
    target = Target
    
    def run(dt):
        Richard.x += Richard.vx*dt
        Richard.vx += Richard.power/Richard.mass*dt
        friction = Richard.friction*dt
        if(friction > abs(Richard.vx)):
            Richard.vx = 0
        elif Richard.vx < 0:
            Richard.vx += friction
        else:
            Richard.vx -= friction
        if Richard.target.check():
            Richard.target.new()
            try:
                RichardCode.NewTarget()
            except Exception as e:
                disable()
                print(e)
                tk.messagebox.showinfo("Runtime Error", str(e))
        
    def runCode():
        if not Richard.enabled:
            return
        try:
            RichardCode.Periodic()
        except Exception as e:
            disable()
            print(e)
            tk.messagebox.showinfo("Runtime Error", str(e))
        
    drawIds = []
    WHEELSIZE = 30
    bodyx = 10
    bodyy = 40
    headx = 15
    heady = 20
    BODYCOORDS = [(-bodyx, 0),
                  (-bodyx, bodyy),
                  (-headx, bodyy),
                  (-headx, bodyy+heady),
                  (headx, bodyy+heady),
                  (headx, bodyy),
                  (bodyx, bodyy),
                  (bodyx, 0)]
    def draw():
        #Remove old drawn Richard
        for i in Richard.drawIds:
            canvas.delete(i)
        Richard.drawIds.clear()
        
        #Draw New Richard
        RichardScale = math.sqrt(Richard.mass)
        wheelr = Richard.WHEELSIZE/2 * RichardScale
        wheelx = Richard.x*scale
        wheely = FLOORHEIGHT - wheelr
        #Rotate body
        ang = math.atan(Richard.vx)*(2/3)
        newBody = []
        for point in Richard.BODYCOORDS:
            point = (point[0]*RichardScale, point[1]*RichardScale)
            newPoint = rotatePoint(*point, ang)
            newPoint[0] += wheelx
            #Flip y axis
            newPoint[1] *= -1
            newPoint[1] += wheely
            newBody.append(newPoint)
        #Add to richard's drawing ids & draw
        Richard.drawIds += [
            canvas.create_oval(wheelx - wheelr,
                               wheely - wheelr,
                               wheelx + wheelr,
                               wheely + wheelr),
            canvas.create_polygon(*newBody,
                                  fill = "#FFFFFF",
                                  width = 1,
                                  outline = "#000000")
            ]

class RichardController():
    def getDistance():
        return Richard.x - Target.x
    
    def setPower(x):
        threshold = Richard.mass*30
        if x > threshold:
            x = threshold
        elif x < -threshold:
            x = -threshold
        Richard.power = x
        
    def getPosition():
        return Richard.x
    
    def getTarget():
        return Richard.target.x

lastFrame = time.time()
DIVISIONS = 10
def main():
    global lastFrame
    dt = time.time() - lastFrame
    lastFrame = time.time()
    
    Richard.runCode()
    for i in range(DIVISIONS):
        Richard.run(dt/DIVISIONS)
    Richard.draw()
    
    try:
        window.after(20, main)
    except:
        pass

class Dashboard():
    values = {}
    entries = {}
    
    def putNumber(name, value):
        Dashboard.values[name] = value
        if name not in Dashboard.entries.keys():
            frame = tk.Frame(DashboardFrame)
            frame.pack()
            tk.Label(frame, text = name).pack(side = 'left')
            textVar = tk.StringVar()
            tk.Entry(frame, textvariable = textVar).pack()
            Dashboard.entries[name] = textVar
        Dashboard.entries[name].set(str(value))
            
        
    def getNumber(name):
        val = Dashboard.entries[name].get()
        try:
            return float(val)
        except:
            return Dashboard.values[name]
        
if __name__ == "__main__":
    #Window
    window = tk.Tk()
    window.title("Richard the Robot")
    window.state('zoomed')
    WIDTH = window.winfo_screenwidth()
    HEIGHT = window.winfo_screenheight()
    
    #Canvas
    CANVASWIDTH = int(WIDTH*(2/3))
    CANVASHEIGHT = HEIGHT
    
    canvas = tk.Canvas(window, width = CANVASWIDTH, height= CANVASHEIGHT,
                       relief = "solid",
                       borderwidth = 2)
    canvas.pack(side = 'left')
    
    #Floor
    FLOORHEIGHT = CANVASWIDTH*(2/4)
    canvas.create_line(0, FLOORHEIGHT, CANVASWIDTH, FLOORHEIGHT)
    for i in range(-60, 60):
        canvas.create_line(i*scale, FLOORHEIGHT,
                           i*scale, FLOORHEIGHT + 10)
        canvas.create_text(i*scale, FLOORHEIGHT + 20,
                           text = str(i),
                           fill="black")
    
    #Settings & Code
    OTHERWIDTH = WIDTH - CANVASWIDTH
    OTHERHEIGHT = HEIGHT
    stuff = tk.Frame(window, width = OTHERWIDTH, height = OTHERHEIGHT)
    stuff.pack(side = 'top')
    
    Richard.target.draw()
    
    #Code text editor
    CODEHEIGHT = int(OTHERHEIGHT*(1/2))
    code = tk.Frame(stuff,
                    relief = 'groove',
                    borderwidth = 5)
    code.pack(side = 'top')
    scrollbar = tk.Scrollbar(code)
    scrollbar.pack(side='right', fill = tk.Y)
    scrollbar2 = tk.Scrollbar(code, orient='horizontal')
    scrollbar2.pack(side='bottom', fill = tk.X)
    FONTSIZE = 10
    editor = tk.Text(code,
                     width = int(OTHERWIDTH/FONTSIZE) + 2,
                     height = int(CODEHEIGHT/FONTSIZE*(3/4)),
                     font = ("Courier", FONTSIZE),
                     wrap="none",
                     yscrollcommand = scrollbar.set,
                     xscrollcommand = scrollbar2.set)
    scrollbar.config(command=editor.yview)
    scrollbar2.config(command=editor.xview)
    editor.pack(fill=tk.BOTH)
    with open("RichardCode.py") as text:
        for n, i in enumerate(text):
            editor.insert(float(n+1), i)
    
    #Enable/Disable/Deploy Code
    def enable():
        Richard.enabled = True
        enableB["state"] = "disabled"
        disableB["state"] = "normal"
        deployB["state"] = "disabled"
        try:
            RichardCode.Init()
        except Exception as e:
            disable()
            tk.messagebox.showinfo("Runtime Error", str(e))
        print("Enabled")
        
    def disable():
        Richard.enabled = False
        enableB["state"] = "normal"
        disableB["state"] = "disabled"
        deployB["state"] = "normal"
        RichardController.setPower(0)
        print("Disabled")
        
    def deploy():
        global RichardCode
        try:
            sys.modules.pop("RichardCode")
        except:
            pass
        text = editor.get(1.0, tk.END)
        with open("RichardCode.py", "w+") as code:
            code.write(text)
        try:
            import RichardCode
        except Exception as e:
            disable()
            tk.messagebox.showinfo("Compile Error", str(e))
        RichardCode.RichardController = RichardController
        RichardCode.Dashboard = Dashboard
        print("Deployed")
    deploy()
        
    def reset():
        Richard.x = (CANVASWIDTH/2)/scale
        Richard.vx = 0
        Richard.power = 0
        disable()
        print("Reset")
        
    BUTTONHEIGHTS = 1
    BUTTONWIDTHS = 10
    buttons = tk.Frame(stuff)
    buttons.pack()
    enableB = tk.Button(buttons, text = "Enable", 
                        height = BUTTONHEIGHTS,
                        width = BUTTONWIDTHS,
                        command = enable)
    disableB = tk.Button(buttons, text = "Disable",
                         height = BUTTONHEIGHTS,
                         width = BUTTONWIDTHS,
                         command = disable,
                         state = "disabled")
    deployB = tk.Button(buttons, text = "Deploy Code",
                         height = BUTTONHEIGHTS,
                         width = BUTTONWIDTHS,
                         command = deploy)
    resetB = tk.Button(buttons, text = "Reset",
                         height = BUTTONHEIGHTS,
                         width = BUTTONWIDTHS,
                         command = reset)
    enableB.pack(side = 'left')
    disableB.pack(side = 'left')
    deployB.pack(side = 'left')
    resetB.pack(side = 'left')
    
    DashboardFrame = tk.Frame(stuff,
                              width = OTHERWIDTH,
                              bg = "#000000",
                              relief = 'ridge',
                              borderwidth = 4)
    DashboardFrame.pack(side = 'top')
    
    window.after(1, main)
    
    window.mainloop()
    window.quit()