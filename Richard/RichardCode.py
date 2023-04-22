'''
RichardController ->
getPosition() - Returns position of Richard
getTarget() - Returns position of the target
setPower() - Sets the power of Richard (capped sadly)
'''

'''
Init is called when Richard is enabled
'''
def Init():
    RC = RichardController
    global sumError
    sumError = 0
    global prevError
    prevError = 0
'''
Periodic is called every 20 ms
'''
def Periodic():
    global sumError
    global prevError
    RC = RichardController
    target = RC.getTarget()
    pos = RC.getPosition()
    kp = Dashboard.getNumber("kp")
    ki = Dashboard.getNumber("ki")
    kd = Dashboard.getNumber("kd")
    error = target - pos
    sumError += error * 0.02
    dError = (error - prevError)/0.02
    power = kp*error + ki*sumError + kd*dError
    RC.setPower(power)
    prevError = error

'''
Called when a new target is set
'''
def NewTarget():
    global sumError
    sumError = 0
    pass































































































