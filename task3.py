"""task3 controller."""

from controller import Robot
from controller import Camera
from controller import DistanceSensor
from controller import LightSensor
from controller import LED
from controller import Keyboard
from controller import Supervisor
from controller import Field
from controller import LidarPoint
from controller import Motor
from math import pi, sin
from array import *
import time
import random
 
# create the Robot instance.
robot = Supervisor()

# get handle on robotNode
robotNode = robot.getFromDef('boebot')

# get the time step of the current world. 
timestep = 64

#0 is north, 1 is east, 2 is south, 3 is west
global orientation
orientation = 0

global currCell
currCell = 13

global stopper
stopper = 0

# define max speed
MAX_SPEED = 6.283
#pi = 3.14159265359
wheelCircumferenceMeters = 0.20891591146
cellLengthInRadians = (0.4572) * (2*pi)/(wheelCircumferenceMeters)  #?????

# enable keyboard for controls
keyboard = Keyboard()
keyboard.enable(timestep)

# get front left and right distance sensors
# note: boebot has built in lds and rds at front l/R angles. Currently not enabled.
frontDistanceSensor = robot.getDistanceSensor('front_ds')
leftDistanceSensor = robot.getDistanceSensor('left_ds')
rightDistanceSensor = robot.getDistanceSensor('right_ds')

frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# get left and right light sensors
leftLightSensor = robot.getLightSensor('lls')
rightLightSensor = robot.getLightSensor('rls')
leftLightSensor.enable(timestep)
rightLightSensor.enable(timestep)

# get left and right LED
leftLED = robot.getLED('left_led')
rightLED = robot.getLED('right_led')
leftLED.set(1)
rightLED.set(1)

# enable camera and recognition
camera = robot.getCamera('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

global wallCounter# global variable to use as ID for wall instances

# get timestep
def getTimestep():
    print('Timestep is currently: %d' % timestep)
    
# set timestep
def setTimestep(step):
    global timestep
    timestep = step
    
def setStopper(stop):
    global stopper
    stopper = stop
    
# movement controls. speed in radians per second
def setSpeed(leftSpeed, rightSpeed):
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    


def setOr(i):
    global orientation
    orientation = i

def setCurrCell(i):
    global currCell
    currCell = i

#rotate 90 degrees a certain number of times
def rotate(times):
    start = time.monotonic()
    endtime = time.monotonic()
    setSpeed(pi/2, -pi/2)
    while robot.step(timestep) != -1:
        endtime = time.monotonic()
        
        key=keyboard.getKey()
        if(key==ord('S')):
            setStopper(999)
            return 
        
        if(endtime - start >= 1.5 * times):
            setSpeed(0, 0)
            break
        key=keyboard.getKey()
    setOr((orientation + times) % 4)

def rotateCounter(times):
    start = time.monotonic()
    endtime = time.monotonic()
    setSpeed(-pi/2, pi/2)
    while robot.step(timestep) != -1:
        endtime = time.monotonic()
        
        key=keyboard.getKey()
        if(key==ord('S')):
            setStopper(999)
            return 
        
        if(endtime - start >= 1.5 * times):
            setSpeed(0, 0)
            break
    setOr((orientation - times) % 4)
    



 
 
#setup
start = time.monotonic()
while robot.step(timestep) != -1:
    endtime = time.monotonic()
    if(endtime - start >= 2):
        break

 
#task 2 attempt

def cellFor2(speed):
    start = time.monotonic()
    while robot.step(timestep) != 1:
        setSpeed(speed, speed)
        if (leftDistanceSensor.getValue() < 450):
            setSpeed((speed + 0.08), speed - 0.08)
        elif (rightDistanceSensor.getValue() < 450):
            setSpeed(speed - 0.08, (speed + 0.08))
        endtime = time.monotonic()
        if(endtime - start >= cellLengthInRadians/speed):
            setSpeed(0, 0)
            break
            
        key=keyboard.getKey()
        if(key==ord('S')):
            setStopper(999)
            return 

def printVisits(cellArray):
    i = 0
    while(i < 16):
        s = ''
        if(cellArray[i] == 0):
            s = s + '. '
        else:
            s = s + 'X '
        if(cellArray[i + 1] == 0):
            s = s + '. '
        else:
            s = s + 'X '
        if(cellArray[i + 2] == 0):
            s = s + '. '
        else:
            s = s + 'X '
        if(cellArray[i + 3] == 0):
            s = s + '. '
        else:
            s = s + 'X '
        print(s)
        i = i + 4
    
    
    
def getMotion(prev, lWall, fWall, rWall, orient, oldMotion):
    motionProbs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
    combination = lWall + fWall + rWall
    prob = 1.0
    if(combination == 0):
        prob = 0.33
    elif(combination == 1):
        prob = 0.5
        
    if(lWall == 0):
        if(orient == 0):
            motionProbs[prev-1-1] = prob * oldMotion
        elif(orient == 1):
            motionProbs[prev-1-4] = prob * oldMotion
        elif(orient == 2):
            motionProbs[prev-1+1] = prob * oldMotion
        else:
            motionProbs[prev-1+4] = prob * oldMotion
    
    if(fWall == 0):
        if(orient == 0):
            motionProbs[prev-1-4] = prob * oldMotion
        elif(orient == 1):
            motionProbs[prev-1+1] = prob * oldMotion
        elif(orient == 2):
            motionProbs[prev-1+4] = prob * oldMotion
        else:
            motionProbs[prev-1-1] = prob * oldMotion
            
    if(rWall == 0):
        if(orient == 0):
            motionProbs[prev-1+1] = prob * oldMotion
        elif(orient == 1):
            motionProbs[prev-1+4] = prob * oldMotion
        elif(orient == 2):
            motionProbs[prev-1-1] = prob * oldMotion
        else:
            motionProbs[prev-1-4] = prob * oldMotion
            
    if(combination == 3):
        if(orient == 0):
            motionProbs[prev-1+4] = prob * oldMotion
        elif(orient == 1):
            motionProbs[prev-1-1] = prob * oldMotion
        elif(orient == 2):
            motionProbs[prev-1-4] = prob * oldMotion
        else:
            motionProbs[prev-1+1] = prob * oldMotion
 
    return motionProbs




def printProbs(probs):
    i = 0
    while(i < 16):
        s = str(probs[i]) + ' ' + str(probs[i+1]) + ' ' + str(probs[i+2]) + ' ' + str(probs[i+3])
        print(s)
        i = i + 4


def getMeasurement(cellWalls, orient):
    measurementProbs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lwall = 0
    fwall = 0
    rwall = 0
    bwall = 0
    if leftDistanceSensor.getValue() < 900:
        lwall = 1
    if frontDistanceSensor.getValue() < 900:
        fwall = 1
    if rightDistanceSensor.getValue() < 900:
        rwall = 1
        
    if(orient == 1):
        temp = lwall
        lwall = bwall
        bwall = rwall
        rwall = fwall
        fwall = temp
    elif(orient == 2):
        temp1 = lwall
        temp2 = bwall
        lwall = rwall
        bwall = fwall
        rwall = temp1
        fwall = temp2
    elif(orient == 3):
        temp = lwall
        lwall = fwall
        fwall = rwall
        rwall = bwall
        bwall = temp
    
    i = 0
    while(i < 16):
        p = 1.0
        if(lwall == cellWalls[i][0]):
            p = p * 0.9
        else:
            p = p * 0.1  
        
        if(fwall == cellWalls[i][1]):
            p = p * 0.9
        else:
            p = p * 0.1  
            
        if(rwall == cellWalls[i][2]):
            p = p * 0.9
        else:
            p = p * 0.1
            
        if(bwall == cellWalls[i][3]):
            p = p * 0.9
        else:
            p = p * 0.1
        
        p = round(p, 2)
        measurementProbs[i] = p
        i = i + 1
    return measurementProbs
    
    
    
def getWeighted(motion, measure):
    weightedProbs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    i = 0
    while(i < 16):
        weightedProbs[i] = round((motion[i] * measure[i]), 2)
        i = i + 1
    return weightedProbs
    
def getNormalized(probs):
    sum = 0
    i = 0
    while(i < 16):
        sum = sum + probs[i]
        i = i + 1
    i = 0
    while(i < 16):
        probs[i] = round((probs[i] / sum), 2)
        i = i + 1
    return probs


def recMap(cellMap, cellNum, b, orient):
    lwall = 0
    fwall = 0
    rwall = 0
    bwall = b
    if leftDistanceSensor.getValue() < 900:
        lwall = 1
    if frontDistanceSensor.getValue() < 900:
        fwall = 1
    if rightDistanceSensor.getValue() < 900:
        rwall = 1
        
    if(orient == 1):
        temp = lwall
        lwall = bwall
        bwall = rwall
        rwall = fwall
        fwall = temp
    elif(orient == 2):
        temp1 = lwall
        temp2 = bwall
        lwall = rwall
        bwall = fwall
        rwall = temp1
        fwall = temp2
    elif(orient == 3):
        temp = lwall
        lwall = fwall
        fwall = rwall
        rwall = bwall
        bwall = temp
        
    cellMap[cellNum-1][0] = str(lwall)
    cellMap[cellNum-1][1] = str(fwall)
    cellMap[cellNum-1][2] = str(rwall)
    cellMap[cellNum-1][3] = str(bwall)



def wallCheck3(cellMap, prevCell, cellsVisited):
    cellNum = prevCell
    lwall = 0
    fwall = 0
    rwall = 0
    bwall = 0
    decider = 99
    if leftDistanceSensor.getValue() < 900:
        lwall = 1
    if frontDistanceSensor.getValue() < 900:
        fwall = 1
    if rightDistanceSensor.getValue() < 900:
        rwall = 1
    
    l = 0
    f = 0
    r = 0
    if(orientation == 0):
        l = prevCell-1
        f = prevCell-4
        r = prevCell+1
    elif(orientation == 1):
        l = prevCell-4
        f = prevCell+1
        r = prevCell+4
    elif(orientation == 2):
        l = prevCell+1
        f = prevCell+4
        r = prevCell-1
    elif(orientation == 3):
        l = prevCell+4
        f = prevCell-1
        r = prevCell-4
    
    if lwall == 0 and (cellsVisited[l-1] == 0):
        decider = 1
    elif fwall == 0 and (cellsVisited[f-1] == 0):
        decider = 2
    elif rwall == 0 and (cellsVisited[r-1] == 0):
        decider = 3
    else:
        #print("random")
        if(lwall == 0) and (fwall == 0) and (rwall == 0):
            decider = random.randint(1,3)
        elif(lwall == 1) and (fwall == 0) and (rwall == 0):
            decider = random.randint(2,3)
        elif(lwall == 0) and (fwall == 1) and (rwall == 0):
            decider = random.randint(1,2)
            if decider == 2:
                decider = 3
        elif(lwall == 0) and (fwall == 0) and (rwall == 1):
            decider = random.randint(1,2)
        elif(lwall == 1) and (fwall == 1) and (rwall == 0):
            decider = 3
        elif(lwall == 1) and (fwall == 0) and (rwall == 1):
            decider = 2
        elif(lwall == 0) and (fwall == 1) and (rwall == 1):
            decider = 1
        elif(lwall == 1) and (fwall == 1) and (rwall == 1):
            decider = 4
        
    if(decider == 1):
        rotateCounter(1)
    elif(decider == 2):
        pass
    elif(decider == 3):
        rotate(1)
    elif(decider == 4):
        rotate(2)

    cellFor2(cellLengthInRadians/4)
    if(stopper == 999):
        return cellNum
        
    if(orientation == 0):
        cellNum = cellNum - 4
    elif(orientation == 1):
        cellNum = cellNum + 1
    elif(orientation == 2):
        cellNum = cellNum + 4
    elif(orientation == 3):
        cellNum = cellNum - 1

    recMap(cellMap, cellNum, bwall, orientation)
    return cellNum


def printMap(cellMap):
    print('Cell Walls (WNES)')
    i = 0
    while(i < 16):
        s = ''
        j = 0
        while(j < 4):
            if(cellMap[i][j] == '0'):
                s = s + 'O'
            elif(cellMap[i][j] == '1'):
                s = s + 'W'
            else:
                s = s + '?'
            j = j + 1
        print(str(i+1) + '\t' + s)
        i = i + 1
    



#W,N,E,S
cellMap = [['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], 
           ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], 
           ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], 
           ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?'], ['?', '?', '?', '?']]
cellsVisited = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
cellsVisited[13-1] = 1
thisCell = 13
prevCell = thisCell
recMap(cellMap, thisCell, 1, 0)

print('Cell: ' + str(thisCell))
print('')
while robot.step(timestep) != -1:
    
    key=keyboard.getKey()
    
    j = 0
    visits = 0
    while(j < 16):
        visits = visits + cellsVisited[j]
        j = j + 1
    
    if(key==ord('S') or stopper==999 or visits==16):
        print('DONE')
        setSpeed(0, 0)
        stopper = 0
        break
    
    prevCell = thisCell
    thisCell = wallCheck3(cellMap, thisCell, cellsVisited)
    
    
    
    print('Cell: ' + str(thisCell))
    #print(cellMap[thisCell-1][0] + ' ' + cellMap[thisCell-1][1] + ' ' + cellMap[thisCell-1][2] + ' ' + cellMap[thisCell-1][3])
    if(orientation == 0):
        print('Orientation: North')
    elif(orientation == 1):
        print('Orientation: East')
    elif(orientation == 2):
        print('Orientation: South')
    else:
        print('Orientation: West')
    
    print('')
    print('Visited Cells ("X"):')
    cellsVisited[thisCell-1] = 1    
    printVisits(cellsVisited)

    print('')
    print('')

print('Waiting for user to press key P')
while robot.step(timestep) != -1:
    key=keyboard.getKey()
    if(key==ord('P')):
        printMap(cellMap)
        break

