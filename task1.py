"""task1 controller."""

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
from controller import CameraRecognitionObject
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



global bluex
bluex = -1 + 1.8288/2
global bluey
bluey = 0.9 - 1.8288/2

global pinkx
pinkx = -0.9 + 1.8288/2
global pinky
pinky = -0.97 - 1.8288/2

global greenx
greenx = 0.9 + 1.8288/2
global greeny
greeny = -0.97 - 1.8288/2


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

# read distance sensors
def readFDS():
    fdsVal = frontDistanceSensor.getValue()
    print(fdsVal)
    
def readLDS():
    ldsVal = leftDistanceSensor.getValue()
    print(ldsVal)    

def readRDS():
    rdsVal = rightDistanceSensor.getValue()
    print(rdsVal)
    
# movement controls. speed in radians per second
def setSpeed(leftSpeed, rightSpeed):
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    

def goForward(speed): 
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(speed)

def goBackward(speed):
    leftMotor.setVelocity(-speed)
    rightMotor.setVelocity(-speed)
    
def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
def spinLeft(speed):
    leftMotor.setVelocity(-speed)
    rightMotor.setVelocity(speed)
    
def spinRight(speed):
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(-speed)

# get position x, y, z
def getPosition():
    translationField = robotNode.getField('translation')
    translation = translationField.getSFVec3f()
    print('Position is: %g %g %g' % (translation[0], translation[1], translation[2]))

# set position (translation)
def setPosition(coordinates):
    translationField = robotNode.getField('translation')
    translationField.setSFVec3f(coordinates)
    print('Robot moved to: %g %g %g' % (coordinates[0], coordinates[1], coordinates[2]))
    
# set translation and rotation
def setPositionAndAngle(coordinates, rotationList):
    translationField = robotNode.getField('translation')
    rotationField = robotNode.getField('rotation')
    translationField.setSFVec3f(coordinates)
    rotationField.setSFRotation(rotationList)

# get orientation angle
def getOrientation():
    rotationField = robotNode.getField('rotation')
    orientation = rotationField.getSFRotation() #index 3 is angle in radians      
    print('Angle in radians is: %g' % orientation[3])

# set orientation angle
def setOrientation(rotationList): #x, y, z, angle
    rotationField = robotNode.getField('rotation')
    rotationField.setSFRotation(rotationList)

def addWall(translation, rotation, size):
    rootNode = robot.getRoot()
    rootChildrenField = rootNode.getField('children')
    rootChildrenField.importMFNode(-1, 'wallNode.wbo') # -1 tree index
    wallNode = robot.getFromDef('wall')
    wallTranslationField = wallNode.getField('translation')
    wallTranslationField.setSFVec3f(translation)
    wallRotationField = wallNode.getField('rotation')
    wallRotationField.setSFRotation(rotation)
    wallSizeField = wallNode.getField('size')
    wallSizeField.setSFVec3f(size)
    
    
    

















  



def setOr(i):
    global orientation
    orientation = i

def setCurrCell(i):
    global currCell
    currCell = i

#rotate how many 90 degree times
def rotate(times):
    start = time.monotonic()
    endtime = time.monotonic()
    setSpeed(pi/2, -pi/2)
    while robot.step(timestep) != -1:
        endtime = time.monotonic()
        
        if(times > 2):
            if(endtime - start >= (1.43) * times):
                setSpeed(0, 0)
                break
        else:
            if(endtime - start >= (1.5) * times):
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
        
        
        if(times > 2):
            if(endtime - start >= (1.43) * times):
                setSpeed(0, 0)
                break
        else:
            if(endtime - start >= (1.5) * times):
                setSpeed(0, 0)
                break
    setOr((orientation - times) % 4)
    
    
       
 
#setup
start = time.monotonic()
while robot.step(timestep) != -1:
    endtime = time.monotonic()
    if(endtime - start >= 2):
        break

 
def cellFor2(speed):
    start = time.monotonic()
    while robot.step(timestep) != 1:
        setSpeed(speed, speed)
        
        if leftDistanceSensor.getValue() < 23:
            setSpeed((speed + 0.1), speed - 0.1)
        elif rightDistanceSensor.getValue() < 23:
            setSpeed(speed - 0.1, (speed + 0.1))
        endtime = time.monotonic()
        if(endtime - start >= cellLengthInRadians/speed):
            setSpeed(0, 0)
            break



lwall = 0
fwall = 0
rwall = 0



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



    
    
def getWeighted(motion, measure):
    weightedProbs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    i = 0
    while(i < 16):
        weightedProbs[i] = round((motion[i] * measure[i]), 9)
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
        if(sum == 0):
            probs[i] = 0.0
        else:
            probs[i] = round((probs[i] / sum), 2)
        i = i + 1
    return probs




def wallCheck1(prevCell, cellsVisited):
    cellNum = prevCell
    lwall = 0
    fwall = 0
    rwall = 0
    bwall = 0
    decider = 99
    if leftDistanceSensor.getValue() < 40:
        lwall = 1
    if frontDistanceSensor.getValue() < 40:
        fwall = 1
    if rightDistanceSensor.getValue() < 40:
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

    return cellNum








def getMeasurementCamera():
    measurementProbs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    bluedistance = 0
    pinkdistance = 0
    greendistance = 0

    start = time.monotonic()
    endtime = time.monotonic()
    setSpeed(pi/2, -pi/2)

    while (bluedistance == 0) and (pinkdistance == 0) and (greendistance == 0):
        while robot.step(timestep) != -1:
            
            endtime = time.monotonic()
            if(endtime - start >= (1.43) * 4):
                setSpeed(0, 0)
                break
        
            if(len(camera.getRecognitionObjects()) > 0):
                firstObject = camera.getRecognitionObjects()[0]
                id = firstObject.get_id()
                
                if(firstObject.get_position()[0] < 0.08 and firstObject.get_position()[0] > -0.08) and (firstObject.get_position()[1] < 0.05 and firstObject.get_position()[1] > -0.05):
                    redBool = (firstObject.get_colors()[0] == 1)
                    greenBool = (firstObject.get_colors()[1] == 1)
                    blueBool = (firstObject.get_colors()[2] == 1)
                    if(greenBool and blueBool):
                        bluedistance = frontDistanceSensor.getValue()
                        bluedistance = ((bluedistance-10)/250) * 2.58631376286791622524852833883
                    elif(redBool and blueBool):
                        pinkdistance = frontDistanceSensor.getValue()
                        pinkdistance = ((pinkdistance-10)/250) * 2.58631376286791622524852833883
                    elif(greenBool):
                        greendistance = frontDistanceSensor.getValue()
                        greendistance = ((greendistance-10)/250) * 2.58631376286791622524852833883
    

    A = (-2 * pinkx) + (2 * greenx)
    B = (-2 * pinky) + (2 * greeny)
    C = (pinkdistance * pinkdistance) - (greendistance * greendistance) - (pinkx * pinkx) + (greenx * greenx) - (pinky * pinky) + (greeny * greeny)
    D = (-2 * greenx) + (2 * bluex)
    E = (-2 * greeny) + (2 * bluey)
    F = (greendistance * greendistance) - (bluedistance * bluedistance) - (greenx * greenx) + (bluex * bluex) - (greeny * greeny) + (bluey * bluey)
    xposition = ((C * E) - (F * B)) / ((E * A) - (B * D))
    yposition = ((C * D) - (A * F)) / ((B * D) - (A * E))
    yposition = -yposition
    mainCell = 0
    mainProb = round(1/9, 2)
    if(xposition > 1.316 and yposition > 1.316):
        mainCell = 4-1
        measurementProbs[mainCell] = mainProb*6 #C
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell+4] = mainProb #S
    elif(xposition > 1.316 and yposition > 0.9144):
        mainCell = 8-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 1.316 and yposition > 0.4572):
        mainCell = 12-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 1.3716 and yposition > 0):
        mainCell = 16-1
        measurementProbs[mainCell] = mainProb*6 #C
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
        measurementProbs[mainCell-4] = mainProb #N
    elif(xposition > 0.9144 and yposition > 1.316):
        mainCell = 3-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
    elif(xposition > 0.9144 and yposition > 0.9144):
        mainCell = 7-1
        measurementProbs[mainCell] = mainProb #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0.9144 and yposition > 0.4572):
        mainCell = 11-1
        measurementProbs[mainCell] = mainProb #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0.9144 and yposition > 0):
        mainCell = 15-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0.4572 and yposition > 1.316):
        mainCell = 2-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
    elif(xposition > 0.4572 and yposition > 0.9144):
        mainCell = 6-1
        measurementProbs[mainCell] = mainProb #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0.4572 and yposition > 0.4572):
        mainCell = 10-1
        measurementProbs[mainCell] = mainProb #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
        measurementProbs[mainCell+3] = mainProb #SW
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0.4572 and yposition > 0):
        mainCell = 14-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell-1] = mainProb #W
        measurementProbs[mainCell-5] = mainProb #NW
    elif(xposition > 0 and yposition > 1.316):
        mainCell = 1-1
        measurementProbs[mainCell] = mainProb*6 #C
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
    elif(xposition > 0 and yposition > 0.9144):
        mainCell = 5-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
    elif(xposition > 0 and yposition > 0.4572):
        mainCell = 9-1
        measurementProbs[mainCell] = mainProb*4 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
        measurementProbs[mainCell+5] = mainProb #SE
        measurementProbs[mainCell+4] = mainProb #S
    else:
        mainCell = 13-1
        measurementProbs[mainCell] = mainProb*6 #C
        measurementProbs[mainCell-4] = mainProb #N
        measurementProbs[mainCell-3] = mainProb #NE
        measurementProbs[mainCell+1] = mainProb #E
    
    return measurementProbs





start = time.monotonic()
endtime = time.monotonic()
setCurrCell(13)#starting point
cellOrder = "Cell Sequence: 13"
oldMotion = 1
cellsVisited = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
cellsVisited[13-1] = 1


# Main loop:
# - perform simulation steps until Webots is stopping the controller

prevCell = 13
thisCell = prevCell

while robot.step(timestep) != -1:
    j = 0
    visits = 0
    while(j < 16):
        visits = visits + cellsVisited[j]
        j = j + 1
    
    oldOrient = orientation
    lwall = 0
    fwall = 0
    rwall = 0
    decider = 99
    if leftDistanceSensor.getValue() < 40:
        lwall = 1
    if frontDistanceSensor.getValue() < 40:
        fwall = 1
    if rightDistanceSensor.getValue() < 40:
        rwall = 1
    prevCell = currCell
    
    if(stopper==999 or visits==16):
        print('DONE')
        setSpeed(0, 0)
        stopper = 0
        break
    
    prevCell = thisCell
    thisCell = wallCheck1(thisCell, cellsVisited)
    
    
    
    print('Cell: ' + str(thisCell))
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
    print('Motion Probabilities:')
    motionProb = getMotion(prevCell, lwall, fwall, rwall, oldOrient, oldMotion)
    k = 0
    l = 0
    while(k < 16):
        if(motionProb[k] != 0):
            oldMotion = motionProb[k]
        k = k + 1
    printProbs(motionProb)
    
    print('')
    print('Measurement Probabilities:')
    measurementProb = getMeasurementCamera()
    printProbs(measurementProb)
    
    print('')
    print('Weighted Probabilities:')
    weightedProb = getWeighted(motionProb, measurementProb)
    printProbs(weightedProb)
    
    print('')
    print('Normalized Probabilities:')
    normalizedProb = getNormalized(weightedProb)
    printProbs(normalizedProb)

    print('')
    print('')
    print('')
    
    endtime = time.monotonic()
    if(endtime - start >= 180):
        setSpeed(0, 0)
        print('Maximum Time')
        print(cellOrder)
        break

    print('')
    print('')

    
#############################################################

