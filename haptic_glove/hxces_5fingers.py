#script to control a simulated robot hand using a VR glove
#see https://twitter.com/erwincoumans/status/821953216271106048
#and https://www.youtube.com/watch?v=I6s37aBXbV8
#vr glove was custom build using Spectra Symbolflex sensors (4.5")
#inside a Under Armour Batting Glove, using DFRobot Bluno BLE/Beetle
#with BLE Link to receive serial (for wireless bluetooth serial)

import serial
import time
import pybullet as p
import pybullet_data
import os
import math
import pyfirmata


#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
print(c)
if (c < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  

#set gravity(optional)
p.setGravity(0,0,0)

#camera Setting
p.resetDebugVisualizerCamera(cameraDistance=1.5,cameraYaw=0,cameraPitch=-40,cameraTargetPosition=[0.4,-0.85,0.2])

#basic object
tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"),basePosition=[0.35,-0.5,-0.625])
trayUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"tray/traybox.urdf"),basePosition=[0.6,-0.5,0])
objectUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"random_urdfs/000/000.urdf"),basePosition=[0.6,-0.5,0.1])

#load the MuJoCo MJCF hand
handUid = p.loadMJCF("C:\\Users\\wowjy\\.mujoco\\mujoco237\\Hand_example\\MPL.xml")

hand = handUid[0]
#clamp in range 400-600
#minV = 400
#maxV = 600
minVarray = [275, 300, 280, 350, 290]
maxVarray = [450, 450, 550, 500, 400]

#fingerIndex
pinkId = 0
ringposId= 1
middleId = 2
indexId = 3
thumbId = 4

p.setRealTimeSimulation(1)


def getSerialOrNone(portname):
  try:
    return serial.Serial(port=portname,
                         baudrate=115200,
                         parity=serial.PARITY_ODD,
                         stopbits=serial.STOPBITS_TWO,
                         bytesize=serial.SEVENBITS)
  except:
    return None


def convertSensor(x, fingerIndex):
  minV = minVarray[fingerIndex]
  maxV = maxVarray[fingerIndex]

  v = minV
  try:
    v = float(x)
  except ValueError:
    v = minV
  if (v < minV):
    v = minV
  if (v > maxV):
    v = maxV
  b = (v - minV) / float(maxV - minV)
  return (b)


ser = None
portindex = 0
while (ser is None and portindex < 30):
  portname = 'COM' + str(portindex)
  print(portname)
  ser = getSerialOrNone(portname)
  if (ser is None):
    portname = "/dev/cu.usbmodem14" + str(portindex)
    print(portname)
    ser = getSerialOrNone(portname)
    if (ser is not None):
      print("COnnected!")
  portindex = portindex + 1

if (ser is None):
  ser = serial.Serial(port="/dev/cu.usbmodem1421",
                      baudrate=115200,
                      parity=serial.PARITY_ODD,
                      stopbits=serial.STOPBITS_TWO,
                      bytesize=serial.SEVENBITS)
pi = 3.141592

if (ser is not None and ser.isOpen()):
  while True:
    while ser.inWaiting() > 0:
      line = str(ser.readline())
      words = line.split(",")
      if (len(words) == 6):
        pink = convertSensor(words[1], pinkId)
        ringpos = convertSensor(words[2], ringposId)
        middle = convertSensor(words[3], middleId)
        index = convertSensor(words[4], indexId)
        thumb = convertSensor(words[5], thumbId)

        p.setJointMotorControl2(hand, 7, p.POSITION_CONTROL, pi / 4.)
        p.setJointMotorControl2(hand, 9, p.POSITION_CONTROL, thumb + pi / 10)
        p.setJointMotorControl2(hand, 11, p.POSITION_CONTROL, thumb)
        p.setJointMotorControl2(hand, 13, p.POSITION_CONTROL, thumb)

        p.setJointMotorControl2(hand, 17, p.POSITION_CONTROL, index)
        p.setJointMotorControl2(hand, 19, p.POSITION_CONTROL, index)
        p.setJointMotorControl2(hand, 21, p.POSITION_CONTROL, index)

        p.setJointMotorControl2(hand, 24, p.POSITION_CONTROL, middle)
        p.setJointMotorControl2(hand, 26, p.POSITION_CONTROL, middle)
        p.setJointMotorControl2(hand, 28, p.POSITION_CONTROL, middle)

        p.setJointMotorControl2(hand, 32, p.POSITION_CONTROL, ringpos)
        p.setJointMotorControl2(hand, 34, p.POSITION_CONTROL, ringpos)
        p.setJointMotorControl2(hand, 36, p.POSITION_CONTROL, ringpos)

        p.setJointMotorControl2(hand, 40, p.POSITION_CONTROL, pink)
        p.setJointMotorControl2(hand, 42, p.POSITION_CONTROL, pink)
        p.setJointMotorControl2(hand, 44, p.POSITION_CONTROL, pink)


        #print(middle)
        #print(pink)
        #print(index)
        #print(thumb)
else:
  print("Cannot find port")
