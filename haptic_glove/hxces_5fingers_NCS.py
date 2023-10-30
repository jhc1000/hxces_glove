#script to control a simulated robot hand using a VR glove
#see https://twitter.com/erwincoumans/status/821953216271106048
#and https://www.youtube.com/watch?v=I6s37aBXbV8
#vr glove was custom build using Spectra Symbolflex sensors (4.5")
#inside a Under Armour Batting Glove, using DFRobot Bluno BLE/Beetle
#with BLE Link to receive serial (for wireless bluetooth serial)

import os
import serial
import time
import pybullet as p
import pybullet_data
import os
import math
from math import pi
from math import radians
import pyfirmata


import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# CONSTANT
INI_0_FLOAT = 0.0
INI_0_INT = 0

# tinyForce= 2*10e-7
tinyForce= 20
move = 0.01#0.00035


#first try to connect to shared memory (VR), if it fails use local GUI
c = p.connect(p.SHARED_MEMORY)
print(c)
if (c < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  

#set gravity(optional)
p.setGravity(0,0,-9.8)

#camera Setting
p.resetDebugVisualizerCamera(cameraDistance=1.5,cameraYaw=0,cameraPitch=-40,cameraTargetPosition=[0.4,-0.85,0.2])

#basic object
tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"),basePosition=[0.1,-0.2,-0.625])
trayUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"tray/traybox.urdf"),basePosition=[-0.1,-0.2,0])
# objectUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"random_urdfs/000/000.urdf"),basePosition=[0.6,-0.5,0.1])
objectUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"sphere_small.urdf"),basePosition=[0.0,-0.3,0.4])

#load the MuJoCo MJCF hand
# handUid = p.loadMJCF('D:\주형찬\한양대\Rodel\HX-CES\hxces_glove\Hand_example\MPL.xml')
handUid = p.loadMJCF('/home/chan/rodel/hxces_glove/Hand_example/MPL.xml')
# handUid = p.loadMJCF("C:\\Users\\wowjy\\.mujoco\\mujoco237\\Hand_example\\MPL.xml")

hand = handUid[0]
hand_cid = p.createConstraint(hand,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0])
hand_po = p.getBasePositionAndOrientation(hand)
ho = p.getQuaternionFromEuler([0.0, pi, pi])
p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=200)

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

#fingerValue
finger_value = [INI_0_INT for i in range(15)]

#collisionDetection
collision_bool = [1 for i in range(5)]

p.setRealTimeSimulation(1)


def getSerialOrNone(portname):
  try:
    return serial.Serial(port=portname,
                         baudrate=115200,
                        #  parity=serial.PARITY_ODD,
                        #  stopbits=serial.STOPBITS_TWO,
                        #  bytesize=serial.SEVENBITS
                         timeout=0.1,
                        )       
  except:
    return None


def convertSensor(x, fingerIndex):
  minV = minVarray[fingerIndex]
  maxV = maxVarray[fingerIndex]

  v = minV
  try:
    v = int(x)
  except ValueError:
    v = minV
  if (v < minV):
    v = minV
  if (v > maxV):
    v = maxV
  b = (v - minV) / int(maxV - minV)
  return int(b)

# connecting Serial port 
ser = None
portindex = 0
while (ser is None and portindex < 30):
  if sys.platform == 'win32':
    # window serial port
    portname = 'COM' + str(portindex)
    print(portname)
    ser = getSerialOrNone(portname)
  else:
    # linux serial port
    portname = '/dev/ttyUSB' + str(portindex)
    print(portname)
    ser = getSerialOrNone(portname)
  if (ser is not None):
    print("Connected!")
  portindex = portindex + 1


# data processing between esp32 and PC
if (ser is not None and ser.isOpen()):
  try:
    while True:
      key = p.getKeyboardEvents()
      print(key)
      for k in key.keys():
          hand_po = p.getBasePositionAndOrientation(hand)
          if k == 65296: #left 
            p.changeConstraint(hand_cid,(hand_po[0][0]+move,hand_po[0][1],hand_po[0][2]),ho, maxForce=tinyForce)
          elif k == 65295: #right        
            p.changeConstraint(hand_cid,(hand_po[0][0]-move,hand_po[0][1],hand_po[0][2]),ho, maxForce=tinyForce)
          elif k == 65297: #up        
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1]+move,hand_po[0][2]),ho, maxForce=tinyForce)
          elif k == 65298: #down         
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1]-move,hand_po[0][2]),ho, maxForce=tinyForce)
          elif k == 44: #< z up key       
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]+move),ho, maxForce=tinyForce)            
          elif k == 46: #> z down key           
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]-move),ho, maxForce=tinyForce)
          elif k == 107: #k tilt + key       
            ho = p.getQuaternionFromEuler([p.getEulerFromQuaternion(ho)[0], 
                                           p.getEulerFromQuaternion(ho)[1]+move*pi, 
                                           p.getEulerFromQuaternion(ho)[2]])    
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=tinyForce)
          elif k == 108: #l tilt - key 
            ho = p.getQuaternionFromEuler([p.getEulerFromQuaternion(ho)[0], 
                                           p.getEulerFromQuaternion(ho)[1]-move*pi, 
                                           p.getEulerFromQuaternion(ho)[2]])          
            p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=tinyForce)

              
      if ser.inWaiting():
        # read serial data from esp32
        read_value_byte = ser.readline()
        # print(read_value_byte)
        str_check = b"A"
        end_check = b"\n"
        # check and convert the data from ESP32
        if (read_value_byte.startswith(str_check)) and (read_value_byte.endswith(end_check)) :
            read_value_string = read_value_byte.decode()
            read_value_string_sliced = read_value_string[2:-1]
            words = read_value_string_sliced.split(',')
            # print(words)
            for i in range(len(words)):
              finger_value[i] = int(words[i])
            print(finger_value)
            if (len(finger_value) == 15):
              # pink = convertSensor(finger_value[0], pinkId)
              # ringpos = convertSensor(finger_value[1], ringposId)
              # middle = convertSensor(finger_value[2], middleId)
              # index = convertSensor(finger_value[3], indexId)
              # thumb = convertSensor(finger_value[4], thumbId)

              #thumb
              p.setJointMotorControl2(hand, 7, p.POSITION_CONTROL, radians(finger_value[0]))
              p.setJointMotorControl2(hand, 9, p.POSITION_CONTROL, radians(finger_value[1]))
              p.setJointMotorControl2(hand, 11, p.POSITION_CONTROL, radians(finger_value[2]))
              p.setJointMotorControl2(hand, 13, p.POSITION_CONTROL, pi/4)
              #index
              p.setJointMotorControl2(hand, 17, p.POSITION_CONTROL, radians(finger_value[3]))
              p.setJointMotorControl2(hand, 19, p.POSITION_CONTROL, radians(finger_value[4]))
              p.setJointMotorControl2(hand, 21, p.POSITION_CONTROL, radians(finger_value[5]))
              #middle
              p.setJointMotorControl2(hand, 24, p.POSITION_CONTROL, radians(finger_value[6]))
              p.setJointMotorControl2(hand, 26, p.POSITION_CONTROL, radians(finger_value[7]))
              p.setJointMotorControl2(hand, 28, p.POSITION_CONTROL, radians(finger_value[8]))
              #ringpos
              p.setJointMotorControl2(hand, 32, p.POSITION_CONTROL, radians(finger_value[9]))
              p.setJointMotorControl2(hand, 34, p.POSITION_CONTROL, radians(finger_value[10]))
              p.setJointMotorControl2(hand, 36, p.POSITION_CONTROL, radians(finger_value[11]))
              #pink
              p.setJointMotorControl2(hand, 40, p.POSITION_CONTROL, radians(finger_value[12]))
              p.setJointMotorControl2(hand, 42, p.POSITION_CONTROL, radians(finger_value[13]))
              p.setJointMotorControl2(hand, 44, p.POSITION_CONTROL, radians(finger_value[14]))


              # print(middle)
              # print(pink)
              # print(index)
              # print(thumb)

      # sending collision bool to esp32
      if any (collision_bool) > 0:
        
        msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=collision_bool[0], b=collision_bool[1], 
                                              c=collision_bool[2], d=collision_bool[3], 
                                              e=collision_bool[4])
        # print(msg)
        ser.write(msg.encode('utf-8'))
        time.sleep(0.01)

  except KeyboardInterrupt:
    print(KeyboardInterrupt)

  finally:
    i = 0
    msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
    # print(msg)
    ser.write(msg.encode('utf-8'))
    time.sleep(0.01)
      



else:
  print("Cannot find port")
