import os
import serial
import time
import numpy as np
import pybullet as p
import pybullet_data
import pyb_utils
import math
from math import pi, radians
import pyfirmata
import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TIMESTEP = 1.0 / 60

# CONSTANT
INI_0_FLOAT = 0.0
INI_0_INT = 0

# tinyForce= 2*10e-7
tinyForce= 20
move = 0.01#0.00035


TIMESTEP = 1.0 / 60

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
col_finger_value = [INI_0_INT for i in range(5)]

def getSerialOrNone(portname):
  try:
    return serial.Serial(port=portname,
                         baudrate=115200,
                        #  parity=serial.PARITY_ODD,
                        #  stopbits=serial.STOPBITS_TWO,
                        #  bytesize=serial.SEVENBITS
                         timeout=0.01,
                        )       
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

def get_key_pressed():
        pressed_keys = []
        events = p.getKeyboardEvents()
        key_codes = events.keys()
        for key in key_codes:
            pressed_keys.append(key)
        return pressed_keys


############################################################################################################################################


def load_environment(client_id):
    p.setTimeStep(TIMESTEP, physicsClientId=client_id)
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath(), physicsClientId=client_id
    )

    #set gravity(optional)
    p.setGravity(0,0,-9.8, physicsClientId=client_id)

    #camera Setting
    p.resetDebugVisualizerCamera(
        cameraDistance=1,
        cameraYaw=-90,
        cameraPitch=-45,
        cameraTargetPosition=[0.15,-0.2,-0.2]
    )

    # ground plane
    ground_id = p.loadURDF(
        "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
    )

    #load the MuJoCo MJCF hand

    global hand_id
    global hand
    global hand_cid
    global hand_po
    global ho
    global robot

    # hand_id = p.loadMJCF('D:\주형찬\한양대\Rodel\HX-CES\hxces_glove\Hand_example\MPL.xml', physicsClientId=client_id)
    hand_id = p.loadMJCF('/home/chan/rodel/hxces_glove/Hand_example/MPL.xml', physicsClientId=client_id)
    # hand_id = p.loadMJCF("C:\\Users\\wowjy\\.mujoco\\mujoco237\\Hand_example\\MPL.xml", physicsClientId=client_id)
    hand = hand_id[0]
    hand_cid = p.createConstraint(hand,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0], physicsClientId=client_id)
    hand_po = p.getBasePositionAndOrientation(hand, physicsClientId=client_id)
    ho = p.getQuaternionFromEuler([0.0, 0.0, pi], physicsClientId=client_id)
    p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]),ho, maxForce=200, physicsClientId=client_id)
    robot = pyb_utils.Robot(hand_id[0], client_id=client_id)

    # some object (cube1 = table, cube2 = tray, cube3 = sphere)
    #cube1_id = p.loadURDF(
    #    os.path.join(pybullet_data.getDataPath(),"table/table.urdf"),basePosition=[0.35,-0.5,-0.625], useFixedBase=True, physicsClientId=client_id
    #)
    #cube2_id = p.loadURDF(
    #    os.path.join(pybullet_data.getDataPath(),"tray/traybox.urdf"),basePosition=[0.6,-0.5,0], useFixedBase=True, physicsClientId=client_id
    #)
    cube3_id = p.loadURDF(
        os.path.join(pybullet_data.getDataPath(),"sphere_small.urdf"),basePosition=[0.0,-0.2,0.1], useFixedBase=False, physicsClientId=client_id
    )

    # store body indices in a dict with more convenient key names
    obstacles = {
        "ground": ground_id,
    #    "cube1": cube1_id,
    #    "cube2": cube2_id,
        "cube3": cube3_id,
    }
    return robot, obstacles


def create_robot_params_gui(robot):
    """Create debug params to set the robot joint positions from the GUI."""
    params = {}
    for name in robot.moveable_joint_names:
        params[name] = p.addUserDebugParameter(
            name,
            rangeMin=-2 * np.pi,
            rangeMax=2 * np.pi,
            startValue=0,
            physicsClientId=robot.client_id,
        )
    return params


def read_robot_params_gui(robot_params_gui, client_id):
    """Read robot configuration from the GUI."""
    return np.array(
        [
            p.readUserDebugParameter(
                param,
                physicsClientId=client_id,
            )
            for param in robot_params_gui.values()
        ]
    )


def main():

    # main simulation server, with a GUI
    gui_id = p.connect(p.GUI)

    # simulation server only used for collision detection
    col_id = p.connect(p.DIRECT)

    # add bodies to both of the environments
    robot, _ = load_environment(gui_id)
    col_robot, col_obstacles = load_environment(col_id)

    p.setRealTimeSimulation(1)

    # create user debug parameters
    collision_margin_param_gui = p.addUserDebugParameter(
        "collision_margin",
        rangeMin=0,
        rangeMax=0.2,
        startValue=0.01,
        physicsClientId=gui_id,
    )
    robot_params_gui = create_robot_params_gui(robot)

    # define bodies (and links) to use for shortest distance computations and
    # collision checking
    ground = col_obstacles["ground"]
    #cube1 = col_obstacles["cube1"]
    #cube2 = col_obstacles["cube2"]
    cube3 = col_obstacles["cube3"]
    link7 = (col_robot.uid, "pinky3")

    #(link7, cube1), (link7, cube2), 빼놓음
    col_detector = pyb_utils.CollisionDetector(
        col_id,
        [(link7, ground), (link7, cube3)],
    )

    last_dists = 0

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
                q = read_robot_params_gui(robot_params_gui, client_id=gui_id)

                # move to the requested configuration if it is not in collision,
                # otherwise display a warning
                # the key is that we can check collisions using the separate physics
                # client, so we don't have to set the robot to a configuration in the
                # main GUI sim to check if that configuration is in collision
                col_robot.reset_joint_configuration(q)
                if not col_detector.in_collision(
                    margin=p.readUserDebugParameter(collision_margin_param_gui),
                ):
                    robot.reset_joint_configuration(q)
                else:
                    p.addUserDebugText(
                        "Avoiding collision",
                        textPosition=[0, 0, 1.5],
                        textColorRGB=[1, 0, 0],
                        textSize=2,
                    lifeTime=0.2,
                    )

                # compute shortest distances for user-selected configuration
                dists = col_detector.compute_distances()
                if not np.allclose(last_dists, dists):
                    print(f"Distance to obstacles = {dists}")
                    last_dists = dists

                time.sleep(TIMESTEP)
                
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
                    elif k == 44: #< 4 key       
                        p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]+move),ho, maxForce=tinyForce)            
                    elif k == 46: #> 5 key           
                        p.changeConstraint(hand_cid,(hand_po[0][0],hand_po[0][1],hand_po[0][2]-move),ho, maxForce=tinyForce)
                
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
            
                    # i = 1
                    # # sending collision bool to esp32
                    # if True:
                    #     msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
                    #     # print(msg)
                    #     ser.write(msg.encode('utf-8'))
                    #     time.sleep(0.01)
                    # time.sleep(0.1)
                    # i = 0
                    # # sending collision bool to esp32
                    # if True:
                    #     msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
                    #     # print(msg)
                    #     ser.write(msg.encode('utf-8'))
                    #     time.sleep(0.01)
                    # time.sleep(0.1)

                    if any(col_finger_value) > 0:
                        msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
                        # print(msg)
                        ser.write(msg.encode('utf-8'))
                        time.sleep(TIMESTEP)
                
        except KeyboardInterrupt:
            print(KeyboardInterrupt)

        finally:
            i = 0
            msg = 'A{a}B{b}C{c}D{d}E{e}\n'.format(a=i, b=i, c=i, d=i, e=i)
            # print(msg)
            ser.write(msg.encode('utf-8'))
            time.sleep(TIMESTEP)

    else:
        print("Cannot find port")


if __name__ == "__main__":
    main()
