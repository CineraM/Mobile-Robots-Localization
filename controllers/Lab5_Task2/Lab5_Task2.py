# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math
#######################################################
# Creates Robot
#######################################################
robot = Robot()
#######################################################
# Sets the time step of the current world
#######################################################
timestep = int(robot.getBasicTimeStep())
#######################################################
# Gets Robots Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/distancesensor
#######################################################
frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)
#######################################################
# Gets Robots Lidar Distance Sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/lidar
#######################################################
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()
# print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res, "\nLidar Image Number of Layers: ", lidar_num_layers)
# print("Lidar Range: [",lidar_min_dist," ,", lidar_max_dist,'] in meters')
#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera_front = robot.getDevice('cameraFront')
camera_front.enable(timestep)
camera_front.recognitionEnable(timestep)
camera_right = robot.getDevice('cameraRight')
camera_right.enable(timestep)
camera_right.recognitionEnable(timestep)
camera_rear = robot.getDevice('cameraRear')
camera_rear.enable(timestep)
camera_rear.recognitionEnable(timestep)
camera_left = robot.getDevice('cameraLeft')
camera_left.enable(timestep)
camera_left.recognitionEnable(timestep)
#######################################################
# Gets Robots Motors
# Documentation:
#  https://cyberbotics.com/doc/reference/motor
#######################################################
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
#######################################################
# Gets Robot's the position sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/positionsensor
#######################################################
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# global variables
distBtwWhe = 2.28
dmid = distBtwWhe/2
w_dia = 1.6
w_r = w_dia/2
pi = math.pi
half_of_robot = 0.037*39.3701 
toIn = 39.3701


################# HELPER FUNCTIONS #################
def getLidar():
    image = lidar.getRangeImage()
    ret = []
    ret.append(image[0]*toIn - half_of_robot)   # front
    ret.append(image[270]*toIn - half_of_robot) # left
    ret.append(image[90]*toIn - half_of_robot)  # right
    ret.append(image[180]*toIn - half_of_robot) # back
    
    return ret

# set speed to both motors, input in Inches
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

# calculates distance after task
# For this lab only calculate when abs(vl) = abs(vr)
def distAfterTask(vl, vr):
    vl = round(vl, 8)
    vr = round(vr, 8)
    
    if vl == vr:
        return 0.032*vl
    if vl == -vr or math.isnan(vl):
        return 0
    return 0

# returns the decorders values
def getPositionSensors():
    return leftposition_sensor.getValue(), rightposition_sensor.getValue()

# return the imu reading in degrees instead of radians
def imuCleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)

# code that generates the 3 points for all tiles, 16 tiles
def generateTiles():
    y = 20
    tiles = []
    for i in range(4):
        x = -20
        for j in range(4):
            tiles.append([[x, y], [x+10, y], [x, y-10], [x+10, y-10]])
            x+=10
        y-=10
    return tiles
################# HELPER FUNCTIONS #################

# 0 = undiscovered, 1 = discovered
grid = [[0, 0, 0, 0],
       [0, 0, 0, 0], 
       [0, 0, 0, 0], 
       [0, 0, 0, 0]]

################ robot class & functions #####################
class Landmark:
  def __init__(self, color, x, y, r):
    self.color = color
    self.x = x 
    self.y = y
    self.r = r
# 1=yellow, 2=red, 3=blue, 4=green
lnm1 = Landmark(1, -20, 20, -1)
lnm2 = Landmark(2, 20, 20, -1)
lnm3 = Landmark(3, 20, -20, -1)
lnm4 = Landmark(4, -20, -20, -1)

# landmark objects
def triliteration(l1, l2, l3):
    A = (-2*l1.x + 2*l2.x)
    B = (-2*l1.y + 2*l2.y)
    C = pow(l1.r, 2) - pow(l2.r, 2) - pow(l1.x, 2) + pow(l2.x, 2) - pow(l1.y, 2) + pow(l2.y, 2)

    D = (-2*l2.x + 2*l3.x)
    E = (-2*l2.y + 2*l3.y)
    F = pow(l2.r, 2) - pow(l3.r, 2) - pow(l2.x, 2) + pow(l3.x, 2) - pow(l2.y, 2) + pow(l3.y, 2)

    x = (C*E-F*B)/(E*A-B*D)
    y = (C*D-A*F)/(B*D-A*E)
    return x, y


# yellow = 379
# red = 372
# blue = 393
# green = 386
def updateLmR(id, new_r):
    global lnm1, lnm2, lnm3, lnm4
    new_r+=half_of_robot
    if id == 379:
        lnm1.r = new_r
        return lnm1
    elif id == 372:
        lnm2.r = new_r
        return lnm2
    elif id == 393:
        lnm3.r = new_r
        return lnm3
    elif id == 386:
        lnm4.r = new_r 
        return lnm4

def findLandmarks():
    fc_objects = camera_front.getRecognitionObjects()
    lc_objects = camera_left.getRecognitionObjects()
    rc_objects = camera_right.getRecognitionObjects()
    bc_objects = camera_rear.getRecognitionObjects()

    myset = []
    landmarks = []
    if len(fc_objects) > 0:
        if fc_objects[0].getId() not in myset:
            myset.append(fc_objects[0].getId())
            landmarks.append(updateLmR(fc_objects[0].getId(), fc_objects[0].getPosition()[0] * 39.3701))
    if len(lc_objects) > 0:
        if lc_objects[0].getId() not in myset:
            myset.append(lc_objects[0].getId())
            landmarks.append(updateLmR(lc_objects[0].getId(), lc_objects[0].getPosition()[0] * 39.3701))
    if len(rc_objects) > 0:
        if rc_objects[0].getId() not in myset:
            myset.append(rc_objects[0].getId())
            landmarks.append(updateLmR(rc_objects[0].getId(), rc_objects[0].getPosition()[0] * 39.3701))
    if len(bc_objects) > 0:
        if bc_objects[0].getId() not in myset:
            myset.append(bc_objects[0].getId())
            landmarks.append(updateLmR(bc_objects[0].getId(), bc_objects[0].getPosition()[0] * 39.3701))
    
    if len(landmarks) >= 3:
        landmarks = sorted(landmarks,key=lambda x: (x.color)) # sort based on first index, always maintain the same order
        return landmarks
    else:
        return []


class RobotPose:
  def __init__(self, x, y, tile, theta):
    self.x = x
    self.y = y
    self.tile = tile
    self.theta = theta 

# change 
def updateGrid(tile):
    global grid
    i = tile//4
    j = tile%4
    grid[i][j] = 1

tiles_coordinates = generateTiles()

#print the grid & robot pose
def printRobotPose(obj):
    global grid
    print(f'x: {obj.x:.2f}\ty: {obj.y:.2f}\ttile: {obj.tile}\ttheta: {obj.theta:.2f}')
    for list in grid:
        print("\t" + str(list))
    print("-----------------------------------------------")

ROBOT_POSE = RobotPose(15.0, -15.0, 16, 90)
updateGrid(ROBOT_POSE.tile-1)
prev_l, prev_r = getPositionSensors()

# bottom left, top right, robot
def updateTile(pose):
    global tiles_coordinates
    # up, down, left, right instead looking though all the tiles
    # the search space is extremly small, this will not affect performance
    for i in range(len(tiles_coordinates)):
        tl = tiles_coordinates[i][0]
        br = tiles_coordinates[i][3]
        x, y = pose.x, pose.y

        if x > tl[0] and x < br[0]:
            if y < tl[1] and y > br[1]:
                return i+1
    return -1

# Update pose and grid
def updatePose(obj):
    global prev_l, prev_r
    cur_l, cur_r = getPositionSensors()
    vl = (cur_l-prev_l)/0.032   # 32 ms 
    vr = (cur_r-prev_r)/0.032
    imu_reading = imuCleaner(imu.getRollPitchYaw()[2])
    dist = distAfterTask(vl*w_r, vr*w_r)
    obj.theta = imu_reading
    
    prev_l = cur_l
    prev_r = cur_r

    if imu_reading < 94 and imu_reading > 86:
        obj.y += dist
    elif imu_reading < 184 and imu_reading > 176:
        obj.x -= dist
    elif imu_reading < 274 and imu_reading > 266:
        obj.y -= dist
    elif imu_reading <= 360 and imu_reading > 356 or imu_reading < 4 and imu_reading >= 0:
        obj.x += dist

    tile = updateTile(obj)
    if tile != -1: 
        obj.tile = tile
        updateGrid(tile-1)
################ robot class & functions #####################



################ motion functions #####################
# 5.024 = max speed in in per second
def straightMotionD(d):
    global ROBOT_POSE
    v = 5.024
    is_neg = False
    if d < 0:
        is_neg = True
        d = abs(d)

    time = d/v  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            setSpeedIPS(0,0)
            updatePose(ROBOT_POSE)
            # printRobotPose(ROBOT_POSE)
            break
        if is_neg:
            setSpeedIPS(-v, -v)
        else:
            setSpeedIPS(v, v)
        updatePose(ROBOT_POSE)
        # printRobotPose(ROBOT_POSE)

# assume angle is in radians
def rotationInPlace(direction, angle, v):
    global ROBOT_POSE
    s = angle*dmid
    time = s/v
    s_time = robot.getTime()

    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            updatePose(ROBOT_POSE)
            # printRobotPose(ROBOT_POSE)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)
            
        updatePose(ROBOT_POSE)
        # printRobotPose(ROBOT_POSE)
################ motion functions #####################

############## traversal logic ############
def checkWalls(theta):
    # front, left, right, back
    lidar = getLidar()
    no_wall = []
    for lid in lidar:
        if lid < 4:
            no_wall.append(False)
        else:
            no_wall.append(True)
    
    if theta < 94 and theta > 86:
        return no_wall 
    elif theta < 184 and theta > 176:
        return [no_wall[2], no_wall[0], no_wall[3], no_wall[1]]
    elif theta < 274 and theta > 266:
        return [no_wall[3], no_wall[2], no_wall[1], no_wall[0]]
    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        return [no_wall[1], no_wall[3], no_wall[0], no_wall[2]]

# forward, left, right, backward
# check if there are walls
def neighTiles(tile, theta=90):
    valid_neigh = []
    n = 4 # could use as an input, not for this lab
    i = tile//n
    j = tile%n

    #up
    if i == 0: valid_neigh.append(False)
    else:
        if grid[i-1][j] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # left 
    if j == 0: valid_neigh.append(False)
    else:
        if grid[i][j-1] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # right
    if j == n-1:valid_neigh.append(False)
    else:
        if grid[i][j+1] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    # down
    if i == n-1:valid_neigh.append(False)
    else:
        if grid[i+1][j] == 0:
            valid_neigh.append(True)
        else:
            valid_neigh.append(False)
    
    # print(valid_neigh)
    valid_walls = checkWalls(theta)
    for i in range(len(valid_walls)):
        if valid_walls[i] == False:
            valid_neigh[i] = False
        
    # print(valid_walls)
    # print(valid_neigh)
    # print("-------------------------")

    return valid_neigh

stack = []
def traversalStrightHelper():
    global stack
    straightMotionD(10)
    stack.append(1)
    
def traversalRotationtHelper():
    global stack
    rotationInPlace('left', pi/2, 0.6)
    stack.append(0)


def traversalRotationtHelper(theta, neighbors):

    if theta < 94 and theta > 86:
        if neighbors[1]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[2]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[3]:
            rotationInPlace('left', pi/2, 0.6)
            rotationInPlace('left', pi/2, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta < 184 and theta > 176:
        if neighbors[3]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
            return
        elif neighbors[0]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[2]:
            rotationInPlace('left', pi/2, 0.6)
            rotationInPlace('left', pi/2, 0.6)
            stack.append(-1)
            stack.append(-1)

    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if neighbors[0]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[3]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[1]:
            rotationInPlace('righ', pi/2, 0.6)
            rotationInPlace('righ', pi/2, 0.6)
            stack.append(0)
            stack.append(0)

    elif theta < 274 and theta > 266:
        if neighbors[2]:
            rotationInPlace('left', pi/2, 0.6)
            stack.append(0)
        elif neighbors[1]:
            rotationInPlace('right', pi/2, 0.6)
            stack.append(-1)
        elif neighbors[0]:
            rotationInPlace('righ', pi/2, 0.6)
            rotationInPlace('righ', pi/2, 0.6)
            stack.append(0)
            stack.append(0)

def traverse():
    global grid, stack, ROBOT_POSE
    printRobotPose(ROBOT_POSE)
    flag = False
    for list in grid:
        if 0 in list: 
            flag = True
            break
    if flag == False: # victory spin :) 
        setSpeedIPS(-2, 2)
        return

    n_tiles = neighTiles(ROBOT_POSE.tile-1, ROBOT_POSE.theta)
    theta = ROBOT_POSE.theta

    # print(stack)
    # BACK TRACK
    if True not in n_tiles: 
        top = stack.pop()
        if top == 1:
            straightMotionD(-10)
        elif top == 0:
            rotationInPlace('right', pi/2, 0.6)
        elif top == -1:
            rotationInPlace('left', pi/2, 0.6)
    
    elif theta < 94 and theta > 86:
        if n_tiles[0]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta < 184 and theta > 176:
        if n_tiles[1]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if n_tiles[2]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
    elif theta < 274 and theta > 266:
        if n_tiles[3]:
            traversalStrightHelper()
        else:
            traversalRotationtHelper(theta, n_tiles)
############## traversal logic ############


############## wall-follow ############
def vSaturation(v, max):
    if math.isinf(v):
        return max
    if v > max:
        return max
    if v < -max:
        return -max
    return v

def frontLidar():
    image = lidar.getRangeImage()
    return (image[0]*39.3701) - half_of_robot

def getLidarMin():
    image = lidar.getRangeImage()
    toIn = 39.3701

    min_left = 999
    min_right = 999

    for i in range(270, 360):
        if min_left > image[i]:
            min_left = image[i]
    
    for i in range(0, 91):
        if min_right > image[i]:
            min_right = image[i]

    return [min_left*toIn - half_of_robot, min_right*toIn - half_of_robot]
def wallFollowLidar(wall, flid, k):
    lids = getLidarMin()

    left_lid = lids[0]
    right_lid = lids[1]
    dist_to_wall = 1.5
    v = vSaturation(flid, 4)
    error = (v - 2.5)  # target distance to wall = 2.5 inches
    pid = v-abs(error)*k
    if wall == 'right':    
        if flid > 3:
            if right_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(pid, v)
            elif right_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(v, pid)
            elif left_lid < dist_to_wall:   # too close to opposite wall
                setSpeedIPS(v, pid)
        else:
            setSpeedIPS(v, v)

def wallFollow():
    fpid = frontLidar()
    wall = 'right'
    if fpid < 2.5:  # to close to wall, rotate 45 deg away from it
        rotationInPlace('left', pi/4, 0.9)
    else:   # else follow wall
        wallFollowLidar(wall, frontLidar(), 1)
############## wall-follow ############


while robot.step(timestep) != -1:
    landmarks = findLandmarks()
    # for lan in landmarks:
    #     print(lan.color)

    if len(landmarks) >= 3:
        x, y = triliteration(landmarks[0], landmarks[1], landmarks[2])
        print(x, y)
    else:
        print("not enough landmarks")
    # traverse()
    wallFollow()