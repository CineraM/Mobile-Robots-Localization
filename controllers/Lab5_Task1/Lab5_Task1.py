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


################# HELPER FUNCTIONS #################
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
################# END  #################

# 0 = undiscovered, 1 = discovered
grid = [[0, 0, 0, 0],
       [0, 0, 0, 0], 
       [0, 0, 0, 0], 
       [0, 0, 0, 0]]

################# Updating-Pose Code #################
# change 
def updateGrid(tile):
    global grid
    i = tile//4
    j = tile%4
    grid[i][j] = 1


def checkWall():
    pass
# forward, left, right, backward
# check if there are walls
def neighTiles(tile):
    tiles = []
    n = 4 # could use as an input, not for this lab
    i = tile//n
    j = tile%n

    # print(i, j)
    #up
    if i == 0:
        tiles.append(False)
    else:
        if grid[i-1][j] == 0:
            tiles.append(True)
        else:
            tiles.append(False)
    # left 
    if j == 0:
        tiles.append(False)
    else:
        if grid[i][j-1] == 0:
            tiles.append(True)
        else:
            tiles.append(False)
    # right
    if j == n-1:
        tiles.append(False)
    else:
        if grid[i][j+1] == 0:
            tiles.append(True)
        else:
            tiles.append(False)
    # down
    if i == n-1:
        tiles.append(False)
    else:
        if grid[i+1][j] == 0:
            tiles.append(True)
        else:
            tiles.append(False)

    return tiles

tiles_coordinates = generateTiles()

# robot class & functions
class RobotPose:
  def __init__(self, x, y, tile, theta):
    self.x = x
    self.y = y
    self.tile = tile
    self.theta = theta 

#print the grid & robot pose
def printRobotPose(obj):
    global grid
    print(f'x: {obj.x:.2f}\ty: {obj.y:.2f}\ttile: {obj.tile}\ttheta: {obj.theta:.2f}')
    for list in grid:
        print(list)
    print("-----------------------------------------------")

ROBOT_POSE = RobotPose(-15.0, -15.0, 13, 90)
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

# 5.024 = max speed in in per second
def straightMotionD(d):
    global ROBOT_POSE
    v = 5.024
    time = d/v  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            setSpeedIPS(0,0)
            updatePose(ROBOT_POSE)
            printRobotPose(ROBOT_POSE)
            break
        setSpeedIPS(v, v)
        updatePose(ROBOT_POSE)
        printRobotPose(ROBOT_POSE)

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
            printRobotPose(ROBOT_POSE)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)
        updatePose(ROBOT_POSE)
        printRobotPose(ROBOT_POSE)


def frontLidar():
    image = lidar.getRangeImage()
    return (image[0]*39.3701) - half_of_robot

stack = []
def idk():
    global grid, stack, ROBOT_POSE
    flag = False
    for list in grid:
        if 0 in list: 
            flag = True
            break
    if flag == False: return

    n_tiles = neighTiles(ROBOT_POSE.tile-1)
    theta = ROBOT_POSE.theta

    if 0 not in n_tiles: # back track
        print("Test")

        temp = stack.top()
        if len(temp) == 1:
            straightMotionD(-10)
        else:
            rotationInPlace('right', pi/2, 0.6)
        stack.pop()
    
    elif theta < 94 and theta > 86:
        if n_tiles[0]:
            straightMotionD(10)
            stack.append([10])
        else:
            rotationInPlace('left', pi/2, 0.6)
            stack.append([0.6, 0]) 
    elif theta < 184 and theta > 176:
        if n_tiles[1]:
            straightMotionD(10)
            stack.append([10])
        else:
            rotationInPlace('left', pi/2, 0.6)
            stack.append([0.6, 1])
    elif theta <= 360 and theta > 356 or theta < 4 and theta >= 0:
        if n_tiles[2]:
            straightMotionD(10)
            stack.append([10])
        else:
            rotationInPlace('left', pi/2, 0.6)
            stack.append([0.6, 1])
    elif theta < 274 and theta > 266:
        if n_tiles[3]:
            straightMotionD(10)
            stack.append([10])
        else:
            rotationInPlace('left', pi/2, 0.6)
            stack.append([0.6, 1])
                
while robot.step(timestep) != -1:
    idk()

# stuff for task 2
class Landmark:
  def __init__(self, color, x, y, r):
    self.color = color
    self.x = x 
    self.y = y
    self.r = r

lnm1 = Landmark('yellow', -20, 20)
lnm2 = Landmark('red', 20, 20)
lnm3 = Landmark('green', -20, -20)
lnm4 = Landmark('blue', 20, -20)

# lnm
#[x, y, radius]
def trilateration(c1, c2, c3):
    
    A = (-2*c1.x + 2*c2.x)
    B = (-2*c1.y + 2*c2.y)
    C = ( pow(c1.r, 2) -  pow(c2.r, 2) - pow(c1.x, 2) + pow(c2.x, 2) - pow(c1.y, 2) + pow(c2.y, 2))

    D = (-2*c2.x + 2*c3.x)
    E = (-2*c2.y + 2*c3.y)
    F = ( pow(c2.r, 2) -  pow(c3.r, 2) - pow(c2.x, 2) + pow(c2.x, 2) - pow(c2.y, 2) + pow(c3.y, 2))

    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F)
    return x, y