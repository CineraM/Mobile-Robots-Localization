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

def imu_cleaner(imu_reading = imu.getRollPitchYaw()[2]):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)

def imu_rad():
    return math.radians(imu_cleaner(imu.getRollPitchYaw()[2]))


# stuff for task 2
class Landmark:
  def __init__(self, color, x, y):
    self.color = color
    self.x = x 
    self.y = y 

lm1 = Landmark('yellow', -20, 20)
lm2 = Landmark('red', 20, 20)
lm3 = Landmark('green', -20, -20)
lm4 = Landmark('yellow', 20, -20)

# X is undiscovered
map = [['x', 'x', 'x', 'x'],
       ['x', 'x', 'x', 'x'], 
       ['x', 'x', 'x', 'x'], 
       ['x', 'x', 'x', 'x']]

class RobotPose:
  def __init__(self, x, y, grid, theta):
    self.x = x
    self.y = y
    self.grid = grid
    self.theta = theta 

def print_robot_pose(obj):
    print(f'x: {obj.x}, y: {obj.x}, tile: {obj.grid}, theta: {obj.theta}')

robot_pose = RobotPose(15, -15, 16, imu_cleaner(imu.getRollPitchYaw()[2]))


# set speed to both motors, input in Inches
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

def vSaturation(v, max):
    if math.isinf(v):
        return max
    if v > max:
        return max
    if v < -max:
        return -max
    return v

# 5.024 = max speed in in per second
def straightMotionD(d, v = 5.024):
    time = d/v  # 5.024 = v*r ==> max linear speed
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            setSpeedIPS(0,0)
            break
        setSpeedIPS(v, v)

# assume angle is in radians
def rotationInPlace(direction, angle, v):
    s = angle*dmid
    time = s/v
    s_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime()-s_time > time:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break 
        if direction == "left":
            setSpeedIPS(-v, v)
        else:
            setSpeedIPS(v, -v)


# return the distance in inches from the front pid
def frontDist():
    return frontDistanceSensor.getValue()*39.3701

def frontLidar():
    image = lidar.getRangeImage()
    return (image[0]*39.3701) - half_of_robot


# Main loop:
# perform simulation steps until Webots is stopping the controller


count = 0
while robot.step(timestep) != -1:
    print(imu_cleaner())
    # print_robot_pose(robot_pose)
    # rotationInPlace('left', pi/2, 0.8)
    if count==0:
        straightMotionD(30)
        rotationInPlace('left', pi/2, 0.3)
        count+=1
    elif count==1:
        straightMotionD(10)
        rotationInPlace('left', pi/2, 0.3)
        count+=1
    elif count==2:
        straightMotionD(30)
        count+=1












    # # Read the sensors:
    # # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
    # full_range_image = lidar.getRangeImage()
    # # print size of Range Image
    # print('#################################################################')
    # print("Lidar's Full Range Image Size: ", len(full_range_image))
    # # Compare Distance Sensors to Lidar Ranges
    # front_dist = frontDistanceSensor.getValue()
    # right_dist = rightDistanceSensor.getValue()
    # rear_dist = rearDistanceSensor.getValue()
    # left_dist = leftDistanceSensor.getValue()

    # print("Distance Sensor vs Lidar")
    # print("\tFront:\t", front_dist, "\t|", full_range_image[0])
    # print("\tRight:\t", right_dist, "\t|", full_range_image[90])
    # print("\tRear:\t", rear_dist, "\t|", full_range_image[180])
    # print("\tLeft:\t", left_dist, "\t|", full_range_image[270])

    # # Enter here functions to send actuator commands, like:
    # leftMotor.setVelocity(6)
    # rightMotor.setVelocity(6)

    # if full_range_image[0] < .07:

    #     leftMotor.setVelocity(0)
    #     rightMotor.setVelocity(0)
    #     break
# Enter here exit cleanup code.
