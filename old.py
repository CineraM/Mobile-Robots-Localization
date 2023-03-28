"""
Matias Cinera - U 6931_8506
CAP-6626
Instructor: Dr.Alfredo Weitzenfeld 
Ta:         Chance Hamilton
Assigment:  lab4_task2 controller.
"""
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

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

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

def getLidar():
    image = lidar.getRangeImage()
    toIn = 39.3701
    return [image[270]*toIn - half_of_robot, image[90]*toIn - half_of_robot]

# set speed to both motors
def setSpeedIPS(vl, vr):
    vl /= w_r
    vr /= w_r
    leftMotor.setVelocity(vl)
    rightMotor.setVelocity(vr)

# saturation fnc
def vSaturation(v, max):
    if math.isinf(v):
        return max
    if v > max:
        return max
    if v < -max:
        return -max
    return v

# return the distance in inches from the front pid
def frontDist():
    return frontDistanceSensor.getValue()*39.3701

def frontLidar():
    image = lidar.getRangeImage()
    return (image[0]*39.3701) - half_of_robot

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
    elif wall == 'left':
        if flid > 3:
            if left_lid < dist_to_wall:    # too close to target wall
                setSpeedIPS(v, pid)
            elif left_lid > dist_to_wall:  # too far to target wall
                setSpeedIPS(pid, v)
            elif right_lid < dist_to_wall: # too close to opposite wall
                setSpeedIPS(pid, v)
        else:
            setSpeedIPS(v, v)

def wallFollow():
    
    kps_vals = [0.1, 0.5, 1.0, 2.0, 2.5, 5.0]
    fpid = frontDist()
    wall = 'right'
    if fpid < 2.5:  # to close to wall, rotate 45 deg away from it
        rotationInPlace('left', pi/4, 0.9)
    else:   # else follow wall
        wallFollowLidar(wall, frontLidar(), kps_vals[2])


def distToObject():
    try:
        return camera.getRecognitionObjects()[0].getPosition()[0] * 39.3701
    except:
        return 999
    
def motionToGoal(goal_dist):
    pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
    # rotate to look at the object
    if pos_image < 38:
        rotationInPlace('left', pi/60, 0.9)
        return
    elif pos_image > 42:    
        rotationInPlace('right', pi/60, 0.9)
        return
    # pid logic
    # print(f'dist sensor: {distance}, camera: {pos_view*39.3701}') # debug
    error = distToObject() - goal_dist
    v = (2*error) # 2 == kp value
    v = vSaturation(v, 5.024)
    if abs(error) < 0.1:    # stop motors if 0.1in awa from goal
        setSpeedIPS(0, 0)
    else:
        setSpeedIPS(v, v)

def findGoal():
    count=0
    while robot.step(timestep) != -1 and count<8:
        obj_in_view = len(camera.getRecognitionObjects())
        if obj_in_view > 0: return True
        rotationInPlace('left', pi/4, 0.9)
        count+=1
    
    return False

def main():
    WF = False
    MG = False
    obj_in_view = len(camera.getRecognitionObjects())
    if obj_in_view > 0: MG = True

    while robot.step(timestep) != -1:
        dist_to_goal = -1
        front_dist = frontDist()
        dist_to_goal = distToObject()
        error = abs(front_dist - dist_to_goal) - 1.8
        obj_in_view = len(camera.getRecognitionObjects())
        
        if WF:
            pos_image = -1
            try: pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
            except:pass
            wallFollow()
            # change to Motion to goal if the object is in the camera fov
            if pos_image >= 30 and pos_image <= 50: # ideally 38 or 42
                print(f'Motion to Goal\t\t\tTime: {robot.getTime():.2f}')  
                MG, WF = True, False
        elif MG:
            if error <= 0.2: # goal found, stop the motors
                motionToGoal(5)
                continue
            # if close to a wall, change to wall follow
            elif front_dist <= 5.2 and front_dist < dist_to_goal: # 5.2 good value when aporaching corners
                rotationInPlace('left', pi/4, 0.9) # half left turn
                WF, MG = True, False
                print(f'Following Right Wall\t\tTime: {robot.getTime():.2f}')  
            elif obj_in_view > 0:
                motionToGoal(5)
            else:
                WF, MG = False, False
        else: 
            # either find goal or wall follow
            if obj_in_view > 0: # if goal is already in view, MG
                print(f'Motion to Goal\t\t\tTime: {robot.getTime():.2f}') 
                MG, WF = True, False
                continue

            # else try to find goal
            found = findGoal()  
            if found:
                print(f'Motion to Goal\t\t\tTime: {robot.getTime():.2f}') 
                MG, WF = True, False
            else: # if goal was not found, go forward until the robot is close to a wall, then wall follow
                if front_dist > 3: # wall follow
                    while frontDist() < 3:
                        setSpeedIPS(2, 2)
                    setSpeedIPS(0, 0)

                print(f'Following Right Wall\t\tTime: {robot.getTime():.2f}')    
                WF, MG = True, False                

if __name__ == "__main__":
    main()