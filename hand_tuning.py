from lx16a import *
import time
import numpy as np
import time

"""
this code is for hand tuning and get phisical positions and read positions of the robot, and
"""

try:
    servo_start_angle = np.loadtxt("initial_pos.csv", dtype=float)
    servo_start_angle = np.delete(servo_start_angle, 0).reshape(-1, 8)
except:
    servo_start_angle =  np.asarray([ 93.36,  88.8,  107.04, 112.56, 130.56, 122.4,  127.92,  84.])
    print("Unable to find the initial position.csv, set the position to default")

off_set = servo_start_angle * -1 

def angle_command_to_physic(command=list):
    physic = command - off_set
    return physic

def angle_physic_to_command(physic=list):
    command = physic + off_set
    return  command

LX16A.initialize("COM4")
print('ready?')
try:
    servo1 = LX16A(11)
    servo2 = LX16A(12)
    servo3 = LX16A(13)
    servo4 = LX16A(14)
    servo5 = LX16A(21)
    servo6 = LX16A(22)
    servo7 = LX16A(23)
    servo8 = LX16A(24)

    servo_list = [servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8]

    for servo_ in servo_list:
        servo_.set_angle_limits(0, 240)

except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()


position_space = np.empty(1)
print('ready')
t = 0

while True:
    if input() == 'stop':
        np.savetxt("position_space.csv", position_space, delimiter=",")
        print("the keyframe positions(relative positions) has been saved into the position_space.csv")
    physical_position = np.array(list(servo.get_physical_angle() for servo in servo_list))
    read_position = angle_physic_to_command(physical_position)
    print('physical position:', physical_position)
    print("read position:", read_position)
    position_space = np.append(position_space, read_position)
    time.sleep(1)
    t += 0.1