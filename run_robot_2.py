from math import sin, cos, pi
from lx16a import *
import time
import numpy as np
from scipy import interpolate
import try_wave as ctrl

LX16A.initialize("COM4", 0.1)

"""
positions for initialization. Use hand-tuning to get the initial position.
"""

try:
    servo_start_angle = np.loadtxt("initial_pos.csv", dtype=float)
    servo_start_angle = np.delete(servo_start_angle, 0).reshape(-1, 8)
except:
    servo_start_angle =  np.asarray([ 93.36,  88.8,  107.04, 112.56, 130.56, 122.4,  127.92,  84.])
    print("Unable to find the initial position.csv, set the position to default")

off_set = servo_start_angle * -1 
initialize = np.asarray([0, 0, 0, 0, 0, 0, 0, 0])

# we give command to the motor(with offset) and the robot will receive the physic command. Notice 
# that the physic command does not equal to physical reading
def angle_command_to_physic(command=list):
    physic = command - off_set
    return physic


def angle_physic_to_command(physic=list):
    command = physic + off_set
    return  command


def get_BSpline_command(num_of_points, phase_difference=0):
    position_space = np.genfromtxt("position_space.csv", delimiter=",") #for reading
    position_space = np.delete(position_space, 0).reshape(-1, 8)

    run_command = np.array([])

    for i in range(8):
        x = np.linspace(1, 10, len(position_space[:, i]))
        tck = interpolate.splrep(x, position_space[:, i], s=0, k=2) 
        x_new = np.linspace(min(x), max(x), num_of_points)
        y_fit = interpolate.BSpline(*tck)(x_new)
        run_command = np.append(run_command, y_fit)
    run_command = run_command.reshape(8, -1)
    # run_command[0] = 0
    run_command = np.transpose(run_command)

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

    for i in range(len(servo_list)):
        servo_list[i].move(servo_start_angle[i])



except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

def convert_to_equivalent_angle(degrees):
    """
    Converts angle in degrees to equivalent angle in range (-180, 180]
    """
    degrees = np.mod(degrees, 360) # take modulo 360 to get within 0-360 range
    degrees = np.where(degrees > 180, degrees - 360, degrees) # adjust angles > 180 to be negative
    return degrees

def Walking_test_Keyframe(input_angle):
    physic_angle = angle_command_to_physic(input_angle)
    for i in range(len(servo_list)):
        servo_list[i].move(physic_angle[i])
    time.sleep(0.01)


if __name__=='__main__':
    #initialize parameters
    start = time.time()
    while time.time() - start <= 2:
        Walking_test_Keyframe(initialize) #set the walking leg to the inital condition
    t = 0
    sampling_frequency = 80
    phase_difference = int(sampling_frequency/2)
    command = get_BSpline_command(num_of_points=sampling_frequency, phase_difference=phase_difference)

    print("############################Caution#########################")
    print("Start Running......")
    while True:
        if t == sampling_frequency-1:
            t = 0
        Walking_test_Keyframe(command[t])
        t += 1
        time.sleep(0.001)

        










