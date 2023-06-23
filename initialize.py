from run_robot import *
import time
import numpy as np

try:
    servo_start_angle = np.loadtxt("initial_pos.csv", dtype=float)
    servo_start_angle = np.delete(servo_start_angle, 0).reshape(-1, 8)
except:
    servo_start_angle =  np.asarray([ 93.36,  88.8,  107.04, 112.56, 130.56, 122.4,  127.92,  84.])
    print("Unable to find the initial position.csv, set the position to default")

initialize = np.asarray([0, 0, 0, 0, 0, 0, 0, 0])

start = time.time()
while time.time() - start <= 2:
        Walking_test_Keyframe(initialize) #set the walking leg to the inital condition