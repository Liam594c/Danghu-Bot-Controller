import time
import numpy as np
import matplotlib.pyplot as plt

# m = mujoco.MjModel.from_xml_path('Codes\mujoco_model\scene.xml')
# d = mujoco.MjData(m)

front_hip_range = (-0.855, 0.27)
back_hip_range = (-1, 0.81)
ankle_range = (-0.63, 0.09)
motor_set_range = (-4.5, 0.18)

def gen_random_coeff(order, angle_range):
    """
    coeff space:
    a11 a12 a13 a14 a21 a22 a23 a24 omega
    """
    len = order*2+1
    random_coeff = np.zeros(len)
    for i in range(order):
        random_coeff[i] = np.random.uniform(angle_range[0], angle_range[1])
        random_coeff[i+order] = np.random.uniform(-np.pi, np.pi)
    omega = np.random.uniform(-10, 10)
    random_coeff[len-1] = omega
    return random_coeff

# def generate_curve3(coeffs, time, range):
#     """
#     3 order curves
#     coeffs: coefficients generated for the sinwave function
#     dt: step time, the same as system step time, which is 0.002
#     time: total length of period, which is normally 10 secs
#     """
#     y = coeffs[0]*np.sin(coeffs[6]*time+coeffs[3])+coeffs[1]*np.sin(2*coeffs[6]*time+coeffs[4])
#     +coeffs[2]*np.sin(3*coeffs[6]*time+coeffs[5])
#     a = range[0]
#     b = range[1]
#     if y > a:
#         y = a
#     if y < b:
#         y = b
#     return y

def generate_curve3(coeffs, dt, time, range):
    """
    3 order curves
    coeffs: coefficients generated for the sinwave function
    dt: step time, the same as system step time, which is 0.002
    time: total length of period, which is normally 10 secs
    """
    num_of_steps = int(time/dt)
    x_values = np.linspace(0, time, num_of_steps)
    y_values = np.zeros_like(x_values)
    index = 0
    for t in x_values:
        y_values[index] = coeffs[0]*np.sin(coeffs[6]*t+coeffs[3])+coeffs[1]*np.sin(2*coeffs[6]*t+coeffs[4])
        +coeffs[2]*np.sin(3*coeffs[6]*t+coeffs[5])
        index+=1
    a = range[0]
    b = range[1]
    y_values_filtered = np.clip(y_values, a, b)
    return y_values_filtered

class controller():
    front_hip_range = (-0.855, 0.27)
    back_hip_range = (-1, 0.81)
    ankle_range = (-0.63, 0.09)
    motor_set_range = (-4.5, 0.18)
    def __init__(self):
        """
        position control, assign coefficients
        """
        self.order = 3
        self.a11 = gen_random_coeff(self.order, motor_set_range) #motors
        self.a12 = gen_random_coeff(self.order, front_hip_range)
        self.a13 = gen_random_coeff(self.order, back_hip_range)
        self.a14 = gen_random_coeff(self.order, ankle_range)
        self.a21 = gen_random_coeff(self.order, motor_set_range)
        self.a22 = gen_random_coeff(self.order, front_hip_range)
        self.a23 = gen_random_coeff(self.order, back_hip_range)
        self.a24 = gen_random_coeff(self.order, ankle_range)
        self.trial = [self.a11, self.a12, self.a13, self.a14, self.a21, self.a22, self.a23, self.a24]
        self.saved = [self.a11, self.a12, self.a13, self.a14, self.a21, self.a22, self.a23, self.a24]
        self.dt = 0.002
    
    def plotting(self, a_xx, range):
        curve = generate_curve3(a_xx, 0.002, 10, range)
        plt.plot(curve)
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title('Generated Curve')
        plt.show()


    def change_coeff(self):
        self.a11 = gen_random_coeff(self.order, motor_set_range) #motors
        self.a12 = gen_random_coeff(self.order, front_hip_range)
        self.a13 = gen_random_coeff(self.order, back_hip_range)
        self.a14 = gen_random_coeff(self.order, ankle_range)
        self.a21 = gen_random_coeff(self.order, motor_set_range)
        self.a22 = gen_random_coeff(self.order, front_hip_range)
        self.a23 = gen_random_coeff(self.order, back_hip_range)
        self.a24 = gen_random_coeff(self.order, ankle_range)
        self.trial = [self.a11, self.a12, self.a13, self.a14, self.a21, self.a22, self.a23, self.a24]
        return
    

    def save_coeff(self):
        self.saved = [self.a11, self.a12, self.a13, self.a14, self.a21, self.a22, self.a23, self.a24]
        return

    def calculate_angle3(self, t, lag):
        """
        3 order curves
        return a 1*6 numpy array
        """
        angle = np.zeros((1, 8))
        front_hip_range = (-0.855, 0.27)
        back_hip_range = (-1, 0.81)
        ankle_range = (-0.63, 0.09)
        motor_set_range = (-4.5, 0.18)
        range_in_sequence = [motor_set_range, front_hip_range, back_hip_range, ankle_range,
                             motor_set_range, front_hip_range, back_hip_range, ankle_range
                             ]
        
        for i in range(4):
            coeffs = self.saved[i]
            omega = coeffs[6]
            T = 2*np.pi/omega
            t1 = t
            t2 = t+T*lag
            y1 = coeffs[0]*np.sin(coeffs[6]*t1+coeffs[3])+coeffs[1]*np.sin(2*coeffs[6]*t1+coeffs[4])
            +coeffs[2]*np.sin(3*coeffs[6]*t1+coeffs[5])
            y2 = coeffs[0]*np.sin(coeffs[6]*t2+coeffs[3])+coeffs[1]*np.sin(2*coeffs[6]*t2+coeffs[4])
            +coeffs[2]*np.sin(3*coeffs[6]*t2+coeffs[5])
            y1 = y1*5
            y2 = y2*5
            angle_range = range_in_sequence[i]
            a = angle_range[0]
            b = angle_range[1]
            if y1 > b:
                y1 = b
            if y1 < a:
                y1 = a
            if y2 > b:
                y2 = b
            if y2 < a:
                y2 = a
            angle[0, i] = y1
            angle[0, i+4] = y2
        return angle

    def calculate_angle_with_saved3(self, t):
        """
        3 order curves
        return a 1*6 numpy array
        """
        angle = np.zeros((1, 8))
        front_hip_range = (-0.855, 0.27)
        back_hip_range = (-1, 0.81)
        ankle_range = (-0.63, 0.09)
        motor_set_range = (-4.5, 0.18)
        range_in_sequence = [motor_set_range, front_hip_range, back_hip_range, ankle_range,
                             motor_set_range, front_hip_range, back_hip_range, ankle_range
                             ]
        for i in range(8):
            coeffs = self.saved[i]
            y_1 = coeffs[0]*np.sin(coeffs[6]*t+coeffs[3])+coeffs[1]*np.sin(2*coeffs[6]*t+coeffs[4])
            +coeffs[2]*np.sin(3*coeffs[6]*t+coeffs[5])
            print(y)
            angle_range = range_in_sequence[i]
            a = angle_range[0]
            b = angle_range[1]
            if y > b:
                y = b
            if y < a:
                y = a
            angle[0, i] = y
        return angle
    
