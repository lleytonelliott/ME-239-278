# Virtual Torque Controller

import time
from hardware import read_motor_current

class TorqueController:
    def __init__(self, kp, kd, Kt):
        self.kp = kp
        self.kd = kd
        self.Kt = Kt
        self.tau_prev = None
        self.pd_time_prev_ns = time.monotonic_ns()
    
    def generate_PD(self, tau_desired):
        motor_current = read_motor_current()
        tau_actual = motor_current*self.Kt
        if self.tau_prev == None:
            self.tau_prev = tau_actual

        error = tau_desired - tau_actual

        pd_time_current_ns = time.monotonic_ns()
        pd_delta_t_ns = pd_time_current_ns - self.pd_time_prev_ns
        self.pd_time_prev_ns = pd_time_current_ns

        if pd_delta_t_ns == 0:
            pd_delta_t_s = 0.0
            d_tau = 0
        else:
            pd_delta_t_s = pd_delta_t_ns/1000000000.0
            d_tau = (tau_actual - self.tau_prev)/pd_delta_t_s
            self.tau_prev = tau_actual
                
        return self.kp*error - self.kd*d_tau