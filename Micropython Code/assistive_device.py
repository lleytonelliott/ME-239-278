# Master Assistive Device Logic

import time
import numpy as np

from phase_state_machine import PhaseStateMachine
from torque_controller import TorqueController
from dynamic_model import DynamicModel

from hardware import drive_motor, reset_button_pressed # and other stuff

class AssistiveDevice:
    def __init__(self, kp, kd, Kt, sitting_threshold, sit2stand_threshold, standing_threshold, stand2sit_threshold, emergency_threshold, mmarray):
        self.kp = kp
        self.kd = kd
        self.Kt = Kt
        self.th_sit = sitting_threshold
        self.th_up = sit2stand_threshold
        self.th_stand = standing_threshold
        self.th_down = stand2sit_threshold
        self.th_emergency = emergency_threshold
        
        self.state_machine = PhaseStateMachine()
        self.torque_controller = TorqueController(self.kp, self.kd, self.Kt)
        self.dynamic_model = DynamicModel(mmarray)

        self.state_vector = np.array([None]*6)
        self.sensor_time_prev_ns = time.monotonic_ns()

        self.execution_map = {
            "null": self.null_state,
            "sitting": self.sitting,
            "sit to stand": self.sit2stand,
            "standing": self.standing,
            "stand to sit": self.stand2sit,
            "shutdown": self.shutdown
        }

        self.update_sensors()

        if # state_vector hits certain threshold for sitting:
            startup_event = "startup_sitting"
        else:
            startup_event = "startup_standing"
        self.state_machine.transition(startup_event)

    def update_sensors(self):
        sensor_time_current_ns = time.monotonic_ns()
        sensor_delta_t_ns = sensor_time_current_ns - self.sensor_time_prev_ns
        self.sensor_time_prev_ns = sensor_time_current_ns

        z_ori_prev, theta_knee_prev, F_seat_prev = self.state_vector[0:3]

        self.state_vector[0] = # Set orientation about z based on sensors
        self.state_vector[1] = # Set theta position based on sensors
        self.state_vector[2] = # Set seat force based on sensors

        if z_ori_prev is None:
            self.state_vector[3:6] = 0
        elif sensor_delta_t_ns > 0:
            sensor_delta_t_s = sensor_delta_t_ns/1000000000.0
            self.state_vector[3] = (self.state_vector[0] - z_ori_prev)/sensor_delta_t_s
            self.state_vector[4] = (self.state_vector[1] - theta_knee_prev)/sensor_delta_t_s
            self.state_vector[5] = (self.state_vector[2] - F_seat_prev)/sensor_delta_t_s
        else:
            pass

    def main_loop(self):
        while True:
            self.update_sensors()

            current_phase = self.state_machine.current_phase

            execution_function = self.execution_map[current_phase]
            execution_function()

            time.sleep(0.01)
    
    def sitting(self):
        if self.check_sit2stand_threshold():
            self.state_machine.transition("stand_threshold_crossed")
            return
        if self.check_emergency_threshold():
            self.state_machine.transition("emergency")
            return

        pwm_signal = self.torque_controller.generate_PD(tau_desired = 0)
        drive_motor(pwm_signal)

    def sit2stand(self):
        if self.check_standing_threshold():
            self.state_machine.transition("finished_standing")
            return
        if self.check_sitting_threshold():
            self.state_machine.transition("finished_sitting")
            return
        if self.check_emergency_threshold():
            self.state_machine.transition("emergency")
            return

        tau_desired = self.dynamic_model.sit2stand(self.state_vector)
        pwm_signal = self.torque_controller.generate_PD(tau_desired)
        drive_motor(pwm_signal)

    def standing(self):
        if self.check_stand2sit_threshold():
            self.state_machine.transition("stand_threshold_crossed")
            return
        if self.check_emergency_threshold():
            self.state_machine.transition("emergency")
            return

        pwm_signal = self.torque_controller.generate_PD(tau_desired = 0)
        drive_motor(pwm_signal)

    def stand2sit(self):
        if self.check_sitting_threshold():
            self.state_machine.transition("finished_sitting")
            return
        if self.check_standing_threshold():
            self.state_machine.transition("finished_standing")
            return
        if self.check_emergency_threshold():
            self.state_machine.transition("emergency")
            return

        tau_desired = self.dynamic_model.stand2sit(self.state_vector)
        pwm_signal = self.torque_controller.generate_PD(tau_desired)
        drive_motor(pwm_signal)

    def shutdown(self):
        if reset_button_pressed(): # implement later
            self.state_machine.transition("startup")
            return

    def check_sitting_threshold(self):
        sv = self.state_vector
        th = self.th_sit

        if sv[0] >= th["z_ori"]: return False
        if sv[2] <= th["F_seat"]: return False
        if sv[3] >= th["z_ori_dot"]: return False
        if sv[5] <= th["F_seat_dot"]: return False
        
        return True

    def check_sit2stand_threshold(self):
        sv = self.state_vector
        th = self.th_up

        if sv[0] <= th["z_ori"]: return False
        if sv[2] >= th["F_seat"]: return False
        if sv[3] <= th["z_ori_dot"]: return False
        if sv[5] >= th["F_seat_dot"]: return False
        
        return True
    
    def check_standing_threshold(self):
        sv = self.state_vector
        th = self.th_stand

        if sv[0] >= th["z_ori"]: return False
        if sv[1] >= th["theta_knee"]: return False
        if sv[3] >= th["z_ori_dot"]: return False
        if sv[4] >= th["theta_knee_dot"]: return False
        
        return True
    
    def check_stand2sit_threshold(self):
        sv = self.state_vector
        th = self.th_down

        if sv[0] <= th["z_ori"]: return False
        if sv[1] <= th["theta_knee"]: return False
        if sv[3] <= th["z_ori_dot"]: return False
        if sv[4] <= th["theta_knee_dot"]: return False
        
        return True
    
    def check_emergency_threshold(self):
        sv = self.state_vector
        th = self.th_emergency

        if sv[0] >= th["z_ori"]: return True
        if sv[1] >= th["theta_knee"]: return True
        if sv[3] >= th["z_ori_dot"]: return True
        if sv[4] >= th["theta_knee_dot"]: return True

        return False