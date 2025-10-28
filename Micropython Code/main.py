# Main Code for Assistive Device

import time
import numpy as np

# Loop Variables

kp = 0
kd = 0
Kt = 0

# Thresholds (will implement later)

sitting_threshold = {
    "z_ori": 0,
    "F_seat": 0,
    "z_ori_dot": 0,
    "F_seat_dot": 0
}

sit2stand_threshold = {
    "z_ori": 0,
    "F_seat": 0,
    "z_ori_dot": 0,
    "F_seat_dot": 0
}

standing_threshold = {
    "z_ori": 0,
    "theta_knee": 0,
    "z_ori_dot": 0,
    "theta_knee_dot": 0,
}

stand2sit_threshold = {
    "z_ori": 0,
    "theta_knee": 0,
    "z_ori_dot": 0,
    "theta_knee_dot": 0,
}

emergency_threshold = {
    "z_ori": 0,
    "theta_knee": 0,
    "z_ori_dot": 0,
    "theta_knee_dot": 0,
}

# Master Class

class AssistiveDevice:
    def __init__(self, kp, kd, Kt, sitting_threshold, sit2stand_threshold, standing_threshold, stand2sit_threshold, emergency_threshold):
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
        self.dynamic_model = DynamicModel()

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
        self.state_machine.transition("startup", self.state_vector)

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
        if resetbuttonpressed(): # implement later
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
        
# Dynamic Model

class DynamicModel:
    def __init__(self):
        pass

    def sit2stand(self, state_vector):
        # Model based on state vector
        return tau_desired

    def stand2sit(self, state_vector):
        # Model based on state vector
        return tau_desired

# State Machine for Phases of Motion

class PhaseStateMachine:
    def __init__(self):
        self.current_phase = "null"

        self.phase_handlers = {
            "null": self.null_state,
            "sitting": self.sitting,
            "sit to stand": self.sit2stand,
            "standing": self.standing,
            "stand to sit": self.stand2sit,
            "shutdown": self.shutdown,
        }

        self.valid_events = {
            "startup",
            "stand_threshold_crossed",
            "finished_standing",
            "sit_threshold_crossed",
            "finished_sitting",
            "emergency"
        }
        
    def transition(self, event, state_vector = None):
        if event not in self.valid_events:
            print(f"Ignored Invalid Event '{event}'.")
            return
        
        handler = self.phase_handlers[self.current_phase]
        new_phase = handler(event, state_vector = state_vector)

        if self.current_phase != new_phase:
            print(f"Transition: '{self.current_phase}' -> '{new_phase}' on event '{event}'.")
            self.current_phase = new_phase
        
    def sitting(self, event, **kwargs):
        if event == "stand_threshold_crossed":
            return "sit to stand"
        elif event == "emergency":
            return "null"
        return "sitting"
    
    def sit2stand(self, event, **kwargs):
        if event == "finished_standing":
            return "standing"
        elif event == "finished_sitting":
            return "sitting"
        elif event == "emergency":
            return "null"
        return "sit to stand"
    
    def standing(self, event, **kwargs):
        if event == "sit_threshold_crossed":
            return "stand to sit"
        elif event == "emergency":
            return "null"
        return "standing"

    def stand2sit(self, event, **kwargs):
        if event == "finished_sitting":
            return "sitting"
        elif event == "finished_standing":
            return "standing"
        elif event == "emergency":
            return "null"
        return "stand to sit"
    
    def null_state(self, event, state_vector):
        if event == "startup":
            if #state_vector hits certain threshold for sitting:
                return "sitting"
            else:
                return "standing"
        elif event == "emergency":
            return "shutdown"
        return "null"
    
    def shutdown(self, event, **kwargs):
        if event == "startup":
            return "null"
        return "shutdown"

# Low-Level Torque Controller

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

# Other functions to figure out:

# def calculate_desired_torque(state_vector, phase):
#   Calculate ideal torque based on state vector and current phase of motion

#   Update IMU (probably separate function for integration)
#   Find encoder position and calculate knee angle
#   Find force vector from seat

# def read_motor_current

# def drive_motor(PD_signal)
#   Some code to drive the motor

if __name__ == "__main__":
    device = AssistiveDevice(kp = kp, kd = kd, Kt = Kt, sitting_threshold = sitting_threshold, sit2stand_threshold = sit2stand_threshold, standing_threshold = standing_threshold, stand2sit_threshold = stand2sit_threshold, emergency_threshold = emergency_threshold)
    device.main_loop()