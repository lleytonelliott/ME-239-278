# Pseudocode

import time
import numpy as np

# Loop Variables

kp = 0
kd = 0
Kt = 0

# Set Up State Machine

class AssistiveDevice:
    def __init__(self, kp, kd, Kt):

        self.state_machine = PhaseStateMachine
        self.torque_controller = TorqueController(kp, kd, Kt)

        self.state_vector = np.array([0, 0, 0, 0, 0, 0])
        self.sensor_time_prev = time.monotonic_ns()

        self.phase_handlers = {
            "null": self.null_state,
            "sitting": self.sitting,
            "sit to stand": self.sit2stand,
            "standing": self.standing,
            "stand to sit": self.stand2sit,
        }

        self.update_sensors()
        self.state_machine.transition("startup", self.state_vector)

    def update_sensors(self)

class PhaseStateMachine:
    def __init__(self):
        self.current_phase = "null"

        self.phase_handlers = {
            "null": self.null_state,
            "sitting": self.sitting,
            "sit to stand": self.sit2stand,
            "standing": self.standing,
            "stand to sit": self.stand2sit,
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
            return "null"
        return "null"

# Low-Level Torque Controller

class TorqueController:
    def __init__(self, kp, kd, Kt):
        self.kp = kp
        self.kd = kd
        self.Kt = Kt
        self.tau_prev = None
        self.time_prev = time.monotonic_ns()
    
    def generate_PD(self, tau_desired):
        motor_current = read_motor_current()
        tau_actual = motor_current*self.Kt
        if self.tau_prev == None:
            self.tau_prev = tau_actual
        error = tau_desired - tau_actual

        time_current = time.monotonic_ns()
        delta_t = (time_current - self.time_prev)/1000000000.0
        self.time_prev = time_current

        if delta_t == 0:
            d_tau = 0
            self.tau_prev = tau_actual
        else:
            d_tau = (tau_actual - self.tau_prev)/delta_t
            self.tau_prev = tau_actual
        
        return self.kp*error - self.kd*d_tau

# Functions

def setup(kp, kd, Kt):

    state_machine = PhaseStateMachine()
    torque_controller = TorqueController(kp, kd, Kt)

    state_vector = np.array([0, 0, 0, None, None, None])

    state_vector = update_sensors(state_vector, None)
    state_machine.transition("startup", state_vector = state_vector)

    return state_machine, torque_controller, state_vector

def main(kp, kd, Kt):

    state_machine, torque_controller, state_vector = setup(kp, kd, Kt)

    # run the loop
    # while True:
    # include fail safe - "emergency" event that turns off active control, and await button press to restart program

# def calculate_desired_torque(state_vector, phase):
#   Calculate ideal torque based on state vector and current phase of motion

def update_sensors(state_vector, delta_t_sensors):
    z_ori_prev = state_vector[0]
    theta_knee_prev = state_vector[1]
    F_seat_prev = state_vector[2]
    
    state_vector[0] = # Set orientation about z based on sensors
    state_vector[1] = # Set theta position based on sensors
    state_vector[2] = # Set seat force based on sensors

    if z_ori_prev == theta_knee_prev == F_seat_prev == delta_t_sensors == None:
        state_vector[3:6] = 0
    else:
        state_vector[3] = (state_vector[0] - z_ori_prev)/delta_t_sensors
        state_vector[4] = (state_vector[1] - theta_knee_prev)/delta_t_sensors
        state_vector[5] = (state_vector[2] - F_seat_prev)/delta_t_sensors

    return state_vector

#   Update IMU (probably separate function for integration)
#   Find encoder position and calculate knee angle
#   Find force vector from seat

# def read_motor_current:

# def drive_motor(PD_signal)
#   Some code to drive the motor

if __name__ == "__main__":
    main()