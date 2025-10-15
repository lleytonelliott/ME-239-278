# Pseudocode

import time

# Loop Variables

kp = 0
kd = 0
Kt = 0

# Set Up State Machine

# class PhaseStateMachine:
#     def __init__(self):
#         self.phases = {
#             "null": self.null_state,
#             "sitting": self.sitting,
#             "sit to stand": self.sit2stand,
#             "standing": self.standing,
#             "stand to sit": self.stand2sit,
#         }
#         self.current_phase = "null"
    
#     def determine_phase(self, state_vector, F_seat):
#         if self.current_phase == "null":
#             # determine whether sitting or standing based on state vector
#         else:
#             return self.current_phase

# Low-Level Torque Controller

class TorqueController:
    def __init__(self, kp, kd, Kt):
        self.kp = kp
        self.kd = kd
        self.Kt = Kt
        self.error_prev = 0
        self.time_prev = time.monotonic_ns()
    
    def generate_PD(self, tau_desired):
        motor_current = read_motor_current()
        tau_actual = motor_current*self.Kt
        error = tau_desired - tau_actual

        time_current = time.monotonic_ns()
        delta_t = (time_current - self.time_prev)/1000000000
        self.time_prev = time_current

        d_error = (error - self.error_prev)/delta_t
        self.error_prev = error
        
        return self.kp*error + self.kd*d_error

# Functions

def setup(kp, kd, Kt):

    state_machine = PhaseStateMachine()
    torque_controller = TorqueController(kp, kd, Kt)
    state_vector = [0, 0, 0, 0, 0, 0]

    return state_machine, torque_controller, state_vector

def main(kp, kd, Kt):

    state_machine, torque_controller, state_vector = setup(kp, kd, Kt)

    # run the loop
    # while True:

# def calculate_desired_torque(state_vector, phase):
#   Calculate ideal torque based on state vector and current phase of motion

# def update_sensors(x_com_prev, y_com_prev, theta_knee_prev):
#   Update IMU (probably separate function for integration)
#   Find encoder position and calculate knee angle
#   Find force vector from seat

# def read_motor_current:

# def calculate_motor_torque:

# def drive_motor(PD_signal)
#   Some code to drive the motor

if __name__ == "__main__":
    main()