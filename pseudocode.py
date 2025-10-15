# Pseudocode

import time

# State Variables

x_com = None
y_com = None
theta_knee = None
xdot_com = None
ydot_com = None
thetadot_knee = None
F_seat = None
state_vector = [x_com, y_com, theta_knee, xdot_com, ydot_com, thetadot_knee]

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

# Functions

# def main(state_vector, phase):
#   Calculate ideal torque based on state vector and current phase of motion

# def update_sensors(x_com_prev, y_com_prev, theta_knee_prev):
#   Update IMU (probably separate function for integration)
#   Find encoder position and calculate knee angle
#   Find force vector from seat

# def calculate_motor_torque:

# def generate_PD(tau_desired, tau_prev, time_prev):
#   time_current = time.monotonic_ns()
#   tau_current = calculate_motor_torque
#   error = tau_desired - tau_current
#   delta_us = (time_current - time_prev)/1000000000
#   d_error = (tau_current - tau_prev)/delta_us
#   PD_signal = kp*error + kd*d_error
#   return PD_signal

# def drive_motor(PD_signal)
#   Some code to drive the motor