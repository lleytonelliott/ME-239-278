# Pseudocode

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

# class StateMachine:
#     def __init__(self):
#         self.phases = {
#             "null": self.null_state,
#             "sitting": self.sitting,
#             "sit to stand": self.sit2stand,
#             "standing": self.standing,
#             "stand to sit": self.stand2sit,
#         }
#         self.current_phase = "null"
    
#     def determine_state(self, state_vector, F_seat):
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

# def drive_motor(tau_desired):
#   tau_current = calculate_motor_torque