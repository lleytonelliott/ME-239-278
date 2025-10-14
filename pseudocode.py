# Pseudocode

# State Variables

x_com = None
y_com = None
theta_knee = None
xdot_com = None
ydot_com = None
thetadot_knee = None
F_seat = None
state_vector = [x_com, y_com, theta_knee]

# Set Up State Machine

# class StateMachine:
#     def __init__(self):
#         self.states = {
#             "null": self.null_state,
#             "sitting": self.sitting,
#             "sit to stand": self.sit2stand,
#             "standing": self.standing,
#             "stand to sit": self.stand2sit,
#         }
#         self.current_state = "null"
    
#     def determine_state(self, state_vector, F_seat):
#         if self.current_state == "null":
#             # determine whether sitting or standing based on state vector
#         else:
#             return self.current_state

# Functions

def main(state_vector):