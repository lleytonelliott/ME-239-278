# Main Code

import time
import numpy as np

from assistive_device import AssistiveDevice
from dynamic_model import mmdata

# Motor Control Variables

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

if __name__ == "__main__":
    device = AssistiveDevice(
        kp=kp, kd=kd, Kt=Kt,
        sitting_threshold=sitting_threshold,
        sit2stand_threshold=sit2stand_threshold,
        standing_threshold=standing_threshold,
        stand2sit_threshold=stand2sit_threshold,
        emergency_threshold=emergency_threshold
        mmarray=mmdata
    )

    device.main_loop()