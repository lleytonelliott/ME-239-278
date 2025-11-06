# Dynamic Model

from scipy.spatial import KDTree
import numpy as np

model_states = np.array([

])

standup_torques = np.array([

])

sitdown_torques = np.array([

])

mmdata = (model_states, standup_torques, sitdown_torques)

class DynamicModel:
    def __init__(self, mmarray):
        self.state_trajectory = mmarray[0]
        self.standup_trajectory = mmarray[1]
        self.sitdown_trajectory = mmarray[2]

        self.state_tree = KDTree(self.state_trajectory)

    def sit2stand(self, state_vector):
        distance, index = self.state_tree.query(state_vector, k=1)
        tau_desired = self.standup_trajectory[index]
        return tau_desired

    def stand2sit(self, state_vector):
        distance, index = self.state_tree.query(state_vector, k=1)
        tau_desired = self.sitdown_trajectory[index]
        return tau_desired