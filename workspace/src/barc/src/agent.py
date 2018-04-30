#!usr/bin/env python

'''
    File name: agent.py
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Python Version: 2.7.12
'''

import numpy as np
import pdb

from transformations import s_to_xy, xy_to_s


class Agent:
    """docstring for Agent"""

    dt = 0.1  # TODO: Define globally

    l_front = 0.125
    l_rear = 0.125
    width = 0.1

    mass = 1.98
    mu = 0.85
    g = 9.81  # TODO: Maybe as general constant
    I_z = 0.03

    # Pajceka parameters
    B = 6.0
    C = 1.6

    # TODO: Might be unncessary
    max_slip_angle = 10.0

    input_lower_bound = np.array([- 0.6, - np.pi / 3])
    input_upper_bound = np.array([0.6, np.pi / 3])
    s_lower_bound = np.array([- np.inf, - np.inf, - np.inf, - np.inf, 0., 0.])
    s_upper_bound = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

    is_dynamic = True

    current_lap = 0
    current_iteration = 0
    current_input = np.zeros(2)
    final_input = np.zeros(2)
    final_s = np.zeros(6)
    final_xy = np.zeros(6)

    def __init__(self, index, color, maximum_num_iterations, horizon, num_laps,
                 num_loaded_laps, num_considered_states, v_max):
        self.index = index
        self.color = color
        self.horizon = horizon
        self.maximum_num_iterations = maximum_num_iterations

        self.v_max = v_max

        # x-y state: z_xy = [x y v_x v_y psi psi_dot]
        self.states_xy = np.zeros((maximum_num_iterations, 6))
        # s-e_y state: z_s = [s e_y e_psi psi_dot v_x v_y]
        self.states_s = np.zeros((maximum_num_iterations, 6))
        # input: u = [acceleration steering_angle]
        self.inputs = np.zeros((maximum_num_iterations, 2))
        # s-e_y true state: z_s = [s e_y e_psi psi_dot v_x v_y]
        # self.states_true_s = np.zeros(maximum_num_iterations, 6)

        self.optimal_inputs = np.zeros((self.horizon, 2))
        self.predicted_xy = np.zeros((self.horizon + 1, 6))
        self.predicted_s = np.zeros((self.horizon + 1, 6))
        # TODO: Reason for prev predicted_s?
        self.prev_predicted_s = np.zeros((self.horizon + 1, 6))

        total_num_laps = num_laps + num_loaded_laps
        self.iterations_needed = np.zeros(total_num_laps)
        self.trajectories_s = np.zeros((total_num_laps,
                                        maximum_num_iterations, 6))
        self.trajectories_xy = np.zeros((total_num_laps,
                                         maximum_num_iterations, 6))
        self.trajectories_cost = np.zeros((total_num_laps,
                                           maximum_num_iterations))

        self.selected_states_s = np.zeros((num_considered_states, 6))
        self.selected_states_xy = np.zeros((num_considered_states, 6))
        self.selected_states_cost = np.zeros(num_considered_states)

    def get_state_xy(self, iteration):
        assert iteration <= self.maximum_num_iterations
        return self.states_xy[iteration, :]

    def get_state_s(self, iteration):
        assert iteration <= self.maximum_num_iterations
        return self.states_s[iteration, :]

    def get_current_s(self):
        return self.get_state_s(self.current_iteration)

    def get_current_xy(self):
        return self.get_state_xy(self.current_iteration)

    def get_current_input(self):
        return self.current_input

    def set_initial_state_xy(self, xy_state, track):
        # Only sets the first state of the first lap
        assert self.current_lap == 0 and self.current_iteration == 0
        self.states_xy[0, :] = xy_state
        self.states_s[0, :] = xy_to_s(xy_state, track)

    def set_initial_state_s(self, s_state, track):
        # Only sets the first state of the first lap
        assert self.current_lap == 0 and self.current_iteration == 0
        self.states_s[0, :] = s_state
        self.states_xy[0, :] = s_to_xy(s_state, track)

    def get_slip_angle(self, steering_angle):
        beta = np.arctan(self.l_rear / (self.l_front + self.l_rear) *
               np.tan(steering_angle))

        return beta

    def select_states(self):
        return 0

    # TODO: Add another attribute which also saves the cost for each state
    def states_cost(self):
        return 0

    def save_trajectories(self, filename):
        return 0

    def load_trajectories(self, filename):
        return 0


if __name__ == "__main__":
    from track import Track

    # track = Track(1. / 10, "test", 0.8)
    track = Track.from_json("track.json")
    agent = Agent(index=0, color="blue", maximum_num_iterations=500, horizon=10,
                  num_laps=10, num_loaded_laps=0, num_considered_states=100,
                  v_max=5.0)

    state = np.array([8.05, 0.2, 0.0, 0.0, 0.0, 0.0])
    agent.set_initial_state_s(state, track)

    print agent.states_s[0, :]
    print agent.states_xy[0, :]

    import json
    from copy import copy

    def json_default(object):
        return object.__dict__

    track_dict = copy(track.__dict__)
    print track_dict.keys()

    for key in track_dict.keys():
        if key not in ["shape", "width", "ds"]:
            del track_dict[key]

    with open("track.json", "w") as f:
        json.dump(track_dict, f, indent=2)

    pdb.set_trace()

    json_track = json.dumps(track_dict, indent=2)

    pdb.set_trace()
