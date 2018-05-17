#!usr/bin/env python

'''
    File name: transformations.py
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Python Version: 2.7.12
'''

import numpy as np
from numpy import linalg as LA

from track import Track


def s_to_xy(s_coord, track):
    last_index = False
    s, e_y, e_psi, psi_dot, v_x, v_y = s_coord

    s = track.get_s(s)

    distances_to_track = np.array([LA.norm(track.s_coords[i] - s)
                                   for i in range(len(track.s_coords))])
    two_closest_indices = np.argsort(distances_to_track)[: 2]

    # if 0 in two_closest_indices:
    #    if s <= track.ds:
    #        xy1_index = 0
    #    else:
    #        xy1_index = track.s_coords.shape[0] - 2
    # if s > track.s_coords[- 1]:
    #    last_index = True
    #    xy1_index = track.s_coords.shape[0] - 1
    #else:
    xy1_index = np.min(two_closest_indices)
    s1_xy = track.xy_coords[xy1_index, :]

    # plt.plot(s1_xy[0], s1_xy[1], "r*")
    # plt.plot(track.xy_coords[xy1_index + 1, 0], track.xy_coords[xy1_index + 1, 1], "b*")

    # if last_index:
    #    theta = track.thetas[xy1_index]
    #    delta_theta = 0.0
    #else:
    theta = track.thetas[xy1_index]
    abs_delta_theta = abs(track.thetas[xy1_index + 1] - theta)
    delta_s = s - track.s_coords[xy1_index]    

    if abs(abs_delta_theta) > 1e-5 and abs(track.get_curvature(track.s_coords[xy1_index + 1])) > 1e-5:
        if abs(track.get_curvature(track.s_coords[xy1_index])) < 1e-5 and \
                abs(track.get_curvature(track.s_coords[xy1_index + 1])) > 1e-5:
            curvature = track.get_curvature(track.s_coords[xy1_index + 1])
        elif abs(track.get_curvature(track.s_coords[xy1_index])) > 1e-5 and \
                abs(track.get_curvature(track.s_coords[xy1_index + 1])) > 1e-5 and \
                abs(track.get_curvature(track.s_coords[xy1_index]) - track.get_curvature(track.s_coords[xy1_index + 1])) < 1e-5:
            curvature = track.get_curvature(track.s_coords[xy1_index + 1])
        else:
            curvature = track.get_curvature(track.s_coords[xy1_index + 1])

        if curvature < 0.0:
            delta_theta = - abs_delta_theta
        else:
            delta_theta = abs_delta_theta

        delta_theta_prime = delta_theta / track.ds * delta_s

        r = 1. / curvature
        chord = 2.0 * r * np.sin(delta_theta_prime / 2.)
    else:
        chord = delta_s
        delta_theta = abs_delta_theta
        delta_theta_prime = delta_theta / track.ds * delta_s

    theta_prime = theta + delta_theta_prime

    delta_xy = chord * np.array([np.cos(theta_prime), np.sin(theta_prime)])

    initial_point = track.xy_coords[0, :]
    e_y_direction = np.array([initial_point[0], initial_point[1] - e_y])
    e_y_rotated = rotate_around_z(e_y_direction, theta_prime)

    xy_from_origin = s1_xy + delta_xy
    e_y_prime = e_y_rotated + xy_from_origin
    x = e_y_prime[0]
    y = e_y_prime[1]

    # psi = e_psi + theta + delta_theta_prime
    psi = e_psi + theta_prime

    # plt.plot([x, track.xy_coords[xy1_index, 0]], [y, track.xy_coords[xy1_index, 1]])
    # plt.plot([x, track.xy_coords[xy1_index + 1, 0]], [y, track.xy_coords[xy1_index + 1, 1]])

    return np.array([x, y, v_x, v_y, psi, psi_dot])


def xy_to_s(xy_coord, track):
    x, y, v_x, v_y, psi, psi_dot = xy_coord

    distances_to_track = np.array([LA.norm(track.xy_coords[i, :] - xy_coord[: 2])
                                   for i in range(len(track.s_coords))])
    two_closest_indices = np.argsort(distances_to_track)[: 2]

    if 0 in two_closest_indices:
        if x >= 0.:
            s1_index = 0
        else:
            s1_index = track.xy_coords.shape[0] - 2
    else:
        s1_index = np.min(two_closest_indices)

    s1_s = track.s_coords[s1_index]
    theta = track.thetas[s1_index]
    delta_theta = track.thetas[s1_index + 1] - theta

    dist_to_s1 = distances_to_track[s1_index]
    dist_to_s2 = distances_to_track[s1_index + 1]

    if abs(delta_theta) > 1e-5:
        if abs(track.get_curvature(track.s_coords[s1_index + 1]) - track.get_curvature(track.s_coords[s1_index])) > 1e-5:
            if abs(track.get_curvature(track.s_coords[s1_index + 1])) > 1e-5:
                curvature = track.get_curvature(track.s_coords[s1_index + 1])
            else:
                curvature = track.get_curvature(track.s_coords[s1_index])
        else:
            curvature = track.get_curvature(track.s_coords[s1_index])

        radius = 1. / curvature
        radius_abs = abs(radius)

        # Find the center of the circle segment
        initial_point = track.xy_coords[0, :]
        e_y_direction = np.array([initial_point[0], initial_point[1] + radius])
        e_y_rotated = rotate_around_z(e_y_direction, theta)
        center_of_segment = e_y_rotated + track.xy_coords[s1_index, :]

        # Applying the law of cosines
        dist_to_center_segment = LA.norm(center_of_segment - xy_coord[: 2])

        nominator = radius_abs ** 2 + dist_to_center_segment ** 2 - dist_to_s1 ** 2
        denominator = 2 * radius_abs * dist_to_center_segment
        if abs(nominator - denominator) < 1e-5:
            abs_phi = 0.0
        else:
            abs_phi = np.arccos(nominator / denominator)

        # print("curvature: ", curvature)    

        if curvature < 0.0:
            phi = - abs_phi
        else:
            phi = abs_phi

        delta_s = radius_abs * abs_phi
        if curvature < 0.0:
            e_y = radius_abs - dist_to_center_segment
        else:
            e_y = - (radius_abs - dist_to_center_segment)

        e_psi = psi - (theta + phi)

        theta = (theta + phi)
    else:
        if dist_to_s1 < 1e-5:
            delta_s = 0.0
            abs_e_y = 0.0
            phi = 0.0
        elif dist_to_s1 + dist_to_s2 <= track.ds + 1e-5: # added tolerance
            delta_s = dist_to_s1
            abs_e_y = 0.0
            phi = 0.0
        else:
            # print("Here")
            abs_phi = np.arccos((dist_to_s1 ** 2 + track.ds ** 2 - dist_to_s2 ** 2) /
                                (2 * dist_to_s1 * track.ds))
            if track.get_curvature(track.s_coords[s1_index + 1]) < 0.0:
                phi = abs_phi
            else:
                phi = abs_phi 
                
            delta_s = dist_to_s1 * np.cos(phi)
            abs_e_y = dist_to_s1 * np.sin(phi)

        curvature = track.get_curvature(track.s_coords[s1_index + 1])  # track.curvature[s1_index + 1]
        theta = track.thetas[s1_index]

        e_psi = psi - theta

        initial_point = track.xy_coords[0, :]
        # print("initial_point: ", initial_point)
        e_y_direction = np.array([initial_point[0],
                                  initial_point[1] - track.width / 2])  # to inner bound
        # print("Ey direction: ", e_y_direction)
        e_y_rotated = rotate_around_z(e_y_direction, theta)
        # print("Ey rotated: ", e_y_rotated)
        # xy_position = track.xy_coords[s1_index, :] + delta_s * \
        #              np.array([np.cos(theta), np.sin(theta)])
        xy_position = track.xy_coords[s1_index, :] + delta_s * np.array([np.cos(theta), np.sin(theta)])  # np.array([delta_s, 0.0])
        center_of_segment = e_y_rotated + xy_position
        # print("center of segment: ", center_of_segment)
        distance_to_inner_bound = LA.norm(center_of_segment - np.array([x, y])) # track.xy_coords[: 2])
        # print("disance to inner bound: ", distance_to_inner_bound)
        if distance_to_inner_bound <= track.width / 2:
            e_y = abs_e_y
        else:
            e_y = - abs_e_y

    s = s1_s + delta_s

    # print("S: ", s)
    

    return np.array([s, - e_y, e_psi, psi_dot, v_x, v_y])


def rotate_around_z(xy_coords, angle):
    rotation_matrix = np.array([[np.cos(angle), - np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])

    return np.dot(rotation_matrix, xy_coords)


if __name__ == "__main__":
    import pdb
    import matplotlib.pyplot as plt

    track = Track(0.1, "test", 1.0)
    # track = Track(0.1, "oval", 1.0)

    fig = plt.figure("Race")
    # ax = fig.add_subplot(1, 1, 1)
    ax = fig.add_subplot(2, 1, 1)
    ax_2 = fig.add_subplot(2, 1, 2)

    # fig_2 = plt.figure("Angles")
    # axis = fig_2.add_subplot(1, 1, 1)

    """
    test = np.array([-0.02592649572766198,-0.002358129563365882,
                      0.5311734600406155,0.00415122594607335,-6.262578301959858,
                      -0.03384403792516429])
    test_2 = np.array([0.23317530843876547,-0.0046964218147232855,0.5307739418952622,
                       -0.0009789572258890383,-6.26154372355634,-0.021930483057318338])
    test_3 = np.array([0.23317530843876547,-0.0046964218147232855,0.5307739418952622,
                       -0.0009789572258890383,2 * np.pi -6.26154372355634,-0.021930483057318338])

    print(xy_to_s(test, track))
    print(xy_to_s(test_2, track))
    print(xy_to_s(test_3, track))
    pdb.set_trace()

    ax.plot(track.xy_coords[:, 0], track.xy_coords[:, 1], "k--")
    ax.plot(track.xy_inner[:, 0], track.xy_inner[:, 1], "k-")
    ax.plot(track.xy_outer[:, 0], track.xy_outer[:, 1], "k-")

    test = np.array([ 3.0114, - 0.4, 0., 0., 0., 0.])
    """

    num_points = 500
    # delta_s = 0.02  # track.total_length / num_points
    delta_s = track.total_length / num_points

    # run_s = 3.0592
    # run_s = 23.8044
    # run_s = 5.49
    run_s = 1.9

    # plt.plot(track.xy_coords[89:99, 0], track.xy_coords[89:99, 1], "ko")
    ss_state = np.zeros((num_points, 6))
    curvature_log = np.zeros((num_points))
    theta_log = np.zeros((num_points))
    center_of_segment_log = np.zeros((num_points, 2))

    for i in range(num_points):
        print("i: ", i, run_s)
        state = np.array([run_s, 0.4, 0.0, 0.0, 0.0, 0.0])
        # state = np.array([run_s, - 0.02, 0.0, 0.0, 0.0, 0.0])
        print("S: ", state)
        xy_state = s_to_xy(state, track)
        print("XY: ", xy_state)
        print("S: ", ss_state[i, :])
        xxyy_state = s_to_xy(ss_state[i, :], track)
        print("XY: ", xxyy_state)
        ax.plot(xy_state[0], xy_state[1], "ro")
        ax.plot(xxyy_state[0], xxyy_state[1], "bo")
        ax_2.plot(state[0], 0.0, "ro")

        run_s += delta_s

    ax.plot(center_of_segment_log[:, 0], center_of_segment_log[:, 1], "b*")

    r = 4.5 / np.pi
    alpha = np.arange(0.0, 2 * np.pi, 0.01)
    ax.plot(r * np.cos(alpha) + 2, r * np.sin(alpha) - r, "g-")
    ax.plot(r * np.cos(alpha) - 2, r * np.sin(alpha) - r, "g-")

    ax.plot(track.xy_coords[:, 0], track.xy_coords[:, 1], "ko")

    """
    
    ax.plot(r * np.cos(alpha) + 1, r * np.sin(alpha) - r, "g-")
    ax.plot(r * np.cos(alpha) - 1, r * np.sin(alpha) - r, "g-")
    ax.plot(track.xy_coords[:, 0], track.xy_coords[:, 1], "ko")
    ax.plot(track.xy_coords[12, 0], track.xy_coords[12, 1], "bo")
    ax.plot(track.xy_coords[13, 0], track.xy_coords[13, 1], "co")

    print("Theta 11: ", track.thetas[11])
    print("Theta 12: ", track.thetas[12])
    print("Theta 13: ", track.thetas[13])
    print("Theta 14: ", track.thetas[14])

    print("xy 11: ", track.xy_coords[11, :])
    print("xy 12: ", track.xy_coords[12, :])
    print("xy 13: ", track.xy_coords[13, :])
    print("xy 14: ", track.xy_coords[14, :])
    """

    #for i in range(len(track.xy_coords[:, 1])):
    #    print("s: ", track.s_coords[i], " theta: ", track.thetas[i])

    # axis.plot(ss_state[:, 0], curvature_log, "b-")
    # axis.plot(ss_state[:, 0], theta_log, "r-")

    ax_2.plot(ss_state[:, 0], ss_state[:, 1], "bo")

    s = track.s_coords


    # s = 9.05

    # state = np.array([s, - 0.4, 0.0, 0.0, 0.0, 0.0])
    # xy_state = s_to_xy(state, track)
    # print(xy_state)

    # ax.plot(xy_state[0], xy_state[1], "bo")

    # s_index = np.int(np.floor(s / track.ds))
    # ax.plot(track.xy_coords[s_index, 0], track.xy_coords[s_index, 1], "bo")
    # ax.plot(track.xy_coords[s_index + 1, 0], track.xy_coords[s_index + 1, 1], "bo")
    
    plt.axis("equal")
    plt.show()

    # pdb.set_trace()
