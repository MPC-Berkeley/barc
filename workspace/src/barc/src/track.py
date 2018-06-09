#!usr/bin/env python

'''
    File name: track.py
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Python Version: 2.7.12
'''

import numpy as np
from numpy import linalg as LA
import os
import json
import pdb
import matplotlib.pyplot as plt


class Track:
    """docstring for Track"""

    total_length = 0.
    s_coords = np.zeros(1)

    thetas = [0.0]
    curvature = []
    xy_coords = np.zeros(2)
    xy_outer = np.zeros(2)
    xy_inner = np.zeros(2)

    # tracks_dir = os.listdir(os.getcwd() + "/../tracks")
    tracks_dir = "/home/lukas/barc/workspace/src/barc/tracks"

    def __init__(self, ds, shape, width):
        self.ds = ds
        assert self.ds > 0.
        self.shape = shape
        self.width = width

        if self.shape + ".npy" in self.tracks_dir:
            print "Loading track " + self.shape
        # self.load_track()
        else:
            print "Creating track " + self.shape
            self.create_track()
        # self.save_track()

        print "Total length of track: " + str(self.total_length)

    @classmethod
    def from_json(cls, filename):
        with open(filename) as f:
            track_dict = json.load(f)

        ds = track_dict["ds"]
        shape = track_dict["shape"]
        width = track_dict["width"]
        track = cls(ds, shape, width)

        return track

    def create_track(self):
        if self.shape == "oval":
            self.oval_track()
        elif self.shape == "test":
            self.test_track()
        elif self.shape == "l_shape":
            self.l_shape()
        else:
            raise ValueError("Track %s is not available" % self.shape)

        num_points = len(self.thetas)

        # Transform lists to numpy arrays
        self.thetas = np.asarray(self.thetas)
        self.curvature = np.asarray(self.curvature)

        self.xy_coords = np.zeros((num_points, 2))
        self.xy_outer = np.zeros((num_points, 2))
        self.xy_inner = np.zeros((num_points, 2))
        self.xy_outer[0, 1] = self.width / 2
        self.xy_inner[0, 1] = - self.width / 2

        self.s_coords = self.ds * np.ones(num_points)
        self.s_coords[0] = 0.0
        self.s_coords = np.cumsum(self.s_coords)

        sign_x = 1.0
        sign_y = 1.0

        for i in range(1, num_points):
            theta = self.thetas[i]
            delta_theta = theta - self.thetas[i - 1]
        
            if abs(delta_theta) > 1e-5 and abs(self.get_curvature_check(self.s_coords[i])) > 1e-5:
                radius = abs(1. / self.get_curvature_check(self.s_coords[i]))

                chord = 2 * radius * np.sin(abs(delta_theta) / 2)

                initial_point = self.xy_coords[0, :]
                s_direction = np.array([initial_point[0] + chord, initial_point[1]])
                s_rotated = rotate_around_z(s_direction, self.thetas[i - 1] + delta_theta / 2)
                self.xy_coords[i, :] = s_rotated + self.xy_coords[i - 1, :]
            else:
                self.xy_coords[i, :] = self.xy_coords[i - 1, :] + self.ds * np.array([np.cos(theta), np.sin(theta)])

            self.xy_outer[i, :] = self.width / 2 * np.array([np.cos(theta + np.pi / 2),
                                                             np.sin(theta + np.pi / 2)])
            self.xy_inner[i, :] = self.width / 2 * np.array([np.cos(theta - np.pi / 2),
                                                             np.sin(theta - np.pi / 2)])

        self.xy_outer += self.xy_coords
        self.xy_inner += self.xy_coords

        # assert LA.norm(self.xy_coords[0, :] - self.xy_coords[- 1, :]) < 1e-3

        if sum(abs(self.xy_coords[0, :] - self.xy_coords[- 1, :])) > 1e-2:
            print(self.xy_coords[0, :])
            print(self.xy_coords[- 1, :])
            print("Track starting and end point don't align.")
            # exit()

    def oval_track(self):
        self.add_segment(1.0, 0.0)
        self.add_segment(4.5, np.pi)
        self.add_segment(2.0, 0.0)
        self.add_segment(4.5, np.pi)
        self.add_segment(1.0, 0.0)

        """
        self.add_segment(1.0, 0.0)
        self.add_segment(4.5, - np.pi)
        self.add_segment(2.0, 0.0)
        self.add_segment(4.5, - np.pi)
        self.add_segment(1.0, 0.0)
        """

        """
        self.add_segment(2.0, 0.0)
        self.add_segment(9.0, - np.pi)
        self.add_segment(4.0, 0.0)
        self.add_segment(9.0, - np.pi)
        self.add_segment(2.0, 0.0)
        """
        """
        self.add_segment(6.0, 0.0)
        self.add_segment(14.0, - np.pi)
        self.add_segment(12.0, 0.0)
        self.add_segment(14.0, - np.pi)
        self.add_segment(6.0, 0.0)
        """

    def l_shape(self):
        self.add_segment(1.5, 0.0)
        self.add_segment(2.2, np.pi / 2)
        self.add_segment(0.1, 0.0)
        self.add_segment(2.2, np.pi / 2)
        self.add_segment(0.1, 0.0)
        self.add_segment(2.2, - np.pi / 2)
        self.add_segment(0.1, 0.0)
        self.add_segment(2.2, np.pi / 2)
        self.add_segment(0.1, 0.0)
        self.add_segment(2.2, np.pi / 2)
        self.add_segment(3.0, 0.0)
        self.add_segment(2.2, np.pi / 2)
        self.add_segment(1.5, 0.0)

    def test_track(self):
        denominator = 3.5
        small = False

        if small:
            self.add_segment(1.7, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(0.5, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.2, np.pi / (2 * denominator))
            self.add_segment(1.2, - np.pi / denominator)
            self.add_segment(1.2, np.pi / (2 * denominator))
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(0.5, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.8, 0.0)
        else:
            self.add_segment(1.5, 0.0)
            self.add_segment(2.0, np.pi / 2)
            self.add_segment(1.5, 0.0)
            self.add_segment(2.0, np.pi / 2)
            self.add_segment(3.0, 0.0)
            self.add_segment(2.0, np.pi / 2)
            self.add_segment(1.5, 0.0)
            self.add_segment(2.0, np.pi / 2)
            self.add_segment(1.5, 0.0)

            """
            self.add_segment(2.5, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.0, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.5, np.pi / (2 * denominator))
            self.add_segment(1.5, - np.pi / denominator)
            self.add_segment(1.5, np.pi / (2 * denominator))
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.0, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(1.9, 0.0)
            """

            """
            self.add_segment(3.0, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(2.0, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(2.0, np.pi / (2 * denominator))
            self.add_segment(2.0, - np.pi / denominator)
            self.add_segment(2.0, np.pi / (2 * denominator))
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(2.0, 0.0)
            self.add_segment(2.0, - np.pi / 2)
            self.add_segment(2.8, 0.0)
            """

    def add_segment(self, length, angle):
        num_pieces = int(round(length / self.ds))
        # radius = length / angle
        curvature = angle / length  # = 1 / radius

        angle_per_segment = angle / (num_pieces)
        thetas_segment = angle_per_segment * np.ones(num_pieces)
        thetas_segment[0] += self.thetas[- 1]
        self.thetas += (np.cumsum(thetas_segment)).tolist()

        # self.curvature += (curvature * np.ones(num_pieces)).tolist()

        start_interval = self.total_length
        end_interval = start_interval + length
        kappa = [(start_interval, end_interval), curvature]
        self.curvature.append(kappa)
        self.total_length += length

    def get_s(self, s):
        if isinstance(s, np.float64):
            s = abs(s) % self.total_length
            return s
        elif s.shape[0] == 6:
            s[0] = abs(s[0]) % self.total_length
            return s
        else:
            ValueError("The s-coordinate is not matching any of the "
                       "predefined types.")

    def get_curvature(self, s):
        s = self.get_s(s)

        for i in range(len(self.curvature)):
            interval = self.curvature[i][0]
            start_interval = interval[0]
            end_interval = interval[1]

            if s >= start_interval and s < end_interval:
                curvature = self.curvature[i][1]
                return curvature
            if s <= end_interval and abs(self.curvature[i][1]) > 1e-5:
                curvature = self.curvature[i][1]
                return curvature
            """
            elif s == 0.0:
                curvature = 0.0
                return curvature
            """
    def get_curvature_check(self, s):
        delta = 1e-5
        s_plus = s + delta
        s_minus = s - delta

        curvature_plus = self.get_curvature(s_plus)
        curvature_minus = self.get_curvature(s_minus)

        if abs(curvature_plus) > abs(curvature_minus):
            return curvature_plus
        else:
            return curvature_minus

    def get_theta(self, s):
        return 0

    def save_track(self):
        return 0

    def load_track(self):
        return 0


def rotate_around_z(xy_coords, angle):
    rotation_matrix = np.array([[np.cos(angle), - np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])

    return np.dot(rotation_matrix, xy_coords)


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    track = Track(1. / 10, "oval", 0.8)
    # track = Track.from_json("track.json")

    # plt.plot(track.xy_coords[:, 0], track.xy_coords[:, 1], "ro")
    # plt.plot(track.xy_inner[:, 0], track.xy_inner[:, 1], "bo")
    # plt.plot(track.xy_outer[:, 0], track.xy_outer[:, 1], "go")
    # plt.axis("equal")
    # plt.show()

    plt.plot(track.curvature, "ro")
    # plt.axis("equal")
    plt.show()

    pdb.set_trace()
