#!/usr/bin/env python

'''
    File name: plotter.py
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Python Version: 2.7.12
'''

import rospy
from barc.msg import xy_prediction
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.transforms as transforms
import matplotlib.animation as animation
import matplotlib as mpl
import matplotlib.collections as collections
import pdb

from track import Track
from transformations import rotate_around_z


def set_transformation(axis, angle, translate):
    # Try to rotate rectangle using matplotlib's transformations
    t1 = mpl.transforms.Affine2D()
    t1.rotate(angle)
    t1.translate(translate[0], translate[1])

    # Apparently one also has to transform between data coordinate
    # system and display coordinate system
    t2 = axis.transData
    t3 = t1.__add__(t2)

    return t3


class PlottedAgent:
    """docstring for PlottedAgent"""
    wheel_length = 0.04
    wheel_width = 0.02
    bumper = 0.01
    l_front = 0.125
    l_rear = 0.125
    width = 0.1

    center_of_mass = np.array([0.0, 0.0])
    psi = np.array(0.0)
    psi_and_steering = np.array([0.0])
    front = np.array([0.0, 0.0])
    rear = np.array([0.0, 0.0])
    center_front_wheel = np.array([[0.0, 0.0], [0.0, 0.0]])
    center_rear_wheel = np.array([[0.0, 0.0], [0.0, 0.0]])
    center_front_wheel_left = np.array([0.0, 0.0])
    center_front_wheel_right = np.array([0.0, 0.0])
    center_rear_wheel_left = np.array([0.0, 0.0])
    center_rear_wheel_right = np.array([0.0, 0.0])

    def __init__(self, color, ax):
        xy_sub = rospy.Subscriber("xy_prediction", xy_prediction, self.xy_callback)

        self.color = color
        if color == "blue":
            self.dot = "bo"
            self.star = "b*"
            self.line = "b-"
            self.dot_extra = "co"
            self.line_extra = "c-"
        elif color == "red":
            self.dot = "ro"
            self.star = "r*"
            self.line = "r-"
            self.dot_extra = "mo"
            self.line_extra = "m-"
        else:
            ValueError("Color %s not defined" %color)

        print (color)

        self.agent_length = self.l_front + self.l_rear

        self.plt_front, = ax.plot([self.center_of_mass[0], self.front[0]],
                             [self.center_of_mass[1], self.front[1]], "k-", lw=3.0)
        self.plt_rear, = ax.plot([self.center_of_mass[0], self.rear[0]],
                             [self.center_of_mass[1], self.rear[1]], "k-", lw=3.0)

        self.plt_front_axis, = ax.plot(self.center_front_wheel[:, 0],
								   self.center_front_wheel[:, 1], "k-", lw=3.0)
        self.plt_rear_axis, = ax.plot(self.center_rear_wheel[:, 0],
								  self.center_rear_wheel[:, 1], "k-", lw=3.0)

        # TODO: Add this part
        """
        self.selected_states =
        self.sel_states_connected =
        self.predicted_xy =
        self.predicted_xy_con =
        self.predicted_true_xy =
        self.predicted_true_xy_con =
        self.trajectory =
        self.center_of_mass =
        self.deleted_states =
        self.com = np.zeros(length(agents), 2)  # center of mass
        """

        self.rect = patch.Rectangle([- self.l_rear - self.wheel_length / 2 - self.bumper,
                                     - self.width / 2 - self.wheel_width / 2],
                                     self.agent_length + self.wheel_length + 2 * self.bumper,
                                     self.width + self.wheel_width,
                                     color=self.color, alpha=0.5, ec="black")
        self.front_wheel_left = patch.Rectangle([- self.wheel_length / 2,
							                    - self.wheel_width / 2],
							                    self.wheel_length, self.wheel_width,
							                    color="gray", alpha=1.0, ec="black")
        self.front_wheel_right = patch.Rectangle([- self.wheel_length / 2,
							                    - self.wheel_width / 2],
							                    self.wheel_length, self.wheel_width,
							                    color="gray", alpha=1.0, ec="black")
        self.rear_wheel_left = patch.Rectangle([- self.wheel_length / 2,
							                    - self.wheel_width / 2],
							                    self.wheel_length, self.wheel_width,
							                    color="gray", alpha=1.0, ec="black")
        self.rear_wheel_right = patch.Rectangle([- self.wheel_length / 2,
							                    - self.wheel_width / 2],
							                    self.wheel_length, self.wheel_width,
							                    color="gray", alpha=1.0, ec="black")

        # self.transform(ax)

        ax.add_artist(self.rect)
        ax.add_artist(self.front_wheel_left)
        ax.add_artist(self.front_wheel_right)
        ax.add_artist(self.rear_wheel_left)
        ax.add_artist(self.rear_wheel_right)

    def xy_callback(self, msg):
        self.center_of_mass = np.array(msg.x[0], msg.y[0])
        self.psi = np.array(msg.psi[0])
        self.psi_and_steering = self.psi + msg.steering[0]

        self.front = self.center_of_mass + \
                     rotate_around_z(np.array([self.l_front, 0.0]), self.psi)
        self.rear = self.center_of_mass - \
                    rotate_around_z(np.array([self.l_rear, 0.0]), self.psi)

        self.center_front_wheel = np.array([[0.0, self.width / 2],
                                            [0.0, - self.width / 2]])
        self.center_front_wheel = rotate_around_z(self.center_front_wheel.T, self.psi)
        self.center_front_wheel = np.reshape(np.repeat(self.front, 2), (2, 2)).T + \
                                  self.center_front_wheel.T
        self.center_rear_wheel = np.array([[0.0, self.width / 2],
                                           [0.0, - self.width / 2]])
        self.center_rear_wheel = rotate_around_z(self.center_rear_wheel.T, self.psi)
        self.center_rear_wheel = np.reshape(np.repeat(self.rear, 2), (2, 2)).T + \
                                 self.center_rear_wheel.T

        self.center_front_wheel_left = self.center_front_wheel[0, :]
        self.center_front_wheel_right = self.center_front_wheel[1, :]
        self.center_rear_wheel_left = self.center_rear_wheel[0, :]
        self.center_rear_wheel_right = self.center_rear_wheel[1, :]

    def transform(self, ax):
        self.rect.set_transform((set_transformation(ax, self.psi,
                                                    self.center_of_mass)))
        self.front_wheel_left.set_transform(set_transformation(ax, self.psi_and_steering,
                                                               self.center_front_wheel_left))
        self.front_wheel_right.set_transform(set_transformation(ax, self.psi_and_steering,
                                                                self.center_front_wheel_right))
        self.rear_wheel_left.set_transform(set_transformation(ax, self.psi,
                                                              self.center_rear_wheel_left))
        self.rear_wheel_right.set_transform(set_transformation(ax, self.psi,
                                                               self.center_rear_wheel_right))

    def update(self, ax):
        self.plt_front.set_data([self.center_of_mass[0], self.front[0]],
                                [self.center_of_mass[1], self.front[1]])
        self.plt_rear.set_data([self.center_of_mass[0], self.rear[0]],
                               [self.center_of_mass[1], self.rear[1]])

        self.plt_front_axis.set_data(self.center_front_wheel[:, 0],
                                     self.center_front_wheel[:, 1])
        self.plt_rear_axis.set_data(self.center_rear_wheel[:, 0],
                                    self.center_rear_wheel[:, 1])

        self.transform(ax)


class Plotter:
    """docstring for Plotter"""
    plt.ion()  # interactive plotting on

    fig = plt.figure("Race")
    ax = fig.add_subplot(1, 1, 1)

    plotted_agents = None

    def __init__(self, track, colors=None):
        self.draw_track(track)
        self.draw_finish_line(track)

        if colors is not None:
            self.plotted_agents = [PlottedAgent(color, self.ax) for color in colors]

        plt.axis("equal")
        plt.grid()
        plt.show()
        plt.pause(0.0001)

    def draw_track(self, track):
        self.ax.plot(track.xy_coords[:, 0], track.xy_coords[:, 1], "k--")
        self.ax.plot(track.xy_inner[:, 0], track.xy_inner[:, 1], "k-", lw=2.0)
        self.ax.plot(track.xy_outer[:, 0], track.xy_outer[:, 1], "k-", lw=2.0)

    def draw_finish_line(self, track):
        self.ax.plot(np.array([0, 0]), track.width / 2 * np.array([1, - 1]),
                     "k--", lw=5.0)

    def update(self):
        if self.plotted_agents is not None:
            for i in range(len(self.plotted_agents)):
                self.plotted_agents[i].update(self.ax)
        plt.pause(0.0001)


if __name__ == "__main__":

    try:
        rospy.init_node("plotting_stuff")
        colors = ["blue"]
        track = Track(ds=0.1, shape="test", width=1.0)
        plotter = Plotter(track, colors)

        loop_rate = 20
        rate = rospy.Rate(loop_rate)

        while not rospy.is_shutdown():
            plotter.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
