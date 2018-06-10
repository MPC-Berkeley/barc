#!/usr/bin/env python

'''
    File name: plotter.py
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Python Version: 2.7.12
'''

import rospy
from barc.msg import xy_prediction, pos_info, theta, selected_states
from marvelmind_nav.msg import hedge_imu_fusion
import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.transforms as transforms
import matplotlib.animation as animation
import matplotlib as mpl
import matplotlib.collections as collections
import pdb

from track import Track
from transformations import rotate_around_z, s_to_xy, xy_to_s

HORIZON = 10
NUM_CONSIDERED_STATES = 100
NUM_LAPS = 10


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
    wheel_length = 0.04
    wheel_width = 0.02
    bumper = 0.01
    l_front = 0.125
    l_rear = 0.125
    width = 0.1

    x_gps = 0.0
    y_gps = 0.0

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

    trajectory_xy = np.zeros((500, 2))
    trajectory_gps_xy = np.zeros((500, 2))
    traj_counter = 0
    gps_counter = 0
    traj_gps_counter = 0
    prediction_available = False

    num_points = 500
    theta_vx = np.zeros((num_points, 3))
    theta_vy = np.zeros((num_points, 4))
    theta_psi_dot = np.zeros((num_points, 3))

    theta_counter = 0

    selection_x = np.zeros((NUM_CONSIDERED_STATES))
    selection_y = np.zeros((NUM_CONSIDERED_STATES))

    # track = Track(ds=0.1, shape="test", width=1.0)

    steering_angles = np.zeros(500)
    s = np.linspace(start=0.0, stop=23.9, num=500)
    counter = 0

    time_start = None

    def __init__(self, color_string, ax, track):
        node_name = color_string.split("/")[0]
        sim_string = rospy.get_param(rospy.get_name() + "/" + node_name)
        if sim_string == "simulation":
            xy_sub = rospy.Subscriber(node_name + "/real_val", pos_info, self.xy_callback)
        else:
            xy_sub = rospy.Subscriber(node_name + "/pos_info", pos_info, self.xy_callback)

        xy_gps_sub = rospy.Subscriber(node_name + "/hedge_imu_fusion", hedge_imu_fusion, self.xy_gps_callback)
        prediction_sub = rospy.Subscriber(node_name + "/xy_prediction", xy_prediction, self.prediction_callback)
        selection_sub = rospy.Subscriber(node_name + "/selected_states", selected_states, self.selection_callback)

        # theta_sub = rospy.Subscriber("theta", theta, self.theta_callback)

        self.track = track

        # self.color = color
        self.color = rospy.get_param(color_string)
        if self.color == "blue":
            self.dot = "bo"
            self.star = "b*"
            self.line = "b-"
            self.dot_extra = "co"
            self.line_extra = "c-"
        elif self.color == "red":
            self.dot = "ro"
            self.star = "r*"
            self.line = "r-"
            self.dot_extra = "mo"
            self.line_extra = "m-"
        else:
            ValueError("Color %s not defined" %self.color)

        print (self.color)

        self.selected_states_plot, = ax.plot(self.selection_x, self.selection_y, 
                                             self.dot_extra)

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
        self.prediction_xy, = ax.plot(self.center_of_mass[0] * np.ones(HORIZON + 1),
                                     self.center_of_mass[1] * np.ones(HORIZON + 1), 
                                     self.dot)
        self.prediction_xy_line, = ax.plot(self.center_of_mass[0] * np.ones(HORIZON + 1), 
                                          self.center_of_mass[1] * np.ones(HORIZON + 1), 
                                          self.line)

        self.trajectory, = ax.plot(self.trajectory_xy[:, 0], self.trajectory_xy[:, 1], self.line)

        self.current_lap = 0

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

        self.gps_pos, = ax.plot(self.x_gps, self.y_gps, self.star)
        self.trajectory_gps, = ax.plot(self.trajectory_gps_xy[:, 0], self.trajectory_gps_xy[:, 1], self.line_extra)

        # self.transform(ax)

        ax.add_artist(self.rect)
        ax.add_artist(self.front_wheel_left)
        ax.add_artist(self.front_wheel_right)
        ax.add_artist(self.rear_wheel_left)
        ax.add_artist(self.rear_wheel_right)

    def xy_callback(self, msg):
        self.center_of_mass = np.array([msg.x, msg.y])

        self.psi = msg.psi
        self.psi_and_steering = self.psi + msg.u_df

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

    def xy_gps_callback(self, msg):
        self.x_gps = msg.x_m
        self.y_gps = msg.y_m

        # print "gps counter: " self.gps_counter

        if self.prediction_available:
	        if self.gps_counter % 10 == 0:
	            self.trajectory_gps_xy[self.traj_gps_counter, :] = np.array([msg.x_m, msg.y_m])
	            self.trajectory_gps_xy[self.traj_gps_counter:, :] = np.array([msg.x_m, msg.y_m])
	            self.traj_gps_counter += 1

	        self.gps_counter += 1

        
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

    def prediction_callback(self, msg):
        # print(np.array(msg.x))
        # test_s = np.zeros((len(msg.x), 6))
        # test_xy = np.zeros((len(msg.x), 6))
        # test_s[:, 0] = msg.x
        # test_s[:, 1] = msg.y
        # for i in range(len(msg.x)):
        #     test_xy[i, :] = s_to_xy(test_s[i, :], self.track)
        # self.prediction_xy.set_data(test_xy[:, 0], test_xy[:, 1])
        # self.prediction_xy_line.set_data(test_xy[:, 0], test_xy[:, 1])

        # self.steering_angles[self.counter] = msg.steering[0]

        # xy_state = np.array([msg.x[0], msg.y[0], 0.0, 0.0, 0.0, 0.0])
        # s_state = xy_to_s(xy_state, self.track)
        # self.s[self.counter] = s_state[0]
        # self.counter += 1

        self.prediction_available = True

        try:
            self.prediction_xy.set_data(np.array(msg.x), np.array(msg.y))
            self.prediction_xy_line.set_data(np.array(msg.x), np.array(msg.y))
        except RuntimeError:
            self.prediction_xy.set_data(np.zeros(HORIZON), np.zeros(HORIZON))
            self.prediction_xy_line.set_data(np.zeros(HORIZON), np.zeros(HORIZON))

        if msg.current_lap > self.current_lap:
            self.trajectory_xy = np.zeros_like(self.trajectory_xy)
            self.trajectory_gps_xy = np.zeros_like(self.trajectory_gps_xy)
            self.traj_counter = 0
            self.gps_counter = 0
            self.traj_gps_counter = 0

        self.trajectory_xy[self.traj_counter, :] = np.array([np.array(msg.x)[0], np.array(msg.y)[0]])
        self.trajectory_xy[self.traj_counter:, :] = np.array([np.array(msg.x)[0], np.array(msg.y)[0]])
        self.traj_counter += 1
        
        self.current_lap = msg.current_lap

        if self.time_start is None:
            self.time_start = rospy.get_time()

        # def theta_callback(self, msg):
        #    self.theta_vx[self.theta_counter, :] = msg.theta_vx
        #    self.theta_vy[self.theta_counter, :] = msg.theta_vy
        #    self.theta_psi_dot[self.theta_counter, :] = msg.theta_psi_dot
        #
        #    self.vx_plot_1.set_data(np.arange(self.num_points), self.theta_vx[:, 0])
        #    self.vx_plot_2.set_data(np.arange(self.num_points), self.theta_vx[:, 1])
        #    self.vx_plot_3.set_data(np.arange(self.num_points), self.theta_vx[:, 2])

        #    self.theta_counter += 1


    def selection_callback(self, msg):
        self.selection_x = msg.x
        self.selection_y = msg.y

        self.selected_states_plot.set_data(self.selection_x, self.selection_y)

    def update(self, ax):
        self.plt_front.set_data([self.center_of_mass[0], self.front[0]],
                                [self.center_of_mass[1], self.front[1]])
        self.plt_rear.set_data([self.center_of_mass[0], self.rear[0]],
                               [self.center_of_mass[1], self.rear[1]])

        self.plt_front_axis.set_data(self.center_front_wheel[:, 0],
                                     self.center_front_wheel[:, 1])
        self.plt_rear_axis.set_data(self.center_rear_wheel[:, 0],
                                    self.center_rear_wheel[:, 1])


        self.trajectory.set_data(self.trajectory_xy[:, 0], self.trajectory_xy[:, 1])

        self.gps_pos.set_data(self.x_gps, self.y_gps)
        self.trajectory_gps.set_data(self.trajectory_gps_xy[:, 0], self.trajectory_gps_xy[:, 1])

        self.transform(ax)


class Plotter:
    """docstring for Plotter"""
    plt.ion()  # interactive plotting on

    fig = plt.figure("Race")
    ax = fig.add_subplot(1, 1, 1)
    # ax = fig.add_subplot(2, 1, 1)
    # ax_2 = fig.add_subplot(2, 1, 2)

    # ax_2.set_ylim([-1, 1])

    plotted_agents = None
    count = 0

    time_start = None

    def __init__(self, track, colors=None):
        self.track = track
        self.draw_track(track)
        self.draw_finish_line(track)

        color_strings = []
        """
        try:
            colors = [rospy.get_param("agent_1/color"), 
                      rospy.get_param("agent_2/color")]
            color_strings = ["agent_1/color", "agent_2/color"]
        except:
            colors = [rospy.get_param("agent_1/color")]
            color_strings = ["agent_1/color"]
        """
        node_name = rospy.get_name()
        self.num_agents = rospy.get_param(node_name + "/num_agents")
        if self.num_agents == 1:
            index = rospy.get_param(node_name + "/index") 
            color_strings = ["agent_" + str(i) + "/color" for i in [index]]
        else:
            color_strings = ["agent_" + str(i + 1) + "/color" for i in range(self.num_agents)]
        # color_strings = ["agent_1/color", "agent_2/color"]
        # print("plotter color: ", colors)

        if color_strings is not None:
            self.plotted_agents = [PlottedAgent(color_string, self.ax, track) for color_string in color_strings]

        # print(len(np.arange(len(track.curvature))))
        # print(len(self.plotted_agents[0].steering_angles))
        curvature = np.array([track.get_curvature(s) for s in self.plotted_agents[0].s])
        # self.steering_plot, = self.ax_2.plot(np.linspace(0.0, 23.9, 500), 
        #                                self.plotted_agents[0].steering_angles, 
        #                                "b-")
        # self.ax_2.plot(np.linspace(0.0, 23.9, 500), 10 * curvature)

        leading_index = 0
        following_index = 0

        elapsed_time = 0.0
        self.time_string = "Time : %.2f s" % (round(elapsed_time, 2))

        # plotter.ax[:annotate](time_string, xy=[0.5, 0.9], 
        self.time_s = self.ax.annotate("", xy=[0.6, 0.85], xycoords= "axes fraction", 
                                  fontsize=20, verticalalignment="top")
        # plotter.ax[:annotate](race_string, xy=[0.8, 0.9], xycoords="axes fraction", 
        self.race_s = self.ax.annotate("", xy=[0.6, 0.75], xycoords="axes fraction", 
                                  fontsize=20, verticalalignment="top")

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

    def create_race_info_strings(self, num_laps):
        if self.num_agents == 2:
            agent_1_lap = self.plotted_agents[0].current_lap
            agent_2_lap = self.plotted_agents[1].current_lap
            if agent_1_lap > agent_2_lap:
                leading_index = 1
                following_index = 2
            elif agent_1_lap == agent_2_lap:
                agent_1_xy = np.zeros(6)
                agent_1_xy[: 2] = self.plotted_agents[0].center_of_mass
                agent_1_s = xy_to_s(agent_1_xy, self.track)[0]
                agent_2_xy = np.zeros(6)
                agent_2_xy[: 2] = self.plotted_agents[1].center_of_mass
                agent_2_s = xy_to_s(agent_2_xy, self.track)[0]
                if agent_1_s > agent_2_s:
                    leading_index = 1
                    following_index = 2
                else:
                    leading_index = 2
                    following_index = 1
            else:
                leading_index = 2
                following_index = 1

            leading_agent = self.plotted_agents[leading_index - 1].color.capitalize()
            leading_lap = self.plotted_agents[leading_index - 1].current_lap
            following_agent = self.plotted_agents[following_index - 1].color.capitalize()
            following_lap = self.plotted_agents[following_index - 1].current_lap

            self.race_string = "%-13s %-10s\n1. %-10s %d/%d\n2. %-10s %d/%d" %("Race", "Lap", 
                                                                          leading_agent, 
                                                                          leading_lap, num_laps,
                                                                          following_agent,
                                                                          following_lap,
                                                                          num_laps)
        else:
            leading_agent = self.plotted_agents[0].color.capitalize()
            leading_lap = self.plotted_agents[0].current_lap
            self.race_string = "%-13s %-10s\n1. %-10s %d/%d" %("Race", "Lap", 
                                                          leading_agent, 
                                                          leading_lap, num_laps)

    def update_time_string(self):
        time_now = rospy.get_time()
        if self.time_start is not None:
            elapsed_time = time_now - self.time_start
            self.time_string = "Time : %.2f s" % (round(elapsed_time, 2))

    def update(self, track):
        if self.plotted_agents is not None:
            for i in range(len(self.plotted_agents)):
                self.plotted_agents[i].update(self.ax)

            if self.num_agents == 2:
                num_laps = max(self.plotted_agents[0].current_lap, 
                               self.plotted_agents[1].current_lap)
                if self.time_start is None:
                    if self.plotted_agents[0].time_start is not None and \
                        self.plotted_agents[1].time_start is not None:
                        self.time_start = min(self.plotted_agents[0].time_start,
                                              self.plotted_agents[1].time_start)
                    elif self.plotted_agents[0].time_start is not None:
                        self.time_start = self.plotted_agents[0].time_start
                    elif self.plotted_agents[1].time_start is not None:
                        self.time_start = self.plotted_agents[1].time_start
            else:
                num_laps = self.plotted_agents[0].current_lap
                if self.time_start is None:
                    if self.plotted_agents[0].time_start is not None:
                        self.time_start = self.plotted_agents[0].time_start

            self.create_race_info_strings(30)
            self.update_time_string()
            self.time_s.set_text(self.time_string)
            self.race_s.set_text(self.race_string)



        # self.steering_plot.set_data(self.plotted_agents[0].s, 
        #                             self.plotted_agents[0].steering_angles)

        # self.ax.set_xlim(self.plotted_agents[0].center_of_mass[0] - 1.0, self.plotted_agents[0].center_of_mass[0] + 1.0)
        # self.ax.set_ylim(self.plotted_agents[0].center_of_mass[1] - 1.0, self.plotted_agents[0].center_of_mass[1] + 1.0)

        # self.ax_2.set_xlim(0.0, track.total_length)
        # self.ax_2.set_ylim(- np.pi / 3, np.pi / 3)

        # plt.savefig("/home/lukas/images/image_" + str(self.count) + ".png")

        # print("STEERING: ", self.plotted_agents[0].steering_angles[np.max(self.plotted_agents[0].counter - 1, 0)])
        # print("MAX STEERING: ", np.max(self.plotted_agents[0].steering_angles))
        # print("MIN STEERING: ", np.min(self.plotted_agents[0].steering_angles))
        
        try:
            plt.pause(0.0001)
        except:
            RuntimeError("Plot not updated.")

        self.count += 1


if __name__ == "__main__":

    try:
        rospy.init_node("plotting_stuff")
        colors = ["blue"]
        # track = Track(ds=0.1, shape="test", width=1.0)
        # track = Track(ds=0.1, shape="oval", width=1.0)
        track = Track(ds=0.1, shape="l_shape", width=1.2)
        plotter = Plotter(track, colors)

        loop_rate = 20
        rate = rospy.Rate(loop_rate)

        while not rospy.is_shutdown():
            plotter.update(track)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
