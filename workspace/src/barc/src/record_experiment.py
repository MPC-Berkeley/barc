#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import subprocess, os
import json

import rospy
import rosbag

import time

from data_service.srv import DataForward, RegisterVideo
from data_service.msg import TimeSignal
from tf import transformations

import cv2
import numpy as np

rosbag_dir = os.path.expanduser("~") + '/rosbag'
video_dir = os.path.expanduser("~") + '/video'
image_dir = os.path.expanduser("~") + '/images'

class RecordExperiment():
    def __init__(self):
        # get parameters
        self.experiment_name     = rospy.get_param("/record_experiment/experiment_name")
        self.camera_on           = rospy.get_param("/record_experiment/camera_on")

        # wait for ROS services
        rospy.wait_for_service('send_data')
        rospy.wait_for_service('register_video')

        # resigter proxy service
        self.send_data = rospy.ServiceProxy('send_data', DataForward, persistent=True)
        if self.camera_on:
            self.start_record_video()
            self.register_video = rospy.ServiceProxy('register_video', RegisterVideo)
            try:
                self.register_video(self.experiment_name, video_dir + '/%s.avi' % self.experiment_name)
            except Exception as e:
                pass
 
        # ensure data storage directories exist
        if not os.path.isdir(video_dir):
            os.makedirs(video_dir)
        if not os.path.isdir(rosbag_dir):
            os.makedirs(rosbag_dir)
        if not os.path.isdir(image_dir):
            os.makedirs(image_dir)
         
        # start rosrecord for following topics
        self.topics = ['/imu/data', '/encoder', '/ecu', '/ecu_pwm', '/image_transformed/compressed/', '/fix', '/vel_est']
        self.rosbag_file_path = os.path.abspath(rosbag_dir + '/' + self.experiment_name + '.bag')
        self.start_record_data()
        
        # upload video to local server on shutdown
        rospy.on_shutdown( self.process_data )
        rospy.spin()

    def start_record_data(self):
        # construct rosbag record command 
        command = 'rosbag record '
        self.time = time.time()
        for topic in self.topics:
            command += topic + ' '
        command += ' -O %s' % self.experiment_name
        self.proc_bag = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=rosbag_dir)

    def start_record_video(self):
        command = 'rosrun image_view video_recorder image:=/image_raw _max_depth_range:=0 _fps:=30'
        self.proc_vid = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=video_dir)

    def process_data(self):
        # stop recording 
        self.proc_bag.kill()
        if self.camera_on:
            self.proc_vid.kill()
            time.sleep(0.5)
            command = 'mv output.avi %s.avi' % self.experiment_name
            subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=video_dir)
        
        # wait for bagfile to get created
        for i in range(100):
            if os.path.isfile(self.rosbag_file_path):
                break
            time.sleep(1.0)
        if not os.path.isfile(self.rosbag_file_path):
            print("file not created on time")
            return

        # get bag file  
        self.bag = rosbag.Bag(self.rosbag_file_path)

        # extract images
        if self.camera_on:
            self.extract_images()

        # upload all data
        print("starting to upload data ...")
        self.upload_data()

    def extract_images(self):
        # create directory for images
        experiment_img_dir = image_dir + "/" + self.experiment_name  
        if not os.path.isdir(experiment_img_dir):
            os.makedirs(experiment_img_dir)

        # extract images
        idx = 0
        for topic, msg, t in self.bag.read_messages( topics='/image_transformed/compressed/' ):
            nparr = np.fromstring(msg.data, np.uint8)
            img_data = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cv2.imwrite( experiment_img_dir + '/%5d.jpg' % idx, img_data)
            idx += 1

    def upload_data(self):
        # declare data storage containers
        chunk_size = 50
        chunk_dict = dict()
        chunk_msg = dict()
        chunk_ts = dict()
        
        img_idx = 0.0
        for topic, msg, t in self.bag.read_messages( topics= self.topics):
            ts = t.secs + t.nsecs/(10.0**9)

            if topic not in chunk_dict:
                chunk_dict[topic] = 1

            if topic not in chunk_msg:
                chunk_msg[topic] = []

            if topic not in chunk_ts:
                chunk_ts[topic] = []

            chunk_dict[topic] += 1

            if topic =='/image_transformed/compressed/' :
                chunk_msg[topic].append( img_idx )
                chunk_ts[topic].append( ts )
                img_idx += 1
            else:
                chunk_msg[topic].append( msg )
                chunk_ts[topic].append( ts )

            for k, v in chunk_dict.items():
                if v > chunk_size:
                    self.upload_message(k, chunk_msg[k], chunk_ts[k])
                    del chunk_msg[k]
                    del chunk_dict[k]
                    del chunk_ts[k]

        for k, v in chunk_dict.items():
            self.upload_message(k, chunk_msg[k], chunk_ts[k])

        print("done uploading!")

    def upload_message(self, topic, msgs, tss):
        vars_list = ['roll', 'pitch', 'yaw',
                     'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                     'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
                     'encoder_FL', 'encoder_FR','encoder_BL','encoder_BR', 'velocity_FL', 'velocity_FR',
                     'velocity_BL', 'velocity_BR', 'motor', 'servo','motor_pwm','servo_pwm',
                     'image_id',
                     'longitude','latitude','altitude','gps_status','gps_service']

        signal_dict = dict()

        for v in vars_list:
            signal_dict[v] = []

        for msg in msgs:
            # Inertia measurement unit
            if topic == '/imu/data':
                ori         = msg.orientation
                quaternion  = (ori.x, ori.y, ori.z, ori.w)
                (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
                angular_velocity_x = msg.angular_velocity.x
                angular_velocity_y = msg.angular_velocity.y
                angular_velocity_z = msg.angular_velocity.z
                linear_acceleration_x = msg.linear_acceleration.x
                linear_acceleration_y = msg.linear_acceleration.y
                linear_acceleration_z = msg.linear_acceleration.z

            # Encoder
            if topic == '/encoder':
                encoder_FL = msg.FL
                encoder_FR = msg.FR
                encoder_BL = msg.BL
                encoder_BR = msg.BR
                
            # Velocity
            if topic == '/vel_est':
                velocity_FL = msg.FL
                velocity_FR = msg.FR
                velocity_BL = msg.BL
                velocity_BR = msg.BR

            # Ultrasound
            if topic == '/ultrasound':
                ultrasound_front = msg.front
                ultrasound_back = msg.back
                ultrasound_left = msg.left
                ultrasound_right = msg.right

            # Electronic control unit (high level commands)
            if topic == '/ecu':
                motor = msg.motor
                servo = msg.servo

            # Electronic control unit (low level commands)
            if topic == '/ecu_pwm':
                motor_pwm = msg.motor
                servo_pwm = msg.servo

            # Camera ID
            if topic =='/image_transformed/compressed/':
                image_id = msg  

            # GPS
            if topic == '/fix':
                longitude = msg.longitude
                latitude = msg.latitude
                altitude = msg.altitude
                gps_status = msg.status.status
                gps_service = msg.status.service

            # Python introspection from list 'vars_list'
            for v in vars_list:
                if v in dict(globals(), **locals()):
                    signal_dict[v].append(dict(globals(), **locals())[v])

        for v in vars_list:
            if signal_dict[v]:
                time_signal = TimeSignal()
                time_signal.name = v
                time_signal.timestamps = tss
                time_signal.signal = json.dumps(signal_dict[v])
                try:
                    self.send_data(time_signal, None, self.experiment_name)
                except Exception as e:
                    pass

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('record_experiment')

    try:
        node = RecordExperiment()
    except rospy.ROSInterruptException:
        pass


