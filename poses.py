#!/usr/bin/env python


import rospy
import tf
import math
import yaml

import os.path

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray

torso_2_joint = "torso_2_joint"
torso_3_joint = "torso_3_joint"
file_path = '/home/fmw-hb/catkin_msh/src/choreography/document.yaml'

class choreo_poses():
    def __init__(self):
        rospy.init_node('choreography_poses', anonymous=True)
        rospy.Subscriber("/torso/joint_states", JointState , self.torso_callback)
        rospy.Subscriber("/head/joint_states", JointState, self.head_callback)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.base_callback)
        self.head_joint_pos = [0,0,0]
        self.torso_joint_pos = [0,0,0]
        self.base_odometry = None
        self.pose_name = ""
        print "choreography poses started"

    def head_callback(self, joint_state):

        self.head_joint_pos = joint_state.position

    def torso_callback(self, joint_state):

        self.torso_joint_pos = joint_state.position

    def base_callback(self, odometry):

        self.base_odometry = odometry

    def safe_pose(self):
        self.pose_name = raw_input("Please enter pose name (press 'c' to cancel)")
        if (self.pose_name == 'c'):
            return

        torso_positions = []
        base_poses = []
        # read from file, if old file exists
        if(os.path.isfile(file_path)):
            with open(file_path, 'r') as stream:
                yaml_data = yaml.load(stream)
            for it in yaml_data["torso"]:
                name = it["name"]
                x = it["position"][torso_2_joint]
                y = it["position"][torso_3_joint]
                torso_position={'position':{torso_2_joint:x, torso_3_joint:y}, 'name':name}
                torso_positions.append(torso_position)

            for it in yaml_data["base"]:
                name = it["name"]
                x = it["position"]["x"]
                y = it["position"]["y"]
                w = it["position"]["yaw"]
                base_pose={'position':{"x":x, "y":y, "yaw":w}, 'name':name}
                base_poses.append(base_pose)

        # read new positions from topic
        torso_position={'position':{torso_2_joint:self.torso_joint_pos[0], torso_3_joint:self.torso_joint_pos[1]}, 'name':self.pose_name}
        torso_positions.append(torso_position)
        quaternion = (
            self.base_odometry.pose.pose.orientation.x,
            self.base_odometry.pose.pose.orientation.y,
            self.base_odometry.pose.pose.orientation.z,
            self.base_odometry.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        base_pose={'position':{"x":self.base_odometry.pose.pose.position.x, "y":self.base_odometry.pose.pose.position.y, "yaw":euler[2]}, 'name':self.pose_name}
        base_poses.append(base_pose)
        print "name: "+str(self.pose_name)+", "+torso_2_joint+": "+str(self.torso_joint_pos[0])+", "+torso_3_joint+": "+str(self.torso_joint_pos[1])



        # save everything in the file
        with open(file_path, 'w') as stream:
            stream.write(yaml.dump({'torso':torso_positions, 'base':base_poses}, default_flow_style=False))


    def execute(self):
        while not rospy.is_shutdown():
            keyboard_input = raw_input("Press 's' to save pose. (press 'e' to exit)")
            if(keyboard_input == 's'):
                self.safe_pose()
            elif (keyboard_input == 'e'):
                print "Exiting choreography_poses..."
                return
            else:
                print "Unknown keyboard input"





#TODO: wenn taste gedrueckt rosspin once!!!

if __name__ == '__main__':
    try:
        t = choreo_poses()
        t.execute()
    except rospy.ROSInterruptException:
        pass
