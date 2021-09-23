#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
from stanley import StanleyControl
from xycar_msgs.msg import xycar_motor

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64

class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.k_gain = 0.5

        with open("/home/selfdriving/xycar_ws/src/grepp_project_ws/src/path_follower/src/path.pkl") as f:
            self.path = pickle.load(f)

        self.ego_pose_sub = rospy.Subscriber("ego_pose", PoseStamped, self.PoseCallBack)
        self.ego_speed_sub = rospy.Subscriber("ego_speed", Float64, self.SpeedCallBack)

    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def SpeedCallBack(self, msg):
        self.v = msg.data

    def GetSteeringAngleMsg(self):
        steps = 50
        last = 4701
        delta = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v,
                               self.path['x'][:last:steps], self.path['y'][:last:steps], self.path['yaw'][:last:steps],
                               L=3.14, k=self.k_gain)
        steering_angle_msg = Float64()
        steering_angle_msg.data = delta
        return steering_angle_msg


class PIDController(object):
    def __init__(self):
        self.current_speed = 0.0
        self.p_gain = 0.5
        self.speed_sub = rospy.Subscriber("ego_speed", Float64, self.CurSpeedCallBack)

    def GetAccCmdMsg(self, target_speed):
        acc_cmd = -self.p_gain * (self.current_speed - target_speed)

        acc_cmd_msg = Float64()
        acc_cmd_msg.data = acc_cmd

        return acc_cmd_msg

    def CurSpeedCallBack(self, msg):
        self.current_speed = msg.data

if __name__ == '__main__':
    rospy.init_node("stanley_follower_node")

    steering_angle_pub = rospy.Publisher("steering_angle_cmd", Float64, queue_size=1)
    acc_cmd_pub = rospy.Publisher("acc_cmd", Float64, queue_size=1)
    xycar_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

    stanley = StanleyController()
    controller = PIDController()

    target_speed = 20.0

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        steering_angle_msg = stanley.GetSteeringAngleMsg()
        steering_angle_pub.publish(steering_angle_msg)

        acc_cmd_msg = controller.GetAccCmdMsg(target_speed)
        acc_cmd_pub.publish(acc_cmd_msg)
        
        xycar_msg = xycar_motor()
        xycar_msg.speed = acc_cmd_msg
        xycar_msg.angle = steering_angle_msg
        xycar_pub.publish(xycar_msg)
        r.sleep()

