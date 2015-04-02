#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
from std_msgs.msg import Header
from mico_js_controller.srv import InputMethodSwitchService

DEBUG = True


class InputGenerator(object):
    __metaclass__ = ABCMeta

    # able to remap names
    input_pose_topicname = 'input_PoseStamped'
    input_twist_topicname = 'input_TwistStamped'
    current_pose_topicname = "current_PoseStamped"
    inout_method_switching_servicename = 'input_method_switcher'

    def __init__(self, data_from_device_topic_name, data_from_device_topic_class):
        self.current_pose = Pose()
        self.input_pose = Pose()
        self.input_twist = Twist()
        self.init_input_values()

        self.twist_mode_flag = True
        self.position_mode_flag = True

        self.topic_name = data_from_device_topic_name
        self.topic_class = data_from_device_topic_class

    def init_input_values(self):
        self.input_twist.linear.x = 0
        self.input_twist.linear.y = 0
        self.input_twist.linear.z = 0
        self.input_twist.angular.x = 0
        self.input_twist.angular.y = 0
        self.input_twist.angular.z = 0

    def initialize_publishers(self):
        self.pub_pose = rospy.Publisher(self.input_pose_topicname, PoseStamped, queue_size=10)
        self.pub_twist = rospy.Publisher(self.input_twist_topicname, TwistStamped, queue_size=10)

    def initialize_subscriber(self):
        self.sub_pose = rospy.Subscriber(self.current_pose_topicname, PoseStamped, self.ee_pose_callback)
        self.sub_data_from_device = rospy.Subscriber(self.topic_name, self.topic_class, self.data_from_device_callback)

    def ee_pose_callback(self, ee_pose):
        print ee_pose
        self.current_pose = ee_pose.pose

    def data_from_device_callback(self, data):
        if self.twist_mode_flag:
            self.generate_twist(data)
        else:
            self.generate_pose(data)

    def run_input_method_switch_server(self):
        rospy.Service(self.inout_method_switching_servicename, InputMethodSwitchService,
                      self.input_method_switch_handler)

    @abstractmethod
    def generate_pose(self, data_from_device):
        self.input_pose = self.current_pose

    @abstractmethod
    def generate_twist(self, data_from_device):
        self.input_twist.linear.x = 0
        self.input_twist.linear.y = 0
        self.input_twist.linear.z = 0
        self.input_twist.angular.x = 0
        self.input_twist.angular.y = 0
        self.input_twist.angular.z = 0

    def input_method_switch_handler(self, req):
        if DEBUG: print('switch!')

        if req.twist_pose_switcher == 1:
            self.twist_mode_flag = True
        elif req.twist_pose_switcher == 2:
            self.twist_mode_flag = False

        if req.pos_ori_switcher == 1:
            self.position_mode_flag = True
        elif req.pos_ori_switcher == 2:
            self.position_mode_flag = False

    def switch_twist_input(self, _header):
        self.pub_pose.unregister()
        self.pub_twist = rospy.Publisher(self.input_twist_topicname, TwistStamped, queue_size=10)
        self.pub_twist.publish(TwistStamped(header=_header, twist=self.input_twist))

    def switch_pose_input(self, _header):
        self.pub_twist.unregister()
        self.pub_pose = rospy.Publisher(self.input_pose_topicname, PoseStamped, queue_size=10)
        self.pub_pose.publish(PoseStamped(header=_header, pose=self.input_pose))


class JoyStickInputGen(InputGenerator):
    def __init__(self):
        super(JoyStickInputGen, self).__init__('joystick', Joy)
        self.scale_pose = Pose()
        self.scale_twist = Twist()
        self.init_input_values()
        self.set_scale_values()

    def set_scale_values(self):
        self.scale_pose.position.x = 1.0
        self.scale_pose.position.y = 1.0
        self.scale_pose.position.z = 1.0

        self.scale_twist.linear.x = 0.10
        self.scale_twist.linear.y = 0.10
        self.scale_twist.linear.z = 0.10
        self.scale_twist.angular.x = 1.0
        self.scale_twist.angular.y = 1.0
        self.scale_twist.angular.z = 1.0

    def generate_pose(self, joy_data):
        self.input_pose = self.current_pose

    def generate_twist(self, joy_data):
        if self.position_mode_flag:
            self.input_twist.linear.x = self.scale_twist.linear.x * joy_data.axes[3]
            self.input_twist.linear.y = self.scale_twist.linear.y * joy_data.axes[2]
            self.input_twist.linear.z = self.scale_twist.linear.z * joy_data.axes[5]
        else:
            self.input_twist.angular.x = self.scale_twist.angular.x * joy_data.axes[2]
            self.input_twist.angular.y = self.scale_twist.angular.y * joy_data.axes[5]
            self.input_twist.angular.z = self.scale_twist.angular.z * joy_data.axes[4]

# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('input_generator', anonymous=True)
    rate_mgr = rospy.Rate(10)

    js_input_gen = JoyStickInputGen()
    try:
        js_input_gen.initialize_publishers()
        js_input_gen.initialize_subscriber()
        _header = Header()

        js_input_gen.run_input_method_switch_server()

        print('Run input_gen')
        while not rospy.is_shutdown():
            _header.stamp = rospy.Time.now()
            if js_input_gen.twist_mode_flag:
                js_input_gen.switch_twist_input(_header)
            else:
                js_input_gen.switch_pose_input(_header)
            rate_mgr.sleep()
    except rospy.ROSInterruptException, e:
        print("fail!!: " + str(e))



