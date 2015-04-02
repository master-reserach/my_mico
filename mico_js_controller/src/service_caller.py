#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABCMeta, abstractmethod

import rospy
from sensor_msgs.msg import Joy
from jaco_msgs.srv import Start, Stop, HomeArm
from mico_js_controller.srv import InputMethodSwitchService


class ServiceCaller(object):
    __metaclass__ = ABCMeta

    start_arm_servicename = 'start_arm'
    stop_arm_servicename = 'stop_arm'
    home_arm_servicename = 'home_arm'
    switch_mode_servicename = 'switch_mode'

    def __init__(self, data_from_device_topic_name, data_from_device_topic_class):
        self.topic_name = data_from_device_topic_name
        self.topic_class = data_from_device_topic_class

        print('Declare service_caller')
        rospy.wait_for_service(self.start_arm_servicename)
        rospy.wait_for_service(self.stop_arm_servicename)
        rospy.wait_for_service(self.home_arm_servicename)
        rospy.wait_for_service(self.switch_mode_servicename)

        self.start_arm_srv = rospy.ServiceProxy(self.start_arm_servicename, Start)
        self.stop_arm_srv = rospy.ServiceProxy(self.stop_arm_servicename, Stop)
        self.home_arm_srv = rospy.ServiceProxy(self.home_arm_servicename, HomeArm)
        self.switch_mode_srv = rospy.ServiceProxy(self.switch_mode_servicename, InputMethodSwitchService)
        print('Declared service_caller')

    def initialize_subscriber(self):
        self.sub_data_from_device = rospy.Subscriber(self.topic_name, self.topic_class, self.data_from_device_callback)

    def data_from_device_callback(self, data):
        print('subscribe!')
        self.call_service(data)

    @abstractmethod
    def call_service(self, data):
        pass


class JoyStickSrvCaller(ServiceCaller):
    def __init__(self):
        super(JoyStickSrvCaller, self).__init__('joystick', Joy)
        self.can_switch_mode = True
        self.can_start_arm = True

        self.twist_pos_switch_cnt = 0
        self.pos_ori_switch_cnt = 0

    def call_service(self, joy_data):
        print(str(joy_data.buttons[8]) + str(joy_data.buttons[9]))

        if (self.can_switch_mode is True) and joy_data.buttons[11] == 1:
            self.pos_ori_switch_cnt += 1
            self.switch_mode_srv((self.twist_pos_switch_cnt % 2) + 1, (self.pos_ori_switch_cnt % 2) + 1)
            self.can_switch_mode = False
        elif (self.can_switch_mode is False) and joy_data.buttons[11] == 0:
            self.can_switch_mode = True

        if (self.can_start_arm is True) and joy_data.buttons[8] == 0 and joy_data.buttons[9] == 1:
            self.start_arm_srv()
            self.can_start_arm = False
        elif (self.can_start_arm is False) and joy_data.buttons[8] == 0 and joy_data.buttons[9] == 1:
            self.stop_arm_srv()
            self.can_start_arm = True
        elif (self.can_start_arm is False) and joy_data.buttons[8] == 1 and joy_data.buttons[9] == 1:
            self.home_arm_srv()

# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node('service_caller')
    js_srv_caller = JoyStickSrvCaller()

    try:
        js_srv_caller.initialize_subscriber()
        print('Run service_caller')
        rospy.spin()
    except rospy.ServiceException, e:
        print("fail!!: " + str(e))
















