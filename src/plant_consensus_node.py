#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import random
import numpy as np

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse
from watchplant_demo.srv import SetValue, SetValueResponse
from watchplant_demo.msg import FloatArray


class SimpleConsensusNode(object):
    def __init__(self):
        # Initialize class from ROS parameters.
        self.config = rospy.get_param('/consensus_params')
        self.num_of_agents = len(self.config['mapping'])

        name = rospy.get_namespace().strip('/')
        name = 'local'
        index = int(self.config['mapping'].index(name))
        self.alpha = self.config['alpha']
        self.locked = False
        self.value = 300
        self.all_values = FloatArray()
        self.all_values.data = [self.value] * self.num_of_agents

        # Value visualization with LEDs

        # Create a publisher.
        pub = rospy.Publisher('value', Float32, queue_size=1)
        pubg = rospy.Publisher('/all_values', FloatArray, queue_size=1)

        # Create subscribers.
        for connected, to in zip(self.config['adjacency'][index], self.config['mapping']):
            if connected:
                rospy.Subscriber('/{}/value'.format(to), Float32, self.value_callback, queue_size=3)
        rospy.Subscriber('/all_values', FloatArray, self.all_callback, queue_size=self.num_of_agents)

        # Create services.
        rospy.Service('set_value', SetValue, self.set_value)
        rospy.Service('set_random', SetBool, self.set_random)

        # Main loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(self.value)
            self.all_values.data[index] = self.value
            pubg.publish(self.all_values)
            rate.sleep()

    def value_callback(self, msg):
        if not self.locked:
            self.value = self.value + self.alpha * (msg.data - self.value)

    def all_callback(self, msg):
        self.all_values.data = list(msg.data)


        if np.ptp(self.all_values.data) <= 5:
            self.consensus_achieved = True
            self.locked = False
        else:
            self.consensus_achieved = False

    def set_value(self, req):
        self.value = req.value
        self.locked = True
        return SetValueResponse()

    def set_random(self, req=None):
        self.locked = req.data
        if self.config['initial_values']['range'][2] == 0:
            self.value = random.uniform(*self.config['initial_values']['range'][0:2])
        else:
            self.value = random.randrange(*self.config['initial_values']['range'])
        return SetBoolResponse()


if __name__ == "__main__":
    rospy.init_node("node")

    try:
        node = SimpleConsensusNode()
    except rospy.ROSInterruptException:
        pass
