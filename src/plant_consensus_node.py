#!/usr/bin/env python
# -*- coding: utf-8 -*-
import random
try:
    import queue
except ImportError:
    import Queue as queue
import threading

import rospy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse
from watchplant_demo.srv import SetValue, SetValueResponse
from watchplant_demo.msg import SignedFloat


class SimpleConsensusNode(object):
    def __init__(self):
        # Initialize class from ROS parameters.
        self.config = rospy.get_param('/consensus_params')
        self.num_of_agents = len(self.config['mapping'])

        self.name = rospy.get_namespace().strip('/')
        self.name = 'local'
        self.index = int(self.config['mapping'].index(self.name))
        self.alpha = self.config['alpha']
        self.fixed = False
        self.mutex = threading.Lock()
        self.value = 300
        self.other_values = {agent: self.value for agent in self.config['mapping']}
        del self.other_values[self.name]

        # Create a publisher.
        pub = rospy.Publisher('value', Float32, queue_size=1)
        pubg = rospy.Publisher('/all_values', SignedFloat, queue_size=self.num_of_agents)

        # Create subscribers.
        self.message_queue = queue.Queue()
        for connected, to in zip(self.config['adjacency'][self.index], self.config['mapping']):
            if connected:
                rospy.Subscriber('/{}/value'.format(to), Float32, self.value_callback, queue_size=3)
        rospy.Subscriber('/all_values', SignedFloat, self.all_callback, queue_size=1)

        # Create services.
        rospy.Service('set_value', SetValue, self.set_value)
        rospy.Service('set_random', SetBool, self.set_random)

        # Main loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            while not self.message_queue.empty():
                other_value = self.message_queue.get()
                with self.mutex:
                    if not self.fixed:
                        self.value = self.value + self.alpha * (other_value - self.value)
                    else:
                        if max(map(lambda x: x - self.value, self.other_values.values())) <= 5:
                            self.fixed = False

            pub.publish(self.value)
            pubg.publish(self.value, self.name)
            rate.sleep()

    def value_callback(self, msg):
        self.message_queue.put(msg.data)

    def all_callback(self, msg):
        if msg.sender != self.name:
            with self.mutex:
                self.other_values[msg.sender] = msg.data

    def set_value(self, req):
        with self.mutex:
            self.value = req.value
            self.fixed = True
        return SetValueResponse()

    def set_random(self, req=None):
        self.fixed = req.data
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
