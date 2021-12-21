#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import zmq
import math
import numpy as np
import rospy

from watchplant_demo.msg import SensorData


class ZMQtoROS(object):
    def __init__(self):

        # Create a ZMQ subscriber.
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.bind("tcp://*:5556")
        self.socket.subscribe("")

        # Create a ROS publisher.
        self.pub = rospy.Publisher('plant_data', SensorData, queue_size=1)

        while not rospy.is_shutdown():
            header, additionalSensors, payload = self.receive()
            sender = header['name']
            msg_type = header['msg_type']

            rospy.logdebug("Incoming data from node %s:\n%s", sender, payload)

            # MU data header
            if msg_type == 0:
                payload = payload.split('\r\n')
                rospy.loginfo("Measurement started on node %s at %s", sender, payload[2].split()[1])
                rospy.loginfo("Measurement unit ID: %s", payload[3].split()[1])
            # MU data
            elif msg_type == 1:
                self.parse_data(sender, additionalSensors, payload)
            # MU data/ID/measurement mode
            elif msg_type == 2:
                self.parse_data(sender, additionalSensors, payload)
                rospy.loginfo("Node %s reporting: MU ID is %s, current measurement mode is: %s",
                             sender, payload[3], payload[2])
            # Unknown
            else:
                rospy.logwarn("Unknown message type: %d. Payload:\n%s", msg_type, payload)

    def parse_data(self, sender, additionalSensors, payload):
        # Read and format the data.
        data = [sender] + payload.tolist()
        rospy.logdebug(" \n%s", data)

        msg = SensorData()
        msg.header.stamp = rospy.Time.now()
        # PCB temperature
        msg.pcb_temp = data[19] / 10000     # Degrees Celsius
        # Magnetic fields
        msg.mag.x = data[21] / 1000 * 100   # Micro Tesla
        msg.mag.y = data[22] / 1000 * 100   # Micro Tesla
        msg.mag.z = data[23] / 1000 * 100   # Micro Tesla
        msg.mag_total = math.sqrt(msg.mag.x ** 2 + msg.mag.y ** 2 + msg.mag.z ** 2)
        # Air conditions
        msg.air_temp = data[27] / 10000     # Degrees Celsius
        msg.air_hum = (data[29] * 3 / 4200000 - 0.1515) / (0.006707256 - 0.0000137376 * msg.air_temp)  # Percent
        msg.air_press = data[35] / 100      # Milibars
        # Other external conditions
        msg.light = data[28] / 799.4 - 0.75056  # Lux
        msg.rf_power = data[32]                 # UNKNOWN
        # Differential potentials
        msg.diff_pot_CH1 = data[30] - 512000         # Micro Volts
        msg.diff_pot_CH2 = data[31] - 512000         # Micro Volts
        # Plant conditions
        msg.transpiration = data[33] / 1000 # Percent
        # Soil conditions
        msg.soil_moist = data[36]           # UNKNOWN
        msg.soil_temp = data[37] / 10       # Degrees Celsius

        self.pub.publish(msg)

    def receive(self, flags=0, copy=True, track=False):
        """
        Receive a custom multipart message. Refer to readme for more information.
        """
        header = self.socket.recv_json(flags=flags)
        additionalSensors = False
        if header['msg_type'] == 0:
            payload = self.socket.recv_string(flags=flags)
        else:
            if header['add_sensor']:
                additionalSensors = self.socket.recv_json(flags=flags)
            payload = self.recv_array(flags=flags)
        return header, additionalSensors, payload

    def recv_array(self, flags=0, copy=True, track=False):
        """Receive and desiralize np arrays."""
        md = self.socket.recv_json(flags=flags)
        message = self.socket.recv(flags=flags, copy=copy, track=track)
        array = np.frombuffer(message, dtype=md['dtype'])
        return array.reshape(md['shape'])


if __name__ == "__main__":
    rospy.init_node("zmq_ros_bridge")

    try:
        node = ZMQtoROS()
    except rospy.ROSInterruptException:
        pass
