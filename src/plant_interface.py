#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import math
import time
import datetime
import numpy as np
import rospy
from cybres_mu import Cybres_MU

from std_msgs.msg import Float32
from watchplant_demo.msg import SensorData
from watchplant_demo.srv import SetValue, SetValueRequest


class PlantInterface(object):
    def __init__(self, port='/dev/ttyACM0', baudrate=460800, meas_interval=1000):
        self.mu = Cybres_MU(port, baudrate)
        self.measurment_interval = meas_interval
        self.hostname = 'local'
        self.mu_id = 0
        self.mu_mm = 0

        self.moist_was_low = True
        self.temp_was_low = True
        self.light_was_low = True

        # Publishers.
        self.data = SensorData()
        self.pub = rospy.Publisher('plant_data', SensorData, queue_size=1)

        # Subscribers.
        self.current_color = '111'
        rospy.Subscriber('value', Float32, self.callback, queue_size=1)

        rospy.on_shutdown(self.shutdown)

        # Measure at set interval.
        self.mu.set_measurement_interval(self.measurment_interval)
        self.mu.start_measurement()

        # Main loop.
        while not rospy.is_shutdown():
            # Get the next data set
            next_line = self.mu.get_next()
            header, payload = self.classify_message(next_line)
            sender, msg_type = header

            rospy.logdebug("Incoming data from node %s:\n%s", sender, payload)

            # MU data header
            if msg_type == 0:
                payload = payload.split('\r\n')
                rospy.loginfo("Measurement started on node %s at %s", sender, payload[2].split()[1])
                rospy.loginfo("Measurement unit ID: %s", payload[3].split()[1])
            # MU data
            elif msg_type == 1:
                self.parse_data(sender, payload)
            # MU data/ID/measurement mode
            elif msg_type == 2:
                self.parse_data(sender, payload)
                rospy.loginfo("Node %s reporting: MU ID is %s, current measurement mode is: %s",
                              sender, payload[3], payload[2])
            # Unknown
            else:
                rospy.logwarn("Unknown message type: %d. Payload:\n%s", msg_type, payload)


    def callback(self, msg):
        if msg.data < 30 or msg.data >= 330:
            self.mu.set_color('100')
        elif 30 <= msg.data < 70:
            self.mu.set_color('110')
        elif 70 <= msg.data < 160:
            self.mu.set_color('010')
        elif 160 <= msg.data < 210:
            self.mu.set_color('011')
        elif 210 <= msg.data < 280:
            self.mu.set_color('001')
        elif 280 <= msg.data < 330:
            self.mu.set_color('101')

    def send_color(self, color):
        self.mu.set_color(color)
        values = {'100': 0, '110': 60, '010': 120, '011': 180, '001': 210, '101': 240}
        try:
            rospy.wait_for_service('set_value')
            service_func = rospy.ServiceProxy('set_value', SetValue)
            req = SetValueRequest()
            req.value = values[color]
            resp = service_func(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed with error: %s', e)

    def shutdown(self):
        """
        Perform final clean up on shutdown.
        """
        rospy.loginfo("Measurement stopped at %s.", datetime.datetime.now().strftime("%d.%m.%Y. %H:%M:%S"))
        _ = self.mu.return_serial()
        self.mu.stop_measurement()
        time.sleep(2)
        self.mu.restart()
        self.mu.ser.close()

    def classify_message(self, mu_line):
        """
        Determines the message type.

        Args:
            mu_line (str): Complete MU data line

        Returns:
            A tuple containing a header and payload for the MQTT message.
        """
        counter = mu_line.count('#')
        if counter == 0:
            # Line is pure data message.
            messagetype = 1
            transfromed_data = PlantInterface.transform_data(mu_line)
            # ID and MM are manually added.
            payload = np.append([self.mu_mm, self.mu_id], transfromed_data)

        elif counter == 2:
            # Line is data message/id/measurement mode.
            # Every 100 measurements the MU sends also its own ID and measurement mode.
            messagetype = 2
            messages = mu_line.split('#')
            mu_id = int(messages[1].split(' ')[1])
            mu_mm = int(messages[2].split(' ')[1])
            # ID and mm get attached at the back of the data array
            payload = np.append([mu_mm, mu_id], PlantInterface.transform_data(messages[0]))

        elif counter == 4:
            # Line is header.
            messagetype = 0
            payload = mu_line
            # ID and MM are saved from the header.
            lines = mu_line.split('\r\n')
            self.mu_id = int(lines[3].split()[1])
            self.mu_mm = int(lines[4].split()[1])
        else:
            rospy.logwarn("Unknown data type: \n%s", mu_line)
            return None, None

        header = (self.hostname, messagetype)
        return header, payload

    def parse_data(self, sender, payload):
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
        msg.air_press = data[35] / 100      # Millibars
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
        self.vis_data(msg)

    def vis_data(self, data):
        if data.soil_moist > 350 and self.moist_was_low:
            self.send_color('010')
            self.moist_was_low = False
        elif data.soil_moist < 300 and not self.moist_was_low:
            self.send_color('110')
            self.moist_was_low = True

        if data.light > 1000 and self.light_was_low:
            self.send_color('000')
            self.light_was_low = False
        elif data.light < 800 and not self.light_was_low:
            self.send_color('111')
            self.light_was_low = True

        if data.air_temp > 30 and self.temp_was_low:
            self.send_color('100')
            self.temp_was_low = True
        elif data.air_temp < 27 and not self.temp_was_low:
            self.send_color('001')
            self.temp_was_low = False

    @staticmethod
    def transform_data(string_data):
        """
        Transform MU data from string to numpy array.

        Args:
            string_data (str): MU data in string format.

        Returns:
            A numpy array containing the MU data
        """
        split_data = string_data.split(' ')
        timestamp = [int(time.mktime(datetime.datetime.now().timetuple()))]
        measurements = [int(elem) for elem in split_data[1:]]
        return np.array(timestamp + measurements)

if __name__ == "__main__":
    rospy.init_node("plant_interface")

    try:
        node = PlantInterface()
    except rospy.ROSInterruptException:
        pass
