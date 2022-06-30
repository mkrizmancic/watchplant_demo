#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import rospy

from watchplant_demo.srv import SetValue, SetValueRequest


def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=.1)

    print('[SCRIPT]> Waiting for device')
    print(f'[SCRIPT]> Connected to {ser.name}\n')

    rospy.init_node("ble_reader")

    while not rospy.is_shutdown():
        val = str(ser.readline().decode().strip('\r\n'))  # Capture serial output as a decoded string
        if val:
            print(f"[RECV]> {val}", end="\n", flush=True)

            try:
                rospy.wait_for_service('set_value')
                service_func = rospy.ServiceProxy('set_value', SetValue)
                req = SetValueRequest()
                req.value = float(val)
                service_func(req)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr('Service call failed with error: %s', e)
            except ValueError:
                print("[SCRIPT]> Wrong command!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass