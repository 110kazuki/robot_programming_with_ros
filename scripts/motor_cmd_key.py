#!/usr/bin/env python

import rospy
import std_msgs.msg import Int16
import keyboard

def gen_cmd():
    motor_cmd = 0
    step      = 5

    pub = rospy.Publisher('motor_ctrl', Int16, queue_size=10)
    rospy.init_node('motor_cmd_key', anonymous=True)
    r = rospy.Rate(100) #Hz

    while not rospy.is_shutdown():
        if keyboard.read_key() == "up":
            if motor_cmd < 100:
                motor_cmd = motor_cmd + step
        if keyboard.read_key() == "down":
            if motor_cmd > -100:
                motor_cmd = motor_cmd - step
        if keyboard.read_key() == "space":
            motor_cmd = 0
        pub.publish(motor_cmd)
        status_print(motor_cmd)
        r.sleep()

def status_print(motor_cmd_param):
    print('==============================')
    print('motor speed : %d [%]' %motor_cmd_param)


if __name__ == '__main__':
    try:
        gen_cmd()
    except rospy.ROSInterruptException: pass