#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys

import rospy
from std_msgs.msg import String 
import intera_interface

print_string = ''
def right(nav_name = 'right'):

    #rospy.init_node('navigator_comm', anonymous=True)
    pub = rospy.Publisher('/right_navigator_button', String, queue_size = 10)
    nav = intera_interface.Navigator()

    def back_pressed(v):
    	print ("Button 'Back': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Back': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)


    def rethink_pressed(v):
        print ("Button 'Rethink': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Rethink': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def circle_pressed(v):
        print ("Button 'Circle': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Circle': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def square_pressed(v):
        print ("Button 'Square': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Square': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def x_pressed(v):
        print ("Button 'X': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'X': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def ok_pressed(v):
        print ("Button 'OK': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'OK': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def wheel_moved(v):
        print ("Wheel value: {0}".format(v))
        print_string = "Wheel value: {0}".format(v)
        pub.publish(print_string)
        rospy.loginfo(print_string)

    nav.register_callback(back_pressed, '_'.join([nav_name, 'button_back']))
    nav.register_callback(rethink_pressed, '_'.join([nav_name, 'button_show']))
    nav.register_callback(circle_pressed, '_'.join([nav_name, 'button_circle']))
    nav.register_callback(square_pressed, '_'.join([nav_name, 'button_square']))
    nav.register_callback(x_pressed, '_'.join([nav_name, 'button_triangle']))
    nav.register_callback(ok_pressed, '_'.join([nav_name, 'button_ok']))
    nav.register_callback(wheel_moved, '_'.join([nav_name, 'wheel']))

    print ("Press input buttons on the right navigator, "
           "input will be echoed here.")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
		rate.sleep()


def head(nav_name = 'head'):
    

    #rospy.init_node('navigator_comm', anonymous=True)
    pub = rospy.Publisher('/head_navigator_button', String, queue_size = 10)
    nav = intera_interface.Navigator()


    def back_pressed(v):
    	print ("Button 'Back': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Back': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def rethink_pressed(v):
        print ("Button 'Rethink': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Rethink': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def circle_pressed(v):
        print ("Button 'Circle': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Circle': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def square_pressed(v):
        print ("Button 'Square': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'Square': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def x_pressed(v):
        print ("Button 'X': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'X': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def ok_pressed(v):
        print ("Button 'OK': {0}".format(nav.button_string_lookup(v)))
        print_string = "Button 'OK': {0}".format(nav.button_string_lookup(v))
        pub.publish(print_string)
        rospy.loginfo(print_string)

    def wheel_moved(v):
        print ("Wheel value: {0}".format(v))
        print_string = "Wheel value: {0}".format(v)
        pub.publish(print_string)
        rospy.loginfo(print_string)

    nav.register_callback(back_pressed, '_'.join([nav_name, 'button_back']))
    nav.register_callback(rethink_pressed, '_'.join([nav_name, 'button_show']))
    nav.register_callback(circle_pressed, '_'.join([nav_name, 'button_circle']))
    nav.register_callback(square_pressed, '_'.join([nav_name, 'button_square']))
    nav.register_callback(x_pressed, '_'.join([nav_name, 'button_triangle']))
    nav.register_callback(ok_pressed, '_'.join([nav_name, 'button_ok']))
    nav.register_callback(wheel_moved, '_'.join([nav_name, 'wheel']))

    print ("Press input buttons on the head navigator, "
           "input will be echoed here.")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and i <10:
        rate.sleep()