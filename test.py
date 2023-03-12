#!/usr/bin/env python

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
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
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import time
import math
import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from sensor_msgs.msg import LaserScan


OUTER_RANGE_RADIUS = 1


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def get_angle(num):
        return (num * 180) / math.pi


    def scan_callback(data):
            range_center = data.ranges[len(data.ranges)/2] # angle 0
            range_right = data.ranges[len(data.ranges)/4] # angle -3.14/2
            range_left = data.ranges[3 * (len(data.ranges)/4)] # angle 3.14/2
            range_behind = data.ranges[0]   # angle -3.14
            print("center- %0.1f" %range_center," behind - %0.1f" %range_behind, " left - %0.1f" %range_left, " right - %0.1f" %range_right)
            
            scan_count = ((data.scan_time // data.time_increment)) // 2
            min_angle_index = len(data.ranges)/4
            min_angle = math.pi/2
            print(get_angle(min_angle))
            
            debounce = 0 # Counter for if lidar picks up a value outside OUTER_RANGE_RADIUS while scanning box 
            box = [] # Array containing distances/angles lidar pick up of a single box
            
            # Fails if >= 2 boxes are overlapped in view of the lidar
            for i in range(int(scan_count)): 
                if(data.ranges[i + min_angle_index] < OUTER_RANGE_RADIUS and debounce < 5):
                    print(data.ranges[i + min_angle_index])
                    print(get_angle((i * data.angle_increment) + min_angle))
                    box.append(get_angle((i * data.angle_increment) + min_angle))
                    
                    if(len(box)>= 50 and debounce < 5):
                        index = len(box) // 2 
                        midAngle =  get_angle((index * data.angle_increment) + min_angle)
                        print("The angle needed is: " + midAngle)
                        
                    continue
                    
                elif(data.ranges[(i-1) + min_angle_index] < OUTER_RANGE_RADIUS and debounce < 5):
                    debounce += 1
                    
                else:
                    debounce = 0
                
                
                #if(debounce >= 3)
                #    seeing_box = !seeing_box
                #    print(data.ranges[i + min_angle_index])
                #    print(get_angle((i * data.angle_increment) + min_angle))

                


def main():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
