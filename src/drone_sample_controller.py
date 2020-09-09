#!/usr/bin/env python

import ConfigParser
import os
import rospy
import subprocess
import threading
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def takeoff():

    rospy.loginfo("Taking Off")
    takeoff = rospy.Publisher('drone/takeoff', Empty, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        takeoff.publish()
        if stop_takingoff:
            break
        rate.sleep()

def fly():

        rospy.loginfo("Flying")
        move = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        circle_command = Twist()
        circle_command.linear.x = 1.0
        circle_command.linear.y = 0.0
        circle_command.linear.z = 0.0

        circle_command.angular.x = 0.0
        circle_command.angular.y = 0.0
        circle_command.angular.z = 0.4

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            move.publish(circle_command)
            if stop_flying:
                circle_command.linear.x = 0.0
                circle_command.angular.z = 0.0
                move.publish(circle_command)
                break
            rate.sleep()

def land():

    rospy.loginfo("Landing")
    land = rospy.Publisher('drone/land', Empty, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        land.publish()
        if stop_landing:
            break
        rate.sleep()

    rospy.loginfo("Finished")

rospy.init_node('drone_controller', anonymous=False)
rospy.loginfo("Node initialized")

# rospy.loginfo("Parsing Config")
# dirname = os.path.dirname(__file__) + '/../'
# configParser = ConfigParser.RawConfigParser()
# configFilePath = dirname + 'src/config.config'
# configParser.read(configFilePath)
# rospy.loginfo("Parsing completed")
#
# rospy.loginfo('Waiting for flight payment, press Ctrl+\ to interrupt')
#
# program =  configParser.get('keys_and_addresses', 'ROBONOMICS_DIR') + "/robonomics io read launch"
# process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
# while True:
#     try:
#         output = process.stdout.readline()
#         if output.strip() == configParser.get('keys_and_addresses', 'EMPLOYER_ADDRESS') + " >> " + configParser.get('keys_and_addresses', 'DRONE_ADDRESS') + " : true":
#             rospy.loginfo("Flight Paid!")
#             process.kill()
#             break
#         if output.strip():
#             rospy.loginfo("Not my flight is paid!")
#     except KeyboardInterrupt:
#         exit()


takingoff = flying = threading.Thread(target=takeoff)
flying = threading.Thread(target=fly)
landing = threading.Thread(target=land)
stop_takingoff = False
stop_flying = False
stop_landing = False


takingoff.start()
time.sleep(1)
stop_takingoff = True
takingoff.join()

flying.start()
time.sleep(10)
stop_flying = True
flying.join()

landing.start()
time.sleep(1)
stop_landing = True
landing.join()
