#!/usr/bin/env python

import ConfigParser #to parse config file with drone and employer's addresses and keys
import cv2 #for image converting
import ipfshttpclient #to send data to IPFS
import os #to locate files
import rospy #Python client library for ROS
import subprocess #to call shell commands from terminal and use robonomics binary
import threading #threading to publish topics
import time #to sleep

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist #message type for /cmd_vel
from sensor_msgs.msg import Image #message type for /drone/front_camera/image_raw
from std_msgs.msg import Empty #message type for /drone/takeoff and /drone/land

def takeoff():
    rospy.loginfo("Taking Off")
    takeoff = rospy.Publisher('drone/takeoff', Empty, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        takeoff.publish()
        if stop_takingoff:
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

def take_pictures():
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if stop_taking_pictures:
            break
        rospy.Subscriber("/drone/front_camera/image_raw", Image, image_callback)
        rate.sleep()

def image_callback(msg):
    global i
    global dirname
    if (i - time.time() < -1):
        i = time.time()
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(dirname + 'src/drone_images/front_camera_image' + str(time.time()) + '.jpeg', cv2_img)
            rospy.loginfo("Image saved!")
        except CvBridgeError, e:
            print(e)

rospy.init_node('drone_controller', anonymous = False)
rospy.loginfo('Node initialized')

global i
global dirname
i = time.time()
bridge = CvBridge()

#waiting for transaction
rospy.loginfo("Parsing Config")
dirname = os.path.dirname(__file__) + '/../'
configParser = ConfigParser.RawConfigParser()
configFilePath = dirname + 'src/config.config'
configParser.read(configFilePath)
rospy.loginfo("Parsing Completed")

rospy.loginfo("Creating directory for pictures")
os.mkdir(dirname + 'src/drone_images')

rospy.loginfo("Waiting for flight payment")

program = configParser.get('key_and_addresses', 'ROBONOMICS_DIR') + "/robonomics io read launch" #that's the bash command to launch Robonomics IO and read the transactions
process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
while True:
    try:
        output = process.stdout.readline()
        if output.strip() == configParser.get('key_and_addresses', 'EMPLOYER_ADDRESS') + " >> " + configParser.get('key_and_addresses', 'DRONE_ADDRESS') + " : true": #checking the correct payment to the drone address
            rospy.loginfo("Flight Paid!")
            process.kill()
            break #after that the script will continue running
        if output.strip():
            rospy.loginfo("Not my flight is paid!")
    except KeyboardInterrupt:
        process.kill()
        exit


takingoff = threading.Thread(target=takeoff)
flying = threading.Thread(target=fly)
landing = threading.Thread(target=land)
taking_pictures = threading.Thread(target=take_pictures)

stop_takingoff = False
stop_flying = False
stop_landing = False
stop_taking_pictures = False #flages used to stop threads

taking_pictures.start()
rospy.loginfo("Started taking pictures")

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

stop_taking_pictures = True
taking_pictures.join()

rospy.loginfo("Pushing files to IPFS")
try:
    client = ipfshttpclient.connect()
    res = client.add(dirname + 'src/drone_images', recursive=True)
except Exception, e:
    print(e)
rospy.loginfo ("Files pushed. IPFS hash is " + res[-1].values()[0].encode('utf8'))

rospy.loginfo("Removing directory")
try:
    piclist = [f for f in os.listdir(dirname + 'src/drone_images')]
    for f in piclist:
        os.remove(os.path.join(dirname + 'src/drone_images', f))
    os.rmdir(dirname + 'src/drone_images')
except Exception, e:
    print(e)

rospy.loginfo ("Publishing IPFS hash to chain")
try:
    program = "echo \"" + res[-1].values()[0].encode('utf8') + "\" | " + configParser.get('key_and_addresses', 'ROBONOMICS_DIR') + "/robonomics io write datalog -s " + configParser.get('key_and_addresses', 'DRONE_KEY')
    process = subprocess.Popen(program, shell=True, stdout=subprocess.PIPE)
    output = process.stdout.readline()
    rospy.loginfo("Published data to chain. Transaction hash is " + output.strip())
except Exception, e:
    print(e)
rospy.loginfo("Job done. Check DAPP for IPFS data hash")
