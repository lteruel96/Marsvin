#!/usr/bin/env python
import rospy
#import cv2
import os
import time
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from exomy.msg import RoverCommand
from locomotion_modes import LocomotionMode


# Define globals
global tracking_enabled
tracking_enabled = False

def callbackA(message):

    global tracking_enabled

    rover_cmd_t = RoverCommand()

    if (message.locomotion_mode == LocomotionMode.TRACKING.value) and (tracking_enabled == False):
        tracking_enabled = True
        rospy.loginfo("Tracking enabled!")

    elif (message.locomotion_mode != LocomotionMode.TRACKING.value) and (tracking_enabled == True):
            tracking_enabled = False
            rospy.loginfo("Tracking disabled!")

    if (tracking_enabled == False):
        rover_cmd_t = message

    else:
        rover_cmd_t.connected = message.connected
        rover_cmd_t.motors_enabled = message.motors_enabled
        rover_cmd_t.locomotion_mode = message.locomotion_mode
        #Provisional
        rover_cmd_t.vel = 0
        rover_cmd_t.steering = 0
        
    robot_pub.publish(rover_cmd_t)



def callbackB(data):

    global tracking_enabled

    if (tracking_enabled == True):
        rospy.loginfo("In!")
        # bridge = CvBridge()
        # #rospy.loginfo("In!")
        # try:
        #     cv_image = bridge.imgmsg_to_cv2(data)
        #     #rospy.loginfo("Taken!")
        # except CvBridgeError as e:
        #     #rospy.loginfo("Error!")
        #     print(e)

        # (rows, cols, channels) = cv_image.shape
        # print("Rows: {r}, Cols: {c}, Channels: {ch}".format(r=rows, c=cols, ch=channels))
        # #Send the picture to the model perform detection
        # classIds, confs, bbox = net.detect(cv_image,confThreshold=thres)
        # #Check that it has detected something and that it is a person
        # if len(classIds) != 0:
        #     #Create bounding box and write name
        #     for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
        #         if classId==1: #ClassID = 1 -> person
        #             print("Person detected!")
        #             # cv2.rectangle(cv_image,box,color=(0,255,0),thickness=3)
                    
        #             #Calc. width
        #             width = box[2] - box[0]
        #             #Decide if the robot should go forward
        #             if width < 100:
        #                 width = 10
        #                 print("Adelante!")
        #             else:
        #                 print("Stop!")
                    
        #             #Calc. position
        #             midX = ((box[0] + box[2])/2)-320
        #             mid = 220
        #             if -mid<midX<mid:
        #                 print("Middle!", width)
        #             elif midX>mid:
        #                 print("Right!", width)
                          
        #             else:
        #                 print("Left!", width)



if __name__ == '__main__':
    global pub

    rospy.init_node('tracking_node')
    rospy.loginfo('tracking_node started')

    #New
    # OBJECT DETECTION
    rospy.loginfo('loading object-detector-model')
    # Object detection threshold
    thres = 0.5
    # Coco-dataset names import them automatically into an array
    classNames = []
    classFile = os.path.dirname(os.path.realpath(__file__))+'/model/coco.names'
    with open(classFile,'rt') as f:
        classNames = f.read().rstrip('\n').split('\n')
    # Configuration file
    configPath = os.path.dirname(os.path.realpath(__file__))+'/model/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
    weightsPath = os.path.dirname(os.path.realpath(__file__))+'/model/frozen_inference_graph.pb'
    # Load the model - not working
    # net = cv2.dnn_DetectionModel(weightsPath, configPath)
    # net.setInputSize(320,320)
    # net.setInputScale(1.0/127.5)
    # net.setInputMean((127.5, 127.5, 127.5))
    # net.setInputSwapRB(True)

    # Wait to let the camera warmup
    time.sleep(0.1)
    #New

    #callbackA >> callbackB
    joy_sub = rospy.Subscriber("/rover_command", RoverCommand, callbackA, queue_size=1)
    sub_cam = rospy.Subscriber("/pi_cam/image_raw", Image, callbackB, queue_size=1)
    
    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/rover_command_t", RoverCommand, queue_size=1)
    
    rospy.spin()