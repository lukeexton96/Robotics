# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 09:21:05 2017

@author: Lord Baker
"""
import rospy, cv2, cv_bridge, numpy
import math
from std_msgs.msg import Float32
import tf.transformations
import numpy as np
import numpy
import argparse
import cv2
from cv2 import namedWindow, cvtColor, imshow, waitKey
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from cv2 import blur, Canny
from numpy import mean
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal
#---------------------------------------------------------------------------------------------------
wheel_radius = 0.035
robot_radius = 0.23
laser_range_mean = 0.0
#---------------------------------------------------------------------------------------------------
class Follower:          
    
  def __init__(self):
      
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    #cv2.namedWindow("mask", 1)
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel',
                                       Twist, queue_size=1)
    self.scan_pub = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserscan_callback)
    self.twist = Twist()
    self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=1)      
    self.result = rospy.Subscriber('/turtlebot/move_base/result', MoveBaseActionResult,self.result_callback, queue_size=1)
    self.redFound = False
    self.blueFound = False
    self.greenFound = False
    self.yellowFound = False     
    self.searching = False
    rospy.sleep(3)
    
    self.goalArrived = False
    self.goalSent = False
   # self.searching = False   
    
    self.waypoints = [
    [1.5, -4.5],
    [1.5, -1.5],
    [-4, 0],
    [-3, 2.0],
    [-0.5, 2.0],
    [3, 4.5]    
    ]
          
    
  def goto (self, x, y, quat):
      
      message = PoseStamped()
      message.header.frame_id = "/map"
      
      message.pose.position.x = x
      message.pose.position.y = y
      message.pose.position.z = 0
     
      message.pose.orientation.w = 1
      message.pose.orientation.x = quat[1]
      message.pose.orientation.y = quat[2]
      message.pose.orientation.z = quat[3]
      
      
     # self.goal_pub.publish(message)
#      if self.searching == True:
      while self.goalArrived == False:
                 #print "searching"
                 if self.goalSent == False:
                      #self.searching = True
                      self.goal_pub.publish(message)
                      self.goalSent = True
                      print "SENT GOAL"
      
     # self.searching = False 
      self.goalArrived = False
      self.goalSent= False


#------------------------------------------------------------------------------------------------


  def image_callback(self, msg):
    #default image/hsvs for variable
    self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    #Colour Thresholds
    lower_green = numpy.array([ 40, 100,  80])
    upper_green = numpy.array([ 80, 255, 250])
    
    lower_blue = numpy.array([ 110, 100,  100])
    upper_blue = numpy.array([ 130, 255, 255])
    
    lower_yellow = numpy.array([ 30, 100,  100])
    upper_yellow = numpy.array([ 50, 255, 255])
    
    lower_red = numpy.array([ 0, 100,  100])
    upper_red = numpy.array([ 5, 255, 255])
    
    
    self.maskgreen = cv2.inRange(hsv, lower_green, upper_green)
    self.maskblue = cv2.inRange(hsv, lower_blue, upper_blue) 
    self.maskyellow = cv2.inRange(hsv, lower_yellow, upper_yellow)  
    self.maskred = cv2.inRange(hsv, lower_red, upper_red)
    
    self.h, self.w, self.d = self.image.shape
    
    #Masks
    self.MG = cv2.moments(self.maskgreen)
    self.MB = cv2.moments(self.maskblue)
    self.MY = cv2.moments(self.maskyellow)
    self.MR = cv2.moments(self.maskred)
    
#------------------------------------------------------------------------------------------------    
  def search(self):
#        if self.searching == False:
        #print "Laser_range_mean", laser_range_mean
#            print "testing"

            if laser_range_mean > 0.9:
                #If nothing is infront of you then move
                self.twist.angular.x = 0.0
                self.twist.linear.z = 0.0
                
                #r = rospy.Rate(10)
                self.cmd_vel_pub.publish(self.twist)
                #rospy.sleep(3)
                #print "Searching"
        #        
        #       'Find Green
                if self.greenFound == False:
                        print "Searching for green"
                        if self.MG['m00'] > 0:
                            print "GREEN SEEN"
                            self.searching = False
                            cx = int(self.MG['m10']/self.MG['m00'])
                            cy = int(self.MG['m01']/self.MG['m00'])
                            cv2.circle(self.image, (cx, cy), 20, (255,255,255), -1)
                     # BEGIN CONTROL
                            err = cx - self.w/2
                            self.twist.linear.x = 0.2
                            self.twist.angular.z = -float(err) / 100
                     #    print "cx", cx
                      #   print "cy", cy
                            self.cmd_vel_pub.publish(self.twist)
                            if self.MG['m00'] > 10000000:
                                print "GREEN DONE"
                                self.searching = False
                                self.greenFound = True
                                 
                        else:
                            print "Green else statement turning back on"
    #                #spin
                         
                    # END CONTROL
        
                        cv2.imshow("window", self.image)
                        cv2.imshow("maskgreen", self.maskgreen)
                        cv2.waitKey(1)
    #        
    #            #find blue
                if self.blueFound == False:    
                        print "Searching for blue"
                        if self.MB['m00'] > 0:
                            print "BLUE SEEN"
                            #self.searching = False
                            cx = int(self.MB['m10']/self.MB['m00'])
                            cy = int(self.MB['m01']/self.MB['m00'])
                            cv2.circle(self.image, (cx, cy), 20, (255,255,255), -1)
                            # BEGIN CONTROL
                            err = cx - self.w/2
                            self.twist.linear.x = 0.2
                            self.twist.angular.z = -float(err) / 100
                            #print "cy", cy
                            self.cmd_vel_pub.publish(self.twist)
                            print "BLUE MB:", self.MB['m00']
                            if self.MB['m00'] > 10000000:
                                print "BLUE DONE"
                                self.searching = False
                                self.blueFound = True
                                
                        else:
                            #spin
                            print "Blue else statement turning back on"
                            
                    # END CONTROL
                        cv2.imshow("window", self.image)
                        cv2.imshow("maskblue", self.maskblue)
                        cv2.waitKey(1)  
    #                
    #            #find yellow
                if self.yellowFound == False:
                        print "Searching for yellow"
                        if self.MY['m00'] > 0:
                            print "YELLOW SEEN"
                            #self.searching = False
                            cx = int(self.MY['m10']/self.MY['m00'])
                            cy = int(self.MY['m01']/self.MY['m00'])
                            cv2.circle(self.image, (cx, cy), 20, (255,255,255), -1)
                            # BEGIN CONTROL
                            err = cx - self.w/2
                            self.twist.linear.x = 0.2
                            self.twist.angular.z = -float(err) / 100
                            #print "cy", cy
                            self.cmd_vel_pub.publish(self.twist)  
                            print "YELLOW MR:", self.MY['m00']
                            if self.MY['m00'] > 10000000:
                                 print "YELLOW DONE"
                                 self.searching = False
                                 self.yellowFound = True
                                    
                        else:
        #                #spin
                            print "yellow else statement turning search back on"
                            #self.searching = False
                        # END CONTROL
            
                        cv2.imshow("window", self.image)
                        cv2.imshow("maskyellow", self.maskyellow)
                        cv2.waitKey(1)
                    
                    #find red    
                if self.redFound == False:
                        print "Searching for red"
                        if self.MR['m00'] > 0:
                            print "RED SEEN"
                            #self.searching = False
                            cx = int(self.MR['m10']/self.MR['m00'])
                            cy = int(self.MR['m01']/self.MR['m00'])
                            cv2.circle(self.image, (cx, cy), 20, (255,255,255), -1)
                            # BEGIN CONTROL
                            err = cx - self.w/2
                            self.twist.linear.x = 0.2
                            self.twist.angular.z = -float(err) / 100
                            self.cmd_vel_pub.publish(self.twist) 
                            print "RED MR:", self.MR['m00']
                            if self.MR['m00'] > 10000000:
                                print "RED DONE"
                                self.searching = False
                                self.redFound = True
                                #if self.redFound == True:
    #                                print "Turn Searching Back on"
    #                                print self.searching
                                    #self.goal_pub.publish(self.searching)
                                   # self.twist.angular.z = 0.2
                                   # self.twist.linear.x = 0.0
                                   # self.cmd_vel_pub.publish(self.twist)
                                    
                        else:
                        #spin
                            print "red else statement turning search back on"
                            self.searching = False
                        # END CONTROL
                        cv2.imshow("window", self.image)
                        cv2.imshow("maskred", self.maskred)
                        cv2.waitKey(1)   
  
  def laserscan_callback(self, msg):
    global laser_range_mean
    self.laser = msg
  #  print self.laser.ranges[len(self.laser.ranges) / 2]
    laser_range_mean = self.laser.ranges[len(self.laser.ranges) / 2]
    
  def result_callback(self, msg):
      self.goalArrived = True
      #self.searching = False

  

if __name__ == '__main__':
   try:
       rospy.init_node('follower', anonymous=True)
       follower = Follower() 
       count = 0
       for i in follower.waypoints:
           print "omw to waypoint" + str(count)
           count + 1
           
           follower.goto(i[0], i[1], [0,0,0,0])
           follower.searching = True           
           while follower.searching == True:
               follower.search()
               #print follower.search()
                          
       
       print "Completed Journey"
           

       rospy.sleep(1)
       rospy.spin()
       cv2.destroyAllWindows()
   except rospy.ROSInterruptException:
       rospy.loginfo("CTRL + C Caught, QUITTING")