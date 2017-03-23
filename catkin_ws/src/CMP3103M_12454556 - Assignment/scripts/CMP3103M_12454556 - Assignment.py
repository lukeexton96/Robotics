#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import time

wheel_radius = 0.05
robot_radius = 0.25

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    
    # Initialise found variable flags
    self.greenFound = False
    self.blueFound = False
    self.yellowFound = False
    self.redFound = False
    
    # intial global methods and variables
    self.laser = LaserScan()
    self.distance = [0]    
    self.objectFound = False
    self.goalSeeking = True
    self.inSearchMode = False
    self.elapsedScanTime = time.time()
    
    # Initialise global array which contains co-ordinates for all way-points
    self.atPoint = False
    self.goalPositions = [[2.01, -4.04],
                          [0.996, -1.05], 
                          [-2.24, -1.57], 
                          [-3.95, -1.04], 
                          [-3.56, 2.93], 
                          [-3.97, -1.57], 
                          [-0.476, 0.804],
                          [-0.298, 3.93]]
                    
    ## Arrays to hold values 
    self.lowerBoundColours = [[40, 200, 100], [100, 200, 100], [25, 200, 100], [0, 210, 100]]
    self.upperBoundColours = [[60, 255, 255], [120, 255, 255], [30, 255, 255], [4, 255, 255]]
    
    
 ## For SIMULATION  
    self.infrared_camera = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserRange)    
    rospy.sleep(2)
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)
    
    # For Move_Base to set goals/waypoints throughout map
    self.setGoal = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=1)
    self.confirmGoal = rospy.Subscriber('/turtlebot/move_base/result', MoveBaseActionResult, self.goalCallback)
    
    # Initialise Twist() method
    self.twist = Twist()
    
    # Sleep for 3 seconds 
    rospy.sleep(3)

  def laserRange(self, data):
      self.laser = data
     
#####################################################################################     
## Method used to set a way-point on the map for the robot to head to
  def setWaypoint(self, x, y):
      pos = PoseStamped()
      
      pos.header.frame_id = '/map'
      # Set orientation of robot if needed
      pos.pose.orientation.w = 1

      # Position of robot 
      pos.pose.position.x = x
      pos.pose.position.y = y
      pos.pose.position.z = 0
      
      # Publish position
      self.setGoal.publish(pos)
      
#####################################################################################      
## Method used as a callback function from confirmGoal to confirm robot has met goal
  def goalCallback(self, data):
      # Return True once it arrives at goal
      print "At goal!" 
      self.atPoint = True
      self.goalSeeking = False
          
#####################################################################################
## Method fired to check whether Pillar/Monument has been found
  def pillarFound(self, colour, infoFlag):
      # Green 
      if colour == 'green' and infoFlag == False:
          self.greenFound = True
          print 'Green Found!'
          self.atPoint = False
      # Blue
      elif colour == 'blue' and infoFlag == False: 
          self.blueFound = True
          print 'Blue Found!'
          self.atPoint = False
      # Yellow
      elif colour == 'yellow' and infoFlag == False:
          self.yellowFound = True
          print 'Yellow Found!'
          self.atPoint = False
      # Red
      elif colour == 'red' and infoFlag == False:
          self.redFound = True
          print 'Red Found!'
          self.atPoint = False
      else:
          print 'No colour on spectrum found'

######################################################################################  
## Method called to send message that all monuments have been found
  def completeMessage(self):
      self.inSearchMode = False
      self.objectFound = True
      if self.greenFound == True and self.blueFound == True and self.yellowFound == True and self.redFound == True:
          print "All monuments found."

######################################################################################
## Control method used to identify and approach monuments should they appear within the mask
  def control(self, M, image):
      
      h, w, d = image.shape
      
      # If the robot is at the way-point, enter this method
      if self.atPoint == True:
          # If the robot has an object within it's view, enter further
          if M['m00'] > 0:
              # Approach the object until it's distance is 50cm away. 
              if min(self.distance) > 0.5 or math.isnan(min(self.distance)):  
                  
                ## Centre 'x' pixel 
                  cx = int(M['m10']/M['m00'])
                  
                ## BEGIN CONTROL
                  err = cx - w / 2
                  self.twist.linear.x = 0.6
                  self.twist.angular.z = -float(err) / 100
                  self.cmd_vel_pub.publish(self.twist)
          # If object not found, continue to search the map
          else:
                self.searchMap()
              
#######################################################################################  
  def image_callback(self, msg):
     ## error handling
    try:
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except self.bridge, e:
        print e
        
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    
    
    # initialise combined mask cv2 object
    combinedMasks = cv2.inRange(hsv, numpy.array([180, 255, 255]), numpy.array([180, 255, 255]))
    self.distance = 0    
    
    #   Upper and Lower bounds defined as:
    #   [0] = Green, [1] = Blue, [2] = Yellow, [3] = Red
    
    self.distance = self.laser.ranges
    
    # Get middle pixel for colour testing
    middle = (len(self.distance) - 1) / 2
    mid = self.distance[middle]
    
    if math.isnan(mid):
        mid = 100

########################################################################################
## Check if colours have been found    
    ## If Green is false, append to mask for view
    if self.greenFound == False:
        
        # Set bounds for colour
        lower_green = numpy.array(self.lowerBoundColours[0])
        upper_green = numpy.array(self.upperBoundColours[0])
        
        # set to colour specific mask
        maskGreen = cv2.inRange(hsv, lower_green, upper_green)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskGreen)
        
        self.control(cv2.moments(combinedMasks), hsv)
        
        if mid <=1 and cv2.moments(maskGreen)['m00'] > 0:
            self.pillarFound('green', self.greenFound)
            self.greenFound = True
            self.completeMessage()
        
    ## if Blue is false, append to mask for view
    if self.blueFound == False:
        
        # Set bounds for colour
        lower_blue = numpy.array(self.lowerBoundColours[1])
        upper_blue = numpy.array(self.upperBoundColours[1])
        
        # set to colour specific mask
        maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)    
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskBlue)
        
        self.control(cv2.moments(combinedMasks), hsv)
        
        if mid <=1 and cv2.moments(maskBlue)['m00'] > 0:
            self.pillarFound('blue', self.blueFound)
            self.blueFound = True
            self.completeMessage()
    
    ## if Yellow is false, append to mask for view
    if self.yellowFound == False:
        
        # Set bounds for colour
        lower_yellow = numpy.array(self.lowerBoundColours[2])
        upper_yellow = numpy.array(self.upperBoundColours[2])
        
        # set to colour specific mask
        maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskYellow)
        
        self.control(cv2.moments(combinedMasks), hsv)
        
        if mid <=1 and cv2.moments(maskYellow)['m00'] > 0:
            self.pillarFound('yellow', self.yellowFound)
            self.yellowFound = True
            self.completeMessage()

    ## if Red is false, append to mask for view
    if self.redFound == False:
            
        # Set bounds for colour        
        lower_red = numpy.array(self.lowerBoundColours[3])
        upper_red = numpy.array(self.upperBoundColours[3])
        
        # set to colour specific mask
        maskRed = cv2.inRange(hsv, lower_red, upper_red)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskRed)
        
        self.control(cv2.moments(combinedMasks), hsv)
        
        if mid <=1 and cv2.moments(maskRed)['m00'] > 0:
            self.pillarFound('red', self.redFound)
            self.redFound = True
            self.completeMessage()
    
    # CONTROL
    self.control(cv2.moments(combinedMasks), hsv)
    # Show windows of live images
    cv2.imshow("HSV Image", hsv)
    cv2.imshow("Combined Masks", combinedMasks)
    cv2.waitKey(1)
    
####################################################################################
    
  def searchMap(self):
      #do stuff
      if self.inSearchMode == True and not follower.goalSeeking:
              
          #searching logic
          a = Twist()
          a.angular.z = 0.5
          self.cmd_vel_pub.publish(a)
    
####################################################################################

if __name__ == "__main__":
    try:
        rospy.init_node('Colour_Checker', anonymous = False)
        follower = Follower()
        
        #set count to allow for incremental way-point allocation
        count = 0
        # for each item within the goal positions array, set way-point
        for i in follower.goalPositions:
            count + 1
            print "Heading to Waypoint %s" % count

            
            follower.setWaypoint(i[0], i[1])
            follower.foundObject = False
            follower.goalSeeking = True
            while follower.goalSeeking == True:
                # Call function to begin search and stuff
                if follower.atPoint == True:
                    follower.inSearchMode = True
                    follower.elapsedScanTime = time.time()
                if follower.foundObject == True:
                    follower.goalSeeking = False
                    
        
        print "Journey is now complete"
        
        rospy.sleep(1)
        # Keep client running
        rospy.spin() 
        cv2.destroyAllWindows()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("CTRL + C Caught, QUITTING")