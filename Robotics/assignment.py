#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
 
wheel_radius = 0.05
robot_radius = 0.25

class Follower:
  def __init__(self):
    rospy.init_node('follower')
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
        
    self.greenFound = False
    self.blueFound = False
    self.yellowFound = False
    self.redFound = False
    
    self.laser = LaserScan()
    self.distance = [0]    
    
## For SIMULATION 
    self.infrared_camera = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserRange)    
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

    self.twist = Twist()
    self.laser.ranges = []
    

  def laserRange(self, data):
      self.laser = data
  
  def whatColour(self, image, cx, cy):
      # get Hue Value
      hueVal = image[cx, cy, 0]
      # Green 
      if 40 <= hueVal <= 60:
          self.greenFound = True
          return 'Green Found!'
      # Blue
      elif 100 <= hueVal <= 120: 
          self.blueFound = True
          return 'Blue Found!'
      # Yellow
      elif 25 <= hueVal <= 30:
          self.yellowFound = True
          return 'Yellow Found!'
      # Red
      elif 0 <= hueVal <= 4:
          self.redFound = True
          return 'Red Found!'
      else:
          return 'No colour on spectrum found'
  
  def image_callback(self, msg):
    
    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    ## Arrays to hold values 
    lowerBoundColours = [[40, 200, 100], [100, 200, 100], [25, 200, 100], [0, 210, 100]]
    upperBoundColours = [[60, 255, 255], [120, 255, 255], [30, 255, 255], [4, 255, 255]]
    
    # initialise combined mask cv2 object
    combinedMasks = cv2.inRange(hsv, numpy.array([180, 255, 255]), numpy.array([180, 255, 255]))
    self.distance = [0]    
    
    
    #   Upper and Lower bounds defined as
#   [0] = Green
#   [1] = Blue
#   [2] = Yellow
#   [3] = Red

########################################################################################
## Check if colours have been found    
    ## If Green is false, append to mask for view
    if self.greenFound == False:
        
        # Set bounds for colour
        lower_green = numpy.array(lowerBoundColours[0])
        upper_green = numpy.array(upperBoundColours[0])
        
        # set to colour specific mask
        maskGreen = cv2.inRange(hsv, lower_green, upper_green)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskGreen)
    
    ## if Blue is false, append to mask for view
    if self.blueFound == False:
        
        # Set bounds for colour
        lower_blue = numpy.array(lowerBoundColours[1])
        upper_blue = numpy.array(upperBoundColours[1])
        
        # set to colour specific mask
        maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)    
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskBlue)
    
    ## if Yellow is false, append to mask for view
    if self.yellowFound == False:
        
        # Set bounds for colour
        lower_yellow = numpy.array(lowerBoundColours[2])
        upper_yellow = numpy.array(upperBoundColours[2])
        
        # set to colour specific mask
        maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskYellow)

    ## if Red is false, append to mask for view
    if self.redFound == False:
            
        # Set bounds for colour        
        lower_red = numpy.array(lowerBoundColours[3])
        upper_red = numpy.array(upperBoundColours[3])
        
        # set to colour specific mask
        maskRed = cv2.inRange(hsv, lower_red, upper_red)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskRed)
        
    # END CONTROL
    cv2.imshow("HSV Image", hsv)
    cv2.imshow("Combined Masks", combinedMasks)
    cv2.waitKey(1)
        
########################################################################################
        
## Print message if NO coloured monuments found
    ## Not syntactically correct (everything needs to equal False)
   # if self.greenFound and self.blueFound and self.yellowFound and self.redFound == False:
    #    print "No monuments found."
        
## Print message if ALL coloured monuments found
    if self.greenFound == True and self.blueFound == True and self.yellowFound == True and self.redFound == True:
        print "All monuments found. End of session."  

########################################################################################

    h, w, d = image.shape
    
## Trying to see colours on screen 
    M = cv2.moments(combinedMasks)
## Use distance metric as a measure of depth using lasers
    self.distance = self.laser.ranges
        
    if M['m00'] > 0:
      if min(self.distance) > 0.5 or math.isnan(min(self.distance)):  
          
        ## Centre 'x' pixel 
          cx = int(M['m10']/M['m00'])
        ## Centre 'y' pixel
          cy = int(M['m01']/M['m00'])
        ## Draw cirlce on image
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          
        ## BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.6
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
          
          #Check what colour is in front of the robot
          feedback = self.whatColour(hsv, cx, cy)
          print feedback
    
    # Class selector used to run class
if __name__ == "__main__":
    W=Follower()
    rospy.spin()    
    
    