#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
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
    
## For SIMULATION 
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.infrared_camera = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserRange)    
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

    self.twist = Twist()
    self.laser.ranges = []
    self.laser = LaserScan()
    

  def laserRange(self, data):
      self.laser = data
  
  def whatColour(self, image, cx, cy):
      # get Hue Value
      hueVal = image[cx, cy, 0]
      # Red
      if 0 < hueVal <= 5:
          self.redFound = True
          print 'Red Found!'
      # Green 
      elif 50 < hueVal <= 60:
          self.greenFound = True
          print 'Green Found!'
      # Blue
      elif 120 < hueVal <= 140: 
          self.blueFound = True
          print 'Blue Found!'
      # Yellow
      elif 30 < hueVal <= 40:
          self.yellowFound = True
          print 'Yellow Found!'
  
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    ## Arrays to hold values 
    upperBoundColours = [[50,  100, 100], [120, 100, 100], [30, 100, 100], [0, 100, 100]]
    lowerBoundColours = [[60, 255, 255], [140, 255, 250], [40, 255, 255], [5, 255, 255]]
    
    # initialise combined mask cv2 object
    combinedMasks = cv2.inRange(hsv, numpy.array([180, 255, 255]),numpy.array([180, 255, 255]))
    
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
        lower_green = numpy.array(upperBoundColours[0])
        upper_green = numpy.array(lowerBoundColours[0])
        
        # set to colour specific mask
        maskGreen = cv2.inRange(hsv, lower_green, upper_green)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskGreen)
    
    ## if Blue is false, append to mask for view
    if self.blueFound == False:
        
        # Set bounds for colour
        lower_blue = numpy.array(upperBoundColours[1])
        upper_blue = numpy.array(lowerBoundColours[1])
        
        # set to colour specific mask
        maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)    
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskBlue)
    
    ## if Yellow is false, append to mask for view
    if self.yellowFound == False:
        
        # Set bounds for colour
        lower_yellow = numpy.array(upperBoundColours[2])
        upper_yellow = numpy.array(lowerBoundColours[2])
        
        # set to colour specific mask
        maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskYellow)

    ## if Red is false, append to mask for view
    if self.redFound == False:
            
        # Set bounds for colour        
        lower_red = numpy.array(upperBoundColours[3])
        upper_red = numpy.array(lowerBoundColours[3])
        
        # set to colour specific mask
        maskRed = cv2.inRange(hsv, lower_red, upper_red)
        
        # Append colour to mask array
        combinedMasks = cv2.add(combinedMasks, maskRed)
        
########################################################################################
        
## Print message if NO coloured monuments found
    ## Not syntactically correct (everything needs to equal False)
    if self.greenFound and self.blueFound and self.yellowFound and self.redFound == False:
        print "No monuments found."
        
## Print message if ALL coloured monuments found
    ## Not syntactically correct (everything needs to equal True)
    if self.greenFound and self.blueFound and self.yellowFound and self.redFound == True:
        print "All monuments found. End of session."  

########################################################################################

    h, w, d = image.shape
    
## Trying to see colours on screen 
    M = cv2.moments(combinedMasks)
## Use distance metric as a measure of depth using lasers
    Distance = self.laser.ranges
        
    if M['m00'] > 0:
      if min(Distance) > 0.8 or math.isnan(min(Distance)):  
          
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
          self.whatColour(hsv, cx, cy)
          
# END CONTROL
    cv2.imshow("window", image)
    cv2.imshow("window", combinedMasks)
    cv2.waitKey(1)   
    
    # Class selector used to run class
if __name__ == "__main__":
    W=Follower()
    rospy.spin()    
    
    