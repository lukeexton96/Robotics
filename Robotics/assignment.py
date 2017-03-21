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
    
    colours = {'Green': False, 'Blue': False, 'Yellow': False, 'Red': False}
    
## For SIMULATION 
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', Image, self.image_callback)
    self.infrared_camera = rospy.Subscriber('/turtlebot/scan', LaserScan, self.laserRange)    
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

    self.twist = Twist()
    self.laser.ranges = []
    self.laser = LaserScan()
   
    
  def laserRange(self, data):
      self.laser = data
      
  def testColour(self, data):
      # test colour
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
## Define colour 'Green' Identifiers
    lower_green = numpy.array([ 50,  100,  100])
    upper_green = numpy.array([180, 255, 255])
    maskGreen = cv2.inRange(hsv, lower_green, upper_green)
    
## Define colour 'Blue' Identifiers
    lower_blue = numpy.array([ 120,  100,  100])
    upper_blue = numpy.array([140, 255, 250])
    maskBlue = cv2.inRange(hsv, lower_blue, upper_blue) 
    
## Define colour 'Yellow' Identifiers
    lower_yellow = numpy.array([ 30,  100,  100])
    upper_yellow = numpy.array([255, 255, 255])
    maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow) 

## Define colour 'red' Identifiers
    lower_red = numpy.array([ 0,  100,  100])
    upper_red = numpy.array([5, 255, 255])
    maskRed = cv2.inRange(hsv, lower_red, upper_red) 

## Set mask to contain only the colours defined above
    masks = maskGreen + maskBlue + maskYellow + maskRed
    
    h, w, d = image.shape
    
## Cut light
    search_top = 3*h/4 - 175
    search_bottom = 3*h/4 + 50
    masks[0:search_top, 0:w] = 0
    masks[search_bottom:h, 0:w] = 0
    
    M = cv2.moments(masks)
    
    Distance = self.laser.ranges
    
## Print distance info    
    #print min(Distance)
    
    if M['m00'] > 0:
      if min(Distance) > 1 or math.isnan(min(Distance)):  
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.6
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
          
# END CONTROL
    cv2.imshow("window", image)
    cv2.imshow("window", masks)
    cv2.waitKey(1)   
    
    # Class selector used to run class
if __name__ == "__main__":
    W=Follower()
    rospy.spin()    
    
    