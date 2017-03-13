#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
 
#wheel_radius = 0.05
#robot_radius = 0.25
#     
#class w4:
#     
#    def forward_kinematics(self,w_l, w_r):
#        c_l = wheel_radius * w_l
#        c_r = wheel_radius * w_r
#        v = (c_l + c_r) / 2
#        a = (c_l - c_r) / robot_radius
#        return (v, a)
#     
#    def inverse_kinematics(self,v, a):
#        c_l = v + (robot_radius * a) / 2
#        c_r = v - (robot_radius * a) / 2
#        w_l = c_l / wheel_radius
#        w_r = c_r / wheel_radius
#        return (w_l, w_r)
# 
#    def inverse_kinematics_from_twist(self,t):
#        return w4.inverse_kinematics(t.linear.x, t.angular.z)   
#         
#    def __init__(self): 
#        rospy.init_node('kinematics')              
#        self.vleft_sub = rospy.Subscriber("/turtlebot_1/wheel_vel_left", Float32, self.callback)
#        self.cmdvel_pub = rospy.Publisher("/turtlebot_1/cmd_vel", Twist)
#         
#    def callback(self, data):
##        print data.data
#         
#        (v, a) = self.forward_kinematics(data.data, 0.0)
#        tmsg = Twist()
#        tmsg.linear.x = v
#        tmsg.angular.z = a
#        self.cmdvel_pub.publish(tmsg)
#        print v,a

class Follower:
  def __init__(self):
    rospy.init_node('follower')
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/turtlebot_1/camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist, queue_size=1)
    self.infrared_camera = rospy.Subscriber('/turtlebot_1/scan', LaserScan, self.laserRange)
#    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
#    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#    self.infrared_camera = rospy.Subscriber('/scan', LaserScan, self.laserRange)
    self.twist = Twist()
    
  def laserRange(self, data):
      self.laser = data
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = numpy.array([ 50,  100,  0])
    upper_green = numpy.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    h, w, d = image.shape
#    search_top = 3*h/4
#    search_bot = 3*h/4 + 20
#    mask[0:search_top, 0:w] = 0
#    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    Distance = self.laser.ranges
    print min(Distance)
    if M['m00'] > 0:
      if min(Distance) > 1:  
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)
          # END CONTROL
          cv2.imshow("window", image)
          cv2.imshow("window", mask)
          cv2.waitKey(1)         
    
if __name__ == "__main__":
    W=Follower()
    rospy.spin()    
    
    