import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)'
    #change "chatter" to whichever topic the bounding box coordinates are being published at.
    self.image_sub = rospy.Subscriber('chatter',String, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
    
    self.twist = Twist()
  def image_callback(self, msg):
    #take cx and cy from topic.Consider msg=[x_min,y_min,x_max,y_max]
    values=msg.split(",")
    cx = (values[0] + values[2])/2
    cy = (values[1] + values[3])/2
    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    h, w, d = image.shape
    err = cx - w/2
    self.twist.linear.x = 1.0
    self.twist.angular.z = -float(err) / 100
    self.cmd_vel_pub.publish(self.twist)
   
    cv2.imshow('target',image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
