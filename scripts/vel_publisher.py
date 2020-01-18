#! /usr/bin/env python

from time import sleep
import serial
import rospy
from geometry_msgs.msg import Twist

K=0.1
#To be decided
MIN_VEL = 0 #To be decided
MAX_VEL = 1 #To be decided
MIN_PWM = 50
MAX_PWM = 200

class Vels(object):
    """ Pyserial to Arduino """
    def __init__(self):
        rospy.on_shutdown(self.quit)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 57600) # Establish the connection on a specific port
        except:
            self.ser = serial.Serial('/dev/ttyUSB1', 57600)
        #self.wait_for_arduino()
        try:
            #   while self.ser.readline() is not "READY": 	
            rospy.loginfo("Waiting for Arduino to become ready")

            print self.ser.readline()
            self.ser.write("0,0,0,0")
        except:
            rospy.logerr("Cannot Start Arduino")
        self.vel = Twist()
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel" , Twist , self.vel_setter)

    def vel_setter(self , data):
        """ Velocity Callback """
        self.vel = data

    def my_map(self, a , b= MIN_VEL, c= MAX_VEL, d=MIN_PWM , e=MAX_PWM):
        """  """
        return d + ((a - b)/(c - b))*(e - d)
	
        
    def main(self):
        """  """
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            
            v1 = self.vel.linear.x - K*self.vel.angular.z
            v2 = self.vel.linear.x + K*self.vel.angular.z


            pwm1 = int(self.my_map(abs(v1), MIN_VEL , MAX_VEL , MIN_PWM + 5 , MAX_PWM))
            pwm2 = int(self.my_map(abs(v2) , MIN_VEL , MAX_VEL , MIN_PWM + 5 , MAX_PWM))

            pwm1 = "1,"+str(pwm1) if v1>0 else "0,"+str(pwm1)
            pwm2 = "1,"+str(pwm2) if v2>0 else "0,"+str(pwm2)
            
            if self.vel.linear.x==0 and self.vel.angular.z==0:
                pwm1 = "1,"+"0"
                pwm2 = "1,"+"0"
            
            print(pwm1+","+pwm2)
            self.ser.write(pwm1+","+pwm2)
            print "Returned-->"+self.ser.readline()
            # print self.ser.readline()
            # print self.ser.readline()

            r.sleep()

    def quit(self):
        """ Quitting """
        self.ser.write("1,0,1,0")
        self.ser.close()
        rospy.logwarn("Killing!!")

if __name__ == "__main__":
    rospy.init_node("vel_publisher" , anonymous=True , disable_signals=True)

    # rospy.loginfo(ser.readline()) # Read the newest output from the Arduino
    o = Vels()
    o.main()
    
    # rospy.spin()
