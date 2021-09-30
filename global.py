import rospy
from std_msgs.msg import Float64
import time
from motor import *
from socket import *
import calculate as cal
optimal2 = 0

def callback(msg):
    global optimal2
    optimal2 = msg.data

def listener():
    rospy.init_node('optimal_subscriber_node', anonymous=True)
    rospy.Subscriber("optimal_angle", Float64, callback)
    print(optimal2)
    dozzy(optimal2)
    


def cmd_l(optimal):
    if 0 < optimal <= 30:
	self.motor.motor_move(-10,15)

    elif 30 < optimal <= 60:
	self.motor.motor_move(-20,15)
    
    elif 60 < optimal < 90:
	self.motor.motor_move(-30,15)
    

def cmd_r(optimal):
    if 0 < optimal <= 30:
	self.motor.motor_move(10,15)

    elif 30 < optimal <= 60:
	self.motor.motor_move(20,15)

    elif 60 < optimal < 90:
	self.motor.motor_move(30,15)

def dozzy(optimal):
    global optimal2
    if 0 <= optimal2 <= 90:
        cmd_r(optimal2)

    elif 90 < optimal2 <= 180:
        cmd_l(optimal2-90)

if __name__ == '__main__':
    while True:
        listener()
        time.sleep(1)
