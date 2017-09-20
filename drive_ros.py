#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import telescope_nanten.antenna_nanten
ant = telescope_nanten.antenna_nanten.antenna_nanten()


def drive_main(message):
    rospy.loginfo(message.data)
    if message.data:
        drive_on()
    else:
        drive_off()

def drive_on():
    ant.drive_on()
def drive_off():
    ant.drive_off()
    
rospy.init_node('drive')
sub = rospy.Subscriber('drive', String, drive_main)
rospy.spin()
