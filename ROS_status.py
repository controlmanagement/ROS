#!/usr/bin/env python

import rospy
from ros_start.msg import Status_antenna_msg

class status_main(object):
parameters = {
    limit_az:0,
    limit_el:0,
    command_az:0,
    command_el:0,
    current_az:0,
    current_el:0,
    emergency :0
    }

    def __init__(self):
        pass
    
    def status_check(self):
        rospy.loginfo(self.parameters)

    def callback(self,req):
        self.limit_az = req.limit_az
        self.limit_el = req.limit_el
        self.command_az = req.command_az
        self.command_el = req.command_el
        self.current_az = req.current_az
        self.current_el = req.current_el
        self.emergency = req.emergency
        self.status_check
        
    

if __name__ == '__main__':
    s = status_main()
    rospy.init_node('Status')
    sub = rospy.Subscriber('status',Status_antenna_msg,s.callback)
    rospy.spin()
