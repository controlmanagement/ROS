#!/usr/bin/env python                                                            

import time
import rospy
from ros_start.msg import Status_antenna_msg

class status_main(object):
    param1 = {
        "limit_az":0,
        "limit_el":0,
        "command_az":0,
        "command_el":0,
        "current_az":0,
        "current_el":0,
        "emergency" :0
        }
    param2 = {"in_temp":,



    def __init__(self):
        pass

    def status_check(self):
        rospy.loginfo(self.parameters)

    def callback1(self, req):
        self.param1["limit_az"] = req.limit_az
        self.param1["limit_el"] = req.limit_el
        self.param1["command_az"] = req.command_az
        self.param1["command_el"] = req.command_el
        self.param1["current_az"] = req.current_az
        self.param1["current_el"] = req.current_el
        self.param1["emergency"] = req.emergency
        self.status_check()

    def callback2(self, req):
        pass




if __name__ == '__main__':
    st = status_main()
    rospy.init_node('Status')
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback1)
    time.sleep(0.01)
    sub2 = rospy.Subscriber('status_weather', Status_antenna_msg, st.callback2)
    print(sub2)
    rospy.spin()
