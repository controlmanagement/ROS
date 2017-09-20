#!/usr/bin/env python                                                            

import time
import rospy
from ros_start.msg import Status_antenna_msg
from ros_start.msg import Weather_msg

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
    param2 = {"in_temp":0,
              "out_temp":0,
              "in_humi":0,         
              "out_humi":0,
              "wind_sp":0,
              "wind_dir":0,
              "press":0,
              "rain":0,
              "cabin_temp1":0,
              "cabin_temp2":0,
              "dome_temp1":0,
              "dome_temp2":0,
              "gen_temp1":0,
              "gen_temp2":0,
              }


    def __init__(self):
        pass

    def status_check(self):
        #rospy.loginfo(self.param1)
        rospy.loginfo(self.param2)

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
        self.param2["in_temp"] = req.in_temp
        self.param2["out_temp"]= req.out_temp
        self.param2["in_humi"]= req.in_humi
        self.param2["out_humi"]= req.out_humi
        self.param2["wind_sp"]= req.wind_sp
        self.param2["wind_dir"]= req.wind_dir
        self.param2["press"]= req.press
        self.param2["rain"]= req.rain
        self.param2["cabin_temp1"]= req.cabin_temp1
        self.param2["cabin_temp2"]= req.cabin_temp2
        self.param2["dome_temp1"]= req.dome_temp1
        self.param2["dome_temp2"]= req.dome_temp2
        self.param2["gen_temp1"]= req.gen_temp1
        self.param2["gen_temp2"]= req.gen_temp2
        self.status_check()
        pass




if __name__ == '__main__':
    st = status_main()
    rospy.init_node('Status')
    sub1 = rospy.Subscriber('status_antenna', Status_antenna_msg, st.callback1)
    time.sleep(0.01)
    sub2 = rospy.Subscriber('status_weather', Weather_msg, st.callback2)
    print(sub2)
    rospy.spin()
