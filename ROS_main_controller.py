#!/usr/bin/env python

import rospy
import time
import nanten_main_controller_0911###kari
import controller

#import nanten_main_controller
#from ros_start.srv import azel
#from ros_start.srv import azelResponse
from ros_start.srv import list_azel
from ros_start.srv import list_azelResponse
#from std_msgs.msg import String

from ros_start.msg import list_azelmsg

class nanten_main_controller(object):
    nan_ctrl = nanten_main_controller_0911.nanten_main_controller()
    con = controller.controller()
    time_interval = 0.1
    flag = 1
    
    def get_azel(self,req):
        self.flag = 1
        rospy.loginfo('called get_azel func')
        self.list_az = req.az_list
        self.list_el = req.el_list
        self.start_time = req.start_time
        #self.Time = req.now
        #self.Duration = req.dt
        print(self.list_az,self.list_el,'list_az&el')
        #print(self.Time,self.Duration)
        #rospy.loginfo(self.Time)
        #rospy.loginfo(self.Duration)

        return list_azelResponse(success = self.act_azel(),error_msg = "error test")###!!!

    def get_azel2(self,message):
        self.list_az = message.az_list
        self.list_el = message.el_list
        self.start_time = message.start_time
        self.act_azel()
        
    def test2(self):
        self.con.test()

    def callback(self,message):
        rospy.loginfo("I heard %s", message.data)
        self.flag = 0
    

    def limit_check(self):
        ###azel limit check
        #self.limit_flag = 0#(0/1=okey/Not okay)
        for i in range(len(self.list_az)):
            print(self.list_az[i],self.list_el[i])
            if self.list_az[i] >= 280*3600 or  self.list_az[i] <=-280*3600:#kari
                #print('%')
                #self.limit_flag = 1
                rospy.loginfo('!!!limit!!!')
                return False
            if self.list_el[i] >= 89*3600 or  self.list_el[i] <= 0*3600:#kari
                #print('$')
                #self.limit_flag = 1
                rospy.loginfo('!!!limit!!!')
                return False
            else:
                pass
            #print('#')

    def act_azel(self):
        self.success = self.limit_check()
        print(self.success,'#')
        """
        if self.success:
            pass
        else:
            print('!!limit!!')
            return self.success
        """
        """
        if self.success == False:
            rospy.loginfo('!!!limit!!!')
            return self.success
        """
        
        current_time = time.time()
        #d = self.list_time[0] - current_time
        d = self.start_time-current_time
        rospy.loginfo(d)
        
        if d < 0:
            rospy.loginfo('azel_time is backward!!')
            #return azelResponse(success = True)
            self.success = False
            return self.success
        
        elif d > 100:
            rospy.loginfo('timeout %f'%d)
            self.success = False
            return self.success
        
        print(d)
        time.sleep(d)
        
        for i in range(len(self.list_az)):
            if self.flag:
                pass
            else:
                break
            #self.nan_ctrl.azel_move(self.list_az[i],self.list_el[i],10000,12000)
            #track = self.nan_ctrl.azel_move_test(self.list_az[i],self.list_el[i],10000,12000)#return 0 or 1 but I have no idea
            self.con.azel_move(self.list_az[i],self.list_el[i],10000,12000)
            time.sleep(self.time_interval-0.001)###ros time dousuru?

        rospy.loginfo('end azel_list')
        return True
        

        
if __name__ == '__main__':
    rospy.init_node('nanten_main_controller')
    rospy.loginfo('waiting serve')
    n = nanten_main_controller()
    #sub = rospy.Subscriber('chatter', String, n.callback)
    sub = rospy.Subscriber('list_azel',list_azelmsg,n.get_azel2)
    #service_server = rospy.Service('list_azel',list_azel,n.get_azel)
    time.sleep(10)
    #rospy.spin()
