#!/usr/bin/env python
"""
2017/09/20 Shiotani
main_controller part
node name is 'nanten_main_controller'
"""
import rospy
import controller
#import nanten_main_controller
import time
import threading

from ros_start.msg import Status_antenna_msg
from ros_start.msg import list_azelmsg
from std_msgs.msg import String

class nanten_main_controller(object):
    con = controller.controller()
    parameters = {
        'az_list':0,
        'el_list':0,
        'start_time':0,
        'flag':0,
        'target_az':0,
        'target_el':0
        }
    limit_flag = 0###(0/1=okay/NG)
    limit_az = True###(True/False = okay/limit)
    limit_el = True
    command_az = 0
    command_el = 0
    current_az = 0
    current_el = 0

    def __init__(self):
        pass
    
    def start_thread(self):
        th = threading.Thread(target = self.act_azel)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.pub_status)
        th2.setDaemon(True)
        th2.start()

    def test(self):
        return

    def set_parameter(self,req):
        self.parameters['az_list'] = req.az_list
        self.parameters['el_list'] = req.el_list
        self.parameters['start_time'] = req.start_time
        return

    def limit_check(self):
        for i in range(len(self.parameters['target_az'])):
            print(self.parameters['target_az'][i],self.parameters['target_el'][i])
            if self.parameters['target_az'][i] >= 280*3600 or  self.parameters['target_az'][i] <=-280*3600:#kari
                rospy.loginfo('!!!limit az!!!')
                self.limit_flag = 1
                return False
            if self.parameters['target_el'][i] >= 89*3600 or  self.parameters['target_el'][i] <= 0*3600:#kari
                rospy.loginfo('!!!limit el!!!')
                self.limit_flag = 1
                return False
            else:
                return True

    def act_azel(self):
        while True:
            if not self.limit_flag:
                rospy.loginfo('limit_flag')
                time.sleep(1)
                continue
            ###start time check###
            C_time = time.time()
            dt = self.start_time - C_time
            if dt < 0:
                rospy.loginfo('!!!Start_time is backward!!!')
                time.sleep(0.5)###
                continue
            if dt > 10000000:
                rospy.loginfo('!!!Start_time is too distant future')
                time.sleep(0.5)###
                continue
            ###start time check end###
            
            ###limit check###
            if self.limit_check():
                pass
            else:
                continue
            ###limit check end###

            ###move_azel####
            for i in range(len(self.parameters['target_az'])):
                if self.emergency_flag:
                    break
                else:
                    pass
                self.con.azel_move(self.parameters['target_az'][i],self.parameters['target_el'][i],10000,12000)
                self.command_az = self.parameters['target_az'][i]
                self.commamd_el = self.parameters['target_el'][i]

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        while True:
            ###publish parameter
            pub = rospy.Publisher('status_antenna',Status_antenna_msg, queue_size=10, latch = True)
            status = Status_antenna_msg()
            status.limit_az = self.limit_az
            status.limit_el = self.limit_el
            status.command_az = self.parameters['target_az']
            status.command_el = self.parameters['target_el']
            status.current_az = self.current_az#mitei(encoder)
            status.current_el = self.current_el#mitei(encoder)
            pub.publish(status)
            time.sleep(0.5)###kari
            return

if __name__ == '__main__':
    n = nanten_main_controller()
    rospy.init_node('nanten_main_controller')
    rospy.loginfo('waiting publish nanten_main_controller')
    n.start_thread()
    sub = rospy.Subscriber('list_azel', list_azelmsg, n.set_parameter)
    sub = rospy.Subscriber('emergency', String, n.emergency)
    rospy.spin()
    
