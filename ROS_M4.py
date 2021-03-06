#!/usr/bin/env python

import rospy
import time
import threading
import M4
#import board_M4
import test_board

from ros_start.msg import NECST_msg
from std_msgs.msg import String

class m4_controller(object):
    speed = 3000
    low_speed = 100
    acc = 500
    dec = 500

    error =[]

    position = ''
    count = 0
    
    shutdown_flag = False

    M4 = M4.m4_controller()

    def __init__(self):
	#self.mtr = pyinterface.create_gpg7204(ndev)
        #self.mtr.ctrl.set_limit_config('MTR_LOGIC', 0x000c)
        #self.mtr.ctrl.off_inter_lock()
	#self.board_M4 = board_M4.board_M4()
	self.board_M4 = test_board.board()
	self.board_M4.set_limit_config('MTR_LOGIC', 0x000c)
	self.board_M4.off_inter_lock()
        self.get_pos()
        th = threading.Thread(target = self.pub_status)
	th.setDaemon(True)
	th.start()
	pass

    def print_msg(self, msg):
        print(msg)
        return
        
    def print_error(self, msg):
        self.error.append(msg)
        self.print_msg('!!!! ERROR !!!! ' + msg)
        return
    
    def get_count(self):
        self.count = self.board_M4.get_position()
        return self.count
    
    def get_pos(self):
        status = self.board_M4.get_status('MTR_LIMIT_STATUS')
        """
        if status == 0x0008:
            #SMART
            self.position = 'OUT'
        elif status == 0x0004:
            #NAGOYA
            self.position = 'IN'
        elif status == 0x000c:
            self.position = 'MOVE'
        else:
            self.print_error('limit error')
        """
        if status == 0x0004:
            #SMART
            self.position = 'OUT'
        elif status == 0x0008:
            #NAGOYA
            self.position = 'IN'
        elif status == 0x0000:
            self.position = 'MOVE'
        else:
            self.print_error('limit error')
            return
        
        return self.position

    pos = self.get_pos()

    def move(self,req):
        if req == pos:
	    if req == 'OUT':
	        self.print_msg('m4 is alrady out')
	        return
	    elif req == 'IN':
	        self.print_msg('m4 is alrady in')
	        return
            else:
	        self.print_msg('me is alrady move')
                return
        else:
	    if req == 'OUT':
                nstep = 60500
		self.print_msg('m4 move out')
	    elif req == 'IN':
		nstep = -60500
		self.print_msg('m4 move in')
	    else:
		self.print_error('parameter error')
		return
	    self.board_M4.move(self.speed, nstep, self.low_speed, self.acc, self.dec)
            time.sleep(12.)
            count = self.get_count()
            pos= self.get_pos()
	    return

    def m4_out(self):
        self.M4.m4_out()
	return

    def m4_in(self):
	self.M4.m4_in()
	return

    def stop(self):
	self.M4.stop()
	return

    def read_pos(self):
	self.M4.read_pos()
	return

    def read_count(self)
        self.M4.read_count()
	return

    def test(self):
        return


    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop M4!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        pub = rospy.Publisher('status_m4',String, queue_size=10, latch = True)
        status = String()
        status.data = pos
        pub.publish(status.data)
        return

if __name__ == '__main__':
    m4 = m4_controller()
    rospy.init_node('M4')
    rospy.loginfo('waiting publish M4')
    sub = rospy.Subscriber('m4', String, m4.move)
    sub = rospy.Subscriber('emergency', String, m4.emergency)
    rospy.spin()

def m4_client(host,port):
    client = pyinterface.server_client_wrapper.control_client_wrapper(m4_controller, host, port)
    return client

def m4_monitor_client(host,port):
    client = pyinterface.server_client_wrapper.monitor_client_wrapper(m4_controller, host, port)
    return client

def start_m4_server(port1=6003, port2=6004):
    m4 = m4_controller()
    server = pyinterface.server_client_wrapper.server_wrapper(m4,'', port1, port2)
    server.start()
    return server
