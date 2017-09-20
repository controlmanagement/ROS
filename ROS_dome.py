#!/usr/bin/env python
"""
2017/09/20 Shiotani
for dome part
node name is 'Dome'
"""
import rospy
import controller###for check
import time
import threading
import antenna_nanten

from ros_start.msg import NECST_msg
from std_msgs.msg import String

class nanten_main_controller(object):
    con = controller.controller()
    ant = antenna_nanten.antenna_nanten()
    parameters = {
        'target_az':0,
        'flag':0,
        'task_number':0,
        }

    def __init__(self):
        pass

    def start_thread(self):
        th = threading.Thread(target = self.act_dome)
        th.setDaemon(True)
        th.start()
        th2 = threading.Thread(target = self.pub_status)
        th2.setDaemon(True)
        th2.start()
        
    ###dome part
    def dome_open(self):
        """Dome\u306eopen"""
        self.ant.dome_open()
        return

    def dome_close(self):
        """Dome\u306eclose"""
        self.ant.dome_close()
        return

    def memb_open(self):
        """\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopen"""
        self.ant.memb_open()
        return

    def memb_close(self):
        """\u30e1\u30f3\u30d6\u30ec\u30f3\u306eopenclose"""
        self.ant.memb_close()
        return

    def dome_move(self, dome_az):
        """Dome\u3092(dome_az)\u306b\u52d5\u4f5c"""
        self.ant.dome_move(dome_az)
        return

    def dome_stop(self):
        """Dome\u306eclose\u52d5\u4f5c\u3092\u505c\u6b62"""
        self.dome_track_end()
        self.ant.dome_stop()
        return

    '''
    def dome_track(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync"""
        self.ant.dome_track()
        return

    def dome_track_end(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync\u306e\u7d42\u4e86"""
        self.ant.dome_track_end()
        return
    '''
    ###dome part end
    
    def test(self):
        return

    def set_parameter(self,req):
        self.parameters[req.name] = req.value
        return

    def limit_check(self):
        for i in range(len(self.parameters['target_az'])):
            print(self.parameters['target_az'][i],self.parameters['target_el'][i])
            if self.parameters['target_az'][i] >= 280*3600 or  self.parameters['target_az'][i] <=-280*3600:#kari
                rospy.loginfo('!!!limit!!!')
                return False
            if self.parameters['target_el'][i] >= 89*3600 or  self.parameters['target_el'][i] <= 0*3600:#kari
                rospy.loginfo('!!!limit!!!')
                return False
            else:
                pass
        
        pass

    def act_dome(self):
        while True:
            if self.parameters['command'] == 'pass':
                time.sleep(1)
            elif self.parameters['command'] == 'dome_open':
                self.dome_open()
            elif self.parameters['command'] == 'dome_close':
                self.dome_close()
            elif self.parameters['command'] == 'memb_open':
                self.memb_open()
            elif self.parameters['command'] == 'memb_close':
                self.memb_close()
            elif self.parameters['command'] == 'dome_move':
                self.dome_move(self.parameter['target_az'])
            elif self.parameters['command'] == 'dome_stop':
                self.dome_stop()
            time.sleep(1)
            continue
            

    def emergency(self,req):
        rospy.loginfo('!!!emergency!!!')
        rospy.loginfo('!!!stop azel_move!!!')
        self.emergency_flag = 1
        return

    def pub_status(self):
        while True:
            rospy.loginfo('#test#')#forcheck
            time.sleep(1)
        pub = rospy.Publisher('status',String, queue_size=10, latch = True)
        status = String()
        #status.str = self.#kari
        #pub.publish(status.str)
        return

if __name__ == '__main__':
    n = nanten_main_controller()
    rospy.init_node('Dome')
    rospy.loginfo('waiting publish nanten_main_controller')
    n.start_thread()
    sub = rospy.Subscriber('dome_param', NECST_msg, n.set_parameter)
    sub = rospy.Subscriber('emergency', String, n.emergency)
    rospy.spin()
 
