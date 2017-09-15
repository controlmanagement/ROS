#! /usr/bin/env python
# coding:utf-8

"""
------------------------------------------------
[History]
2017/09/15 : kondo takashi
------------------------------------------------
"""

import time
from datetime import datetime as dt
import rospy
from ros_start.srv import NECSTsrv
from ros_start.srv import NECSTsrvResponse
from ros_start.msg import drive_msg
from ros_start.msg import vel_msg
from ros_start.msg import move_msg
from ros_start.msg import dome_drive_msg
from ros_start.msg import dome_move_msg
from ros_start.msg import membrane_msg


class controller(object):
    
    def __init__(self):
        rospy.init_node('controller_client')
        return

    
    def drive_on(self):
        """drive_on"""
        pub = rospy.Publisher("antenna_drive", drive_msg, queue_size = 10)
        drive = drive_msg()
        drive = 1
        pub.publish(drive) 
        return

    def drive_off(self):
        """drive_off"""

        return

    def velocity_move(self,az_speed,el_speed,dist_arcsec = 5 * 3600):
        self.ant.vel_move(az_speed,el_speed,dist_arcsec)
        return

    def radec_move(self, ra, dec, code_mode, off_x = 0, off_y = 0, hosei = 'hosei_230.txt', offcoord = 'HORIZONTAL', lamda=2600, az_rate=12000, el_rate=12000, dcos=1):
        #self.ant.radec_move(ra, dec, code_mode, off_x, off_y, hosei, offcoord, lamda, az_rate, el_rate, dcos)
        pub = rospy.Publisher("antenna_radec", move_msg, queue_size = 10)
        mv = move_msg()
        mv.x = ra
        mv.y = dec
        mv.code_mode = code_mode
        mv.off_x
        mv.off_y
        mv.hosei
        mv.offcoord
        mv.lamda
        mv.dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation 
        pub.publish(mv)
        return
    

    def galactic_move(self, l, b, off_x = 0, off_y = 0, hosei = 'hosei_230.txt', offcoord = 'HORIZONTAL', lamda=2600, az_rate=12000, el_rate=12000, dcos=0):
        pub = rospy.Publisher("antenna_galactic", move_msg, queue_size = 10)
        mv = move_msg()
        mv.x = l
        mv.y = b
        mv.off_x
        mv.off_y
        mv.hosei
        mv.offcoord
        mv.lamda
        mv.dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        pub.publish(mv)
        return

    def planet_move(self, number, off_x = 0, off_y = 0, hosei = 'hosei_230.txt', offcoord = 'HORIZONTAL', lamda=2600, az_rate=12000, el_rate=12000, dcos=0):
        """1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun"""
        pub = rospy.Publisher("antenna_galactic", move_msg, queue_size = 10)
        mv = move_msg()
        mv.ntarg = number
        mv.off_x
        mv.off_y
        mv.hosei
        mv.offcoord
        mv.lamda
        mv.dcos
        #mv.az_rate ... no inplementation
        #mv.el_rate ... no inplementation
        pub.publish(mv)
        return

    def otf_scan(self, lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, n, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg = 0):
        on_start = self.ant.otf_start(lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, n, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg)
        return on_start

    def read_track(self):
        ret = self.ant.read_track()
        if ret[0] == "TRUE" and ret[1] == "TRUE":
            flag = True
        else:
            flag = False
        return flag

    def tracking_end(self):
        """tracking\u306e\u7d42\u4e86"""
        self.ant.tracking_end()
        return

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

    def dome_track(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync"""
        self.ant.dome_track()
        return

    def dome_track_end(self):
        """Dome\u3068\u671b\u9060\u93e1\u306esync\u306e\u7d42\u4e86"""
        self.ant.dome_track_end()
        return

# ===================
# mirror
# ===================

    def move_m4(self, position):
        """mirror4\u3092\u52d5\u304b\u3059("in"or"out")"""
        if position == "in": self.beam.m4_in()
        elif position == "out": self.beam.m4_out()
        else : print('set m4position error')
        return
    
    def move_hot(self, position):
        """hotload\u3092\u52d5\u304b\u3059("in"or"out")"""
        if position == "in": self.beam.hot_in()
        elif position == "out": self.beam.hot_out()
        else : print('set hotposition error')
        return

    def m2_move(self, dist):
        """m2\u3092\u52d5\u304b\u3059(um)"""
        self.beam.m2_move(dist)
        return

# ===================
# encoder
# ===================

    def oneshot(self, repeat=1, exposure=1.0, stime=0.0):
        #\u5206\u5149\u8a08oneshot\u306ecount\u5024\u3092\u914d\u5217\u3067\u51fa\u529b
        #dfs01 = self.rx.oneshot_dfs01(repeat, exposure ,stime)
        #dfs02 = self.rx.oneshot_dfs02(repeat, exposure ,stime)
        data = self.rx.oneshot_dfs(repeat, exposure, stime)
        data_dict = {'dfs1': data[0], 'dfs2': data[1]}
        return data_dict

# ===================
# status
# ===================

    def read_status(self):
        """\u6a5f\u5668and\u5929\u6c17\u306e\u30b9\u30c6\u30fc\u30bf\u30b9\u3092\u53d6\u5f97_"""
        timestamp = time.strftime('%Y/%m/%d %H:%M:%S',time.gmtime())
        ant_status = self.status.read_antenna()
        beam_status = self.status.read_beam()
        # sg_status = self.doppler.get_status()
        ret = self.status.read_weather()
        
        tv = time.time()
        mjd = tv/24./3600. + 40587.0
        ntime = dt.now()
        secofday = ntime.hour*60*60 + ntime.minute*60 + ntime.second + ntime.microsecond*0.000001
        lst_g = 0.67239+1.00273781*(mjd-40000.0)
        l_plb = -67.7222222222/360.0
        lst_plb = lst_g + l_plb
        lst_plb_i = int(lst_plb)
        lst_plb -= lst_plb_i
        lst_plb = 24.0*lst_plb
        lst_hh = int(lst_plb)
        lst_plb = 60.0*(lst_plb - lst_hh)
        lst_mm = int(lst_plb)
        lst_plb = 60.0*(lst_plb -lst_mm)
        lst_ss = int(lst_plb)
        lst_hh = "{0:02d}".format(lst_hh)
        lst_mm = "{0:02d}".format(lst_mm)
        lst_ss = "{0:02d}".format(lst_ss)
        lst = 100
        
        if ant_status[1][0] & ant_status[1][1] == 1:
            drive_ready_az = 'ON'
        else:
            drive_ready_az = 'OFF'

        if ant_status[1][2] & ant_status[1][3] == 1:
            drive_ready_el = 'ON'
        else:
            drive_ready_el = 'OFF'

        if ant_status[1][24] == 1:
            emergency = 'ON'
        else:
            emergency = 'OFF'

        if ant_status[5][1][1] == 'OPEN' and ant_status[5][1][3] == 'OPEN':
            door_dome = 'OPEN'
        elif ant_status[5][1][1] == 'MOVE' or ant_status[5][1][3] == 'MOVE':
            door_dome = 'MOVE'
        elif ant_status[5][1][1] == 'CLOSE' and ant_status[5][1][3] == 'CLOSE':
            door_dome = 'CLOSE'
        else:
            door_dome = 'ERROR'

        statusbox = { "Time" : timestamp,
                   "Limit" : ant_status[0],
                   "Current_Az" : ant_status[4][0]/3600.,
                   "Current_El" : ant_status[4][1]/3600.,
                   "Command_Az" : ant_status[3][2]/3600.,
                   "Command_El" : ant_status[3][3]/3600.,
                   "Deviation_Az" : ant_status[3][4],
                   "Deviation_El" : ant_status[3][5],
                   "Drive_ready_Az" : drive_ready_az,
                   "Drive_ready_El": drive_ready_el,
                   "Authority" : ant_status[2],
                   "Emergency" : emergency,
                   "Current_Dome" : ant_status[6]/3600.,
                   "Door_Dome" : door_dome,
                   "Door_Membrane" : ant_status[5][2][1],
                   "Door_Authority" : ant_status[5][3],
                   "Current_M4" : beam_status[1],
                   "Current_Hot" : beam_status[0],
                   "Year" : ret[0],
                   "Month" : ret[1],
                   "Day" : ret[2],
                   "Hour" : ret[3],
                   "Min" : ret[4],
                   "Sec" : ret[5],
                   "InTemp" : ret[6],
                   "OutTemp" : ret[7],
                   "InHumi" : ret[8],
                   "OutHumi" : ret[9],
                   "WindDir" : ret[10],
                   "WindSp" : ret[11],
                   "Press" : ret[12],
                   "Rain" : ret[13],
                   "CabinTemp1" : ret[14],
                   "CabinTemp2" :ret[15],
                   "DomeTemp1" : ret[16],
                   "DomeTemp2" : ret[17],
                   "GenTemp1" : ret[18],
                   "GenTemp2" : ret[19],
                   "None" : 'None',
                   "Current_M2" : beam_status[2],
                   "MJD" : int(mjd),
                   "LST" : lst,
                   "Secofday" : secofday
                   }
                   
        return statusbox







