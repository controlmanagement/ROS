import math
import time
#import core.controller
from datetime import datetime as dt
import sys
import astropy.coordinates
from pyslalib import slalib
import coord
placoord = coord.coord_calc()
import rospy

from ros_start.msg import vel_msg
from ros_start.msg import move_msg
from ros_start.msg import Status_encoder_msg
from ros_start.msg import list_azelmsg

class antenna_nanten(object):

    off_list = {"off_az":0, "off_el":0, "off_j2000_ra":0, "off_j2000_dec":0, "off_b1950_ra":0, "off_b1950_dec":0,  "off_l":0, "off_b":0}
    longitude = -67.70308139*math.pi/180
    latitude = -22.96995611*math.pi/180
    height = 4863.85
    #temporary dut1
    dut1 = -0.20489  #delta UT:  UT1-UTC (UTC seconds)
    tv = ""
    enc_az = ""
    enc_el = ""



    def __init__(self):                        
        pass 
    
    def read_controller(self):
        rospy.init_node("antenna_server")
        rospy.loginfo(" Read ok ")

        #rospy.Subscriber('antenna_drive', drive_msg, self.drive_check)
        rospy.Subscriber('antenna_vel', vel_msg, self.velocity_move)
        rospy.Subscriber('antenna_radec', move_msg, self.radec_move)
        rospy.Subscriber('antenna_galactic', move_msg, self.galactic_move)
        rospy.Subscriber('antenna_planet', move_msg, self.planet_move)
        #rospy.Subscriber('antenna_dome_drive', dome_drive_msg, self.dome_drive_check)
        #rospy.Subscriber('antenna_dome_move', dome_move_msg, self.dome_move)
        #rospy.Subscriber('antenna_membrane', membrane_msg, self.membrane)
        
        rospy.spin()

    '''
    def drive_check(self, req):
        if req.drive == 1:
            self.antenna.contactor_on()
            self.antenna.drive_on()
        elif req.drive == 0:
            self.antenna.contactor_off()
            self.antenna.drive_off()
        else:
            rospy.logerr("drive_error")
            '''
    def note_encoder(self, req):
        self.enc_az = req.enc_az
        self.enc_el = req.enc_el
        return


    def velocity_move(self, req):
        rospy.Subscriber("status_encoder", Status_encoder_msg, self.note_encoder)
        #az_speed, el_speed, dist_arcsec = 5 * 3600
        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        azel = list_azelmsg()
        dist = 0
        az_list = []
        el_list = []
        #if dist_arcsec < 0:
            #rospy.logerr("Please, dist_arcsec > 0.\n")
            #print("Please, dist_arcsec > 0.\n")
        tv = time.time() + 1
        azel.start_time = tv
        init_az = self.enc_az
        init_el = self.enc_el
        print("ok")
        dt = 1
        time.sleep(0.5)
        print("enc_az",type(self.enc_az))
        print("enc_el",type(self.enc_el))
        init_az = self.enc_az
        init_el = self.enc_el
        while dist < req.dist:
            _az = req.az_speed*dt + init_az
            _el = req.el_speed*dt + init_el
            az_list.append(_az)
            el_list.append(_el)
            dist = abs(req.az_speed*dt) + abs(req.el_speed*dt)
            dt += 0.1
        print(self.enc_az)
        print(self.enc_el)

        azel.az_list = az_list
        azel.el_list = el_list
        pub.publish(azel)
        #print(az_list, el_list,tv)
        return
    
    def radec_move(self, req):
        """
        ra,dec -> degree
        code_mode -> 'J2000' or 'B1950'"""
        gx = req.x*math.pi/180.
        gy = req.y*math.pi/180.
        condition = 10#self.weather.read_weather()
        temp = 300#float(condition[6])+273.
        press = 1013#float(condition[12])
        humid = 20#float(condition[9])/100.
        if req.offcoord.lower() == req.code_mode and req.dcos == 1:
            gy += off_y/3600.*math.pi/180
            gx += (off_x/3600.*math.pi/180)/math.cos(gy)
            off_x = 0
            off_y = 0
        else:
            pass
        ra = gx*180./math.pi
        dec = gy*180./math.pi
        tv = time.time()
        #for i in range(300):
        ret = self.convert(ra, dec, 0, req.code_mode, req.off_x, req.off_y, req.offcoord)
        ret = self.create_azel_list(ret[0], ret[1], req.lamda, req.hosei, num=300)

        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        azel = list_azelmsg()
        azel.az_list = ret[0]
        azel.el_list = ret[1]
        azel.start_time = ret[2]
        print("az : ", ret[0][0]/3600.)
        print("el : ", ret[1][0]/3600.)
        print("time : ", ret[2])
        rospy.loginfo("success publish\n")
        pub.publish(azel)

        '''
        set_status = rospy.ServiceProxy("list_azel", list_azel)
        response = set_status(ret[0], ret[1], ret[2])
        if response.success:
            rospy.loginfo('set [%s] success' %("azel_list"))
        else:
            rospy.logerr('set [%s] failed' %("azel_list"))
            '''

        return
    
    def galactic_move(self, req):
        #pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        #azel = list_azelmsg()
        gx = req.x*math.pi/180.
        gy = req.y*math.pi/180.
        condition = 10#self.weather.read_weather()
        temp = 300#float(condition[6])+273.
        press = 1013#float(condition[12])
        humid = 20#float(condition[9])/100.
        if req.offcoord.lower() == "galactic" and req.dcos == 1:
            gy += off_y/3600.*math.pi/180
            gx += (off_x/3600.*math.pi/180)/math.cos(gy)
            off_x = 0
            off_y = 0
        else:
            pass
        l = gx*180./math.pi
        b = gy*180./math.pi
        ret = self.convert(l, b, 0, "galactic", req.off_x, req.off_y, req.offcoord)
        print("koko")
        ret = self.create_azel_list(ret[0], ret[1], req.lamda, req.hosei, num=300)
        print("okok")
        
        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        azel = list_azelmsg()
        azel.az_list = ret[0]
        azel.el_list = ret[1]
        azel.start_time = ret[2]
        print("az : ", ret[0][0]/3600.)
        print("el : ", ret[1][0]/3600.)
        print("time : ", ret[2])
        rospy.loginfo("success publish\n")
        pub.publish(azel)

        '''
        set_status = rospy.ServiceProxy("list_azel", list_azel)
        response = set_status(ret[0], ret[1], ret[2])
        if response.success:
            rospy.loginfo('set [%s] success' %("azel_list"))
        else:
            rospy.logerr('set [%s] failed' %("azel_list"))
            '''

        return
    
    def planet_move(self, req):
        """
        1.Mercury 2.Venus 3. 4.Mars 5.Jupiter 6.Saturn 7.Uranus 8.Neptune, 9.Pluto, 10.Moon, 11.Sun"""
        condition = 10#self.weather.read_weather()
        temp = 300#float(condition[6])+273.
        press = 1013#float(condition[12])
        humid = 20#float(condition[9])/100.
        ret = self.convert(0, 0, req.ntarg, "planet", req.off_x, req.off_y, req.offcoord)
        ret = self.create_azel_list(ret[0], ret[1], req.lamda, req.hosei, num=300)

        pub = rospy.Publisher("list_azel", list_azelmsg, queue_size = 10, latch=True)
        azel = list_azelmsg()
        azel.az_list = ret[0]
        azel.el_list = ret[1]
        azel.start_time = ret[2]
        print("az : ", ret[0][0]/3600.)
        print("el : ", ret[1][0]/3600.)
        print("time : ", ret[2])
        rospy.loginfo("success publish\n")
        pub.publish(azel)

        '''
        set_status = rospy.ServiceProxy("list_azel", list_azel)
        response = set_status(ret[0], ret[1], ret[2])
        if response.success:
            rospy.loginfo('set [%s] success' %("azel_list"))
        else:
            rospy.logerr('set [%s] failed' %("azel_list"))
            '''

        return

    """
    def coord_check(self, req):
        if req.coord_mode == "j200":
            self.radec_move()
        elif req.coord_mode == "b1950":
            self.radec_move()
        elif req.coord_mode == "galactic":
            self.galactic_move()
        elif req.coord_mode == "planet":
            self.planet_move()
        else:
            rospy.loginfo("coord_error")
            """
    
    def set_offset(self, coord, off_x, off_y):
        self.off_list["off_az"] = self.off_list["off_el"] = self.off_list["off_j2000"] = self.off_list["off_j2000"] = self.off_list["off_b1950"] = self.off_list["off_b1950"] = self.off_list["off_l"] = self.off_list["off_b"] = 0
        if coord.upper() == "HORIZONTAL":
            self.off_list["off_az"] = off_x/3600.
            self.off_list["off_el"] = off_y/3600.
        elif coord.lower() == "j2000":
            self.off_list["off_j2000_ra"] = off_x/3600.
            self.off_list["off_j2000_dec"] = off_y/3600.
        elif coord.lower() == "b1950":
            self.off_list["off_b1950_ra"] = off_x/3600.
            self.off_list["off_b1950_dec"] = off_y/3600.
        elif coord.upper() == "EQUATORIAL":
            self.off_list["off_j2000_ra"] = off_x/3600.
            self.off_list["off_j2000_dec"] = off_y/3600.
        else: #GALACTIC                                                                                                                    
            self.off_list["off_l"] = off_x/3600.
            self.off_list["off_b"] = off_y/3600.
        return
    
    def convert(self, x, y, ntarg, coordsys, gx, gy, cosydel):
        self.set_offset(cosydel, gx, gy)
        print(x, y, ntarg, coordsys, gx, gy, cosydel)
        if coordsys.lower() == 'j2000':
            rospy.loginfo("j2000")
            coord = astropy.coordinates.SkyCoord(x, y, frame='fk5', unit='deg')
        elif coordsys.lower() == 'b1950':
            coord = astropy.coordinates.SkyCoord(x, y, frame='fk4', unit='deg')
        elif coordsys.lower() == 'galactic':
            coord = astropy.coordinates.SkyCoord(x, y, frame='galactic', unit='deg')
        else:#planet
            rospy.loginfo("planet")                                  
            ret = placoord.calc_planet_coordJ2000(ntarg)
            ret = placoord.planet_J2000_geo_to_topo(ret[0], ret[1], ret[2], ret[3], self.dut1, self.longitude, self.latitude, self.height)
            x = ret[2]*180./math.pi
            y = ret[3]*180./math.pi
            coord = astropy.coordinates.SkyCoord(x, y, frame='fk5', unit='deg')
        l = coord.galactic.l.degree + self.off_list["off_l"]
        b = coord.galactic.b.degree + self.off_list["off_b"]
        coord = astropy.coordinates.SkyCoord(l, b, frame='galactic', unit='deg')
        b1950_ra = coord.fk4.ra.degree + self.off_list["off_b1950_ra"]
        b1950_dec = coord.fk4.dec.degree + self.off_list["off_b1950_dec"]
        coord = astropy.coordinates.SkyCoord(b1950_ra, b1950_dec, frame='fk4', unit='deg')
        j2000_ra = coord.fk5.ra.degree + self.off_list["off_j2000_ra"]
        j2000_dec = coord.fk5.dec.degree + self.off_list["off_j2000_dec"]

        return [j2000_ra, j2000_dec]

    def create_azel_list(self, j2000_ra, j2000_dec, lamda, hosei, num):
        condition = 10#self.read_status()
        #time.sleep(0.1)
        temp = 300#float(condition["InTemp"])+273.
        pressure = 1013#float(condition["Press"])
        humid = 20#float(condition["InHumi"])/100.
        lamda = 2600
        """
        f = open("radec.txt","w")
        f.write(str(j2000_ra)+"\n"+str(j2000_dec)+"\n")
        f.close()
        """
        gx = j2000_ra*math.pi/180.
        gy = j2000_dec*math.pi/180.
        az_list = []
        el_list = []
        tv = time.time() + 1
        for i in range (num):
            print(i)
            mjd = (tv+(i/10))/24./3600. + 40587.0 # 40587.0 = MJD0
            tai_utc = 37.0 # tai_utc=TAI-UTC  2017 JAN from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
            ret = slalib.sla_map(gx, gy, 0, 0, 0, 0, 2000, mjd + (tai_utc + 32.184)/(24.*3600.))
            ret = list(ret)
            ret = slalib.sla_aop(ret[0], ret[1], mjd, self.dut1, self.longitude, self.latitude, self.height, 0, 0, temp, pressure, humid, lamda, tlr=0.0065)
            real_az = ret[0]
            real_el = math.pi/2. - ret[1]
            real_az = real_az*180./math.pi*3600. + self.off_list["off_az"]
            real_el = real_el*180./math.pi*3600. + self.off_list["off_el"]
            real_az_n = real_az/3600.*math.pi/180.
            real_el_n = real_el/3600.*math.pi/180.
            #ret = self.coord.apply_kisa(real_az_n, real_el_n, hosei) # until define the set_coord
            target_az = real_az#+ret[0]
            target_el = real_el#+ret[1]
            az_list.append(target_az)
            el_list.append(target_el)
        return [az_list, el_list, tv]


    def otf_start(self, lambda_on, beta_on, dcos, coord_sys, dx, dy, dt, n, rampt, delay, lamda, hosei, code_mode, off_x, off_y, off_coord, ntarg = 0):
        condition = self.weather.read_weather()
        temp = float(condition[6])+273.
        press = float(condition[12])
        humid = float(condition[9])/100.
        
        start_x = off_x-float(dx)/2.-float(dx)/float(dt)*rampt
        start_y = off_y-float(dy)/2.-float(dy)/float(dt)*rampt
        total_t = rampt + dt * n
        end_x = off_x + dx * (n - 0.5)
        end_y = off_y + dy * (n - 0.5)
        mjd = 40587 + time.time()/(24.*3600.)
        
        self.antenna.otf_thread_start(lambda_on, beta_on, mjd+delay/24./3600., mjd+(delay+total_t)/24./3600., start_x, start_y, end_x, end_y, dcos, coord_sys, hosei, temp, press, humid, lamda, code_mode, off_x, off_y, off_coord, ntarg)
        return mjd+(delay+rampt)/24./3600.
    
    def otf_tracking_end(self):
        self.antenna.otf_tracking_end()
        return
    
    def otf_end(self):
        self.antenna.otf_stop()
        return

'''    
# for dome
    
    def dome_drive_check(self, req):
        if req.drive == 1:
            self.dome.dome_open()
        elif req.drive ==0:
            self.dome.dome_close()
        else:
            rospy.logerr("dome_drive_error")
        return

    def membrane(self, req):
        if req.drive == 1:
            self.dome.memb_open()
        elif req.drive == 0:
            self.dome.memb_close()
        else:
            rospy.logerr("memb_drive_error")
        return
    
    def dome_move(self, req):
        self.dome.move(req.dome_az)
        return
    
    def dome_stop(self):
        self.dome.dome_stop()
        return
'''

if __name__ == "__main__":
    aa = antenna_nanten()
    aa.read_controller()
