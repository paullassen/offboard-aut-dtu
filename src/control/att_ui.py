#!/usr/bin/env python
import rospy
import std_msgs.msg as smsg
import geometry_msgs.msg as geo
import mavros_msgs.msg as mav
import offboard.msg as test

import os
import sys
import termios
import fcntl
import atexit
import time

old_settings=None
done = False

def init_anykey():
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON) # lflags
    new_settings[6][termios.VMIN] = 0  # cc
    new_settings[6][termios.VTIME] = 0 # cc
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)

@atexit.register
def term_anykey():
    global old_settings
    if old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def anykey():
    ch_set = []
    ch = os.read(sys.stdin.fileno(), 1)
    while ch != None and len(ch) > 0:
        ch_set.append( ord(ch[0]) )
        ch = os.read(sys.stdin.fileno(), 1)
    return ch_set;

def intro():
    print("============================================================")
    print("Command Line Control Interface")
    print("============================================================")
    print("Press Q to kill")
    print("Press A to abort this program")
    print("Press E/D to change target yaw by +/- 10")
    print("Press T/G to change target height by +/- 0.25")
    print("Press R/F to change baseline thrust by +/- 0.01")
    print("Press Y/H to change Kpxy by +/- 0.1")
    print("Press U/J to change Kdxy by +/- 0.1")
    print("Press I/K to change Kpz  by +/- 0.01")
    print("Press O/L to change Kdz  by +/- 0.01")

def printBool(boolean):
    if boolean:
        return "OK"
    else:
        return "FAIL"


class Status:
    def __init__(self):
        self.health = test.Health()
        self.kp = geo.Point()
        self.kd = geo.Point()
        self.ki = geo.Point()
        self.zz = geo.Point()

        self.bl = smsg.Float32()
        self.kl = smsg.Bool()
        self.st = smsg.Bool()
        self.st.data = False
        self.pos = geo.Point()
        self.vel = geo.Point()
        self.err = geo.Point()
        self.erd = geo.Point()
        self.eri = geo.Point()
        self.yaw = smsg.Float32()
        self.attitude = geo.Point()
        self.mocap_att = geo.Point()
        self.thrust = 0.0 

        self.kp.x = 8.0
        self.kp.y = 8.0
        self.kp.z = 0.05

        self.kd.x = 10.0
        self.kd.y = 10.0
        self.kd.z = 0.1
        
        self.ki.x = 1.0
        self.ki.y = 1.0
        self.ki.z = 0.02
        self.bl.data = 0.14
        self.yaw.data = 0.0

        self.kp_pub = rospy.Publisher('kp', geo.Point, queue_size=1)
        self.kd_pub = rospy.Publisher('kd', geo.Point, queue_size=1)
        self.ki_pub = rospy.Publisher('ki', geo.Point, queue_size=1)
        self.bl_pub = rospy.Publisher('baseline', smsg.Float32, queue_size=1)
        self.zz_pub = rospy.Publisher('target', geo.Point, queue_size=1)
        self.kl_pub = rospy.Publisher('kill', smsg.Bool, queue_size=1)
        self.st_pub = rospy.Publisher('start', smsg.Bool, queue_size=1)
        self.yw_pub = rospy.Publisher('yaw_target', smsg.Float32, queue_size=1)

    def healthCb(self, msg):
        self.health = msg

    def attCb(self, msg):
        self.attitude = msg.point

    def mattCb(self, msg):
        self.mocap_att = msg.point

    def posCb(self, msg):
        self.pos= msg.point

    def velCb(self, msg):
        self.vel = msg.point

    def errCb(self, msg):
        self.err = msg.point

    def erdCb(self, msg):
        self.erd = msg.point

    def eriCb(self, msg):
        self.eri = msg.point

    def thrustCb(self, msg):
        self.thrust = msg.data

    def baselineCb(self, msg):
        self.bl = msg

    def targetCb(self, msg):
        self.zz = msg

    def killCb(self, msg):
        self.kl = msg

    def startCb(self, msg):
        self.st = msg

    def yawCb(self, msg):
        self.yaw = msg

    def publish(self):
        self.kp_pub.publish(self.kp)
        self.kd_pub.publish(self.kd)
        self.ki_pub.publish(self.ki)
        self.bl_pub.publish(self.bl)
        self.zz_pub.publish(self.zz)
        self.kl_pub.publish(self.kl)
        self.st_pub.publish(self.st)
        self.yw_pub.publish(self.yaw)


    def print_status(self, first=False):
        if not first:
            print("\r\033[26A")
        else:
            pass
        zzx = round(self.zz.x,5)
        zzy = round(self.zz.y,5)
        zzz = round(self.zz.z,5)
        kpx = round(self.kp.x,5)
        kpy = round(self.kp.y,5)
        kpz = round(self.kp.z,5)
        kdx = round(self.kd.x,5)
        kdy = round(self.kd.y,5)
        kdz = round(self.kd.z,5)
        kix = round(self.ki.x,5)
        kiy = round(self.ki.y,5)
        kiz = round(self.ki.z,5)
        bl  = round(self.bl.data, 5)


        px = round(self.pos.x, 5)
        py = round(self.pos.y, 5)
        pz = round(self.pos.z, 5)
        vx = round(self.vel.x, 5)
        vy = round(self.vel.y, 5)
        vz = round(self.vel.z, 5)
        ex = round(self.err.x, 5)
        ey = round(self.err.y, 5)
        ez = round(self.err.z, 5)
        exd = round(self.erd.x, 5)
        eyd = round(self.erd.y, 5)
        ezd = round(self.erd.z, 5)
        exi = round(self.eri.x, 5)
        eyi = round(self.eri.y, 5)
        ezi = round(self.eri.z, 5)
        
        t = round(self.thrust, 5)

        pr = round(self.attitude.x, 5)
        pp = round(self.attitude.y, 5)
        pw = round(self.attitude.z, 5)

        mr = round(self.mocap_att.x, 5)
        mp = round(self.mocap_att.y, 5)
        my = round(self.mocap_att.z, 5)

        health = self.health;            
        print("------------------------------------------------------------")
        print("Health".ljust(55))
        print("------------------------------------------------------------")
        print((" Gyro  : "+printBool(health.gyro)).ljust(15) + (" Local    : "+printBool(health.local)).ljust(20))
        print((" Accel : "+printBool(health.accel)).ljust(15) + (" Global   : "+printBool(health.globe)).ljust(20))
        print((" Mag   : "+printBool(health.mag)).ljust(15) + (" Home     : "+printBool(health.home)).ljust(20))
        print((" Level : "+printBool(health.level)).ljust(15) + (" Battery  : "+str(round(health.battery,5)).ljust(20)))
        print("------------------------------------------------------------")
        print("Uav_Ang".ljust(15)+"Mocap_Ang".ljust(15)+"Errors".ljust(15)+"Gains".ljust(10)+'   ')
        print("------------------------------------------------------------")
        print((' r: '+str(pr)).ljust(15) + (' r: '+str(mr)).ljust(15) + 
              (' ex: '+str(ex)).ljust(15) + (' kpx: '+str(kpx)).ljust(10)+'   ')
               
        print((' p: '+str(pp)).ljust(15) + (' p: '+str(mp)).ljust(15) + 
              (' ey: '+str(ey)).ljust(15) + (' kpy: '+str(kpy)).ljust(10)+'   ')

        print((' y: '+str(pw)).ljust(15) + (' y: '+str(my)).ljust(15) + 
              (' ez: '+str(ez)).ljust(15) + (' kpz: '+str(kpz)).ljust(10)+'   ')
        
        print("------------------------------------------------------------")
        print("Current_Pos".ljust(15)+"Target_Pos".ljust(15)+"Errors".ljust(15)+"Gains".ljust(10)+'   ')
        print("------------------------------------------------------------")
        print((' x: '+str(px)).ljust(15) + (' x: '+str(zzx)).ljust(15) + 
              (' exd: '+str(exd)).ljust(15) + (' kdx: '+str(kdx)).ljust(10)+'   ')
               
        print((' y: '+str(py)).ljust(15) + (' y: '+str(zzy)).ljust(15) + 
              (' eyd: '+str(eyd)).ljust(15) + (' kdy: '+str(kdy)).ljust(10)+'   ')

        print((' z: '+str(pz)).ljust(15) + (' z: '+str(zzz)).ljust(15) + 
              (' ezd: '+str(ezd)).ljust(15) + (' kdz: '+str(kdz)).ljust(10)+'   ')
        
        print("------------------------------------------------------------")
        print("Current_Vel".ljust(15)+("Yaw: "+str(self.yaw.data)).ljust(30)+"Kill:"+str(self.kl.data).rjust(6))
        print("------------------------------------------------------------")
        print((' x: '+str(vx)).ljust(15) + (' T: '+str(t)).ljust(15) + 
              ('exi: '+str(exi)).ljust(15) + (' kix: '+str(kix)).ljust(10)+'   ')
        
        print((' y: '+str(vy)).ljust(15) + ('Baseline').ljust(15) + 
              ('eyi: '+str(eyi)).ljust(15) + (' kiy: '+str(kiy)).ljust(10)+'   ')

        print((' z: '+str(vz)).ljust(15) + (' T: '+str(bl)).ljust(15) + 
              ('ezi: '+str(ezi)).ljust(15) + (' kiz: '+str(kiz)).ljust(10)+'   ')

    def kill(self):
        self.kl.data = True
        self.bl.data = 0.0
    def incKpz(self):
        self.kp.z += 0.01 
    
    def decKpz(self):
        self.kp.z -= 0.01

    def incKpxy(self):
        self.kp.x += 0.1
        self.kp.y += 0.1

    def decKpxy(self):
        self.kp.x -= 0.1
        self.kp.y -= 0.1

    def incKdz(self):
        self.kd.z += 0.01 
    
    def decKdz(self):
        self.kd.z -= 0.01
    
    def incKdxy(self):
        self.kd.x += 0.1
        self.kd.y += 0.1

    def decKdxy(self):
        self.kd.x -= 0.1
        self.kd.y -= 0.1

    def incKiz(self):
        self.ki.z += 0.01

    def decKiz(self):
        self.ki.z -= 0.01

    def incKixy(self):
        self.ki.x += 0.01
        self.ki.y += 0.01

    def decKixy(self):
        self.ki.x -= 0.01
        self.ki.y -= 0.01

    def incBl(self):
        self.bl.data += 0.01

    def decBl(self):
        self.bl.data -= 0.01
    
    def incZz(self):
        self.zz.z += 0.12
    
    def decZz(self):
        self.zz.z -= 0.12

    def incXx(self):
        self.zz.x += 0.25

    def decXx(self):
        self.zz.x -= 0.25

    def incYy(self):
        self.zz.y += 25

    def decYy(self):
        self.zz.y -= 25

    def start(self):
        self.st.data = True

    def incYaw(self):
        self.yaw.data += 10

    def decYaw(self):
        self.yaw.data -= 10

def main():
    global done
    init_anykey()

    rospy.init_node('test_ui', anonymous=True)
    
    rate = rospy.Rate(5.0)
    
    stat = Status()

    rospy.Subscriber('health', test.Health, stat.healthCb)
    rospy.Subscriber('attitude', geo.PointStamped, stat.attCb)
    rospy.Subscriber('mocap_att', geo.PointStamped, stat.mattCb)
    rospy.Subscriber('pose', geo.PointStamped, stat.posCb)
    rospy.Subscriber('vel', geo.PointStamped, stat.velCb)
    rospy.Subscriber('err', geo.PointStamped, stat.errCb)
    rospy.Subscriber('erd', geo.PointStamped, stat.erdCb)
    rospy.Subscriber('eri', geo.PointStamped, stat.eriCb)
    rospy.Subscriber('thrust', smsg.Float32, stat.thrustCb)

    rospy.Subscriber('baseline_', smsg.Float32, stat.baselineCb)
    rospy.Subscriber('target_', geo.Point, stat.targetCb)
    rospy.Subscriber('kill_', smsg.Bool, stat.killCb)
    rospy.Subscriber('start_', smsg.Bool, stat.startCb)
    rospy.Subscriber('yaw_target_', smsg.Float32, stat.yawCb)
    intro()
    stat.print_status(True)
    while not done:
        stat.print_status()
        key = anykey()
        if key != []:
            for k in key:
                if chr(k) == 'a':
                    stat.kill()
                    stat.publish()
                    done=True
                    #print("\n"*13)
                elif chr(k) == 'q':
                    stat.kill()
                elif chr(k) == 'r':
                    stat.incBl()
                elif chr(k) == 'f':
                    stat.decBl()
                elif chr(k) == 't':
                    stat.incZz()
                elif chr(k) == 'g':
                    stat.decZz()
                elif chr(k) == 'z':
                    #stat.incYy()
                    stat.incYy()
                elif chr(k) == 'x':
                    #stat.decYy()
                    stat.decYy()
                elif chr(k) == 'y':
                    stat.incKpxy()
                elif chr(k) == 'h':
                    stat.decKpxy()
                elif chr(k) == 'u':
                    stat.incKdxy()
                elif chr(k) == 'j':
                    stat.decKdxy()
                elif chr(k) == 'i':
                    stat.incKpz()
                elif chr(k) == 'k':
                    stat.decKpz()
                elif chr(k) == 'o':
                    stat.incKdz()
                elif chr(k) == 'l':
                    stat.decKdz()
                elif chr(k) == 'n':
                    stat.incKiz()
                elif chr(k) == 'm':
                    stat.decKiz()
                elif chr(k) == 'v':
                    stat.incKixy()
                elif chr(k) == 'b':
                    stat.decKixy()
                elif chr(k) == 'c':
                    stat.start()
                elif chr(k) == 'e':
                    stat.incYaw()
                elif chr(k) == 'd':
                    stat.decYaw()

                

        stat.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
