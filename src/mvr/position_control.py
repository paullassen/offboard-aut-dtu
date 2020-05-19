#!/usr/bin/env python
import rospy
import tf
import math

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
import std_msgs.msg as msgs
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller():
    def __init__(self):
        self.state = State()
        self.kill = False
        self.now = rospy.Time.now().to_sec()
        
        self.target = AttitudeTarget()
        self.target.type_mask = int('00000111', 2)

        self.current_pos = Point()
        self.current_vel = Point()
        self.current_ang = Point()
        self.target_ang = Point()
        self.target_pos  = Point()

        self.ex = 0
        self.ey = 0
        self.ez = 0
        
        self.exd = 0
        self.eyd = 0
        self.ezd = 0

        self.base_thrust = 0
        self.kpz = 0
        self.kdz = 0

        self.kpx = 0
        self.kdx = 0
        self.kpy = 0
        self.kdy = 0
        
        self.att_publisher = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.pos_publisher = rospy.Publisher('test/pos', Point, queue_size=1)
        self.vel_publisher = rospy.Publisher('test/vel', Point, queue_size=1)
        self.err_publisher = rospy.Publisher('test/err', Point, queue_size=1)
        self.erd_publisher = rospy.Publisher('test/erd', Point, queue_size=1)

    def stateCb(self, msg):
        self.state = msg

    def poseCb(self, msg):
        prev_time = self.now
        prev_x = self.current_pos.x
        prev_y = self.current_pos.y
        prev_z = self.current_pos.z


        self.current_pos.x = msg.pose.position.x
        self.current_pos.y = msg.pose.position.y
        self.current_pos.z = msg.pose.position.z
        

        self.now = rospy.Time.now().to_sec()
        dt = self.now-prev_time

        self.current_vel.x = (self.current_pos.x - prev_x)/dt
        self.current_vel.y = (self.current_pos.y - prev_y)/dt
        self.current_vel.z = (self.current_pos.z - prev_z)/dt

        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.current_ang.x = euler[0]
        self.current_ang.y = euler[1]
        self.current_ang.z = euler[2]
        
        self.calculate_errors()

    def killCb(self, msg):
        self.kill = msg.data

    def targetCb(self, msg):
        self.target_pos.x = msg.x
        self.target_pos.y = msg.y
        self.target_pos.z = msg.z

    def kpCb(self, msg):
        self.kpx = msg.x
        self.kpy = msg.y
        self.kpz = msg.z

    def kdCb(self, msg):
        self.kdx = msg.x
        self.kdy = msg.y
        self.kdz = msg.z

    def baselineCb(self, msg):
        self.base_thrust = msg.data

    def calculate_errors(self):
        self.ex = self.target_pos.x - self.current_pos.x 
        self.ey = self.target_pos.y - self.current_pos.y 
        self.ez = self.target_pos.z - self.current_pos.z 

        self.exd = -self.current_vel.x
        self.eyd = -self.current_vel.y
        self.ezd = -self.current_vel.z

    def update_attitude(self):
        if self.kill:
            self.target.thrust = 0;
        else:
            self.target.thrust = self.base_thrust + self.kpz*self.ez + self.kdz*self.ezd
            
            kx = self.kpx*self.ex + self.kdx*self.exd
            ky = self.kpy*self.ey + self.kdy*self.eyd
            sin = math.sin(self.current_ang.z)
            cos = math.cos(self.current_ang.z)

            self.target_ang.x = sin*kx - cos*ky
            self.target_ang.y = cos*kx - sin*ky
        
        #self.target.header.stamp = rospy.Time.now()
        self.update_quaternion()

    def update_quaternion(self):
        quaternion = tf.transformations.quaternion_from_euler(self.target_ang.x,self.target_ang.y,self.target_ang.z)
        self.target.orientation.x = quaternion[0]
        self.target.orientation.y = quaternion[1]
        self.target.orientation.z = quaternion[2]
        self.target.orientation.w = quaternion[3]

    def publish(self):
        self.update_attitude()
        self.att_publisher.publish(self.target)
        self.pos_publisher.publish(self.current_pos)
        self.vel_publisher.publish(self.current_vel)
        err = Point()
        err.x = self.ex
        err.y = self.ey
        err.z = self.ez
        erd = Point()
        erd.x = self.exd
        erd.y = self.eyd
        erd.z = self.ezd
        self.err_publisher.publish(err)
        self.erd_publisher.publish(erd)
    # Get states
    # Update Errors
    # Update Inputs

def main():
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller() 

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    # Subscribe to kill switch
    # Subscribe to drone's local position
    #rospy.Subscriber('mocap_node/local_position/pose', PoseStamped, cnt.poseCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.poseCb)

    rospy.Subscriber('test/kill', msgs.Bool, cnt.killCb)
    rospy.Subscriber('test/target', Point, cnt.targetCb)
    rospy.Subscriber('test/kp', Point, cnt.kpCb)
    rospy.Subscriber('test/kd', Point, cnt.kdCb)
    rospy.Subscriber('test/baseline', msgs.Float32, cnt.baselineCb)


    # Setpoint publisher    

    
    # Make sure the drone is armed
    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    print('armed')
    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<100:
        cnt.publish()
        #rate.sleep()
        k = k + 1
    print('pre-publish')
    
    # activate OFFBOARD mode
    k = 0
    while not cnt.state.mode == "OFFBOARD":
        cnt.publish()
        #modes.setArm()
        modes.setOffboardMode()
        #modes.setStabilizedMode()
    	rate.sleep()
        k += 1
    print('offboarded')

    # ROS main loop
    print('main looping')
    while not rospy.is_shutdown():
    	#cnt.update_attitude()
    	cnt.publish()
        #print(str(cnt.kill) + '  ' + str(cnt.base_thrust))
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

