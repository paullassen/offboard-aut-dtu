#!/usr/bin/env python
import rospy
import tf
import math

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
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

class Ctrl:
    def __init__(self):
        self.target = AttitudeTarget()
        self.target.type_mask = int('00111111', 2)

        self.current_pose = Point()
        self.current_pose.x = 0
        self.current_pose.y = 0
        self.current_pose.z = 0
        
        self.current_ang = Point()
        self.current_ang.x = 0
        self.current_ang.y = 0
        self.current_ang.z = 0
        
        self.pos_target = Point()
        self.pos_target.x = 0
        self.pos_target.y = 0
        self.pos_target.z = 0.15
        
        self.ez = 0
        self.ex = 0
        self.ey = 0
        self.ezd = 0
        self.exd = 0
        self.eyd = 0
        
        self.base_thrust = 0.3
        self.kpz = 0.1
        self.kdz = 0

        self.kill = True
        self.state = State()

    def get_orientation(self, orientation):
        quaternion = ( 
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_ang.x = euler[0]
        self.current_ang.y = euler[1]
        self.current_ang.z = euler[2]

    def set_orientation(self, roll, pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.target.orientation.x = quaternion[0]
        self.target.orientation.y = quaternion[1]
        self.target.orientation.z = quaternion[2]
        self.target.orientation.w = quaternion[3]

    def stateCb(self, msg):
        self.state = msg

    def pos_CB(self, msg):
        if not self.kill:
            self.current_pose.x = msg.pose.position.x
            self.current_pose.y = msg.pose.position.y
            self.current_pose.z = msg.pose.position.z
        
            self.get_orientation(msg.pose.orientation)
        else:
            self.pos_target.z=0
            self.current_pose.x=0
            self.current_pose.y=0
            self.current_pose.z=0

    def input_CB(self, msg):
        if msg.data <= 0:
            self.kill = True
        else:
            self.kill = False
        
    def calculate_error(self):
        ez = self.pos_target.z - self.current_pose.z
        ex = self.pos_target.x - self.current_pose.x
        ey = self.pos_target.y - self.current_pose.y
        
        ez_prev = self.ez
        ex_prev = self.ex
        ey_prev = self.ey
        then = self.target.header.stamp
        self.ez = ez
        self.ex = ex
        self.ey = ey
        self.target.header.stamp = rospy.Time.now()
        now = self.target.header.stamp
        self.ezd = (ez-ez_prev)/(now-then)
        self.exd = (ex-ex_prev)/(now-then)
        self.eyd = (ey-ey_prev)/(now-then)

    def calculate_attitude(self):
        self.set_orientation(0,0,0)
        self.target.thrust = self.base_thrust + self.kpz*self.ez + self.kdz*self.edz; 
        if self.pos_target.z == 0:
            self.target.thrust = 0

    def update_target(self):
        self.calculate_error()
        self.calculate_attitude() 


    
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 3.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def x_dir(self):
    	self.sp.position.x = self.local_pos.x + 5
    	self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Ctrl()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mocap_node/local_position/pose', PoseStamped, cnt.pos_CB)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.target)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    modes.setOffboardMode()
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
    	cnt.update_target()
    	sp_pub.publish(cnt.target)
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
