#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class mocap:
    def __init__(self):
        self.pose = PoseStamped()

    def mocap_cb(self, msg):
        self.pose = msg
    def update_stamp(self):
        self.pose.header.stamp = rospy.Time.now()
        self.pose.pose.position.y = -self.pose.pose.position.y
def main():
    rospy.init_node("mcp", anonymous=True)
    rate = rospy.Rate(20.0)
    mcp = mocap()
    rospy.Subscriber("mocap_node/local_position/pose", PoseStamped, mcp.mocap_cb)
    m_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        mcp.update_stamp()
        m_pub.publish(mcp.pose)
        rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
