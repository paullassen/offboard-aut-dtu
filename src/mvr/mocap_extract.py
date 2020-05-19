#!/usr/bin/env python
import rospy

import std_msgs.msg as geo


def main():
	rospy.init_node('mcp_test', anonymous=True)
    rate = rospy.Rate(20.0)
	rospy.Publisher('/kill', geo.Float32, queue_size=1)

	while not rospy.is_shutdown():
        
		rospy.spin()
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
