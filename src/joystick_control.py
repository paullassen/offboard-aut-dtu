#!/usr/bin/env python
import rospy
import std_msgs.msg as smsg
import geometry_msgs.msg as geo
import sensor_msgs.msg as sens

class Controller:
    def __init__(self):
        self.joystick = sens.Joy()
        self.target = geo.Point()
        self.start = smsg.Bool()
        self.kill = smsg.Bool()
        self.baseline = smsg.Float32()
        self.yaw = smsg.Float32()

        self.target.z = 0.5;
        self.baseline = 0.18;

        self.bl_pub = rospy.Publisher('baseline_', smsg.Float32, queue_size=1)
        self.zz_pub = rospy.Publisher('target_', geo.Point, queue_size=1)
        self.kl_pub = rospy.Publisher('kill_', smsg.Bool, queue_size=1)
        self.st_pub = rospy.Publisher('start_', smsg.Bool, queue_size=1)
        self.yw_pub = rospy.Publisher('yaw_target_', smsg.Float32, queue_size=1)

        self.a = 0;
        self.b = 0;
        self.y = 0;
        self.x = 0;
        self.Lb = 0;
        self.Rb = 0;
        self.Lt = 0;
        self.Rt = 0;
        self.plus = 0;
        self.minus = 0;
        self.upDown = 0;
        self.leftRight = 0;
    
    def publish(self):
        self.bl_pub.publish(self.baseline)
        self.zz_pub.publish(self.target)
        self.kl_pub.publish(self.kill)
        self.st_pub.publish(self.start)
        self.yw_pub.publish(self.yaw)

    def joyCb(self, msg):
        if msg.buttons[1] == 0 and self.a == 1:
            self.aRelease()
        elif msg.buttons[1] == 1 and self.a == 0:
            self.aPress()
        self.a = msg.buttons[1]

        if msg.buttons[0] == 0 and self.b == 1:
            self.bRelease();
        elif msg.buttons[0] == 1 and self.b == 0:
            self.bPress();
        self.b = msg.buttons[0]

        if msg.buttons[2] == 0 and self.y == 1:
            self.yRelease();
        elif msg.buttons[2] == 1 and self.y == 0:
            self.yPress();
        self.y = msg.buttons[2]

        if msg.buttons[3] == 0 and self.x == 1:
            self.xRelease();
        elif msg.buttons[3] == 1 and self.x == 0:
            self.xPress();
        self.x = msg.buttons[3]

        if msg.buttons[4] == 0 and self.Lb == 1:
            self.LbRelease();
        elif msg.buttons[4] == 1 and self.Lb == 0:
            self.LbPress();
        self.Lb = msg.buttons[4]
        
        if msg.buttons[5] == 0 and self.Rb == 1:
            self.RbRelease();
        elif msg.buttons[5] == 1 and self.Rb == 0:
            self.RbPress();
        self.Rb = msg.buttons[5]

        if msg.buttons[6] == 0 and self.Lt == 1:
            self.LtRelease();
        elif msg.buttons[6] == 1 and self.Lt == 0:
            self.LtPress();
        self.Lt = msg.buttons[6]

        if msg.buttons[7] == 0 and self.Rt == 1:
            self.RtRelease();
        elif msg.buttons[7] == 1 and self.Rt == 0:
            self.RtPress();
        self.Rt = msg.buttons[7]

        if msg.buttons[8] == 0 and self.minus == 1:
            self.minusRelease();
        elif msg.buttons[8] == 1 and self.minus == 0:
            self.minusPress();
        self.minus = msg.buttons[8]

        if msg.buttons[9] == 0 and self.plus == 1:
            self.plusRelease();
        elif msg.buttons[9] == 1 and self.plus == 0:
            self.plusPress();
        self.plus = msg.buttons[9]

        if msg.axes[4] > 0 and self.leftRight <= 0:
            self.leftPress()
        elif msg.axes[4] <= 0 and self.leftRight > 0:
            self.leftRelease()
        elif msg.axes[4] < 0 and self.leftRight >= 0:
            self.rightPress()
        elif msg.axes[4] >= 0 and self.leftRight < 0:
            self.rightRelease()

        if msg.axes[5] > 0 and self.upDown <= 0:
            self.upPress()
        elif msg.axes[5] <= 0 and self.upDown > 0:
            self.upRelease()
        elif msg.axes[5] < 0 and self.upDown >= 0:
            self.downPress()
        elif msg.axes[5] >= 0 and self.upDown < 0:
            self.downRelease()

        self.joystick = msg

    def aPress(self):
        print "Pressed a"

    def aRelease(self):
        print "Released a"

    def bPress(self):
        self.kill.data = True

    def bRelease(self):
        print "Released b"

    def yPress(self):
        print "Pressed y"

    def yRelease(self):
        print "Released y"
        
    def xPress(self):
        print "Pressed x"

    def xRelease(self):
        print "Released x"

    def LbPress(self):
        self.yaw.data += 5

    def LbRelease(self):
        print "Released Lb"

    def RbPress(self):
        self.yaw.data -= 5

    def RbRelease(self):
        print "Released Rb"

    def LtPress(self):
        self.target.z -= 0.5

    def LtRelease(self):
        print "Released Lt"

    def RtPress(self):
        self.target.z += 0.5

    def RtRelease(self):
        print "Released Rt"

    def minusPress(self):
        print "Pressed minus"

    def minusRelease(self):
        print "Released minus"

    def plusPress(self):
        self.start.data = True

    def plusRelease(self):
        print "Released plus"
        
    def leftPress(self):
        self.target.x -= 1.0
        if self.target.x < -1.0:
            self.target.x = -1.0

    def leftRelease(self):
        print "Released left"
        
    def rightPress(self):
        self.target.x += 1.0
        if self.target.x > 1.0:
            self.target.x = 1.0

    def rightRelease(self):
        print "Released right"

    def upPress(self):
        self.target.y -= 1.0
        if self.target.y < -1.0:
            self.target.y = -1.0

    def upRelease(self):
        print "Released up"

    def downPress(self):
        self.target.y += 1.0
        if self.target.y > 1.0:
            self.target.y = 1.0

    def downRelease(self):
        print "Released down"


def main():
    rospy.init_node('Joystick_Control')
    controller = Controller();

    rospy.Subscriber('joy', sens.Joy, controller.joyCb)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.publish()
        r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
