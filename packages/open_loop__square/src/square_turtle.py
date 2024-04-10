#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class DuckiebotMovement:
    def __init__(self):
        rospy.init_node('duckiebot_movement_custom', anonymous=True)
        self.pub = rospy.Publisher('/duckiebot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber('/duckiebot/fsm_node/mode', FSMState, self.fsm_callback)
        self.cmd_msg = Twist2DStamped()

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            self.move_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_robot(self):
        for i in range(4):
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.41
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.sleep(2)
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = -8.3
            self.pub.publish(self.cmd_msg)
            rospy.sleep(0.2)

        self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = DuckiebotMovement()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
