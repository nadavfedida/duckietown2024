#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState, Range
from duckietown_msgs.msg import WheelIsCmdStamped, EncoderStamped

class DuckiebotMovement:
    self.distance = 0
    self.Min_range = 0.5

    def __init__(self):
        #self._left_encoder_topic
        rospy.init_node('duckiebot_movement_custom', anonymous=True)
        self.pub = rospy.Publisher('/duckienadav/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
        rospy.Subscriber('/duckienadav/fsm_node/mode', FSMState, self.fsm_callback)

        # subscribe to encoder topic - left wheel 
        rospy.Subscriber('/duckienadav/left_wheel_encoder_node/tick', EncoderStamped, encoder_callback)
        rospy.wait_for_message('/duckienadav/left_wheel_encoder_node/tick', EncoderStamped)


        # Subscribe to ToF distance finder
        #rospy.Subscriber('/duckienadav/front_center_tof_driver_node/range', Range, self.distance_check)
        self.cmd_msg = Twist2DStamped()

    #def distance_check(self, msg):
        #self.distance = msg.range    
        #add function to check distance and stop if less than X

    # Function to save wheel encoder values to global variables
    def encoder_callback(msg):
        global left_encoder_ticks
        global right_encoder_ticks
        left_encoder_ticks = msg.left_ticks
        right_encoder_ticks = msg.right_ticks



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
        #   Set initial tick values
        left_encoder_ticks = 0
        right_encoder_ticks = 0

        desired_ticks = 1000
        speed = 0.3
        while (left_encoder_ticks < desired_ticks):    
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
        self.stop_robot()




        # for i in range(4):
        #     while (self.distance > self.Min_range):    
        #         self.cmd_msg.header.stamp = rospy.Time.now()
        #         self.cmd_msg.v = 0.41
        #         self.cmd_msg.omega = 0.0
        #         self.pub.publish(self.cmd_msg)
        #         rospy.sleep(2)
        #         self.cmd_msg.header.stamp = rospy.Time.now()
        #         self.cmd_msg.v = 0.0
        #         self.cmd_msg.omega = -8.3
        #         self.pub.publish(self.cmd_msg)
        #         rospy.sleep(0.2)
        #     self.stop_robot()
        # self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = DuckiebotMovement()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

