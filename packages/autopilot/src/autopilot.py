#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        
        self.stop_flag = False
        
        # Encoder values
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.ticks_per_meter = 135  # Example value, this needs to be calibrated for your robot

        self.rate = rospy.Rate(10)  # 10 Hz rate for publishing distance
        self.tof_distance = float('inf')

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "duckienadav" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/duckienadav/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/duckienadav/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/duckienadav/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckienadav/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/duckienadav/front_center_tof_driver_node/range', Range, self.tof_callback)
        ################################################################


        rospy.spin() # Spin forever but listen to message callbacks

    def tof_callback(self, msg):
        self.tof_distance = msg.range
        rospy.loginfo(f"Distance to object: {self.tof_distance:.2f} meters")

        if self.tof_distance <= 0.2 and self.stop_flag == False:
            rospy.loginfo(f"Under 0.2M: stopped and changed flag state = {self.stop_flag} ")
            self.move_robot()
        

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
    
    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self):      
        rospy.loginfo(f"move called: stop flag state = {self.stop_flag} ")
        self.stop_robot()
        rospy.sleep(2)  # Wait for 2 seconds
        

        self.stop_flag = True

        # Check if the object is still there
        # if self.tof_distance < 0.2:
        self.set_mode("NORMAL_JOYSTICK_CONTROL")  # Stop lane following
        self.perform_overtake()
        self.set_mode("LANE_FOLLOWING")  # Resume lane following
            # else:
            #     self.drive_straight()
        # elif self.tof_distance > 0.2 and self.stop_flag:
        #     self.drive_straight()
        #     self.stop_flag = False

    def set_mode(self, mode):
        rospy.loginfo(f"Toggle mode ")

        mode_msg = FSMState()
        mode_msg.header.stamp = rospy.Time.now()
        mode_msg.state = mode
        self.state_pub.publish(mode_msg)
        rospy.loginfo(f"NEW MODE = {mode_msg.state} ")


    def perform_overtake(self):
        rospy.loginfo(f"OVERTAKE ")

        self.execute_turn(45)  # Turn left 45 degrees
        self.drive_straight_distance(0.5)  # Move forward 0.5 meters
        self.execute_turn(-45)  # Turn right 45 degrees
        self.drive_straight_distance(0.2)  # Move forward 0.2 meters
        self.execute_turn(-45)  # Turn right 45 degrees
        self.drive_straight_distance(0.5)  # Move forward 0.5 meters
        self.execute_turn(45)  # Turn left 45 degrees
        self.stop_flag = False

    def execute_turn(self, angle):
        rospy.loginfo(f"TURN - Angle {angle} ")

        duration = 1.0  # Set a fixed duration for the turn
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.3
        cmd_msg.omega =  - 8.3 # angle / duration
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(3)
        self.stop_robot()

    def drive_straight_distance(self, distance):
        rospy.loginfo(f"Straight drive {distance} ")

        # self.left_ticks_prev = self.left_ticks
        # self.right_ticks_prev = self.right_ticks
        # target_ticks = distance * self.ticks_per_meter

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.4  # Set a fixed speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        # while not rospy.is_shutdown():
        #     left_delta = self.left_ticks - self.left_ticks_prev
        #     right_delta = self.right_ticks - self.right_ticks_prev

        #     if left_delta >= target_ticks and right_delta >= target_ticks:
        #         break

        rospy.sleep(2)  # Small sleep to prevent hogging the CPU

        self.stop_robot()

    def drive_straight(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.4  # Set a fixed speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
