#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray, WheelEncoderStamped

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        
        # Encoder values
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.ticks_per_meter = 135  # Example value, this needs to be calibrated for your robot

        self.rate = rospy.Rate(10)  # 10 Hz rate for publishing distance

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "duckiebot" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/duckienadav/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/duckienadav/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/duckienadav/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/duckienadav/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/duckienadav/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        for detection in msg.detections:
            distance = detection.transform.translation.z
            rospy.loginfo(f"Distance to object: {distance:.2f} meters")
            self.rate.sleep()
        
        self.move_robot(msg.detections)
    
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

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        for detection in detections:
            distance = detection.transform.translation.z
            if distance < 0.2:
                self.stop_robot()
                rospy.sleep(5)  # Wait for 5 seconds

                # Check if the object is still there
                if self.is_object_still_there():
                    self.set_mode("NORMAL_JOYSTICK_CONTROL")  # Stop lane following
                    self.perform_overtake()
                    self.set_mode("LANE_FOLLOWING")  # Resume lane following
                else:
                    self.drive_straight()
                break

    def is_object_still_there(self):
        rospy.sleep(1)  # Short delay to allow new detection messages to arrive
        msg = rospy.wait_for_message('/duckiebot/apriltag_detector_node/detections', AprilTagDetectionArray, timeout=5.0)
        if len(msg.detections) == 0:
            return False

        for detection in msg.detections:
            distance = detection.transform.translation.z
            if distance < 0.2:
                return True
        return False

    def set_mode(self, mode):
        mode_msg = FSMState()
        mode_msg.header.stamp = rospy.Time.now()
        mode_msg.state = mode
        self.state_pub.publish(mode_msg)

    def perform_overtake(self):
        self.execute_turn(45)  # Turn left 45 degrees
        self.drive_straight_distance(0.5)  # Move forward 0.5 meters
        self.execute_turn(-45)  # Turn right 45 degrees
        self.drive_straight_distance(0.2)  # Move forward 0.2 meters
        self.execute_turn(-45)  # Turn right 45 degrees
        self.drive_straight_distance(0.5)  # Move forward 0.5 meters
        self.execute_turn(45)  # Turn left 45 degrees

    def execute_turn(self, angle):
        duration = 1.0  # Set a fixed duration for the turn
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = angle / duration
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(duration)
        self.stop_robot()

    def drive_straight_distance(self, distance):
        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks
        target_ticks = distance * self.ticks_per_meter

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.2  # Set a fixed speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        while not rospy.is_shutdown():
            left_delta = self.left_ticks - self.left_ticks_prev
            right_delta = self.right_ticks - self.right_ticks_prev

            if left_delta >= target_ticks and right_delta >= target_ticks:
                break

            rospy.sleep(0.01)  # Small sleep to prevent hogging the CPU

        self.stop_robot()

    def drive_straight(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.2  # Set a fixed speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(1.0)  # Duration for driving straight
        self.stop_robot()

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
