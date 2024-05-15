#!/usr/bin/env python3
import math
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):

        # Initialize PID controller parameters for maintaining a desired height
        self.pid_p = 0.1  # Proportional gain
        self.pid_i = 0.0  # Integral gain
        self.pid_d = 0.0  # Derivative gain
        self.error_sum = 0.0
        self.last_error = 0.0

        #Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/duckienadav/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckienadav/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks


    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
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

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        # Extracting tag position from the first detection
        tag_pos_x = detections[0].transform.translation.x

        # Extracting tag height from the first detection
        tag_height = detections[0].transform.translation.z

        # PID control for maintaining a desired height
        desired_height = 0.4
        z_error = desired_height - tag_height
        z_correction = self.pid_controller(z_error)

        # Check if the tag is within acceptable height range
        if abs(z_error) <= 0.05:  # Adjust tolerance as needed
            linear_velocity = 0.0  # Stop moving if within acceptable height range
        elif z_error > 0:
            linear_velocity = 0.1  # Move forwards if tag is below desired height
        else:
            linear_velocity = -0.1  # Move backwards if tag is above desired height

        # Proportional control for centering the tag
        Kp_centering = 0.1  # Tune this value according to the desired response
        angular_velocity = -Kp_centering * tag_pos_x  # Negative sign to move towards the tag

        # Limiting angular velocity to prevent excessive movement
        max_angular_velocity = 1.5  # Define your maximum angular velocity
        if abs(angular_velocity) > max_angular_velocity:
            angular_velocity = max_angular_velocity if angular_velocity > 0 else -max_angular_velocity

        # Publishing velocity commands
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = linear_velocity  # Use linear velocity for distance control
        cmd_msg.omega = angular_velocity
        self.cmd_vel_pub.publish(cmd_msg)
        
    def pid_controller(self, error):
        # Proportional term
        p_term = self.pid_p * error

        # Integral term
        self.error_sum += error
        i_term = self.pid_i * self.error_sum

        # Derivative term
        d_term = self.pid_d * (error - self.last_error)
        self.last_error = error

        # PID output
        output = p_term + i_term + d_term
        return output
    
        #############################

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass