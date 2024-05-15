#!/usr/bin/env python3
import math
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):

        self.pid_p = 0.1  
        self.pid_i = 0.0  
        self.pid_d = 0.0  
        self.error_sum = 0.0
        self.last_error = 0.0

        rospy.init_node('target_follower_node', anonymous=True)

        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/duckienadav/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckienadav/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin() 


    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            return
        tag_pos_x = detections[0].transform.translation.x

        tag_height = detections[0].transform.translation.z

        desired_height = 0.4
        z_error = desired_height - tag_height
        z_correction = self.pid_controller(z_error)

      
        if abs(z_error) <= 0.05: 
            linear_velocity = 0.0  
        elif z_error > 0:
            linear_velocity = 0.1  
        else:
            linear_velocity = -0.1  


        Kp_centering = 0.1  #
        angular_velocity = -Kp_centering * tag_pos_x  

        max_angular_velocity = 1.5  
        if abs(angular_velocity) > max_angular_velocity:
            angular_velocity = max_angular_velocity if angular_velocity > 0 else -max_angular_velocity

        # Publishing velocity commands
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = linear_velocity 
        cmd_msg.omega = angular_velocity
        self.cmd_vel_pub.publish(cmd_msg)
        
    def pid_controller(self, error):
        p_term = self.pid_p * error
        self.error_sum += error
        i_term = self.pid_i * self.error_sum
        d_term = self.pid_d * (error - self.last_error)
        self.last_error = error

        output = p_term + i_term + d_term
        return output
    
if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass