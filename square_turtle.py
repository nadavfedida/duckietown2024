#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist 
import time 

def move_turtle_square(): 
    rospy.init_node('turtlesim_square_node', anonymous=True)
    
    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    rospy.loginfo("Turtles are great at drawing squares!")

    ########## YOUR CODE GOES HERE ##########



    ###########################################

if __name__ == '__main__': 

    try: 
        move_turtle_square() 
    except rospy.ROSInterruptException: 
        pass
        
