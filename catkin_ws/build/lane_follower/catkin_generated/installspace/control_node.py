import rospy
from std_msgs.msg import Float32, Int16

class ControlNode:
    """A class to control the direction and spped of the turtlebot
    1. Subscribe the /lane_offset topic to get the offset of the lane
    2. Set the standard to determine the direction
        a. If the offset is less than abs(1), go straight
        b. If the offset is less than -2, turn left 
        c. If the offset is less than 2, turn right
    
    """
