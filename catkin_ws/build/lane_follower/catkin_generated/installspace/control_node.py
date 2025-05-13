import rospy
from std_msgs.msg import Float32
from lane_follower.msg import MotorPWM_msg

class ControlNode:
    """A class to control the direction and spped of the turtlebot
    1. Subscribe the /lane_offset topic to get the offset of the lane
    2. Set the standard to determine the direction
    3. Determined the direction and corresponded to specific pwm of right and left motor
    """
    def __init__(self):
        # Initialize the node
        rospy.init_node('control_node', anonymous=True)
        # Create a subsciber to the /lane_offset topic
        rospy.Subscriber('/lane_offset', Float32, self.offset_callback)
        # Create a publisher to the /motor_pwm topic
        self.motor_pwm_pub = rospy.Publisher('/motor_pwm', MotorPWM_msg, queue_size=10)
        # Set the rate of the node to 10Hz
        self.rate = rospy.Rate(10)

        # Set the parameter for the motor
        self.base_speed = rospy.get_param('~base_speed', 100)
        self.max_pwm = rospy.get_param('~max_pwm', 255)
        self.min_pwm = rospy.get_param('~min_pwm', 0)

        # Set the parameter for the offset to PWM
        self.k_p = rospy.get_param('k_p', 0.5)
        # If offset is too small. just go straight
        self.deadzone = rospy.get_param('~deadzone', 10)

        # Make loginfo to show the node is running
        rospy.loginfo(f"Two-wheeled control is initialized with base_speed = {self.base_speed}, kp = {self.k_p}")
        rospy.spin()

    def offset_callback(self, msg):
        """
        Create a standard to determine the pwm of the right and left motor
        1. If the offset is smaller than the deadzone, just go straight
        2. If the offset is larger than the deadzone, determine the direction
        using the sign of the offset, and value is k_p * offset
        3. If the offset is larger than the max_pwm, set the pwm to max_pwm
        4. If the offset is smaller than the min_pwm, set the pwm to min_pwm
        """ 
        # Create a MotorPWM_msg object to precalculate the pwm of the right and left motor
        motor_pwm = MotorPWM_msg()
        # Get the offset from the message
        offset = msg.data
        # Check if the offset is smaller than the deadzone
        if abs(offset) < self.deadzone:
            # Go straight
            motor_pwm.left_pwm = self.base_speed
            motor_pwm.right_pwm = self.base_speed
        else:
            # Determine the direction based on the sign of the offset
            # If the offset is positive, turn left
            # If the offset is negative, turn right
            if offset > 0:
                motor_pwm.left_pwm = max(self.base_speed - self.k_p*offset, self.min_pwm)
                motor_pwm.right_pwm = min(self.base_speed + self.k_p*offset, self.max_pwm)
            else:
                motor_pwm.left_pwm = min(self.base_speed + self.k_p*offset, self.max_pwm)
                motor_pwm.right_pwm = max(self.base_speed - self.k_p*offset, self.min_pwm)
            
            # Restrict the pwm to be between min_pwm and max_pwm
            # And convert to int, since motor_pwm is an int
            motor_pwm.left_pwm = int(max(min(motor_pwm.left_pwm, self.max_pwm), self.min_pwm))
            motor_pwm.right_pwm = int(max(min(motor_pwm.right_pwm, self.max_pwm), self.min_pwm))

            # Publish the motor_pwm message
            self.motor_pwm_pub.publish(motor_pwm)

            rospy.loginfo(f"Motor PWM: left_pwm = {motor_pwm.left_pwm}, right_pwm = {motor_pwm.right_pwm}, offset = {offset}")
if __name__ == '__main__':
    try:
        ControlNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Control node terminated.")

