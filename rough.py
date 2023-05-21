#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class MovementDetector(object):
    def __init__(self):
        """Initialize an object of the MovementDetector class."""
        # _mved_distance is for stored distance moved
        # create and initialize it here. Initial value is 0.0
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0

        # Get the inital position. This will be a reference point for calculating
        # the distance moved 
        self.get_init_position()

        # Create a publisher for publishing the distance moved into the topic '/moved_distance'
        self.distance_moved_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)

        # create a subscriber for getting new Odometry messages
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def get_init_position(self):
       
        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")
        
        # Store the received odometry "position" variable in a Point instance 
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self, msg):
        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position

        # Calculate the new distance moved, and add it to _mved_distance and 
        self._mved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        
        # Update the current position of the robot so we have a new reference point
        # (The robot has moved and so we need a new reference for calculations)
        self.updatecurrent_positin(NewPosition)
        
        # If distance moved is big enough, publish it to the designated topic
        # Otherwise publish zero
        if self._mved_distance.data < 0.000001:
            aux = Float64()
            aux.data = 0.0
            self.distance_moved_pub.publish(aux)
        else:
            self.distance_moved_pub.publish(self._mved_distance)

    def updatecurrent_positin(self, new_position):
        """Update the current position of the robot."""
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

    def publish_moved_distance(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    # create a node for running the program
    rospy.init_node('movement_detector_node', anonymous=True)

    # create an instance of the MovementDetector class and set the code
    # in motion
    movement_obj = MovementDetector()
    movement_obj.publish_moved_distance()

# test_movement/src/test_movement.py

# #!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float64

# def my_callback(msg):
#     """Callback function that processes messages from the subscriber."""

#     # get the distance moved from the message
#     distance_moved = msg.data

#     # If distance is less than 2, continue moving the robot
#     # Otherwise, stop it (by pubishing `0`)
#     if msg.data < 2:
#         move.linear.x = 0.1

#     if msg.data > 2:
#         move.linear.x = 0

#     pub.publish(move)

# # create a node for running the program
# rospy.init_node('test_movement')

# # create a subscriber that gets the distance moved
# sub = rospy.Subscriber('/moved_distance', Float64, my_callback)

# # Create a publisher that moves the robot
# pub = rospy.Publisher('/cmd_vel', Twist, queue_size="1")

# # Create a global variable for publising a Twist ("cmd_vel") message 
# move = Twist()

# # Keep the program running
# rospy.spin()
