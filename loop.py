#!/usr/bin/env python3
import rclpy
import math
# import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
# importing the required module
import matplotlib.pyplot as plt
import time
import datetime as dt
class movecircleNode(Node):
    #global step
    #global y
    #radius_in_meter=4.0
    
    def __init__(self):
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0
        self.wheel_seperation=3.10
        self.wheel_radius=0.05
        super().__init__("move_circle")
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pub2 = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.sub = self.create_subscription(Odometry, '/wheel/odometry', 
                                            self.odom_callback, 10)
        self.get_init_position()

    def odom_callback(self, msg: Odometry):
        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        NewPosition = msg.pose.pose.position
        radius_in_meter=1.0

        # Calculate the new distance moved, and add it to _mved_distance and 
        self._mved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        print(self._mved_distance)
        if self._mved_distance.data>float(2*3.14*radius_in_meter):
            # print(dist)
            # print("to radius")
            twist=Twist()
            pose=Pose()
            twist.linear.x = 1.0 # linear velocity in m/s
            # twist.linear.y = 0.0
            twist.angular.z = -1.0  # angular velocity in rad/s
            # msg.pose.pose.orientation.z=1.57
            self.pub2.publish(msg)
            self.pub.publish(twist)
            # print(self._mved_distance.data)
            velocity_left=(twist.linear.x-(self.wheel_seperation/2)*twist.angular.z)/self.wheel_radius
            velocity_right=(twist.linear.x+(self.wheel_seperation/2)*twist.angular.z)/self.wheel_radius
            rpm_left=(60*velocity_left)/(2*3.14)
            rpm_right=(60*velocity_right)/(2*3.14)
            print("RPM of left wheel:"+str(rpm_left))

            print("RPM of right wheel:"+str(rpm_right))
            
            # y1=[0,1,0,0,10,12,15]
            # # y2=[]
            # xs=[1,0,12,15,12,18,17]
            # # xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
            # # xs = xs[-20:]
            # # y1 = y1[-20:]
            # # y1.append(rpm_left)
            # # y2.append(rpm_right)
            # fig, ax = plt.subplots()
            # ax.plot(xs, y1)
            # # ax.plot(xs, rpm_right)

# Add labels and legend
            # ax.set_xlabel('Time (s)')
            # ax.set_ylabel('Se Data')
            # ax.set_title('Sensor Data vs. Time')
            # ax.legend()

           # Show the plot
            # plt.show()


        # elif self._mved_distance.data>=(float(radius_in_meter)-0.01) or self._mved_distance.data<=(float(radius_in_meter)+0.01):
        #     print("rotating")
        #     cmd=Odometry()
        #     cmd.pose.pose.orientation.x=0.0701
        #     cmd.pose.pose.orientation.y=0.0701
        #     cmd.pose.pose.orientation.z=0.0
        #     cmd.pose.pose.orientation.w=0.0
        #     twist=Twist()
        #     # twist.linear.x = 0.0 # linear velocity in m/s
        #     # twist.angular.z = 0.1
        #     self.pub.publish(twist)
        #     self.pub2.publish(cmd)
        else:
            twist=Twist()
            twist.linear.x = 1.0# linear velocity in m/s
            twist.angular.z = 1.0 # angular velocity in rad/s
            # velocity_left=twist.angular.z(radius_in_meter-(self.wheel_seperation/2))
            # velocity_right=twist.angular.z(radius_in_meter+(self.wheel_seperation/2))
            
            # print("in cirle")
            velocity_left=(twist.linear.x+(self.wheel_seperation/2)*twist.angular.z)/self.wheel_radius
            velocity_right=(twist.linear.x-(self.wheel_seperation/2)*twist.angular.z)/self.wheel_radius
            rpm_left=(60*velocity_left)/(2*3.14)
            rpm_right=(60*velocity_right)/(2*3.14)
            print("RPM of left wheel:"+str(rpm_left))

            print("RPM of right wheel:"+str(rpm_right))
            # print(self._mved_distance.data)
            self.pub.publish(twist)
        
        # Update the current position of the robot so we have a new reference point
        # (The robot has moved and so we need a new reference for calculations)
        self.updatecurrent_position(NewPosition)
        

      
    def updatecurrent_position(self, new_position):
        """Update the current position of the robot."""
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z
        

    def get_init_position(self):
         data_odom = Odometry()
         self._current_position = Point()
         self._current_position.x = data_odom.pose.pose.position.x
         self._current_position.y = data_odom.pose.pose.position.y
         self._current_position.z = data_odom.pose.pose.position.z


    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = float(abs(x2 - x1))
        return dist
    

def main(args=None):
    rclpy.init(args=args)
    node = movecircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

   
if __name__== '__main__':
    main()   
    
   

   




