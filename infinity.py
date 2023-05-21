#!/usr/bin/env python3
from ftplib import MSG_OOB
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class movecircleNode(Node):

    
    global X
    global Y
    global Z
    global counter
    
    def __init__(self):
        self._mved_distance = Float64()
        self._mved_distance.data = 0.0
        self.wheel_seperation=3.10
        self.wheel_radius=0.05
        self.counter=1
        super().__init__("move_circle")
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.pub2 = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.sub = self.create_subscription(Odometry, '/wheel/odometry', 
                                            self.odom_callback, 10)
        (self.X,self.Y,self.Z)=self.get_init_position()
        # X=self.pose.pose.position.x
        # Y=self.msg.pose.pose.position.x
        # print(X)
        # print(Y)



    def odom_callback(self, msg: Odometry):
        print("mohit")
        # Process odometry data sent by the subscriber.
        # Get the position information from the odom message
        print(self.counter)
    

        # Calculate the new distance moved, and add it to _mved_distance and 
        # self._mved_distance.data += self.calculate_distance(NewPosition, self._current_position)
        # print(self._mved_distance)
        # var= self.updatecurrent_position(NewPosition)
        # global x
        # global y
        # return self.msg.pose.pose.position.x as x
        # return self.msg.pose.pose.position.y=y

        # # while True:
        # twist=Twist()
        # twist.linear.x = 1.0# linear velocity in m/s
        # # twist.linear.y = 0.0
        # twist.angular.z = 0.1  # angular velocity in rad/s
        # # msg.pose.pose.orientation.z=1.57
        # self.pub2.publish(msg)
        # self.pub.publish(twist)
        # counter=1
        # # print(msg.pose.pose.position.x)

        print(msg.pose.pose.position.x)
        print("initial",self.X)



        if math.floor(abs(msg.pose.pose.position.x))==self.X and math.floor(abs(msg.pose.pose.position.y))==self.Y and self.counter==0:
        # print(dist)
        # print("to radius")
            twist=Twist()
        # pose=Pose()
            print("loop1")
            twist.linear.x = 0.2 # linear velocity in m/s
        # twist.linear.y = 0.0
            twist.angular.z = 0.1  # angular velocity in rad/s
        # msg.pose.pose.orientation.z=1.57
            self.pub2.publish(msg)
            self.pub.publish(twist)
            self.counter=1
            print(self.counter)

        if math.floor(abs(msg.pose.pose.position.x))==self.X and math.floor(abs(msg.pose.pose.position.y))==self.Y and self.counter==1:
            twist=Twist()
            twist.linear.x = 0.2# linear velocity in m/s
            twist.angular.z = -0.1 # angular velocity in rad/s
            print("loop2")
            self.pub.publish(twist)
            self.counter=0
            print(self.counter)
    #     # self.updatecurrent_position(NewPosition)
        

      
    # # def updatecurrent_position(self, new_position):
    # #     """Update the current position of the robot."""
    # #     self._current_position.x = new_position.x
    # #     self._current_position.y = new_position.y
    # #     self._current_position.z = new_position.z
    #     # global prod
    #     # prod=new_position.x*new_position.y*new_position.z
    #     # return prod

    def get_init_position(self):
         data_odom = Odometry()
         self._current_position = Point()
         return (data_odom.pose.pose.position.x,data_odom.pose.pose.position.y,data_odom.pose.pose.position.z)
        #  self._current_position.x = data_odom.pose.pose.position.x
        #  self._current_position.y = data_odom.pose.pose.position.y
        #  self._current_position.z = data_odom.pose.pose.position.z
         


    # def calculate_distance(self, new_position, old_position):
    #     """Calculate the distance between two Points (positions)."""
    #     x2 = new_position.x
    #     x1 = old_position.x
    #     y2 = new_position.y
    #     y1 = old_position.y
    #     dist = math.hypot(x2 - x1, y2 - y1)
    #     return dist
    

def main(args=None):
    rclpy.init(args=args)
    node = movecircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

   
if __name__== '__main__':
    main()   
    
   

   




