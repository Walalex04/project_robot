
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Float32
from geometry_msgs.msg import Twist
import numpy as np 
import time 


class Robot_controller_gaz(Node):

    def __init__(self):
        super().__init__("robot_controller_gaz")
        self.get_logger().info("initializing the robot controller with gazebo")

        #subcribe 
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_state',
            self.joint_state_callback,
            10
        )


        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        positions = [
                -0.5, 0.3, -0.5,  # FR levantada
                -0.5, 0.3, -0.5,  # FL normal
                -0.5, 0.3, -0.5,   # RR normal
                -0.5, 0.3, -0.5    # RL levantada
        ]
        self.send_commands(positions)


    def send_commands(self, position):
        msg = Float64MultiArray()
        msg.data = position
        self.state_pub.publish(msg)
        print(f"send the next command to position_controller {position}")


    def joint_state_callback(self):
        pass



def main():

    rclpy.init()

    robot = Robot_controller_gaz()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("robot contoller is finishing ...")

    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
