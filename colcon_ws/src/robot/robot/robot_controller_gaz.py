
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Float32
from geometry_msgs.msg import Twist
import numpy as np 
import time 
import threading
from math import pi

from tools.InverseKinematics import InverseKinematics
from RobotController.Robot import Robot



# Robot geometry
body = [0.559, 0.12]
legs = [0.,0.1425, 0.426, 0.345]


spot_robot = Robot(body, legs)
inverseKinematics = InverseKinematics(body, legs)

class Robot_controller_gaz(Node):
    def __init__(self):
        super().__init__("robotController")
        self.time_interval = 0.015
        self.publisher = self.create_publisher(Float64MultiArray,  '/forward_position_controller/commands', 10)
        self.timer = self.create_timer(self.time_interval, self.timer_callback)


    def timer_callback(self):
        leg_pos = spot_robot.run()
        #spot_robot.
        dx = spot_robot.state.body_local_position[0]
        dy = spot_robot.state.body_local_position[1]
        dz = spot_robot.state.body_local_position[2]

        roll = spot_robot.state.body_local_orientation[0]
        pitch = spot_robot.state.body_local_orientation[1]
        yaw = spot_robot.state.body_local_orientation[2]
        
        try:
           
            joint_angles = inverseKinematics.inverse_kinematic(leg_pos,
                                                                dx, dy, dz, 
                                                                roll, pitch, yaw)
            print(joint_angles)
            
            joint_angles = [joint_angles[0],joint_angles[1], -(pi - joint_angles[2]), 
                            joint_angles[3],joint_angles[4], -(pi - joint_angles[5]), 
                            joint_angles[6],joint_angles[7], -(pi - joint_angles[8]),
                            joint_angles[9],joint_angles[10], -(pi -joint_angles[11])]

            print(joint_angles)               
            pos_array = Float64MultiArray(data=joint_angles)
            self.publisher.publish(pos_array)
        except:
            import traceback
            traceback.print_exc()
            rclpy.logging._root_logger.info(f"Can not solve inverese kinematics")


def main():
    rclpy.init(args=None)
    controller = Robot_controller_gaz()


    exect = rclpy.executors.MultiThreadedExecutor()
    exect.add_node(controller)

    exect_thread = threading.Thread(target=exect.spin, daemon = True)
    exect_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()