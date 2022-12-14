import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray

class padwqController(Node):

    def __init__(self):
        super().__init__('padwq_controller')

        self.joint_angs = Float64MultiArray()
        self.rx_data = []
        self.prev_joint_angs = None

        self.joint_angs.data = [
            # 0.0, -np.pi/4, np.pi/3,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        # self.sub = self.create_subscription(Float32MultiArray, 'padwq_joint_controller/commands', self.sub_callback, 30)
        self.pub = self.create_publisher(Float64MultiArray, 'gazebo_joint_controller/commands', 30)
        timer_period = 0.01
        self.timer_pub = self.create_timer(timer_period, self.pub_callback)

    def sub_callback(self, msg):
        self.rx_data = []
        for data in msg.data:
            self.rx_data.append(data * np.pi/180)
        self.joint_angs.data = self.rx_data
        self.pub.publish(self.joint_angs)

    def pub_callback(self):
        # self.pub.publish(self.joint_angs)
        pass

def main(args=None):
    rclpy.init(args=args)

    padwq_controller = padwqController()
    rclpy.spin(padwq_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
