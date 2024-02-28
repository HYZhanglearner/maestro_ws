import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pyspacemouse
import time

class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__('space_mouse_publisher')
        self.publisher = self.create_publisher(Joy, 'spaceMouseMotion', 10)
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error('Failed to open SpaceMouse')
            exit(1)

    def publish_space_mouse_data(self):
        rate = self.create_rate(100)  # Adjust the rate as needed
        while rclpy.ok():
            state = pyspacemouse.read()
            joy_msg = Joy()
            joy_msg.axes = [state.x, state.y, state.z, state.roll, state.yaw, state.pitch]
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(joy_msg)
            self.get_logger().info(f'Published: {joy_msg.axes}')
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    space_mouse_publisher = SpaceMousePublisher()
    try:
        space_mouse_publisher.publish_space_mouse_data()
    except KeyboardInterrupt:
        pass
    finally:
        space_mouse_publisher.destroy_node()
        rclpy.shutdown()
        pyspacemouse.close()

if __name__ == '__main__':
    main()
