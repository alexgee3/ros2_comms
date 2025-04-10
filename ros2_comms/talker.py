import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import getch


class KeyboardTalker(Node):
    def __init__(self):
        super().__init__('keyboard_commands')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Keyboard Talker Node Started. '
                               'Type to send messages. Press Esc to quit.')

    def start_publishing(self):
        while rclpy.ok():
            key = getch.getch()
            if key == '\x1b':  # Escape key
                self.get_logger().info('Exiting keyboard talker')
                break
            msg = String()
            msg.data = key
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    keyboard_talker = KeyboardTalker()
    try:
        keyboard_talker.start_publishing()
    finally:
        keyboard_talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
