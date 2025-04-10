import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class Listener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')

        # Create subscriber to listen to 'command' topic
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        try:
            # Set up UART communication
            self.ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
            self.get_logger().info('UART serial connection established')
        except serial.SerialException as a:
            self.get_logger().error(f'Failed to open serial port: {a}')
            self.ser = None

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Send corresponding command to ESP32 over UART
        if command == 'w':
            self.send_uart_command('0xA1')
        elif command == 'a':
            self.send_uart_command('0xA2')
        elif command == 's':
            self.send_uart_command('0xA3')
        elif command == 'd':
            self.send_uart_command('0xA4')
        elif command == '1':
            self.send_uart_command('0x01')
        elif command == '2':
            self.send_uart_command('0x02')
        elif command == '3':
            self.send_uart_command('0x03')

    def send_uart_command(self, command):
        if self.ser:
            try:
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info(f"Sent command: {command}")
            except serial.SerialException as a:
                self.get_logger().error(f'Error sending UART command: {a}')

    def close_uart(self):
        self.ser.close()

    def read_uart(self):
        while rclpy.ok() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode().strip()
                    if data:
                        self.get_logger().info(f"Received from ESP32: {data.decode()}")
            except serial.SerialException as a:
                self.get_logger().error(f'UART read filaed: {a}')
                break


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    try:
        rclpy.spin(listener)
    finally:
        listener.close_uart()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
