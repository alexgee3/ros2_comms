import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading


class Listener(Node):
    def __init__(self):
        super().__init__('listener')

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

        if self.ser and self.ser.is_open:
            self.uart_thread = threading.Thread(target=self.read_uart,
                                                daemon=True)
            self.uart_thread.start()

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Send corresponding command to ESP32 over UART
        if command == 'w':  # Drive forward
            self.send_uart_command(0xA1)
        elif command == 'a':  # Rotate left
            self.send_uart_command(0xA2)
        elif command == 's':  # Drive backward
            self.send_uart_command(0xA3)
        elif command == 'd':  # Rotate right
            self.send_uart_command(0xA4)
        elif command == ' ':  # Stop
            self.send_uart_command(0xA5)
        elif command == 'j':  # Decrease forward duty cycle by 10
            self.send_uart_command(0xA6)
        elif command == 'u':  # Increase forward duty cycle by 10
            self.send_uart_command(0xA7)
        elif command == 'k':  # Decrease rotate duty cycle by 10
            self.send_uart_command(0xA8)
        elif command == 'i':  # Increase rotate duty cycle by 10
            self.send_uart_command(0xA9)
        elif command == 'l':  # Decrease backward duty cycle by 10
            self.send_uart_command(0xAA)
        elif command == 'o':  # Increase backward duty cycle by 10
            self.send_uart_command(0xAB)
        elif command == '1':  # Request Battery Command 1
            self.send_uart_command(0x01)
        elif command == '2':  # Request Battery Command 2
            self.send_uart_command(0x02)
        elif command == '3':  # Request Battery Command 3
            self.send_uart_command(0x03)

    def send_uart_command(self, command_byte):
        if self.ser:
            try:
                self.ser.write(bytes([command_byte]))
                self.get_logger().info(f"Sent command: {command_byte}")
            except serial.SerialException as a:
                self.get_logger().error(f'Error sending UART command: {a}')

    def close_uart(self):
        self.ser.close()

    def read_uart(self):
        while True:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.readline().decode().strip()
                    self.get_logger().info(f"Received from ESP32: {data}")
                except serial.SerialException as a:
                    self.get_logger().error(f'UART read failed: {a}')
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
