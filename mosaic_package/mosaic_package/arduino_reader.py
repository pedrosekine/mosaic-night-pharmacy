#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial
import serial.tools.list_ports


class ArduinoReaderNode(Node):
    """
    ROS2 node that reads angle data from Arduino rotary sensor via serial
    and publishes it to a ROS2 topic.
    """

    def __init__(self):
        super().__init__('arduino_reader_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Default Arduino port on Linux
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)  # Hz

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.angle_publisher = self.create_publisher(Float32, '/arduino/rotary_angle', 10)
        self.medicine_publisher = self.create_publisher(String, '/arduino/medicine', 10)
        self.height_publisher = self.create_publisher(Float32, '/arduino/box_height', 10)

        # Try to connect to Arduino
        try:
            self.serial_connection = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout
            )
            self.get_logger().info(f'Successfully connected to Arduino on {serial_port}')
            self.get_logger().info(f'Baud rate: {baud_rate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino on {serial_port}: {e}')
            self.get_logger().info('Available serial ports:')
            for port in serial.tools.list_ports.comports():
                self.get_logger().info(f'  - {port.device}: {port.description}')
            raise

        # Create timer to read from Arduino
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.read_and_publish)

        self.get_logger().info(f'Arduino reader node started, publishing at {publish_rate} Hz')

    def read_and_publish(self):
        """Read angle data from Arduino and publish to ROS2 topic."""
        try:
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode('utf-8').strip()

                if line.startswith('Angle:'):
                    # Parse "Angle: 95"
                    try:
                        angle = float(line.split(':')[1].strip())
                        msg = Float32()
                        msg.data = angle
                        self.angle_publisher.publish(msg)

                        # Calculate and publish box height
                        height_m = self.angle_to_height(angle)
                        height_msg = Float32()
                        height_msg.data = height_m
                        self.height_publisher.publish(height_msg)

                        self.get_logger().info(
                            f'Angle: {angle}° -> Box height: {height_m * 1000:.1f} mm ({height_m:.4f} m)')
                    except ValueError:
                        self.get_logger().warn(f'Could not parse angle from: {line}')

                elif line.startswith('Medicine:'):
                    # Parse "Medicine: Protection"
                    medicine = line.split(':', 1)[1].strip()
                    msg = String()
                    msg.data = medicine
                    self.medicine_publisher.publish(msg)
                    self.get_logger().info(f'Medicine: {medicine}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

    def angle_to_height(self, angle_deg):
        """
        Convert rotary sensor angle to box height in meters.

        Quadratic fit from calibration data:
          66° -> 49mm,  92° -> 105mm,  93° -> 111mm,
          98° -> 146mm, 102° -> 190mm

        Returns height in meters, clamped to the calibrated range.
        """
        # Clamp angle to calibrated range
        angle_deg = max(66.0, min(102.0, angle_deg))

        # Quadratic: height_mm = a * angle^2 + b * angle + c
        a = 0.176090
        b = -25.7267
        c = 980.0084
        height_mm = a * angle_deg ** 2 + b * angle_deg + c

        return height_mm / 1000.0  # convert to meters

    def destroy_node(self):
        """Clean up serial connection when node is destroyed."""
        if hasattr(self, 'serial_connection') and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArduinoReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
