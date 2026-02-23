#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time

class SerialNavigationNode(Node):

    def __init__(self):
        super().__init__('serial_navigation_node')

        port = '/dev/ttyACM0'
        baudrate = 115200

        try:
            self.ser = serial.Serial(
                port,
                baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            self.get_logger().info(f"Connected to ESP32 on {port}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            raise

        self.thread = threading.Thread(
            target=self.read_serial,
            daemon=True
        )
        self.thread.start()

        # WAIT for CircuitPython reboot
        self.get_logger().info("Waiting 3s for ESP32 boot...")
        time.sleep(3.0)

        self.send_goal(2.0, 90.0)

    def read_serial(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f"ESP32 → {line}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
            time.sleep(0.02)

    def send_goal(self, distance, heading):
        cmd = f"GO {distance} {heading}\n"
        self.ser.reset_input_buffer()
        self.ser.write(cmd.encode())
        self.ser.flush()

        self.get_logger().info(
            f"Sent GO → distance={distance} m, heading={heading} deg"
        )

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

