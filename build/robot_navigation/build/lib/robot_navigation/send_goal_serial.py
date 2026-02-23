import rclpy
from rclpy.node import Node
import serial
import time
import threading

class SendGoalSerial(Node):

    def __init__(self):
        super().__init__('send_goal_serial')

        self.port = '/dev/ttyACM0'
        self.baud = 115200

        self.get_logger().info("Opening serial port...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2)  # ESP32 reset delay

        distance = 1.0
        heading = 0

        cmd = f"GO {distance} {heading}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent command: {cmd.strip()}")

        self.running = True
        self.reader_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.reader_thread.start()

    def read_serial(self):
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f"ESP32: {line}")
            time.sleep(0.05)

    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SendGoalSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

