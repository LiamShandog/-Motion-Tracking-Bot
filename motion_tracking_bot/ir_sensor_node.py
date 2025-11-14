
# Python API for ROS 2
from platform import node
import rclpy
from rclpy.node import Node

# Import the Bool message type from std_msgs
from std_msgs.msg import Bool

# Import the RPi.GPIO library to interact with Raspberry Pi GPIO pins
import RPi.GPIO as GPIO

# Import time module for delays
import time

# Declare IRSensorNode class, that inherits from Node
class IRSensorNode(Node):
    def __init__(self):
        # Inherit from Node class
        super().__init__('ir_sensor_node')

        self.ir_sensor_pin = 17  # GPIO pin number where the IR sensor is connected

        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(self.ir_sensor_pin, GPIO.IN)  # Set pin as input

        self.publisher_ = self.create_publisher(Bool, 'ir_sensor', 10)  # Publisher for IR sensor data
        self.timer = self.create_timer(0.1, self.read_sensor)  # Timer to periodically check sensor

    def read_sensor(self):
        detected = GPIO.input(self.ir_sensor_pin)  # Read the sensor value
        msg = Bool()
        msg.data = bool(detected)  # Convert to boolean
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Motion: {msg.data}')  # Log the sensor state


def main(args=None):
    rclpy.init(args=args) # Initialize the rclpy library
    node = IRSensorNode() # Create an instance of the IRSensorNode
    rclpy.spin(node)      # Keep the node running
    GPIO.cleanup()      # Clean up GPIO on shutdown
    rclpy.shutdown()     # Shutdown the rclpy library

if __name__ == '__main__': # Goes true if this script is run directly and not imported
    main()  # Call the main function