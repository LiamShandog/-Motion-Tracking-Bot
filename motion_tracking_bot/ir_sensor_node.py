
# Python API for ROS 2
import rclpy
from rclpy.node import Node

# Import the Bool message type from std_msgs
from std_msgs.msg import Bool

from gpiozero import MotionSensor

# Declare IRSensorNode class, that inherits from Node
class IRSensorNode(Node):
    def __init__(self):
        # Inherit from Node class
        super().__init__('ir_sensor_node')

        self.declare_parameter('ir_sensor_pin', 17)  # GPIO pin for IR sensor
        self.ir_sensor_pin = self.get_parameter('ir_sensor_pin').get_parameter_value().integer_value

        # Set up gpiozero MotionSensor
        self.pir = MotionSensor(self.ir_sensor_pin)

        self.publisher_ = self.create_publisher(Bool, 'ir_sensor', 10)  # Publisher for IR sensor data
        self.timer = self.create_timer(0.1, self.read_sensor)  # Timer to periodically check sensor

    def read_sensor(self):
        detected = self.pir.motion_detected # Read the sensor value (0 or 1)
        msg = Bool()
        msg.data = bool(detected)  # Convert to boolean
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Motion: {msg.data}')  # Log the sensor state


def main(args=None):
    rclpy.init(args=args) # Initialize the rclpy library
    node = IRSensorNode() # Create an instance of the IRSensorNode
    rclpy.spin(node)      # Keep the node running
    node.destroy_node()   # Destroy the node explicitly
    rclpy.shutdown()     # Shutdown the rclpy library

if __name__ == '__main__': # Goes true if this script is run directly and not imported
    main()  # Call the main function