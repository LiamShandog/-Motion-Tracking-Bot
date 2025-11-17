
# Python API for ROS 2
import rclpy
from rclpy.node import Node

# Import the Bool message type from std_msgs
from std_msgs.msg import Bool

# Import pigpio library for GPIO control (works on Pi and non-Pi with pigpiod daemon)
try:
    import pigpio
except ImportError:
    raise ImportError(
        "pigpio is not installed. Install it with: pip install pigpio\n"
        "On Raspberry Pi: sudo apt install pigpio python3-pigpio\n"
        "To run pigpiod daemon on Pi: sudo pigpiod"
    )

# Declare IRSensorNode class, that inherits from Node
class IRSensorNode(Node):
    def __init__(self):
        # Inherit from Node class
        super().__init__('ir_sensor_node')

        self.declare_parameter('ir_sensor_pin', 17)  # GPIO pin for IR sensor
        self.ir_sensor_pin = self.get_parameter('ir_sensor_pin').get_parameter_value().integer_value

        # Connect to pigpiod daemon (use localhost:8888 for local connection)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "Failed to connect to pigpiod daemon. "
                "On Raspberry Pi, start it with: sudo pigpiod"
            )

        # Set pin as input with pull-up resistor to avoid floating state
        self.pi.set_mode(self.ir_sensor_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.ir_sensor_pin, pigpio.PUD_UP)

        self.publisher_ = self.create_publisher(Bool, 'ir_sensor', 10)  # Publisher for IR sensor data
        self.timer = self.create_timer(0.1, self.read_sensor)  # Timer to periodically check sensor

    def read_sensor(self):
        detected = self.pi.read(self.ir_sensor_pin)  # Read the sensor value (0 or 1)
        msg = Bool()
        msg.data = bool(detected)  # Convert to boolean
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Motion: {msg.data}')  # Log the sensor state


def main(args=None):
    rclpy.init(args=args) # Initialize the rclpy library
    node = IRSensorNode() # Create an instance of the IRSensorNode
    try:
        rclpy.spin(node)      # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pi.stop()  # Disconnect from pigpiod daemon
        except Exception as e:
            node.get_logger().error(f"Error stopping pigpio: {e}")
        try:
            node.destroy_node()  # Destroy the ROS node
        except Exception as e:
            print(f"Error destroying node: {e}")
        rclpy.shutdown()     # Shutdown the rclpy library

if __name__ == '__main__': # Goes true if this script is run directly and not imported
    main()  # Call the main function