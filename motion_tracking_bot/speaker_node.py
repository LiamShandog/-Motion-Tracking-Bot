import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import pigpio 

import time
import subprocess # add code to send data to speaker

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')

        # subscribe to IR sensor topic
        self.subscription = self.create_subscription(
            Bool,
            'ir_sensor',
            self.motion_callback,
            10
        )

        self.declare_parameter('servo_pin', 18) # GPIO pin for servo
        self.declare_parameter('audio_file', 'default_audio.wav' ) # alter on pi
        self.declare_parameter('wave_count', 3) # #  of waves by hand
        self.declare_parameter('wave_delay', 0.3) # delay between waves


        # Servo pulse widths in microseconds (tune for servo positioning)
        self.declare_parameter('servo_pulse_center', 1500)
        self.declare_parameter('servo_pulse_left', 1200)
        self.declare_parameter('servo_pulse_right', 1800)

        self.servo_pin = self.get_parameter('servo_pin').get_parameter_value().integer_value
        self.audio_file = self.get_parameter('audio_file').get_parameter_value().string_value
        self.wave_count = self.get_parameter('wave_count').get_parameter_value().integer_value
        self.wave_delay = self.get_parameter('wave_delay').get_parameter_value().double_value

        self.servo_center = self.get_parameter('servo_pulse_center').get_parameter_value().integer_value
        self.servo_left = self.get_parameter('servo_pulse_left').get_parameter_value().integer_value
        self.servo_right = self.get_parameter('servo_pulse_right').get_parameter_value().integer_value

        # set up pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "Failed to connect to pigpiod daemon")
        else:
            self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_center)
            self.get_logger().info('Servo initialized on GPIO %d' % self.servo_pin)

        # To prevent overlapping triggers
        self.is_handling_motion = False

        self.get_logger().info('Speaker Node has been started.')

    def motion_callback(self, msg):
        if msg.data:
            if not self.is_handling_motion:
                self.is_handling_motion = True
                self.get_logger().info('Motion detected! Activating speaker and servo.')
                try:
                    self.play_sound()
                    self.wave_servo()
                finally:
                    self.is_handling_motion = False

    def play_sound(self):
        if not self.audio_file:
            self.get_logger().warn('No sound file configured, skipping audio.')
            return

        self.get_logger().info(f'Playing sound: {self.audio_file}')
        try:
            subprocess.run(
                ['aplay', self.audio_file],
                check=False
            )
        except Exception as e:
            self.get_logger().error(f'Error while playing sound: {e}')

    # Move servo to wave hand
    def wave_servo(self):
        if not self.pi.connected:
            self.get_logger().warn('pigpio not connected; cannot move servo.')
            return

        self.get_logger().info('Waving servo hand.')

        for i in range(self.wave_count):
            # Move left
            self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_left)
            time.sleep(self.wave_delay)

            # Move right
            self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_right)
            time.sleep(self.wave_delay)

        # Return to center
        self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_center)
        self.get_logger().info('Servo wave complete.')

    def destroy_node(self):
        # Clean up servo when node is shutting down
        if self.pi is not None and self.pi.connected:
            self.get_logger().info('Stopping servo and releasing pigpio.')
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)
            self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()