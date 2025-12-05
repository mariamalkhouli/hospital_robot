#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class EncoderDriver(Node):
    def __init__(self):
        super().__init__('encoder_driver')
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.LEFT_PIN = 24
        self.RIGHT_PIN = 25
        
        GPIO.setup(self.LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        self.WHEEL_DIAMETER = 0.065
        self.WHEEL_CIRCUMFERENCE = 3.14159 * self.WHEEL_DIAMETER
        self.PULSES_PER_REV = 1
        
        self.left_count = 0
        self.right_count = 0
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()
        
        self.left_speed_pub = self.create_publisher(Float32, 'wheel/left/speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, 'wheel/right/speed', 10)
        
        GPIO.add_event_detect(self.LEFT_PIN, GPIO.FALLING, callback=self.left_callback, bouncetime=10)
        GPIO.add_event_detect(self.RIGHT_PIN, GPIO.FALLING, callback=self.right_callback, bouncetime=10)
        
        self.timer = self.create_timer(0.1, self.publish_speeds)
        self.get_logger().info('Encoders Ready')
    
    def left_callback(self, channel):
        self.left_count += 1
    
    def right_callback(self, channel):
        self.right_count += 1
    
    def publish_speeds(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:
            left_pulses = self.left_count - self.last_left
            right_pulses = self.right_count - self.last_right
            
            left_speed = (left_pulses / self.PULSES_PER_REV) * self.WHEEL_CIRCUMFERENCE / dt
            right_speed = (right_pulses / self.PULSES_PER_REV) * self.WHEEL_CIRCUMFERENCE / dt
            
            self.left_speed_pub.publish(Float32(data=left_speed))
            self.right_speed_pub.publish(Float32(data=right_speed))
            
            self.last_left = self.left_count
            self.last_right = self.right_count
            self.last_time = current_time
    
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
