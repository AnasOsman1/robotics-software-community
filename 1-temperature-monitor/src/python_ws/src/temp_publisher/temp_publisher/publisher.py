#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_interface.msg import TempData
import argparse
import random
import math
from builtin_interfaces.msg import Time
from rclpy.time import Time as rclpyTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

topicName = 'temp_publisher'
# This is a ROS2 node that publishes temperature data with a sinusoidal pattern and noise.
class PublisherTemprature(Node): 

    def __init__(self, sensor_id="default_sensor", temp_min=20.0, temp_max=30.0, frq_hz=1.0,) :
        super().__init__("publisher_node") 
        # Initialize the publisher with a QoS profile
        # The QoS profile ensures reliable communication with a history depth of 10 messages.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisherCreated= self.create_publisher(TempData, topicName, qos_profile)
        timer_period = 1.0 / frq_hz  # Use frequency parameter properly
        self.timer = self.create_timer(timer_period, self.callBackFunctionPublisher)
        self.temp_max = temp_max
        self.temp_min = temp_min
        self.frq_hz = frq_hz
        self.sensor_id = sensor_id
        self.get_logger().info(f"Publisher Node Initialized with Sensor ID: {self.sensor_id}, Temp Range: {self.temp_min}°C to {self.temp_max}°C, Frequency: {self.frq_hz}Hz")
    
    def temp_generator(self):
        base = (self.temp_max + self.temp_min) / 2
        amplitude = (self.temp_max - self.temp_min) / 2
        noise = random.gauss(0, 0.5) # gaussian noise
        t = self.get_clock().now().nanoseconds / 1e9

        return base + amplitude * math.sin(t / 10) + noise
    
    def callBackFunctionPublisher(self):
        msg= TempData()
        msg.temperature = self.temp_generator()
        msg.unit = 'Celsius'  
        msg.sensor_id = self.sensor_id  
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        self.publisherCreated.publish(msg)
        # Format timestamp nicely
        stamp = msg.header.stamp
        stamp_str = f"{stamp.sec}.{str(stamp.nanosec).zfill(9)}"

        self.get_logger().info(
            f" \n [TemperatureData] \n Time: {stamp_str} \n Temperature: {msg.temperature:.2f}°{msg.unit} \n temp_min: {self.temp_min:.2f}°{msg.unit}  \n  temp_max: {self.temp_max:.2f}°{msg.unit}  \n "
        )
        
        return
    
def main(args=None):
    parser = argparse.ArgumentParser(description="Temperature Publisher Node")
    parser.add_argument('--sensor-id', type=str, default='sensor_001', help='Sensor ID')
    parser.add_argument('--temp-min', type=float, default=20.0, help='Minimum temperature')
    parser.add_argument('--temp-max', type=float, default=30.0, help='Maximum temperature')
    parser.add_argument('--freq', type=float, default=1.0, help='Publishing frequency (Hz)')
    args = parser.parse_args()

    rclpy.init()
    node = PublisherTemprature(
        sensor_id=args.sensor_id,
        temp_min=args.temp_min,
        temp_max=args.temp_max,
        frq_hz=args.freq
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Shutting down").info('completed')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()