# # Wheel diameter 65mm

# import time
# import math
# from typing import List
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist  # Standard message for robot movement
# from std_msgs.msg import Float32, Int32, String
# from std_msgs.msg import Int32MultiArray


# class ODOMeter(Node):
#     def __init__(self):

#         self.counts_per_revolution = 1320
#         self.wheel_diameter = 0.065
#         self.previous_counts: list[float] = [0.0] * 4

#         #self.encoder_sub = self.create_subscriber(Int32MultiArray, '/wheel_encoders',self.process_encoders, 10)


#     def count_distance(self, counts):
#         distance = counts * math.pi * self.wheel_diameter / self.counts_per_revolution
#         return distance

#     def calculate_wheel_speeds(self, delta_encoders, delta_time):
#         velocity = delta_encoders * math.pi * self.wheel_diameter / (self.counts_per_revolution * delta_time)
#         return velocity

#     def process_encoders(self, encoders):
#         pass