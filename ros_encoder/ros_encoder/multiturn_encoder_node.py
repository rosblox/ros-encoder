#!/bin/python3

import pigpio
import yaml

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from std_msgs.msg import Float32

from ros_encoder.encoder_interface import Encoder


class MultiturnEncoder(Node):

    def __init__(self):
        super().__init__('multiturn_encoder_node')

        self.pi = pigpio.pi()  # Initialize pigpio
        self.encoder = Encoder(self.pi, pin_a=17, pin_b=18, scale=1)  # Adjust pins as needed

        self.reset_srv = self.create_service(Trigger, '~/reset', self.reset_callback)
        self.publisher = self.create_publisher(Float32, '~/turns', 5)

        self.file_path = '/tmp/turns.yaml'
        self.turns = self.load_turns_from_file()
        self.publish_turns()        

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_counter = 0


    def timer_callback(self):
        current_position = self.encoder.get_position()  # Get current position from encoder
        self.turns = current_position / 360  # Convert to turns, adjust the divisor based on your encoder
        self.publish_turns()
        
        self.timer_counter += 1
        if self.timer_counter == 10:  # Adjust frequency of writing to file as needed
            self.write_turns_to_file()
            self.timer_counter = 0


    def reset_callback(self, request, response):
        self.get_logger().info(f"Reset turns from {self.turns}")
        self.turns = 0.0
        response.success=True
        return response
    

    def publish_turns(self):
        msg_out = Float32()
        msg_out.data = self.turns
        self.publisher.publish(msg_out)


    def load_turns_from_file(self):
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
                if 'turns' in data:
                    return data['turns']
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            pass
        return 0.0  # Default value if file doesn't exist or is invalid


    def write_turns_to_file(self):
        data = {'turns': self.turns}
        with open(self.file_path, 'w') as file:
            yaml.dump(data, file)


def main(args=None):
    rclpy.init(args=args)

    multiturn_encoder = MultiturnEncoder()

    rclpy.spin(multiturn_encoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multiturn_encoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()