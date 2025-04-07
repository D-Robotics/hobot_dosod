import numpy as np
import rclpy
import time
from collections import Counter

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ai_msgs.msg import PerceptionTargets, Target, Roi
from std_msgs.msg import String

class Object2TextNode(Node):
    def __init__(self):
        super().__init__('object_to_text_node')
        self.get_logger().warn('Object To Text Node has been started.')

        self.sub_topic_name = '/hobot_dosod'
        self.pub_topic_name = '/result'

         # 创建订阅器
        self.subscription = self.create_subscription(
            PerceptionTargets,
            self.sub_topic_name,
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(String, self.pub_topic_name, 10)

    def number_to_words(self, num):
        if num < 0 or num > 9999:
            return "Number out of range"

        ones = ["zero", "one", "two", "three", "four", "five", "six",
                "seven", "eight", "nine"]
        teens = ["ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
                 "sixteen", "seventeen", "eighteen", "nineteen"]
        tens = ["", "", "twenty", "thirty", "forty", "fifty",
                "sixty", "seventy", "eighty", "ninety"]

        def two_digits(n):
            if n < 10:
                return ones[n]
            elif 10 <= n < 20:
                return teens[n - 10]
            else:
                return tens[n // 10] + ('' if n % 10 == 0 else '-' + ones[n % 10])

        def three_digits(n):
            if n < 100:
                return two_digits(n)
            else:
                return ones[n // 100] + ' hundred' + ('' if n % 100 == 0 else ' and ' + two_digits(n % 100))

        if num < 1000:
            return three_digits(num)
        else:
            return ones[num // 1000] + ' thousand' + ('' if num % 1000 == 0 else ' ' + three_digits(num % 1000))


    def listener_callback(self, msg):
        class_list = [target.type for target in msg.targets]
        class_count = Counter(class_list)

        output = String()
        output.data = "There is "
        i = 0
        for object_type, count in class_count.items():
            if i != 0:
              output.data += ", "
            # output.data += str(count) + " " + object_type
            output.data += self.number_to_words(count) + " " + object_type
            i += 1

        output.data += "."
        self.publisher.publish(output)
        self.get_logger().info('Published string:\n' + output.data)

def main(args=None):
    rclpy.init(args=args)
    node = Object2TextNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()