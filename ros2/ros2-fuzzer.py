#!/usr/bin/env python3
"""
ROS2 Topic Fuzzer Node

This node:
  1. Initializes a ROS2 node.
  2. Retrieves the list of published topics and their types.
  3. For each topic that has active subscribers, loads the message type
     and creates a publisher.
  4. Periodically creates a new instance of the message, fills its fields
     with random ("fuzzed") data, and publishes it.

Usage:
  1. Source your ROS2 workspace.
  2. Start your ROS2 daemon (e.g. using: ros2 run <package> <executable>).
  3. Ensure there are active subscribers on some topics.
  4. Run this node.
"""

import rclpy
from rclpy.node import Node
import random
import string
from rosidl_runtime_py.utilities import get_message

def fuzz_value(field_type):
    """
    Generate a random value for a given primitive field type.
    This function supports int, float, bool, str, and list types.
    """
    if field_type == int:
        return random.randint(-100, 100)
    elif field_type == float:
        return random.uniform(-100.0, 100.0)
    elif field_type == bool:
        return random.choice([True, False])
    elif field_type == str:
        return ''.join(random.choices(string.ascii_letters + string.digits, k=10))
    # Check if the type is a list annotation (requires Python 3.7+ with typing)
    elif hasattr(field_type, '__origin__') and field_type.__origin__ is list:
        # If the inner type is available, use it; otherwise default to str.
        inner_type = field_type.__args__[0] if field_type.__args__ else str
        length = random.randint(0, 5)
        return [fuzz_value(inner_type) for _ in range(length)]
    # For nested message types (assumed to be a class), try instantiating and fuzzing recursively.
    elif isinstance(field_type, type):
        try:
            nested_msg = field_type()
            return fuzz_message(nested_msg)
        except Exception:
            return None
    else:
        return None

def fuzz_message(msg):
    """
    Fuzz each field of a ROS2 message using its __slots__ and __annotations__.
    This function assumes that the message type provides type hints in __annotations__.
    """
    annotations = getattr(msg.__class__, '__annotations__', {})
    for field in msg.__slots__:
        field_type = annotations.get(field, None)
        if field_type is not None:
            try:
                setattr(msg, field, fuzz_value(field_type))
            except Exception:
                # If fuzzing a field fails, skip it.
                pass
    return msg

class Ros2TopicFuzzer(Node):
    def __init__(self):
        super().__init__('ros2_topic_fuzzer')
        self.get_logger().info("ROS2 Topic Fuzzer node started.")
        self.targets = []

        # Retrieve the list of topics along with their types.
        topics_and_types = self.get_topic_names_and_types()
        self.get_logger().info("Discovered {} topics.".format(len(topics_and_types)))

        # For each topic, check if there are active subscriptions.
        for topic, types in topics_and_types:
            subs_info = self.get_subscriptions_info_by_topic(topic)
            if subs_info and len(subs_info) > 0:
                # Use the first type if multiple are available.
                msg_type_str = types[0]
                # Dynamically load the message class.
                try:
                    msg_class = get_message(msg_type_str)
                except Exception as e:
                    self.get_logger().warn(
                        "Could not load message class for topic '{}' of type '{}': {}"
                        .format(topic, msg_type_str, e))
                    continue

                if msg_class is None:
                    self.get_logger().warn(
                        "Message class is None for topic '{}' of type '{}'."
                        .format(topic, msg_type_str))
                    continue

                publisher = self.create_publisher(msg_class, topic, 10)
                self.targets.append((topic, publisher, msg_class))
                self.get_logger().info(
                    "Fuzzing topic '{}' of type '{}' with {} subscriber(s)."
                    .format(topic, msg_type_str, len(subs_info)))
        
        if not self.targets:
            self.get_logger().warn("No topics with subscribers found to fuzz.")

        # Create a timer to publish fuzzed messages every second.
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_fuzzed_messages)

    def publish_fuzzed_messages(self):
        for topic, publisher, msg_class in self.targets:
            # Create a new message instance and fuzz its fields.
            msg_instance = msg_class()
            fuzz_message(msg_instance)
            publisher.publish(msg_instance)
            self.get_logger().info("Published fuzzed message to topic '{}'.".format(topic))

def main(args=None):
    rclpy.init(args=args)
    node = Ros2TopicFuzzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
