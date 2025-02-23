#!/usr/bin/env python
"""
A rostopic fuzzer node for ROS.

This script:
  1. Initializes a ROS node.
  2. Retrieves a list of published topics (with their message types).
  3. Uses the ROS Master API to check which topics have subscribers.
  4. For each topic that has subscribers, creates a publisher.
  5. In a loop, creates a “fuzzed” version of the topic’s message and publishes it.

Usage:
  1. Make sure roscore is running.
  2. Have other nodes up (so that some topics have subscribers).
  3. Run this script (e.g., using `rosrun <your_package> rostopic_fuzzer.py` or directly with Python).
"""

import rospy
import roslib.message
import random
import string

def fuzz_value(field_type):
    """
    Generate a random value for a given primitive ROS field type.
    """
    if field_type in ["int8", "int16", "int32", "int64"]:
        return random.randint(-100, 100)
    elif field_type in ["uint8", "uint16", "uint32", "uint64"]:
        return random.randint(0, 200)
    elif field_type in ["float32", "float64"]:
        return random.uniform(-100.0, 100.0)
    elif field_type == "string":
        return ''.join(random.choices(string.ascii_letters + string.digits, k=10))
    elif field_type == "bool":
        return random.choice([True, False])
    # For nested message types, try to fuzz recursively.
    elif "/" in field_type:
        nested_msg_class = roslib.message.get_message_class(field_type)
        if nested_msg_class is not None:
            nested_msg = nested_msg_class()
            return fuzz_message(nested_msg)
        else:
            return None
    else:
        return None

def fuzz_message(msg):
    """
    Fuzz all the fields of a ROS message by assigning random values.
    This function inspects the _slot_names and _slot_types of the message.
    """
    for field_name, field_type in zip(msg._slot_names, msg._slot_types):
        # Handle array types (e.g., "int32[]")
        if field_type.endswith("[]"):
            base_type = field_type[:-2]
            # Create a list of random length (0 to 5 elements)
            length = random.randint(0, 5)
            setattr(msg, field_name, [fuzz_value(base_type) for _ in range(length)])
        # For standard primitive types, use fuzz_value.
        elif field_type in ["int8", "int16", "int32", "int64", 
                            "uint8", "uint16", "uint32", "uint64", 
                            "float32", "float64", "string", "bool"]:
            setattr(msg, field_name, fuzz_value(field_type))
        # For nested message types, instantiate and fuzz recursively.
        elif "/" in field_type:
            nested_msg_class = roslib.message.get_message_class(field_type)
            if nested_msg_class is not None:
                nested_msg = nested_msg_class()
                fuzz_message(nested_msg)
                setattr(msg, field_name, nested_msg)
            else:
                setattr(msg, field_name, None)
        else:
            setattr(msg, field_name, None)
    return msg

def get_system_subscribers():
    """
    Query the ROS master for the current system state and build a dictionary
    mapping each topic to its list of subscribers.
    """
    master = rospy.get_master()
    try:
        code, status_msg, state = master.getSystemState()
        if code != 1:
            rospy.logerr("Error getting system state: %s", status_msg)
            return {}
        # state is a triple: [publishers, subscribers, services]
        _, subscribers_state, _ = state
        subs_dict = {}
        for topic, nodes in subscribers_state:
            subs_dict[topic] = nodes
        return subs_dict
    except Exception as e:
        rospy.logerr("Exception in getting system state: %s", e)
        return {}

def main():
    rospy.init_node('rostopic_fuzzer', anonymous=True)
    rospy.loginfo("RoStopic Fuzzer node started.")

    # Retrieve the list of published topics and their message types.
    topics = rospy.get_published_topics()
    rospy.loginfo("Discovered %d published topics.", len(topics))

    # Retrieve system state to know which topics have subscribers.
    subs_dict = get_system_subscribers()

    # Build a list of targets: topics that have at least one subscriber and whose message class we can load.
    fuzzer_targets = []
    for topic, msg_type in topics:
        if topic in subs_dict and len(subs_dict[topic]) > 0:
            message_class = roslib.message.get_message_class(msg_type)
            if message_class is None:
                rospy.logwarn("Could not load message class for topic '%s' with type '%s'.", topic, msg_type)
                continue
            # Create a publisher for the topic.
            pub = rospy.Publisher(topic, message_class, queue_size=10)
            fuzzer_targets.append((topic, pub, message_class))
            rospy.loginfo("Targeting topic '%s' with type '%s' (subscribers: %s)", topic, msg_type, subs_dict[topic])

    if not fuzzer_targets:
        rospy.logwarn("No topics with subscribers found to fuzz. Exiting.")
        return

    # Set the publishing rate (e.g., 1 Hz).
    rate = rospy.Rate(1)
    rospy.loginfo("Starting fuzzing on %d topics.", len(fuzzer_targets))

    # Main fuzzing loop.
    while not rospy.is_shutdown():
        for topic, pub, message_class in fuzzer_targets:
            # Create a new instance of the message and fuzz its fields.
            msg_instance = message_class()
            fuzz_message(msg_instance)
            # Publish the fuzzed message.
            pub.publish(msg_instance)
            rospy.loginfo("Published fuzzed message to topic '%s'.", topic)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
