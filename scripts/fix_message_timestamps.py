#!/usr/bin/env python
import rospy
import rosbag
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("input_bag", help="The bag to fix the timestamps of")
parser.add_argument("output_bag", help="Where to write the new bag")
args = parser.parse_args()

with rosbag.Bag(args.output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(args.input_bag).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
