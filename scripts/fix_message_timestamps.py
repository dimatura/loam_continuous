#!/usr/bin/env python
import rospy
import rosbag

input_bag = '/home/edbot/logs/husky/2016-07-13-outdoor-george-square-west/run1/filtered/filtered_run1.bag'
output_bag = '/home/edbot/logs/husky/2016-07-13-outdoor-george-square-west/run1/filtered/filtered_run1_fixed.bag'
#input_bag = '/home/edbot/logs/husky/2016-07-13-outdoor-george-square-west/run1/2016-07-13-15-09-32.bag'
#output_bag = '/home/edbot/logs/husky/2016-07-13-outdoor-george-square-west/run1/fixed_timestamps/2016-08-02-17-03-11.bag'
with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
