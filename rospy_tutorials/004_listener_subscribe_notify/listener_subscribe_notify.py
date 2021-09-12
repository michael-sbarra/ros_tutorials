#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## talker that receives notification of new subscriptions

NAME = 'talker_callback'

import sys

import rospy
from std_msgs.msg import String

class ChatterListener(rospy.SubscribeListener):
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.loginfo("a peer subscribed to topic [{0}]".format(topic_name))
        
        subscriber_str = "Hey everyone, we have a new friend!"
        rospy.loginfo(subscriber_str)
        topic_publish(String(subscriber_str))
        greeting_str = "greetings. welcome to topic {0}".format(topic_name)
        rospy.loginfo(greeting_str)
        peer_publish(String(greeting_str))
        
    def peer_unsubscribe(self, topic_name, numPeers):
        rospy.loginfo("a peer unsubscribed from topic [{0}]".format(topic_name))
        if numPeers == 0:
            rospy.loginfo("I have no friends")
    
def talker_callback():
    rospy.init_node(NAME, anonymous=True)
    pub = rospy.Publisher("chatter", String, subscriber_listener=ChatterListener(), queue_size=10)
    count = 0
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world {0}".format(count)
        rospy.loginfo(hello_str)
        pub.publish(String(hello_str))
        count += 1
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker_callback()
    except rospy.ROSInterruptException:
        pass
