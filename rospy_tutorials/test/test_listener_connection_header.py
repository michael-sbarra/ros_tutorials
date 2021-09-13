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
# Revision $Id: test_peer_subscribe_notify.py 3803 2009-02-11 02:04:39Z rob_wheeler $

## Integration test for peer_subscribe_notify

PKG = 'rospy_tutorials'
NAME = 'peer_subscribe_notify_test'

import sys
import time
import unittest

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String


class TestListenerConnectionHeader():

    success = False

    def callback(self, data):
        chatter = data.data
        if 'callerid' in data._connection_header:
            who = data._connection_header['callerid']
            self.success = True
        else:
            who = 'unknown'
        rospy.loginfo("I just heard {0} from {1}".format(chatter, who))

    def test_notify(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber("/chatter", String, self.callback)
        timeout_t = rospy.get_time() + 10.0  # 10 seconds
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown() and not self.success and rospy.get_time() < timeout_t:
            rate.sleep()
        assert (self.success)
