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
# Revision $Id: add_two_ints_client 3804 2009-02-11 02:16:00Z rob_wheeler $

## Extended version of add_two_int_client that shows how to use
## connection header to pass in metadata to service.


PKG = 'rospy_tutorials'
NAME = 'test_client_connection_header'

import sys

from rospy_tutorials.srv import AddTwoInts
import rospy
import pytest


class TestClientConnectionHeader():

    success = False

    def test_client_connection_header(self):
        rospy.init_node(NAME, anonymous=True)
        import random
        x = random.randint(0, 1000)
        y = random.randint(0, 1000)
        service_name = '/add_two_ints'
        rospy.wait_for_service(service_name)

        try:
            # initialize ServiceProxy with extra header information.
            # header is only exchanged on initial connection
            metadata = { 'cookies' : 'peanut butter' }
            add_two_ints = rospy.ServiceProxy(service_name, AddTwoInts, headers=metadata)

            rospy.loginfo("Requesting {0}+{1} with cookies={2}".format(x, y, metadata['cookies']))

            # simplified style
            resp = add_two_ints(x, y)
            rospy.loginfo("Server's connection headers were {0}".format(resp._connection_header))
            assert 'callerid' in resp._connection_header

            return resp.sum
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
            assert False

if __name__ == '__main__':
    if '--node' in sys.argv:
        # run as a normal node instead (test of server_connection_header)
        # the test here is to send the cookies header, which the server test node looks for
        rospy.init_node('sch_test_client', anonymous=True)
        # NOTE different service name
        service_name = 'add_two_ints_header_test'
        try:
            rospy.wait_for_service(service_name)
            metadata = { 'cookies' : 'peanut butter' }
            add_two_ints = rospy.ServiceProxy(service_name, AddTwoInts, headers=metadata)
            rate = rospy.Rate(10)  # 10hz
            while not rospy.is_shutdown():
                resp = add_two_ints(1, 2)
                # call at 10hz until test is over
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
