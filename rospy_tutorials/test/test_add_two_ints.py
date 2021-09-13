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

## Integration test for add_two_ints

PKG = 'rospy_tutorials'
NAME = 'add_two_ints_test'

import sys

import rospy
import pytest
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, BadTwoInts, BadTwoIntsRequest

class TestAddTwoInts():

    @pytest.fixture(scope="class", autouse=True)
    def start_node(self):
        rospy.init_node(NAME, anonymous=True)

    @pytest.fixture(scope="class")
    def s_add_two_ints(self):
        rospy.wait_for_service('add_two_ints')
        yield rospy.ServiceProxy('add_two_ints', AddTwoInts)

    @pytest.fixture(scope="class")
    def s_bad_two_ints(self):
        rospy.wait_for_service('add_two_ints')
        yield rospy.ServiceProxy('add_two_ints', BadTwoInts)

    @pytest.fixture(params=[(1, 2), (0, 0), (-1, -2), (12312, 980123), (sys.maxsize, -sys.maxsize), (sys.maxsize, -1), (sys.maxsize, 0)])
    def vals(self, request):
        yield request.param

    def test_add_two_ints(self, s_add_two_ints, vals):
        x, y = vals
        rospy.loginfo("Requesting {0}+{1}".format(x, y))
        # test both simple and formal call syntax
        resp = s_add_two_ints(x, y)
        resp2 = s_add_two_ints.call(AddTwoIntsRequest(x, y))
        assert resp.sum == resp2.sum
        rospy.loginfo("{0}+{1} = {2}".format(x, y, resp.sum))
        total = x + y
        assert resp.sum == total, "integration failure, returned sum was {0} vs. {1}".format(resp.sum, total)

    def test_add_two_ints_bad_then_good(self, s_bad_two_ints, s_add_two_ints):
        try:
            resp = s_bad_two_ints(1, 2)
            assert False, "service call should have failed with exception but instead returned 1+2={0}".format(resp.sum)
        except rospy.ServiceException as e:
            rospy.loginfo("success -- ros exception was thrown: {0}".format(e))
        resp = s_add_two_ints.call(AddTwoIntsRequest(1, 2))
        assert 3 == resp.sum

    def test_add_two_ints_bad_type(self, s_bad_two_ints, vals):
        x, y = vals
        rospy.loginfo("Requesting {0}+{1}".format(x, y))
        # test both simple and formal call syntax
        try:
            resp = s_bad_two_ints(x, y)
            if resp.sum == x+y:
                assert False, "call 1 with bad type failed: the server appears to be incorrectly deserialing the packet as it returned: {0}".format(resp.sum)
            else:
                assert False, "call 1 with bad type failed to throw exception: {0}".format(resp.sum)
        except rospy.ServiceException as e:
            rospy.loginfo("success -- ros exception was thrown: {0}".format(e))
        try:
            resp = s_bad_two_ints.call(BadTwoIntsRequest(x, y))
            if resp.sum == x+y:
                assert False, "call 2 with bad type failed: the server appears to be incorrectly deserialing the packet as it returned: {0}".format(resp.sum)
            else:
                assert False, "call 2 with bad type failed to throw exception: {0}".format(resp.sum)
        except rospy.ServiceException as e:
            rospy.loginfo("success -- ros exception was thrown: {0}".format(e))
