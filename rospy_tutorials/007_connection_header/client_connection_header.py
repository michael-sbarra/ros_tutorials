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

NAME = 'client_connection_header'

import rospy

from rospy_tutorials.srv import AddTwoInts

## add two numbers using the add_two_ints service
## @param x int: first number to add
## @param y int: second number to add
def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    
    try:
        # initialize ServiceProxy with extra header information.
        # header is only exchanged on initial connection
        metadata = { 'cookies' : 'peanut butter' } 
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts, headers=metadata)
        
        rospy.loginfo("Requesting {0}+{1} with cookies={2}".format(x, y, metadata['cookies']))
        
        # simplified style
        resp = add_two_ints(x, y)
        rospy.loginfo("Server's connection headers were {0}", resp._connection_header)

        return resp.sum
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {0}".format(e))

if __name__ == "__main__":
    
    rospy.init_node(NAME)
    args = rospy.get_param('~')
    if 'x' not in args:
        args['x'] = 1
    if 'y' not in args:
        args['y'] = 2
    if args.pop('random'):
        import random
        args['x'] = random.randint(-50000,50000)
        args['y'] = random.randint(-50000,50000)
    rospy.loginfo("{0} + {1} = {2}".format(args['x'], args['y'], add_two_ints_client(**args)))
