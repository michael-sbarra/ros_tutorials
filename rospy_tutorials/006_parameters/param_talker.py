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
# Revision $Id: listener.py 5263 2009-07-17 23:30:38Z sfkwc $

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

NAME = 'param_talker'

import rospy
from std_msgs.msg import String

def param_talker():
    rospy.init_node(NAME)

    # Fetch values from the Parameter Server. In this example, we fetch
    # parameters from three different namespaces:
    #
    # 1) global (/global_example)
    # 2) parent (/foo/utterance)
    # 3) private (/foo/param_talker/topic_name)

    # fetch a /global parameter
    global_example = rospy.get_param('/global_example')
    rospy.loginfo("{0} is {1}".format(rospy.resolve_name('/global_example'), global_example))

    # fetch the utterance parameter from our parent namespace
    utterance = rospy.get_param('utterance')
    rospy.loginfo("{0} is {1}".format(rospy.resolve_name('utterance'), utterance))

    # fetch topic_name from the ~private namespace
    topic_name = rospy.get_param('~topic_name')
    rospy.loginfo("{0} is {1}".format(rospy.resolve_name('~topic_name'), topic_name))

    # fetch a parameter, using 'default_value' if it doesn't exist
    default_param = rospy.get_param('default_param', 'default_value')
    rospy.loginfo("{0} is {1}".format(rospy.resolve_name('default_param'), default_param))

    # fetch a group (dictionary) of parameters
    gains = rospy.get_param('gains')
    p, i, d = gains['P'], gains['I'], gains['D']
    rospy.loginfo("gains are {0}, {1}, {2}".format(p, i, d))

    # set some parameters
    rospy.loginfo('setting parameters...')
    rospy.set_param('list_of_floats', [1., 2., 3., 4.])
    rospy.set_param('bool_True', True)
    rospy.set_param('~private_bar', 1+2)
    rospy.set_param('to_delete', 'baz')
    rospy.loginfo('...parameters have been set')

    # delete a parameter
    if rospy.has_param('to_delete'):
        rospy.delete_param('to_delete')
        rospy.loginfo("deleted {0} parameter".format(rospy.resolve_name('to_delete')))
    else:
        rospy.loginfo("parameter {0} was already deleted".format(rospy.resolve_name('to_delete')))

    # search for a parameter
    param_name = rospy.search_param('global_example')
    rospy.loginfo("found global_example parameter under key: {0}".format(param_name))

    # publish the value of utterance repeatedly
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(utterance)
        rospy.loginfo(utterance)
        rate.sleep()

if __name__ == '__main__':
    try:
        param_talker()
    except rospy.ROSInterruptException:
        pass
