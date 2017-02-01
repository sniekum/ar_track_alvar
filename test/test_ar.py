#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TORK (Tokyo Opensource Robotics Kyokai Association) 
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
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association
#    nor the names of its contributors may be used to endorse or promote 
#    products derived from this software without specific prior written 
#    permission.
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

import math
import unittest

from geometry_msgs.msg import Pose
import rospy
import tf
from tf.transformations import quaternion_from_euler

class TestArAlvarRos(unittest.TestCase):
    '''
    Tests are based on http://download.ros.org/data/ar_track_alvar/ar_track_alvar_4markers_tork_2017-01-30-15-03-19.bag
    '''

    def setUp(self):
        rospy.init_node('test_armarker_ros_detect')
        self.tflistener = tf.TransformListener()
        rate = rospy.Rate(10.0)

    def tearDown(self):
        True  # TODO impl something meaningful

    def _lookup_tf(self, origin, target):
        '''
        @param origin: lookupTransform's 1st argument.
        @param target: lookupTransform's 2nd argument.
        @rtype: ([str], [str])
        @return: Translation and rotation.
        '''
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform(origin, target, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return (trans, rot)
        
    def test_marker1(self):
        '''
        Aiming the real output taken from a bag file
        tflistener.lookupTransform('camera', 'ar_marker_1', rospy.Time(0))
        Out[4]:
        ((-0.04805772245624329, 0.039528315926071665, 0.26775882622136327),
         (0.48207151562664247, 0.8758763282975102, o-0.016363763970395625, -0.013414118615296202))
        '''
        trans_expected = [-0.04805772245624329, 0.039528315926071665, 0.26775882622136327]
        rot_expected = [0.48207151562664247, 0.8758763282975102, -0.016363763970395625, -0.013414118615296202]
        (trans, rot) = self._lookup_tf('camera', 'ar_marker_1')
        for v_ret, v_expected in zip(trans, trans_expected):
            self.assertAlmostEqual(v_ret, v_expected, 3)
        for v_ret, v_expected in zip(rot, rot_expected):
            self.assertAlmostEqual(v_ret, v_expected, 3)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('ar_track_alvar', 'test_ar_alvar_ros', TestArAlvarRos)
