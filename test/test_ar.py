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
                (trans,rot) = self.tflistener.lookupTransform(origin, target, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(str(e) + ' target={}'.format(target))
                continue
        return (trans, rot)
        
    def test_markers(self):
        '''
        Aiming the real output taken from a bag file.
        '''
        # The values in this list are ordered in the marker's number. 
        tf_expected = [[[-0.04805772245624329, 0.039528315926071665, 0.26775882622136327], [0.48207151562664247, 0.8758763282975102, -0.016363763970395625, -0.013414118615296202]],
                       [[0.00724143569105839, 0.015617918591239179, 0.26620870021332743], [0.08565822924695, 0.996063527939151, 0.005328638541876154, -0.0221747983751627]],
                       [[0.0622400226840759, 0.014597488632830266, 0.2622754265075508], [-0.463809420375053, 0.8850930963424348, 0.03341295306922885, -0.019354765447100585]],
                       [[0.04461061126854959, 0.05335369954390891, 0.2677638768342693], [0.6728107301594857, 0.7391760285155204, -0.013913131296136081, -0.027403376212041232]]]
        for i in range (0, len(tf_expected)):
            (trans, rot) = self._lookup_tf('camera', 'ar_marker_{}'.format(i))
            # Compare each translation element (x, y, z) 
            for v_ret, v_expected in zip(trans, tf_expected[i][0]):
                self.assertAlmostEqual(v_ret, v_expected, 3)
            # Compare each orientation element (x, y, z, w) 
            for v_ret, v_expected in zip(rot, tf_expected[i][1]):
                self.assertAlmostEqual(v_ret, v_expected, 3)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('ar_track_alvar', 'test_ar_alvar_ros', TestArAlvarRos)
