#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from std_srvs.srv import Trigger

def test_resest_client():
    print("waiting for /reset_pose_estimation")
    rospy.wait_for_service('/reset_pose_estimation')
    print("/reset_pose_estimation exists ! ")
    while not rospy.is_shutdown():
        print("press enter to reset pose")
        input()
        reset_srv = rospy.ServiceProxy('/reset_pose_estimation', Trigger)
        resp = reset_srv()
        print(resp)

if __name__ == "__main__":
    test_resest_client()
