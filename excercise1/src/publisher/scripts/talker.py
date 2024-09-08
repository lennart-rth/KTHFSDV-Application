#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16

def publisher(n, k):
    pub = rospy.Publisher('roth', UInt16, queue_size = 10)       # Unit16 range is 0...65535. With 20hz we have 819sec until overflow.
    rospy.init_node('publisher', anonymous = True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.loginfo(k)
        pub.publish(k)
        k += n
        if k >= 2**16 - 1:  # avoid overflow of UInt16
            rospy.loginfo("Overflow detected. Setting n back to 0!")
            k = 0
        rate.sleep()


if __name__ == '__main__':
    n = 4
    k = 0   # k is integer and k > 0
    try:
        publisher(n, k)
    except rospy.ROSInterruptException:
        pass
