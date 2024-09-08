#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

def callback(data, pub, q):
    rospy.loginfo(rospy.get_caller_id() + ': %s', data.data)
    msg = data.data / q     # float now
    pub.publish(msg)
    
def listener(q):
    rospy.init_node('subscriber', anonymous=True)
    pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)

    rospy.Subscriber('roth', UInt16, lambda x: callback(x, pub, q))
    rospy.spin()

if __name__ == '__main__':
    q = 0.15
    listener(q)
