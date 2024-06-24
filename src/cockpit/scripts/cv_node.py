#!/usr/bin/env python3
import math

import actionlib
import rospy
import drone.msg

from std_msgs.msg import String
from geometry_msgs.msg import Transform, Vector3, Quaternion, PolygonStamped, Point32
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header


if __name__ == '__main__':
    try:
        rospy.init_node('cv', anonymous=True)
        rospy.loginfo("cv node started")

        rate = rospy.Rate(1)
        pub_victim = rospy.Publisher('/cv/victim', PolygonStamped, queue_size=10)  # change message type
        pub_line = rospy.Publisher('/cv/lines', PolygonStamped, queue_size=10)  # change message type
        while not rospy.is_shutdown():
            victim = PolygonStamped()
            victim.header = Header(stamp=rospy.Time.now(), frame_id="cv")
            victim.polygon.points = [
                Point32(800.0, 400.0, 0),
                Point32(400.0, 100.0, 0),
                Point32(50.0, 100.0, 0),
                Point32(100.0, 50.0, 0)
            ]
            pub_victim.publish(victim)
            lines = PolygonStamped()
            lines.header = Header(stamp=rospy.Time.now(), frame_id="cv")
            lines.polygon.points = [
                Point32(437.0, 184.0, 0), 
                Point32(827.0, 448.0, 0),
                Point32(346.0, 127.0, 0),
                Point32(781.0, 431.0, 0),
                Point32(344.0, 127.0, 0),
                Point32(780.0, 432.0, 0),
                Point32(344.0, 129.0, 0),
                Point32(673.0, 359.0, 0),
                Point32(369.0, 141.0, 0),
                Point32(781.0, 430.0, 0),
                Point32(316.0, 100.0, 0),
                Point32(435.0, 181.0, 0),
                Point32(562.0, 270.0, 0),
                Point32(825.0, 448.0, 0),
                Point32(872.0, 456.0, 0),
                Point32(949.0, 275.0, 0),
                Point32(825.0, 438.0, 0),
                Point32(958.0, 108.0, 0),
                Point32(832.0, 446.0, 0),
                Point32(958.0, 100.0, 0),
                Point32(893.0, 455.0, 0),
                Point32(959.0, 261.0, 0),
                Point32(836.0, 441.0, 0),
                Point32(958.0, 105.0, 0),
                Point32(873.0, 457.0, 0),
                Point32(956.0, 262.0, 0),
                Point32(826.0, 438.0, 0),
                Point32(895.0, 268.0, 0),
                Point32(894.0, 455.0, 0),
                Point32(958.0, 267.0, 0),
                Point32(353.0, 126.0, 0),
                Point32(518.0, 238.0, 0),
                Point32(827.0, 438.0, 0),
                Point32(959.0, 110.0, 0)
            ]
            pub_line.publish(lines)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
