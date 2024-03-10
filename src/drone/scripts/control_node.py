#!/usr/bin/env python3
import math

import sys
print('sys.path:', sys.path)


import actionlib
import rospy
import drone.msg
import control.msg

from std_msgs.msg import String
from geometry_msgs.msg import Transform, Vector3, Quaternion
from actionlib_msgs.msg import GoalStatus



class DroneControl:
    def __init__(self):
        self.prev = Vector3(0.0, 0.0, 0.0)
        self.course = 0.0
        self.launched = False
        rospy.init_node("control_node", anonymous=True)

        self._feedback = control.msg.TransformFeedback()
        self._result = control.msg.TransformResult()

        # Action Server
        self._as = actionlib.SimpleActionServer(
            'TransformActionServer',
            control.msg.TransformAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )

        self.drone_launch_client = actionlib.SimpleActionClient("launch", drone.msg.LaunchAction)
        self.drone_launch_client.wait_for_server()
        rospy.loginfo("drone launch client initalized")
        self.drone_move_client = actionlib.SimpleActionClient("move", drone.msg.MoveAction)
        self.drone_move_client.wait_for_server()
        rospy.loginfo("drone move client initalized")


    def execute_cb(self, goal):
        """ Sends the calculated rotation and translation values as action messages to
        the drone node

        @rtype: object
        """


  #      Actions:
  #          Takeoff
  #          Land
  #          Movement x, y ,z geometry/translation
  #          
  #          *Waypoints [] translation
  #      
  #      subscriptions:
  #          mapping/odometry/position



        #launch if needed

        # if not self.launched:
        #     success = self.drone_launch_client.send_goal_and_wait(drone.msg.LaunchGoal(takeoff=True))
        #     self.launched = (success == GoalStatus.SUCCEEDED)


        target = goal.target.translation
        course, distance = self.calculate_rotation_and_translation(self.prev, target)
        rbearing = course - self.course
        rbearing = round(self.normalize_angle(rbearing))
        rospy.loginfo("x1: %.2f y1: %.2f x2: %.2f y2: %.2f course: %.2f bear: %.2f dist: %.2f" % (self.prev.x, self.prev.y, target.x, target.y, course, rbearing, distance))


        if rbearing != 0.0:
            while True:
                if self.move_drone(r = rbearing):
                    break
                else:
                    rospy.loginfo('retrying')

        while True:
            if self.move_drone(x = distance):
                break
            else:
                rospy.loginfo('retrying')
        
        self.prev = target
        self.course = course
        self._result.success = True

        self._as.set_succeeded(self._result)

    def move_drone(self, x=0, y=0, z=0, r=0):
        self.drone_move_client.send_goal_and_wait(drone.msg.MoveGoal(target=Transform(Vector3(x, y, z), Quaternion(0, 0, r, 0))))
        result = self.drone_move_client.get_result()
        return result.success
        


    def main_process(self):
        """ The function where several functions run in a loop and are executed

        @rtype: object
        """

        # Start the action server
        self._as.start()


    def calculate_rotation_and_translation(self, prev, target):
        """ Calculates the rotation angle and translation for the drone node
        Return the angle in degrees and hthe translation as float

        @rtype: object
        """
        if prev.x is None or prev.y is None or target.x is None or target.y is None:
            degree_angle = 0.0
            distance = 0.0
        else:
            dx = target.x - prev.x
            dy = target.y - prev.y
            # hypoth = math.sqrt(dx ** 2 + dy ** 2)
            # sine = dy / hypoth
            # rad_angle = math.asin(sine)
            angle = math.atan2(dy, dx)
            degree_angle = (math.degrees(angle) * (-1)) % 360
            distance = math.sqrt((dx) ** 2 + (dy) ** 2)

        return degree_angle, distance

    # calculate euclidian distance between coordinates x1,y1 and x2,y2
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        #return math.sqrt((20) ** 2 + (140) ** 2)

    # calculate angle between two coordinates x1,y1 and x2,y2
    def calc_course(self, x1, y1, x2, y2):
        #return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi;
        return (math.degrees(math.atan2(y2 - y1, x2 - x1)) *(-1)) % 360;


    def normalize_angle(self, angle):
        if angle > 180.0:
            angle -= 360.0
        elif angle < -180.0:
            angle += 360.0
        return angle



if __name__ == '__main__':
    try:
        control_node = DroneControl()
        control_node.main_process()
        rospy.loginfo("control node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
