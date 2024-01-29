#! /usr/bin/env python3

import rospy
import actionlib
import time
from abc import ABC, abstractmethod, abstractproperty



class ActionServer(ABC):
    # create messages that are used to publish feedback/result

    _drone = None


    def __init__(self, name, drone, action):
        self._drone = drone
        self._action_name = name

        #start action server
        self._as = actionlib.SimpleActionServer(self._action_name, action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("ActionServer started: %s", name)

    def command(self, command):
        rospy.loginfo("command: %s", command)
        r = rospy.Rate(1)
        self._drone.send_command_without_return(command) 
        responses = self._drone.get_own_udp_object()['responses']
        aborted = False

        while not responses:
            self.feedback_cb()
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._drone.send_control_command("stop") 
                aborted = True
                break
            r.sleep()  # Sleep during send command

        if (aborted):
            self.success_cb(False)
            return False
        else:    
            first_response = responses.pop(0)  # first datum from socket
            try:
                response = first_response.decode("utf-8")
            except UnicodeDecodeError as e:
                self.LOGGER.error(e)
                return "response decode error"
            response = response.rstrip("\r\n")
            print(response)
            if (response == 'ok'):
                self.success_cb(True)
            else:
                self.success_cb(False)

    @abstractmethod
    def execute_cb(self, goal):
        pass

    @abstractmethod
    def success_cb(self, success):
        pass
