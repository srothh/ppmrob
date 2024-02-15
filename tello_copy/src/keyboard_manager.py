import rospy
from keyboard.msg import Key
from std_msgs.msg import Bool

SAFETY_LAND_PUB_DEFAULT_RATE = 3


class KeyboardManager:
    def __init__(self, safety_land_pub_rate=SAFETY_LAND_PUB_DEFAULT_RATE): # class initialization
        self._safety_land_pub = rospy.Publisher('/tello/safety', Bool, queue_size=1) # TODO queue size?
        self._key_sub = rospy.Subscriber('/keyboard/keydown', Key, callback=self.get_key, queue_size=1) # TODO queue size?
        self._safety_land = Bool()
        self._key_code = -1 # used to hold the keyboard code
        self._safety_land_pub_rate = safety_land_pub_rate
        self.mainloop()

    def get_key(self, msg): # called every time the subscriber receives a message (callback function)
        self._key_code = msg.code

    def mainloop(self):
        rate = rospy.Rate(3) # set rate with given frequency of this loop
        while not rospy.is_shutdown():
            self._safety_land_pub.publish(self._safety_land_pub_rate)
            if self._key_code == Key.KEY_l: # KEY_l pressed (see doc)
                self._safety_land.data = True
            else:
                self._safety_land.data = False
            if self._key_code != -1:
                self._key_code = -1 # reset the key code
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('keyboard_manager_node') # register with roscore for communication with other nodes
    try:
        km = KeyboardManager()
    except rospy.ROSInterruptException:
        pass
