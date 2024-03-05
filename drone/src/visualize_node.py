#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
import matplotlib.pyplot as plt
import control.msg
import drone.msg

positions = []
targets = []
moves = []
twists = []



def odometry_callback(data: PoseStamped):
    global positions
    coordinates = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    positions.append(coordinates)

def control_callback(data: control.msg.TransformActionGoal):
    global targets
    target = (data.goal.target.translation.x, data.goal.target.translation.y, data.goal.target.translation.z)
    targets.append(target)

def drone_move_callback(data: drone.msg.MoveActionGoal):
    global moves
    move = (data.goal.target.translation.x, data.goal.target.translation.y, data.goal.target.translation.z, data.goal.target.rotation.z)
    moves.append(move)

def drone_twist_callback(data: TwistStamped):
    global twists
    twist = (data.twist.linear.x, data.twist.linear.y, data.twist.linear.z)
    twists.append(twist)
    

if __name__ == '__main__':
    try:
        rospy.init_node('visualize_node')  # register the node with roscore, allowing it to communicate with other nodes
        odo_subscriber = rospy.Subscriber('/odometry/return_signal', PoseStamped, callback=odometry_callback)
        control_subscriber = rospy.Subscriber('/TransformActionServer/goal', control.msg.TransformActionGoal, callback=control_callback)
        drone_subscriber = rospy.Subscriber('/move/goal', drone.msg.MoveActionGoal, callback=drone_move_callback)
        twist_subscriber = rospy.Subscriber('/drone/twist', TwistStamped, callback=drone_twist_callback)
        
        targets.append((0,0,0))


        #fig, (ax1, ax2) = plt.subplots(2)
        ax1 = plt.subplot(2, 1, 1)
        ax1.axis("equal")
        ax1.grid(True)
        ax2 = plt.subplot(2, 1, 2, projection='polar')
        
        
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        while not rospy.is_shutdown():
            if positions:
                x, y, z = positions[-1]
                ax1.plot(x, y, "-ob")
            if targets:
                x, y, z = zip(*targets)                
                ax1.plot(x, y, "-r")
            if moves:
                x, y, z, r = moves[-1]                
                ax2.cla()
                plt.polar(r,x,'ro')
            #if twists:
            #    x, y, z = twists[-1]                
            #    rospy.loginfo('x:%d y:%d z:%d' % (x, y, z))
            #    plt.polar(0,x,'bo')
            #    plt.polar(90,y, 'bo')

                

            #plt.plot(ipx, ipy, "or")
            #ax1.savefig("/tmp/viz.png")
            plt.pause(0.1)    
#        rospy.spin()
    except rospy.ROSInterruptException:
        pass
