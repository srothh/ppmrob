#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import UInt8, Bool
import matplotlib.pyplot as plt
from matplotlib import gridspec, transforms
import numpy as np
import control.msg
import drone.msg
import common.config.defaults


positions = []
targets = []
moves = []
twists = []
batteries = []
battery_signal = True



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

def drone_battery_callback(data: UInt8):
    global batteries
    bat = (data.data)
    batteries.append(bat)

def battery_signal_callback(data: Bool):
    global battery_signal
    bat = (data.data)
    battery_signal = bat
    

if __name__ == '__main__':
    try:
        rospy.init_node('visualize_node')  # register the node with roscore, allowing it to communicate with other nodes
        odo_subscriber = rospy.Subscriber('/odometry/return_signal', PoseStamped, callback=odometry_callback)
        control_subscriber = rospy.Subscriber('/TransformActionServer/goal', control.msg.TransformActionGoal, callback=control_callback)
        drone_subscriber = rospy.Subscriber('/'+common.config.defaults.drone_move_action_name + '/goal', drone.msg.MoveActionGoal, callback=drone_move_callback)
        twist_subscriber = rospy.Subscriber(common.config.defaults.drone_twist_sensor_publish_topic_name, TwistStamped, callback=drone_twist_callback)
        drone_battery_subscriber = rospy.Subscriber(common.config.defaults.drone_battery_sensor_publish_topic_name, UInt8, callback=drone_battery_callback)
        battery_signal_subscriber = rospy.Subscriber(common.config.defaults.battery_publish_topic_name, Bool, callback=battery_signal_callback)

        targets.append((0,0,0))


        #fig, (ax1, ax2) = plt.subplots(2)
        fig = plt.figure(figsize=(10, 8)) 
        gs = gridspec.GridSpec(2, 2, width_ratios=[2, 1]) 
        ax1 = fig.add_subplot(gs[:,0])
        ax1.axis("equal")
        ax1.grid(True)
        #ax1.set_ylim(-400, 500)
        #ax1.set_xlim(-400, 500)
        #ax1.invert_xaxis()
        
        ax2 = plt.subplot(gs[0,1], projection='polar')
        ax3 = plt.subplot(gs[1,1])
        ax3.grid(True)
        ax3.set_ylim(0,100)
        ax3.get_xaxis().set_visible(False)
        
        
        #plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        last_arrow = None
        while not rospy.is_shutdown():
            if batteries:
                bats = batteries[-10:].copy()
                ax3.cla()
                ax3.set_ylim(0,100)
                ax3.fill_between(np.arange(0, len(bats)), bats, color = ('orange' if battery_signal else 'olive'))
            if positions:
                # current position
                xcp, ycp, zcp = positions[-1]
                ax1.plot(xcp, ycp, ".", color="red")
                # history
                xp = [i for i, j, k in positions[:-1]]
                yp = [j for i, j, k in positions[:-1]]               
                ax1.plot(xp, yp, ".", color="blue")
            if targets:
                # current target
                if len(targets) > 1:
                    to_x, to_y, to_z = targets[-1]
                    fr_x, fr_y, fr_z = targets[-2]
                    if last_arrow:
                        last_arrow.remove()
                    last_arrow = ax1.arrow(fr_x, fr_y, to_x-fr_x, to_y-fr_y, color='red', length_includes_head=True, head_starts_at_zero=False, width=5)

                    # target history
                    x = [i for i, j, k in targets[:-1]]
                    y = [j for i, j, k in targets[:-1]]               
                    ax1.plot(x, y, "-", color='black')
                else:
                    x, y, z = targets[0]
                    ax1.plot(x, y, "or")

            if moves:
                xm, ym, zm, rm = moves[-1]                
                ax2.cla()
                #ax2.set_rorigin(-1.0)
                ax2.set_theta_zero_location('N')
                ax2.set_theta_direction(-1)
                ax2.set_yticklabels([])
                ax2.set_thetamin(-180)
                ax2.set_thetamax(180)
                ax2.set_xticks(np.arange(np.radians(-180),
                        np.radians(180),
                        np.radians(45),
                       )) 
                if xm > 0:
                    ax2.arrow(0,0,np.deg2rad(rm),xm, width = 0.05)
                else:
                    ax2.arrow(0,0,np.deg2rad(rm),20, width = 0.05, color='r')
                #rospy.loginfo('r:%d x:%d' % (r, x))
            #if twists:
            #    x, y, z = twists[-1]                
            #    rospy.loginfo('x:%d y:%d z:%d' % (x, y, z))
            #    plt.polar(0,x,'bo')
            #    plt.polar(90,y, 'bo')

                

            #plt.plot(ipx, ipy, "or")
            #ax1.savefig("/tmp/viz.png")
            plt.pause(0.5)    
#        rospy.spin()
    except rospy.ROSInterruptException:
        pass
