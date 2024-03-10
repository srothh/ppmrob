#!/usr/bin/env python3

import rospy  # the library should be added as package dependency for the package on which working here
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Bool
import matplotlib.pyplot as plt
from matplotlib import gridspec, transforms
import numpy as np
import drone.msg
import drone.msg
import common.config.defaults
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib.animation import FuncAnimation
import time

positions = []
targets = []
moves = []
twists = []
batteries = []
battery_signal = True
image = []
br = CvBridge()

last_arrow = None
last_azim = None
last_rot = None
last_pos = None
last_pos_history = []

last_time = 0

def odometry_callback(data: PoseStamped):
    global positions
    coordinates = (data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.z)
    positions.append(coordinates)

def control_callback(data: drone.msg.ControlTransformActionGoal):
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

def drone_camera_callback(data: Image):
    global image, br
    image = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    #cv2.imshow("Image window", image)


def update(frame_number):
    global last_arrow, last_azim, last_rot, last_pos, last_pos_history, last_time

    print('%f %d' % (time.time()-last_time, frame_number))
    last_time = time.time()

    # if len(image) > 0:
    #     ax2.imshow(image)
    # if batteries:
    #     bats = batteries[-10:].copy()
    #     ax3.cla()
    #     ax3.set_ylim(0,100)
    #     ax3.fill_between(np.arange(0, len(bats)), bats, color = ('orange' if battery_signal else 'olive'))
    if positions:
        # current position
        xcp, ycp, zcp, ozcp = positions[-1]
        if last_pos:
            last_pos.remove()
            for lp in last_pos_history:
               if len(lp)>0:
                l = lp.pop(0)
                l.remove()
            #ax1.lines.pop(0)
        last_pos, = ax1.plot(xcp, ycp, "x", color="blue", alpha=0.8)
        # history
        xp = [i for i, j, k, l in positions[-100:-1]]
        yp = [j for i, j, k, l in positions[-100:-1]]               
        #ax1.plot(xp, yp, ".", color="lightblue", alpha=0.6)
        step=10
        [last_pos_history.append( ax1.plot(xp[step*i:step*(i+1)],yp[step*i:step*(i+1)], '.',alpha=np.min([0.1+0.01*i,1]),color='tab:blue',lw=1) ) for i in range(int(len(xp)/step))]
        #[ax1.plot(xp[i:(i+1)],yp[i:(i+1)], '.',alpha=np.min([0.1+0.01*i,1]),color='tab:blue',lw=1) for i in range(int(len(xp)))]
        #[print('x%f y%f, a%f' % (xp[i:(i+1)],yp[i:(i+1)],np.min([0.1+0.01*i,1]))) for i in range(int(len(xp)))]
        # orientation
        # if last_azim:
        #     ax4.lines.pop(0)
        #     #last_azim.pop()
        # #ax4.set_theta_zero_location('N')
        # ax4.set_theta_offset(np.deg2rad(ozcp)+np.pi/2)
        # #last_azim = ax4.arrow(0,0,np.deg2rad(ozcp)-np.pi,0.8, alpha = 0.5, width = 0.02, edgecolor = 'black', facecolor = 'green', lw = 2, zorder = 5)
        # last_azim = ax4.plot([0,np.deg2rad(ozcp)], [0,1], color='red')

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
        # if xm > 0:
        #     #movement
        #     try:
        #         last_rot.remove()
        #     except Exception:
        #         pass
        # else:
        #     #rotation
        #     try:
        #         last_rot.remove()
        #     except Exception:
        #         pass
        #     last_rot = ax4.fill_between(np.linspace(np.deg2rad(rm)+np.deg2rad(ozcp)-np.pi, np.deg2rad(ozcp)-np.pi, 100), 0, 1, color='lightblue', alpha=0.9)



if __name__ == '__main__':
    try:
        rospy.init_node('visualize_node')  # register the node with roscore, allowing it to communicate with other nodes
        odo_subscriber = rospy.Subscriber('/odometry/return_signal', PoseStamped, callback=odometry_callback)
        control_subscriber = rospy.Subscriber('/TransformActionServer/goal', drone.msg.ControlTransformActionGoal, callback=control_callback)
        drone_subscriber = rospy.Subscriber('/'+common.config.defaults.drone_move_action_name + '/goal', drone.msg.MoveActionGoal, callback=drone_move_callback)
        twist_subscriber = rospy.Subscriber(common.config.defaults.drone_twist_sensor_publish_topic_name, TwistStamped, callback=drone_twist_callback)
        drone_battery_subscriber = rospy.Subscriber(common.config.defaults.drone_battery_sensor_publish_topic_name, UInt8, callback=drone_battery_callback)
        battery_signal_subscriber = rospy.Subscriber(common.config.defaults.battery_publish_topic_name, Bool, callback=battery_signal_callback)
        #drone_camera_subsriber = rospy.Subscriber(common.config.defaults.drone_image_sensor_publish_topic_name, Image, callback=drone_camera_callback)
        
        targets.append((0,0,0))


        fig = plt.figure(figsize=(10, 8)) 
        gs = gridspec.GridSpec(3, 4) 
        ax1 = fig.add_subplot(gs[:,:-2])
        ax1.axis("equal")
        ax1.grid(True)
        #ax1.set_ylim(-400, 500)
        #ax1.set_xlim(-400, 500)
        #ax1.invert_xaxis()
        
        

        # ax4 = plt.subplot(gs[0,2], projection='polar')
        # ax4.set_theta_direction(-1)
        # ax4.set_yticklabels([])
        # ax4.set_xticks(np.arange(np.radians(0), np.radians(360), np.radians(10))) 
        # ax4.set_xticklabels(np.arange(0,360,10), fontsize='small')
        # ax4.grid(True)
        # ax4.tick_params(labelsize=6)

        # ax2 = plt.subplot(gs[1:,2:])
        # ax2.set_xticklabels([])
        # ax2.set_yticklabels([])

        # ax3 = plt.subplot(gs[0,3])
        # ax3.grid(True)
        # ax3.set_aspect(2)
        # ax3.set_ylim(0,100)
        # ax3.get_xaxis().set_visible(False)
        # ax3.tick_params(labelbottom=False, labeltop=False, labelleft=False, labelright=True,
        #              bottom=False, top=False, left=False, right=True)
        
        #plt.cla()
        # for stopping simulation with the esc key.
        #plt.gcf().canvas.mpl_connect(
        #    "key_release_event",
        #    lambda event: [exit(0) if event.key == "escape" else None],
        #)
        

        animation = FuncAnimation(fig, update, interval=10, save_count=100)
        plt.show()

    except rospy.ROSInterruptException:
        pass
