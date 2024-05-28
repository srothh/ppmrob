#!/usr/bin/env python3

import matplotlib

matplotlib.use("tkagg", force=True)

import rospy  # the library should be added as package dependency for the package on which working here
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped, PolygonStamped, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg import BatteryState
import matplotlib.pyplot as plt
from matplotlib import gridspec, transforms, patches
import numpy as np
import drone.msg

import control.msg
import common.config.defaults as defaults
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib.animation import FuncAnimation
import time
from matplotlib.collections import LineCollection, PatchCollection
import matplotlib.patches as patches
import actionlib
import sys

positions = []
targets = []
moves = []
twists = []
batteries = []
battery_signal = True
image = []
victims = []
lines = []
waypoints = []

br = CvBridge()

last_arrow = None
last_azim = None
last_rot = None
last_pos = None
last_pos_history = []

last_time = 0
move_idx = 0
victim_idx = 0

map_ylim = defaults.Cockpit.map_ylim
map_xlim = defaults.Cockpit.map_xlim


def pairwise(iterable):
    "s -> (s0, s1), (s2, s3), (s4, s5), ..."
    a = iter(iterable)
    return zip(a, a)


def odometry_callback(data: PoseStamped):
    global positions
    coordinates = (
        data.pose.position.x,
        data.pose.position.y,
        data.pose.position.z,
        data.pose.orientation.z,
    )
    positions.append(coordinates)


def control_callback(data: control.msg.PlanningMoveGoal):
    global targets
    for point in data.goal.target:
        target = (point.x, point.y, point.z)
        targets.append(target)


def drone_move_callback(data: drone.msg.MoveActionGoal):
    global moves
    move = (
        data.goal.target.translation.x,
        data.goal.target.translation.y,
        data.goal.target.translation.z,
        data.goal.target.rotation.z,
    )
    moves.append(move)


def drone_twist_callback(data: TwistStamped):
    global twists
    twist = (data.twist.linear.x, data.twist.linear.y, data.twist.linear.z)
    twists.append(twist)


def drone_battery_callback(data: BatteryState):
    global batteries
    bat = data.percentage
    batteries.append(bat)


def battery_signal_callback(data: Bool):
    global battery_signal
    bat = data.data
    battery_signal = bat


def drone_camera_callback(data: Image):
    global image, br
    image = br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    # cv2.imshow("Image window", image)


def cv_victim_callback(data: PolygonStamped):
    global victims
    # rospy.loginfo('points: (%s)' % (data.points))
    victims = []
    # iterate over pairs of data.points
    for f, s in pairwise(data.polygon.points):
        victims.append([(f.x, f.y), (s.x, s.y)])


def cv_lines_callback(data: PolygonStamped):
    global lines
    lines = []
    # iterate over pairs of data.points
    for f, s in pairwise(data.polygon.points):
        lines.append([(f.x, f.y), (s.x, s.y)])


def on_press(event):
    sys.stdout.flush()
    rospy.loginfo("press %s", event.key)
    if event.key == " ":
        rospy.loginfo("EMERGENCY LANDING")
        emergency_client.send_goal_and_wait(drone.msg.EmergencyGoal(soft=True))
    elif event.key == "-":
        resize_map(-100)
    elif event.key == "+":
        resize_map(100)


def on_resize(event):
    resize_map()


def on_click(event):
    ix, iy = event.xdata, event.ydata
    waypoint = Pose()
    waypoint.position.x = ix
    waypoint.position.y = iy
    waypoint.position.z = 0
    waypoint_publisher.publish(waypoint)
    waypoints.append((ix, iy))


def resize_map(amount=0):
    global map_ylim, map_xlim, ax1
    if amount:
        map_xlim = tuple(np.add(map_xlim, (amount, (-1) * amount)))
        map_ylim = tuple(np.add(map_ylim, (amount, (-1) * amount)))
    ax1.set_ylim(map_ylim)
    ax1.set_xlim(map_xlim)
    # fig.tight_layout()
    ax1.figure.canvas.draw_idle()


rospy.init_node(
    "cockpit"
)  # register the node with roscore, allowing it to communicate with other nodes

matplotlib.use("tkAgg")

odo_subscriber = rospy.Subscriber(
    defaults.Odometry.WORLD_POSITION_TOPIC_NAME, PoseStamped, callback=odometry_callback
)
# control_subscriber = rospy.Subscriber('/TransformActionServer/goal', drone.msg.ControlTransformActionGoal, callback=control_callback)
# control_subscriber = rospy.Subscriber('/'+defaults.Control.MOVE_ACTION_NAMESPACE + '/goal', control.msg.PlanningMoveActionGoal, callback=control_callback)
control_subscriber = rospy.Subscriber(
    defaults.Control.MOVE_ACTION_NAMESPACE + "/goal",
    control.msg.PlanningMoveActionGoal,
    callback=control_callback,
)
drone_subscriber = rospy.Subscriber(
    defaults.drone_move_action_name + "/goal",
    drone.msg.MoveActionGoal,
    callback=drone_move_callback,
)
twist_subscriber = rospy.Subscriber(
    defaults.drone_twist_sensor_publish_topic_name,
    TwistStamped,
    callback=drone_twist_callback,
)
drone_battery_subscriber = rospy.Subscriber(
    defaults.drone_battery_sensor_publish_topic_name,
    BatteryState,
    callback=drone_battery_callback,
)
battery_signal_subscriber = rospy.Subscriber(
    defaults.battery_publish_topic_name,
    Bool,
    callback=battery_signal_callback,
)
drone_camera_subsriber = rospy.Subscriber(
    defaults.drone_image_sensor_publish_topic_name,
    Image,
    callback=drone_camera_callback,
)
cv_victim_subsriber = rospy.Subscriber(
    "/cv/victim", PolygonStamped, callback=cv_victim_callback
)
cv_lines_subsriber = rospy.Subscriber(
    "/cv/lines", PolygonStamped, callback=cv_lines_callback
)

waypoint_publisher = rospy.Publisher("/cockpit/waypoint", Pose, queue_size=10)

emergency_client = actionlib.SimpleActionClient(
    defaults.drone_emergency_action_name, drone.msg.EmergencyAction
)
rospy.loginfo("waiting for action server: %s", defaults.drone_emergency_action_name)
emergency_client.wait_for_server()
rospy.loginfo("done")
targets.append((0, 0, 0))
waypoints.append((0, 0))


# plt.cla()
# for stopping simulation with the esc key.
# plt.gcf().canvas.mpl_connect(
#    "key_release_event",
#    lambda event: [exit(0) if event.key == "escape" else None],
# )

fig = plt.figure(figsize=(10, 8))
fig.tight_layout()
fig.canvas.manager.set_window_title("Press SPACE for emergency landing")
fig.canvas.mpl_connect("key_press_event", on_press)
fig.canvas.mpl_connect("button_press_event", on_click)
fig.canvas.mpl_connect("resize_event", on_resize)

gs = gridspec.GridSpec(3, 4)
ax1 = fig.add_subplot(gs[:, :-2])
ax1.axis("equal")
ax1.grid(True)
ax1.set_ylim(defaults.Cockpit.map_ylim)
ax1.set_xlim(defaults.Cockpit.map_xlim)
# ax1.invert_xaxis()


target_current = ax1.arrow(
    0,
    0,
    0,
    0,
    color="red",
    length_includes_head=True,
    head_starts_at_zero=False,
    width=5,
)
(target_history,) = ax1.plot(0, 0, "-", color="black")
(pos_current,) = ax1.plot(0, 0, ".", color="blue", alpha=0.8)
pos_start = ax1.plot(0, 0, "or")
step = 10
xp = np.arange(100)
yp = np.arange(100)
# pos_history = [ax1.plot(xp[step*i:step*(i+1)],yp[step*i:step*(i+1)], '.',alpha=np.min([0.1+0.01*i,1]),color='tab:blue',lw=1)  for i in range(int(len(xp)/step))]
(pos_history,) = ax1.plot(0, 0, ".", alpha=0.6, color="tab:blue", lw=1)

(waypoints_all,) = ax1.plot(0, 0, "x", color="tab:red", lw=1)

# compass
ax4 = plt.subplot(gs[0, 2], projection="polar")
ax4.set_theta_direction(-1)
ax4.set_yticklabels([])
ax4.set_xticks(np.arange(np.radians(0), np.radians(360), np.radians(10)))
ax4.set_xticklabels(np.arange(0, 360, 10), fontsize="small")
ax4.grid(True)
ax4.tick_params(labelsize=6)

azimuth_current = ax4.arrow(0, 0, 0, 0, width=0.01, color="r")
# ax4.plot([0,np.deg2rad(0)], [0,1], color='red')
# rot_current = ax4.fill_between(np.linspace(0, 0, 100), 0, 1, color='lightblue', alpha=0.9)
(rot_current,) = ax4.plot(0, 1, color="lightgreen", alpha=0.9, linewidth=2)

move_current_x = ax4.arrow(0, 0, 0, 0.3, width=0.05)
move_current_x.set_visible(False)
move_current_y = ax4.arrow(0, 0, np.deg2rad(90), 0.3, width=0.05)
move_current_y.set_visible(False)

# camera
ax2 = plt.subplot(gs[1:, 2:])
ax2.set_xticklabels([])
ax2.set_yticklabels([])
# whiteblankimage = np.ones(shape=[720, 960, 3], dtype=np.uint8)
whiteblankimage = np.ones(shape=[240, 320, 3], dtype=np.uint8)
image_current = ax2.imshow(whiteblankimage)
# victim_current = ax2.add_patch(patches.Rectangle((0, 0), 0, 0, linewidth=2, edgecolor='g', facecolor='none'))
lines_current = LineCollection([], color="r", linewidth=1, zorder=1)
ax2.add_collection(lines_current)
victim_current = PatchCollection([], color="g", facecolor="none", linewidth=2, zorder=2)
ax2.add_collection(victim_current)

# battery
ax3 = plt.subplot(gs[0, 3])
ax3.grid(True)
ax3.set_aspect("auto")
ax3.set_ylim(0, 100)
ax3.get_xaxis().set_visible(False)
ax3.tick_params(
    labelbottom=False,
    labeltop=False,
    labelleft=False,
    labelright=True,
    bottom=False,
    top=False,
    left=False,
    right=True,
)

battery_current = ax3.bar(0, 0, color=("red"))


def update(frame_number):
    try:
        global move_idx
        # if (frame_number % 100) == 0:
        #    init()
        if len(targets) > 1:
            to_x, to_y, to_z = targets[-1]
            from_x, from_y, fr_z = targets[-2]
            target_current.set_data(
                x=from_x, y=from_y, dx=to_x - from_x, dy=to_y - from_y, width=5
            )
            x = [i for i, j, k in targets[:-1]]
            y = [j for i, j, k in targets[:-1]]
            target_history.set_data(x, y)

        if len(positions) > 0:
            xcp, ycp, zcp, azimuth = positions[-1]
            # cur_xlim = pos_current.axes.get_xlim()
            # cur_ylim = pos_current.axes.get_ylim()
            # pos_current.axes.set_xlim(np.min([cur_xlim[0], xcp]), np.max([cur_xlim[1], xcp]))
            # pos_current.axes.set_ylim(np.min([cur_ylim[0], ycp]), np.max([cur_ylim[1], ycp]))
            pos_current.set_data([xcp], [ycp])
            # xp = [i for i, j, k, l in positions[:-1]]
            # yp = [j for i, j, k, l in positions[:-1]]
            # pos_history.set_data(xp ,yp)
            # for i in range(0, len(pos_history)):
            #    s = pos_history[i][0]
            #    print(s)
            #    s.set_data(xp[i*10:(i+1)*10],yp[i*10:(i+1)*10])

            # azimuth_current.set_data(x=0,y=0,dx=np.deg2rad(azimuth),dy=1)
            # azimuth_current.axes.set_theta_offset(np.deg2rad(azimuth)+np.pi/2)
            # move_currrent = ax4.fill_between(np.linspace(np.deg2rad(rm)+np.deg2rad(azimuth), np.deg2rad(azimuth), 100), 0, 1, color='lightblue', alpha=0.9)

        if moves:
            # check if there is a new element in moves array
            if len(moves) > move_idx:
                move_idx = len(moves)
                xm, ym, zm, rm = moves[move_idx - 1]
                # rot_current.clear()
                # rot_current = ax4.fill_between(np.linspace(np.deg2rad(rm)+np.deg2rad(azimuth), np.deg2rad(azimuth), 100), 0, 1, color='lightblue', alpha=0.9)
                # rot_current.set_data(np.deg2rad(rm)+np.deg2rad(azimuth), 1)
                # if rm is not 0

                # if rm != 0.0:
                #    move_current_x.set_visible(False)
                #    move_current_y.set_visible(False)
                #    az_diff = rm + azimuth
                #    rot_current.set_data([0,np.deg2rad(rm + azimuth)], [0,np.pi])
                # else:
                #    rot_current.set_data([0],[0])
                #    if xm != 0.0:
                #        move_current_x.set_visible(True)
                #    elif ym != 0.0:
                #        move_current_y.set_visible(True)

        if len(image) > 0:
            image_current.set_data(image)
            if len(lines) > 0:
                lines_current.set_segments(lines)
            else:
                lines_current.set_segments([])
            if victims:
                rects = []
                colors = []
                edges = []
                for victim in victims:
                    f, s = victim
                    # rospy.loginfo('rect: (%d,%d) (%d,%d) ' % (f[0], f[1], s[0], s[1]))
                    rects.append(
                        patches.Rectangle(
                            (f[0], f[1]), abs(s[0] - f[0]), abs(s[1] - f[1])
                        )
                    )
                    colors.append("none")
                    edges.append("g")
                victim_current.set_paths(rects)
                # victim_current.set_facecolors(colors)
                # victim_current.set_edgecolors(edges)
                # victim_current.set_linewidths(2)
            else:
                victim_current.set_paths([])

        if batteries:
            battery_current[0].set_height(batteries[-1])
            battery_current[0].set_color("red" if battery_signal else "olive")

        if waypoints:
            x = [x for x, y in waypoints]
            y = [y for x, y in waypoints]
            waypoints_all.set_data(x, y)
            waypoints_all.set_visible(True)
    except Exception as e:
        rospy.logerr(e)
        pass

    return [
        target_current,
        target_history,
        pos_current,
        pos_history,
        azimuth_current,
        move_current_y,
        move_current_x,
        image_current,
        battery_current[0],
        rot_current,
        victim_current,
        lines_current,
        waypoints_all,
    ]


animation = FuncAnimation(
    fig, update, blit=True, repeat=False, interval=100, save_count=100
)

plt.show()
