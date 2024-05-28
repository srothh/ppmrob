drone_command_timeout = 30
drone_twist_sensor_publish_topic_name = "/drone/twist"
drone_image_sensor_publish_topic_name = "/drone/camera"
drone_battery_sensor_publish_topic_name = "/drone/battery"
drone_launch_action_name = "launch"
drone_move_action_name = "move"
drone_command_action_name = "command"
drone_emergency_action_name = "emergency"
drone_image_sensor_publish_delay = 0.5
map_resolution = 30  # 30cm per grid cell

battery_publish_topic_name = "/battery/return_signal"


class Mapping:
    VICTIM_FOUND_TOPIC_NAME = "/mapping/victim_found"
    OCCUPANCY_GRID_TOPIC_NAME = "/mapping/map"


class Control:
    MOVE_ACTION_NAMESPACE = "/control/planning_move"
    COMMAND_ACTION_NAMESPACE = "/control/planning_command"


class Odometry:
    WORLD_POSITION_TOPIC_NAME = "/odometry/return_signal"


class TelloCommands:
    TAKEOFF = "takeoff"
    LAND = "land"
    STOP = "stop"


class Drone:
    BATTERY_THRESHOLD = 10.0


class Cockpit:
    map_xlim = (-300, 300)
    map_ylim = (-300, 300)


class Planning:
    BT_SETUP_TIMEOUT = 60


class CV:
    VICTIM_LINES_TOPIC_NAME = "/cv/victims"
