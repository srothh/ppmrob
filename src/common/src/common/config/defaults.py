drone_command_timeout = 30
drone_twist_sensor_publish_topic_name = "/drone/twist"
drone_image_sensor_publish_topic_name = "/drone/camera"
drone_battery_sensor_publish_topic_name = "/drone/battery"
drone_launch_action_name = "launch"
drone_move_action_name = "move"
drone_command_action_name = "command"
drone_emergency_action_name = "emergency"

battery_publish_topic_name = "/battery/return_signal"


class Odometry:
    HOME_COORDS_TOPIC_NAME = "/odometry/home_coordinates"


class Mapping:
    VICTIM_FOUND_TOPIC_NAME = "/mapping/victim_found"


class Planning:
    MOVE_ACTION_NAMESPACE = "planning_move"
    COMMAND_ACTION_NAMESPACE = "planning_command"


class TelloCommands:
    TAKEOFF = "takeoff"
    LAND = "land"
    STOP = "stop"
