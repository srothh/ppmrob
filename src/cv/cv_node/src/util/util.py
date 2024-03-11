from geometry_msgs.msg import Point32, Point

def build_coordinate_msg(detected, msg):
    for pair in detected:
        # Check if it is lines or victims
        if isinstance(pair[0], list):
            pair = pair[0]
        detected_victim = PointPair(first=Point(x=pair[0], y=pair[1]), second=Point(x=pair[2], y=pair[3]))
        msg.point_pairs.append(detected_victim)
    return msg

def build_polygon_msg(detected,msg):
    for pair in detected:
        # Check if it is lines or victims
        if isinstance(pair[0], list):
            pair = pair[0]
        first = Point32(x=pair[0], y=pair[1])
        second = Point32(x=pair[2], y=pair[3])
        msg.point_pairs.append(first).append(second)
    return msg
