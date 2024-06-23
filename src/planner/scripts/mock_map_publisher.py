#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

import common.config.defaults as defaults  # TODO add to dockerfile as per issue!

def publish_occupancy_grid():
    rospy.init_node('mock_occupancy_grid_publisher', anonymous=True)
    conf = rospy.get_param("~conf")
    if conf > 4:
        conf = 1
    pub = rospy.Publisher(defaults.Mapping.OCCUPANCY_GRID_TOPIC_NAME, OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo("Started publishing map config: "+str(conf))
    while not rospy.is_shutdown():
        # Create a dummy occupancy grid
        grid = OccupancyGrid()
        grid.header = Header(stamp=rospy.Time.now(), frame_id="world")
        grid.info.resolution = 1.0  # 1 meter per cell
        grid.info.width = 10  # 10 cells wide
        grid.info.height = 10  # 10 cells tall
        grid.info.origin.position.x = 75.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 255.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        #Test grid. 0/0 is in the top left corner.
        #Starting position published by mock control pos publisher is 75/255 which is 2/8 in the grid, or the first 0 bottom left
        #System interprets it the other way around, as 8/2. Gotta know how mapping constructs the grid to fix the dimensions

        #One point in the grid represents a 30cm square irl. Target positions are calculated in the planner to be the center of these squares
        #Therefore, 0/0 is technically represented as 15/15 in real coordinates
        #This coordinate system is for testing purposes only, real life map/world would surely have different dimensions

        #Legend:
        #100 -> obstacle
        #0   -> been there
        #-1  -> unknown
        #50  -> seen that its free but hasn't been there directly

        #This is a showcase of the planning system with all its dependencies mocked
        #The position of the drone isn't actually updating
        #Because of this, the plan also does not update in real time
        #It is however sufficient to showcase these 4 examples, as they are the 4 situations the drone would encounter irl
        #We observe that considering the context of the mocked nodes, the correct paths/actions are calculated

        if conf == 1:
            #Drone explored everything except one spot that it knows is free
            #Should plot a path to that spot following points that it's already explored and knows they are free
            grid.data = [
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0,   0,   0,   0,   0, 100, 100, 100,
                100, 100,   0, 100, 100,  50, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            ]
        elif conf == 2:
            #Drone started out in the bottom left and sees around itself the following states
            #It should move forward to the spot it knows is free using dynamic planning
            grid.data = [
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1,100, 50,100, -1, -1, -1, -1, -1, -1,
                -1,100,  0,100, -1, -1, -1, -1, -1, -1,
                -1,100,100,100, -1, -1, -1, -1, -1, -1,
            ]
        elif conf == 3:
            #Theoretical error setup where the drone has nowhere else to fly but its home position is not reachable (0/0)
            #Should just land
            grid.data = [
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0,   0,   0,   0,   0, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            ]
        elif conf == 4:
            #More complex path to plan, same principles as above
            #Plans a path to the spot marked "known free" which is the 50
            grid.data = [
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                100, 100,  50, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100, 100, 100, 100, 100,
                100, 100,   0, 100, 100, 100,   0, 100, 100, 100,
                100, 100,   0,   0,   0, 100,   0, 100, 100, 100,
                100, 100, 100, 100,   0, 100,   0, 100, 100, 100,
                100, 100, 100, 100,   0,   0,   0, 100, 100, 100,
                100, 100, 100, 100, 100, 100,   0,   0, 100, 100,
                100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
            ]

        # Publish the occupancy grid
        pub.publish(grid)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_occupancy_grid()
    except rospy.ROSInterruptException:
        pass
