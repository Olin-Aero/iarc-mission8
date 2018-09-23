
#!/usr/bin/env python2

from util.Drone import Drone
import rospy

drone = Drone()

# commands to move a drone in the shape of a star
rospy.loginfo("taking off...")
drone.takeoff()

# move to pt. 1
rospy.loginfo("starting star movement...")
drone.move_to(2.0, 2.0, 'launch')

# change altitude
drone.hover(3)

# move to pt. 2
drone.move_to(0.0, 2.0, 'launch')

# move to pt. 3
drone.move_to(1.75, 0.5, 'launch')

# move to pt. 4
drone.move_to(1.0, 3.0, 'launch')

# move to pt. 5
drone.move_to(0.25, 0.5, 'launch')

# move to pt. 6
drone.move_to(2.0, 2.0, 'launch')

# hover in place for 3 seconds before landing
rospy.loginfo("lowering landing...")
drone.hover(3)
drone.land()
