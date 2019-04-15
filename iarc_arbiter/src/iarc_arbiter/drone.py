import math

import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from iarc_arbiter.msg import RegisterBehavior, VelAlt, PosCam
from iarc_main.msg import Roomba
from std_msgs.msg import String, Empty, Header

try:
    from ardrone_autonomy.msg import Navdata
except ImportError:
    rospy.logwarn("Unable to import ardrone_autonomy, assuming running on PX4")
    Navdata = None


class Drone:
    DEFAULT_HEIGHT = 1.5

    def __init__(self, namespace='/bebop/', tfl=None):

        rospy.loginfo("name: {}".format(rospy.get_name()))

        if rospy.get_name().endswith('/unnamed'):
            # Initialize the node
            rospy.init_node('controller', anonymous=True)
            rospy.sleep(0.2)

            # rospy.logwarn('initialing ROS node in Drone object')

        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace

        # World state
        self._remembers_flying = False

        # Remembered control state
        self.last_height = 0.0

        self.prev_target_facing_angle = None  # in radian

        self.FRAME_ID = self.prefix("base_link")

        if tfl is None:
            self.tf = tf.TransformListener()
            rospy.sleep(1.0)
        else:
            self.tf = tfl

        self.posPub = rospy.Publisher(self.namespace+'high_level/cmd_pos', PoseStamped, queue_size=0)
        self.velPub = rospy.Publisher(self.namespace+'high_level/cmd_vel', Twist, queue_size=0)
        self.takeoffPub = rospy.Publisher(self.namespace+'high_level/cmd_takeoff', Empty, queue_size=0)
        self.landPub = rospy.Publisher(self.namespace+'high_level/cmd_land', Empty, queue_size=0)
        self.velAltPub = rospy.Publisher(self.namespace+'high_level/cmd_vel_alt', VelAlt, queue_size=0)
        self.camPosPub = rospy.Publisher(self.namespace+'high_level/cmd_cam_pos', PosCam, queue_size=0)
        self.gimbalPub = rospy.Publisher(self.namespace +'camera_control', Twist, queue_size=0)

        # Give the ROS network time to figure out who the subscribers are
        rospy.sleep(0.5)

        self.navdata = None
        if Navdata is not None:
            rospy.Subscriber('/ardrone/navdata', Navdata, self._on_navdata)

    def is_flying(self):
        """
        Tells whether the drone is currently flying. That includes the state where it is in the process
        of landing, but not the state where it is in the process of taking off.
        :return (Bool): is the drone flying?
        """
        if self.navdata is not None:
            # List of states taken from
            # http://ardrone-autonomy.readthedocs.io/en/latest/reading.html#legacy-navigation-data
            # "flying" = Flying or Hovering or Landing or Looping
            return self.navdata.state in [3, 7, 4, 8, 9]
        else:
            return self._remembers_flying

    def takeoff(self, height=1.5, tol=0.2):
        """
        Commands the drone to takeoff from ground level, and blocks until it has done so.
        :param (float) height: Height in meters
        :param (float) tol: Tolerance to desired height to wait until reaching. Set to 0 to disable.
        :return: None
        """
        self.last_height = height

        self.takeoffPub.publish(Empty())

        rospy.sleep(0.5)

        if tol > 0:
            r = rospy.Rate(10)
            while height - self.get_altitude() > tol and not rospy.is_shutdown():
                self.hover(time=0, height=height)
                r.sleep()
            rospy.loginfo("Drone reached height of {} meters, takeoff finished".format(self.get_altitude()))

        self._remembers_flying = True

    def land(self, block=True):
        """
        Commands the drone to land on the ground. Directly commands the low-level controls,
        and might need changes as hardware gets upgraded.
        :param (bool) block: Wait until vehicle reaches ground?
        :return: None
        """
        self.last_height = 0

        self.landPub.publish(Empty())

        rospy.sleep(0.1)

        if block:
            r = rospy.Rate(10)
            while self.get_altitude() > 0.2 and not rospy.is_shutdown():
                self.landPub.publish(Empty())
                r.sleep()
            rospy.sleep(0.5)

        self._remembers_flying = False

    def hover(self, time=0, height=None):
        """
        Publishes 0 velocity for the given duration
        TODO: Store the starting position, and stay there
        :param (float | rospy.Duration) time: If nonzero, hovers for this amount of time.
        :param height:
        :return:
        """
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        if type(time) != rospy.Duration:
            time = rospy.Duration.from_sec(time)

        # TODO: Consider using position-based hovering in more situations
        if time.to_sec() < 1.0:
            # For short duration hovers, just aim for 0 velocity
            self.velAltPub.publish(VelAlt(height=height))
            rospy.sleep(time.to_sec())
            return
        else:
            # For long duration hovers, try to actively stay in the same place
            hover_start_time = rospy.Time.now()

            start_pos = self.get_pos('odom')
            start_pos.pose.position.z = height

            # Always use the latest available information
            start_pos.header.stamp = rospy.Time(0)

            r = rospy.Rate(10)
            self.posPub.publish(start_pos)
            while rospy.Time.now() - hover_start_time < time:
                r.sleep()
                self.posPub.publish(start_pos)

    def move_relative(self, rel_x=0.0, rel_y=0.0, frame='map', rel_height=0.0, tol=0.4):
        """
        Tells the drone to move to a position relative to its current position on the field,
        and blocks until the drone is within tol of the target, counting vertical and horizontal distance
        """

        frame = self.prefix(frame)
        prev_pos = self.get_pos(frame).pose.position

        des_x, des_y, des_height = prev_pos.x + rel_x, prev_pos.y + rel_y, prev_pos.z + rel_height

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            rel_pos = self.get_pos(frame).pose.position
            dist = math.sqrt(
                (rel_pos.x - des_x) ** 2 +
                (rel_pos.y - des_y) ** 2 +
                (rel_pos.z - des_height) ** 2)
            if dist <= tol:
                break

            self.move_towards(des_x, des_y, frame, des_height)

            r.sleep()


    def move_to(self, des_x=0.0, des_y=0.0, frame='map', height=0.0, tol=0.4):
        """
        Tells the drone to move to a specific position on the field, and blocks until the drone is
        within tol of the target, counting vertical and horizontal distance
        :param des_x:
        :param des_y:
        :param frame:
        :param height:
        :param tol:
        :return:
        """

        frame = self.prefix(frame)
        if height == 0.0:
            height = self.last_height

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            rel_pos = self.get_pos(frame).pose.position
            dist = math.sqrt(
                (rel_pos.x - des_x) ** 2 +
                (rel_pos.y - des_y) ** 2 +
                (rel_pos.z - height) ** 2)
            if dist <= tol:
                break

            self.move_towards(des_x, des_y, frame, height)

            r.sleep()

    def move_towards(self, des_x=0.0, des_y=0.0, frame='map', height=None):
        """
        Tells the drone to begin moving towards a specific position on the field, then returns
        :param height: Flight altitude, meters. Defaults to previously commanded height.
        :param frame: The tf frame associated with the target
        :param des_x: desired position x
        :param des_y: desired position y
        """

        frame = self.prefix(frame)
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = height
        pose_stamped.header.frame_id = frame

        self.posPub.publish(pose_stamped)

    def move_with_velocity(self, x=0.0, y=0.0, z=0.0, angvel=0.0):
        """
        Tells the drone to begin moving towards a specific position on the field, then returns
        :param height: Flight altitude, meters. Defaults to previously commanded height.
        :param frame: The tf frame associated with the target
        :param x: desired velocity x in the drone frame (forward) in m/s
        :param y: desired velocity y in the drone frame (left) in m/s
        :param z: desired velocity z in the drone frame (up) in m/s
        :param angvel: desired angular velocity in the yaw direction in rad/s
        """

        vel = Twist()
        vel.linear.x = x
        vel.linear.y = y
        vel.linear.z = z
        vel.angular.z = angvel

        self.velPub.publish(vel)


    def get_pos(self, frame='map'):
        """
        Gets the position of the drone in the map (or relative to some other coordinate frame)

        :param frame: The world frame in which to return the result
        :return (PoseStamped): The position of the drone at the latest available time
        """
        frame = self.prefix(frame)
        time = None
        while not time and not rospy.is_shutdown():
            try:
                time = self.tf.getLatestCommonTime(frame, self.FRAME_ID)
            except:
                rospy.sleep(0.5)
                rospy.logwarn("Frame missing, delaying...")

        position, quaternion = self.tf.lookupTransform(frame, self.FRAME_ID, time)
        return PoseStamped(
            header=Header(
                stamp=time,
                frame_id=frame
            ),
            pose=Pose(
                position=Point(
                    x=position[0], y=position[1], z=position[2]
                ),
                orientation=Quaternion(
                    x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
                )
            )
        )

    def get_altitude(self):
        return self.get_pos(frame='odom').pose.position.z

    def distance_from(self, frame):
        """
        Returns the linear distance along the ground between the robot and the origin of the provided TF frame.
        TODO: Allow PoseStamped as imput, rather than just using the origin.
        :param (str) frame: The TF frame to measure to
        :return:
        """
        frame = self.prefix(frame)
        linear = self.get_pos(frame).pose.linear

        return math.sqrt(linear.x ** 2 + linear.y ** 2)

    def _on_navdata(self, msg):
        """
        :param (Navdata) msg:
        """
        self.navdata = msg

    def travel_and_look(self, des_x=0.0, des_y=0.0, focus_x=0.0, focus_y=0.0, frame='map', height = None, tol=0.4):
        """
        Tells the drone to move towards a given destination, and look at a given position
        :param des_x: x position to go to
        :param des_y: y position to go to
        :param focus_x: x position to look at
        :param focus_y: y position to look at
        :param frame: The tf frame associated with the target
        :param height: The height for the drone to be, if none, stays steady


        """
        frame = self.prefix(frame)
        if height is None:
            height = self.last_height
        else:
            self.last_height = height
        camMsg = PosCam()
        camMsg.look_at_position.pose.position.x = focus_x
        camMsg.look_at_position.pose.position.y = focus_y
        camMsg.look_at_position.pose.position.z = 0.0

        camMsg.pose_stamped.pose.position.x = des_x
        camMsg.pose_stamped.pose.position.y = des_y
        camMsg.pose_stamped.pose.position.z = height
        camMsg.pose_stamped.header.frame_id = frame
        camMsg.look_at_position.header.frame_id = frame
        self.camPosPub.publish(camMsg)

        rel_pos = self.get_pos(frame).pose.position
        dist = math.sqrt(
            (rel_pos.x - des_x) ** 2 +
            (rel_pos.y - des_y) ** 2 +
            (rel_pos.z - height) ** 2)
        if dist <= tol:
            return True
        else:
            return False


    def turn_to(self, angle, frame = 'map'):
        """
        Tells the drone to turn to face a particular direction.
        :param angle: the euler angle for the drone to face.
        :param frame: The tf frame associated with the target
        """
        frame = self.prefix(frame)
        pose_stamped = self.get_pos(frame)
        orientationQuat = tf.transformations.quaternion_from_euler(0,0,angle)
        pose_stamped.pose.orientation.x = orientationQuat[0]
        pose_stamped.pose.orientation.y = orientationQuat[1]
        pose_stamped.pose.orientation.z = orientationQuat[2]
        pose_stamped.pose.orientation.w = orientationQuat[3]

        self.posPub.publish(pose_stamped)

    def move_camera(self, pitch, yaw):
        """
        Tells the gimbal to make the camera point in a certain direction
        param pitch: the pitch to make the camera go to (degrees), makes the camera look up(+) or down(-)
        param yaw: the yaw to make the camera go to (degrees), makes the camera look left(-) or right(+)
        Roll doesn't do anything
        """
        cameraCoordinates = Twist()
        cameraCoordinates.angular.y = pitch
        cameraCoordinates.angular.z = yaw
        self.gimbalPub.publish(cameraCoordinates)

    def prefix(self, frame):
        """ Appends drone namespace to tf frame id if appropriate """
        if frame in ['odom', 'base_link']:
            frame = self.namespace[1:-1]+'/'+frame
        return frame
