#!/usr/bin/env python2

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import Twist, PoseStamped
from iarc_arbiter.msg import VelAlt, PosCam
from std_msgs.msg import Empty, String
from tf import TransformListener

import transformers
import filters


class ArbiterLite(object):
    """
    The ArbiterLite transforms high-level commands into low-level commands by using utilities from the filters.py file.

    It subscribes to various topics of the form high_level/cmd_**, then republishes as cmd_vel, takeoff, and land
    The publishing of this node will run at the configured frequency, but one-time commands like takeoff and land will always
    be handled immediately.

    One copy of this node is intended to be created for each physical drone being controlled.
    """

    def __init__(self):
        self.tf = TransformListener()
        self.ddynrec = DDynamicReconfigure("arbiterlite_configuration")

        # Transformers are functions capable of processing incoming data in a variety of formats.
        # They are functions that take input of whatever type the topic is, and produce a transformers.Command
        # object.

        alt_pid = transformers.PIDAltController(self.tf, self.ddynrec)
        pos_pid = transformers.PIDPosController(self.tf, self.ddynrec, alt_pid)
        pos_cam_pid = transformers.PIDPosCamController(self.tf, self.ddynrec, pos_pid)

        # topic_name : (type, transformer_func, publish_immediately)
        self.transformers = {
            'high_level/cmd_vel': (Twist, transformers.cmd_vel, False),
            'high_level/cmd_takeoff': (Empty, transformers.cmd_takeoff, True),
            'high_level/cmd_land': (Empty, transformers.cmd_land, True),
            'high_level/cmd_pos': (PoseStamped, pos_pid.cmd_pos, False),
            'high_level/cmd_rel_pos': (PoseStamped, pos_pid.cmd_pos, False),
            'high_level/cmd_vel_alt': (VelAlt, alt_pid.cmd_vel_alt, False),
            'high_level/cmd_cam_pos': (PosCam, pos_cam_pid.cmd_pos_cam, False),
        }

        self.last_topic = None
        self.last_msg = None
        self.last_timestamp = None

        for topic, (msgtype, _, immediate) in self.transformers.items():
            def handler(msg, topic=topic, immediate=immediate):
                self.last_topic = topic
                self.last_msg = msg
                self.last_timestamp = rospy.Time.now()

                if immediate:
                    self.update()

            rospy.Subscriber(topic, msgtype, handler)

        # Filters constrain final output before it goes to the drone.
        # Potential examples include last-minute obstacle avoidance, speed limiters, or arena boundary constraints.
        self.filters = [filters.make_speed_filter(0.2, 0.2, 0.2)]

        self.vel_pub = rospy.Publisher('low_level/cmd_vel', Twist, queue_size=0)
        self.takeoff_pub = rospy.Publisher('low_level/takeoff', Empty, queue_size=0)
        self.land_pub = rospy.Publisher('low_level/land', Empty, queue_size=0)

        self.rate = rospy.get_param('~rate', 20)
        # How long to republish an old message before erroring out (seconds)
        self.timeout = rospy.get_param('~timeout', 1.0)

        self.start_ddynrec()

    def update(self):
        """
        Computes any PID loops and publishes the low-level commands
        """

        if self.last_topic is None:
            rospy.logwarn_throttle(
                5.0, "No messages recieved, unable to publish")
            return

        if rospy.Time.now() - self.last_timestamp > rospy.Duration.from_sec(self.timeout):
            rospy.logwarn_throttle(
                5.0, "Last message to {} was stale, refusing to publish".format(self.last_topic))
            # TODO(eric) Consider publishing a "stop" command here, rather than relying on the timeout in the drone's driver
            return

        _, func, _ = self.transformers[self.last_topic]

        # Convert to a transformers.Command
        cmd = func(self.last_msg)

        # Limit the drone speed, and apply any other filters
        for func in self.filters:
            cmd = func(cmd)

        # Publish the result to the ROS network
        if cmd.takeoff:
            self.takeoff_pub.publish(Empty())
            vel = Twist()
            vel.linear.z = 0.5
            self.vel_pub.publish(vel)
        elif cmd.land:
            self.land_pub.publish(Empty())
            vel = Twist()
            vel.linear.z = -0.5
            self.vel_pub.publish(vel)
        else:
            self.vel_pub.publish(cmd.vel)

    def start_ddynrec(self):
        """
        Helper function to start the ddynamic reconfigure server with a callback
        function that updates the self.ddynrec attribute.

        Needed to tune PID loops in realtime
        """

        def callback(config, level):
            """
            A callback function used to as the parameter in the ddynrec.start() function.
            This custom callback function updates the state of self.ddynrec so we can
            refer to its variables whenever we have access to it. 
            """
            rospy.loginfo("Received reconf call: " + str(config))
            # Update all variables
            var_names = self.ddynrec.get_variable_names()
            for var_name in var_names:
                self.ddynrec.__dict__[var_name] = config[var_name]
            return config

        self.ddynrec.start(callback)

    def run(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            if self.last_topic is not None:
                _, _, immediate = self.transformers[self.last_topic]

                if not immediate:
                    # For any topic that is not supposed to be handled immediately upon reception,
                    # calculate the PID loop here at a defined frequency.
                    self.update()

            try:
                r.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                r = rospy.Rate(self.rate)


if __name__ == '__main__':
    rospy.init_node('arbiter')
    ArbiterLite().run()
