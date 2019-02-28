class Drone:

    takeoffLand = {
        't': (1, 0),
        ' ': (0, 1)
    }

    moveBindings = {
        'I': (1, 0, 0, 0),
        'O': (1, 0, 0, -1),
        'J': (0, 0, 0, 1),
        'L': (0, 0, 0, -1),
        'U': (1, 0, 0, 1),
        '<': (-1, 0, 0, 0),
        '>': (-1, 0, 0, 1),
        'M': (-1, 0, 0, -1),
        'o': (1, -1, 0, 0),
        'i': (1, 0, 0, 0),
        'j': (0, 1, 0, 0),
        'l': (0, -1, 0, 0),
        'u': (1, 1, 0, 0),
        ',': (-1, 0, 0, 0),
        '.': (-1, -1, 0, 0),
        'm': (-1, 1, 0, 0),
        'w': (0, 0, 1, 0),
        's': (0, 0, -1, 0),
        'a': (0, 0, 0, 1),
        'd': (0, 0, 0, -1),
    }

    speedBindings = {
        'q': (1.1, 1.1),
        'z': (.9, .9),
        'r': (1.1, 1),
        'v': (.9, 1),
        'e': (1, 1.1),
        'c': (1, .9),
    }

    def __init__(self, namespace_in, namespace_out, bind):
        self.flight_status = False #initializes in teleop
        self.namespace_in = str(namespace_in) #namespace for subs
        self.namespace_out = str(namespace_out) #namespace for pubs
        self.bind = bind #number bind assigned to controlling drone

        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        self.speed =

        self.cmd_pub = rospy.Publisher('/' + self.namespace_out + '/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/' + self.namespace_out + '/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/' + self.namespace_out + '/land', Empty, queue_size=1)
        self.reset_pub = rospy.Publisher('/' + self.namespace_out + '/reset', Empty, queue_size=1)

        self.takeoff_sub = rospy.Subscriber('/' + self.namespace_in +'/takeoff', Empty, callback_takeoff)
        self.land_sub = rospy.Subscriber('/' + self.namespace_in + '/land', Empty, callback_land)
        self.cmd_sub = rospy.Subscriber('/' + self.namespace_in + '/cmd_vel', Twist, callback_cmd_vel)


    def callback_cmd_vel(self, msg):
        if self.flight_status == True:
            self.cmd_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "velocity command ignored\r")

    def callback_takeoff(self, msg):
        if self.flight_status == True:
            self.takeoff_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "takeoff command ignored\r")

    def callback_land(self, msg):
        if self.flight_status == True:
            self.land_pub.publish(msg)
        else:
            rospy.loginfo_throttle(1, "land command ignored\r")


    def get_vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def handle_key(self, key):
