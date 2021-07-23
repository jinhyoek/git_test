class GlobalPath:
    def __init__(self):
        self.GPS_x = 0.0
        self.GPS_y = 0.0
        self.psi = 0.0
        self.i = 0 #for 'Waypoints List' Index
        self.error_Distance = 30.0 # if error_distance = 0
        self.DirectionAngle = 0.0
        rospy.Subscriber("/GPS_RealTime", GPS, self.gps_callback)
        rospy.Subscriber("/HeadingAngle_RealTime", HeadingAngle, self.HeadingAngle)

        self.direction_pub = rospy.Publisher("/Direction_Angle", DirectionAngle, queue_size = 10)
        self.thruster_pub = rospy.Publisher("/Error_Distance", Float32, queue_size = 10)

    def gps_callback(self, msg):
        self.GPS_x = msg.x
        self.GPS_y = msg.y

    def HeadingAngle_callback(self, msg):
        self.psi = msg.headingAngle # Heading Angle

    # unpack of 'waypoints list'

    def waypoint_step(self, waypoints, error_Distance, tolerance):
        if self.i + 1 != len(waypoints):
            if error_Distance > tolerance:
                waypoint = waypoints[self.i]
            elif error_Distance <= tolerance:
                waypoint = waypoints[self.i + 1]
                self.i = self.i + 1
        elif self.i + 1 == len(waypoints):
            waypoint = waypoints[self.i]

        return waypoint 
