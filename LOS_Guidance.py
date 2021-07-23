# LOS Guidance + PID control

def LOS_Guidance(self, waypoint_x, waypoint_y):
    # Advance direction angle error

    if self.GPS_x == 0.0 and self.GPS_y == 0.0 and self.psi == 0.0
        return

    else:
        error_x = waypoint_x - self.GPS_x
        error_y = waypoint_y - self.GPS_y

        theta = math.atan2(error_x, error_y) # y axis = north
        theta = theta * 180 / math.pi # radian -> angle

        # theta : angle of waypoint direction in Earth Coordinate system

        error_N = theta - self.psi # psi = headingangle
        # error_N : error of y(angle) between the target and the ship
        # if error_N ~= 0 :True direction

        # compensate the angle representaion range from [-pi, pi] to [0, 2 * pi]

        if 180 <= error_N and error_N < 360:
            error_N -= 360
        elif -360 < error_N and error_N < -180
            error_N = error_N + 360 
