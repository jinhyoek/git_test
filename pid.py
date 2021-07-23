class PID:
    """ PID controller """

    def __init__(self, kp, ki, kd):
        # Gains for each term
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Corrections (outputs)
        self.cp = 0.0
        self.ci = 0.0
        self.cd = 0.0

        self.previous_error = 0.0

    def update(self, error):
        #dt = random specified value
        dt = 0.1
        de = error - self.previous_error

        self.cp = error
        self.ci += error * dt
        self.cd = de / dt

        self.previous_error = error

        return (
                (self.kp * self.cp)
                + (self.ki * self.ci)
                + (self.kd * self.cd)
                )
