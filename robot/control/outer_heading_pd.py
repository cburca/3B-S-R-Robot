class HeadingPD:
    def __init__(self, kp, kd, dt, u_limit):
        self.kp = kp
        self.kd = kd
        self.dt = dt
        self.u_limit = u_limit
        self.prev_err = 0.0

    def step(self, theta_ref_rad, theta_rad):
        err = theta_ref_rad - theta_rad
        derr = (err - self.prev_err) / self.dt
        self.prev_err = err

        u = self.kp * err + self.kd * derr
        if u > self.u_limit:
            u = self.u_limit
        if u < -self.u_limit:
            u = -self.u_limit
        return u