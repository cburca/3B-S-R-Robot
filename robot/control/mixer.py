# mixer.py
class DiffDriveMixer:
    def __init__(self, r, L):
        self.r = r
        self.L = L

    def wheel_speed_setpoints(self, v_cmd, yaw_rate_cmd):
        # ωr,sp = v/r + (L/2r) * ωcmd 
        # ωl,sp = v/r - (L/2r) * ωcmd
        w_base = v_cmd / self.r
        dw = (self.L / (2.0 * self.r)) * yaw_rate_cmd
        w_r = w_base - dw
        w_l = w_base + dw
        return w_l, w_r