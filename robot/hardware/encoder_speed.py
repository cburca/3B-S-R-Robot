# encoder_speed.py
import math

class EncoderSpeedEstimator:
    def __init__(self, cpr, dt, alpha=0.3):
        self.cpr = cpr
        self.dt = dt
        self.alpha = alpha
        self.prev_L = None
        self.prev_R = None
        self.wL = 0.0
        self.wR = 0.0

    def update(self, enc):
        if self.prev_L is None:
            self.prev_L = enc.left_counts
            self.prev_R = enc.right_counts
            return self.wL, self.wR

        dL = enc.left_counts - self.prev_L
        dR = enc.right_counts - self.prev_R
        self.prev_L = enc.left_counts
        self.prev_R = enc.right_counts

        # counts -> revolutions -> rad/s
        revL = dL / self.cpr
        revR = dR / self.cpr
        wL_meas = (2.0 * math.pi * revL) / self.dt
        wR_meas = (2.0 * math.pi * revR) / self.dt

        # simple low-pass filter
        # to do
        return self.wL, self.wR