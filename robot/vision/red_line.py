# robot/vision/red_line.py
import cv2 as cv
import numpy as np
import math

def wrap_deg(a: float) -> float:
    """Wrap angle to (-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0

class RedLineDetector:
    """
    Returns:
      angle_deg: heading error in degrees (0 = vertical), + right tilt, - left tilt
      offset_px: x offset at y_ref from image center (pixels), + right, - left
      valid:     True if line detected
      debug:     dictionaty with optional debug frames (mask/edges) if enabled
    """
    def __init__(self, cfg):
        self.cfg = cfg

        # HSV red thresholds (two ranges because red wraps hue)
        self.red_lower1 = np.array(cfg.RED_LOWER1, dtype=np.uint8)
        self.red_upper1 = np.array(cfg.RED_UPPER1, dtype=np.uint8)
        self.red_lower2 = np.array(cfg.RED_LOWER2, dtype=np.uint8)
        self.red_upper2 = np.array(cfg.RED_UPPER2, dtype=np.uint8)

    def process(self, frame_bgr):
        frame = cv.resize(frame_bgr, (self.cfg.VISION_W, self.cfg.VISION_H))
        h, w = frame.shape[:2]
        cx_img = w // 2

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv.bitwise_or(mask1, mask2)

        kernel = np.ones((self.cfg.MORPH_K, self.cfg.MORPH_K), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)

        angle_deg = None
        offset_px = None
        edges = None

        # Quick area gate
        if cv.countNonZero(mask) > self.cfg.MIN_MASK_AREA:
            edges = cv.Canny(mask, self.cfg.CANNY1, self.cfg.CANNY2)

            lines = cv.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi / 180,
                threshold=self.cfg.HOUGH_THRESH,
                minLineLength=self.cfg.MIN_LINE_LEN,
                maxLineGap=self.cfg.MAX_LINE_GAP
            )

            if lines is not None and len(lines) > 0:
                # length-weighted average direction (your logic)
                sum_w = 0.0
                sum_sin = 0.0
                sum_cos = 0.0

                y_ref = int(self.cfg.YREF_FRAC * h)
                sum_xref = 0.0
                sum_wxref = 0.0

                for (x1, y1, x2, y2) in lines[:, 0, :]:
                    dx = x2 - x1
                    dy = y2 - y1
                    length = math.hypot(dx, dy)
                    if length < 1e-6:
                        continue

                    ang_from_x = math.degrees(math.atan2(dy, dx))
                    ang_from_vert = wrap_deg(ang_from_x - 90.0)

                    # keep only lines within ±MAX_ABS_DEG_FROM_VERTICAL
                    if abs(ang_from_vert) > self.cfg.MAX_ABS_DEG_FROM_VERTICAL:
                        continue

                    wgt = length
                    sum_w += wgt

                    rad = math.radians(ang_from_vert)
                    sum_cos += wgt * math.cos(rad)
                    sum_sin += wgt * math.sin(rad)

                    # offset at y_ref (your logic)
                    if abs(dy) > 1e-6:
                        t = (y_ref - y1) / dy
                        x_at_y = x1 + t * dx
                        if -0.25 <= t <= 1.25:
                            sum_xref += wgt * x_at_y
                            sum_wxref += wgt

                    if self.cfg.DEBUG_DRAW:
                        cv.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if sum_w > 0:
                    angle_deg = math.degrees(math.atan2(sum_sin, sum_cos))

                if sum_wxref > 0:
                    x_ref = sum_xref / sum_wxref
                    offset_px = int(round(x_ref - cx_img))

                    if self.cfg.DEBUG_DRAW:
                        cv.circle(frame, (int(round(x_ref)), y_ref), 6, (255, 255, 255), -1)
                        cv.line(frame, (cx_img, 0), (cx_img, h - 1), (255, 255, 255), 1)
                        cv.line(frame, (0, y_ref), (w - 1, y_ref), (255, 255, 255), 1)

        valid = (angle_deg is not None)

        debug = {}
        if self.cfg.DEBUG_SHOW:
            debug = {"frame": frame, "mask": mask, "edges": edges}

        return angle_deg, offset_px, valid, debug