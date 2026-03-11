import cv2 as cv
import numpy as np
import math


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class RedLineDetector:
    """
    Returns:
      angle_deg: heading error in degrees (0 = vertical), + right tilt, - left tilt
      offset_px: x offset at y_ref from image center (pixels), + right, - left
      valid:     True if line detected
      debug:     dictionary with optional debug frames (mask/edges/frame) if enabled
    """
    def __init__(self, cfg):
        self.cfg = cfg

        self.red_lower1 = np.array(cfg.RED_LOWER1, dtype=np.uint8)
        self.red_upper1 = np.array(cfg.RED_UPPER1, dtype=np.uint8)
        self.red_lower2 = np.array(cfg.RED_LOWER2, dtype=np.uint8)
        self.red_upper2 = np.array(cfg.RED_UPPER2, dtype=np.uint8)

        k = int(cfg.MORPH_K)
        self._kernel = np.ones((k, k), np.uint8)

    def process(self, frame_bgr):
        angle_deg = None
        offset_px = None

        if frame_bgr is None:
            return angle_deg, offset_px, False, {}

        if frame_bgr.shape[1] != self.cfg.VISION_W or frame_bgr.shape[0] != self.cfg.VISION_H:
            frame = cv.resize(frame_bgr, (self.cfg.VISION_W, self.cfg.VISION_H))
        else:
            frame = frame_bgr

        h, w = frame.shape[:2]
        cx_img = w // 2

        roi_y0 = int(0.40 * h)
        roi = frame[roi_y0:h, :]
        roi_h, roi_w = roi.shape[:2]
        y_ref_roi = int(self.cfg.YREF_FRAC * roi_h)

        hsv = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        mask = cv.bitwise_or(mask1, mask2)

        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self._kernel, iterations=1)

        if cv.countNonZero(mask) > self.cfg.MIN_MASK_AREA:
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            best = None
            best_area = 0.0
            for cnt in contours:
                area = cv.contourArea(cnt)
                if area > best_area:
                    best_area = area
                    best = cnt

            if best is not None and best_area >= self.cfg.MIN_MASK_AREA and len(best) >= 2:
                vx, vy, x0, y0 = cv.fitLine(best, cv.DIST_L2, 0, 0.01, 0.01)
                vx = float(vx)
                vy = float(vy)
                x0 = float(x0)
                y0 = float(y0)

                if vy < 0.0:
                    vx = -vx
                    vy = -vy

                angle_deg = wrap_deg(math.degrees(math.atan2(vx, vy)))

                if abs(angle_deg) <= self.cfg.MAX_ABS_DEG_FROM_VERTICAL and abs(vy) > 1e-6:
                    t = (y_ref_roi - y0) / vy
                    x_ref = x0 + t * vx
                    offset_px = int(round(x_ref - cx_img))
                else:
                    angle_deg = None
                    offset_px = None

                if self.cfg.DEBUG_DRAW:
                    cv.drawContours(roi, [best], -1, (0, 255, 0), 2)

                    p1 = (int(round(x0 - 200 * vx)), int(round(y0 - 200 * vy)))
                    p2 = (int(round(x0 + 200 * vx)), int(round(y0 + 200 * vy)))
                    cv.line(roi, p1, p2, (255, 0, 0), 2)

                    if offset_px is not None:
                        cv.circle(roi, (int(round(x_ref)), y_ref_roi), 6, (255, 255, 255), -1)

        valid = (angle_deg is not None) and (offset_px is not None)

        debug = {}
        if self.cfg.DEBUG_SHOW:
            dbg_frame = frame.copy()
            cv.line(dbg_frame, (cx_img, 0), (cx_img, h - 1), (255, 255, 255), 1)
            cv.line(dbg_frame, (0, roi_y0 + y_ref_roi), (w - 1, roi_y0 + y_ref_roi), (255, 255, 255), 1)
            debug = {"frame": dbg_frame, "mask": mask, "edges": None}

        return angle_deg, offset_px, valid, debug