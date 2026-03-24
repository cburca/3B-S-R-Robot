from dataclasses import dataclass, field
import math
import cv2 as cv
import numpy as np

@dataclass
class DetectionResult:
    found: bool
    center: tuple[int, int] | None = None
    offset_px: float | None = None
    angle_deg: float | None = None
    area: float | None = None
    confidence: float | None = None
    pickup_ready: bool = False
    debug: dict = field(default_factory=dict)
class FrameContext:
    def __init__(self, frame):
        self.frame = frame
        self._hsv = None
        self._gray = None
    
    @property
    def hsv(self):
        if self._hsv is None:
            self._hsv = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        return self._hsv
        
    @property
    def gray(self):
        if self._gray is None:
            self._gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        return self._gray
    
class LineDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.red_lower1 = np.array(cfg.RED_LOWER1, dtype=np.uint8)
        self.red_upper1 = np.array(cfg.RED_UPPER1, dtype=np.uint8)
        self.red_lower2 = np.array(cfg.RED_LOWER2, dtype=np.uint8)
        self.red_upper2 = np.array(cfg.RED_UPPER2, dtype=np.uint8)
        self.kernel = np.ones((cfg.KERNEL_SIZE, cfg.KERNEL_SIZE), dtype=np.uint8)
        
    def detect(self, ctx: FrameContext) -> DetectionResult:
        frame = ctx.frame
        h, w = frame.shape[:2]
        cx_img = w // 2
        y_ref = int(self.cfg.YREF_FRAC * h)

        mask1 = cv.inRange(ctx.hsv, self.red_lower1, self.red_upper1)
        mask2 = cv.inRange(ctx.hsv, self.red_lower2, self.red_upper2)
        mask = cv.bitwise_or(mask1, mask2)

        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)

        if cv.countNonZero(mask) <= self.cfg.MIN_MASK_AREA:
            return DetectionResult(found=False)

        edges = cv.Canny(mask, self.cfg.CANNY1, self.cfg.CANNY2)

        lines = cv.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.cfg.HOUGH_THRESH,
            minLineLength=self.cfg.MIN_LINE_LEN,
            maxLineGap=self.cfg.MAX_LINE_GAP
        )

        if lines is None or len(lines) == 0:
            return DetectionResult(found=False)

        sum_w = 0.0
        sum_sin = 0.0
        sum_cos = 0.0
        sum_xref = 0.0
        sum_wxref = 0.0

        for x1, y1, x2, y2 in lines[:, 0, :]:
            if y2 < y1:
                x1, y1, x2, y2 = x2, y2, x1, y1

            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = np.hypot(dx, dy)
            if length < 1e-6:
                continue

            ang_from_vert = np.degrees(np.arctan2(dx, dy))
            if abs(ang_from_vert) > self.cfg.MAX_ABS_DEG_FROM_VERTICAL:
                continue

            wgt = length
            sum_w += wgt

            rad = np.radians(ang_from_vert)
            sum_cos += wgt * np.cos(rad)
            sum_sin += wgt * np.sin(rad)

            if abs(dy) > 1e-6:
                t = (y_ref - y1) / dy
                if -0.25 <= t <= 1.25:
                    x_at_y = float(x1) + t * dx
                    sum_xref += wgt * x_at_y
                    sum_wxref += wgt

        if sum_w <= 0.0 or sum_wxref <= 0.0:
            return DetectionResult(found=False)

        angle_deg = -np.degrees(np.arctan2(sum_sin, sum_cos))
        x_ref = sum_xref / sum_wxref
        offset_px = float(x_ref - cx_img)
        
        if self.cfg.DEBUG_SHOW:
            dbg = ctx.frame.copy()
            
        debug = {}
        if self.cfg.DEBUG_DRAW:
            dbg = ctx.frame.copy()            
            # redline debugging visuals
            if angle_deg is not None:
                cv.putText(dbg, f"Heading error (deg): {angle_deg:+.1f}",
                        (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if offset_px is not None:
                    cv.putText(dbg, f"Offset @ y={int(0.85*h)} (px): {offset_px:+.1f}",
                            (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                cv.putText(dbg, "No red line detected",
                        (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            debug["frame"] = dbg
            debug["mask"] = mask
            debug["edges"] = edges

        if self.cfg.DEBUG_DRAW:
            cv.drawContours(dbg, cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 0), 1)
            
        cv.drawMarker(dbg, (int(round(x_ref)), y_ref), (255, 255, 255), cv.MARKER_CROSS, 10, 2)
        cv.line(dbg, (cx_img, 0), (cx_img, h - 1), (255, 255, 255), 1)
        cv.line(dbg, (0, y_ref), (w - 1, y_ref), (255, 255, 255), 1)
            

        return DetectionResult(
            found=True,
            center=(int(round(x_ref)), y_ref),
            offset_px=offset_px,
            angle_deg=angle_deg,
            debug=debug
        )
class BullseyeDetector: # not done yet, WIP
    def __init__(self, cfg):
        self.cfg = cfg
        self.red_lower1 = np.array(cfg.RED_LOWER1, dtype=np.uint8)
        self.red_upper1 = np.array(cfg.RED_UPPER1, dtype=np.uint8)
        self.red_lower2 = np.array(cfg.RED_LOWER2, dtype=np.uint8)
        self.red_upper2 = np.array(cfg.RED_UPPER2, dtype=np.uint8)
        self.blue_lower = np.array(cfg.BLUE_LOWER, dtype=np.uint8)
        self.blue_upper = np.array(cfg.BLUE_UPPER, dtype=np.uint8)
        self.kernel = np.ones((cfg.KERNEL_SIZE, cfg.KERNEL_SIZE), dtype=np.uint8)
    
    def _largest_circle(self, mask):
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_area = 0.0

        for c in contours:
            area = cv.contourArea(c)
            if area < self.cfg.MIN_CIRCLE_AREA:
                continue

            peri = cv.arcLength(c, True)
            if peri <= 1e-6:
                continue

            # circularity = 4.0 * math.pi * area / (peri * peri)
            # if circularity < self.cfg.MIN_CIRCULARITY:
            #     continue

            (x, y), r = cv.minEnclosingCircle(c)

            if area > best_area:
                best = (x, y, r, area)
                best_area = area

        return best
    
    def detect(self, ctx: FrameContext) -> DetectionResult:
        hsv = ctx.hsv
        h, w = ctx.frame.shape[:2]
        
        # combining red hsv masks 
        mask_red1 = cv.inRange(hsv, self.red_lower1, self.red_upper1)
        mask_red2 = cv.inRange(hsv, self.red_lower2, self.red_upper2)
        mask_red = cv.bitwise_or(mask_red1, mask_red2)
        
        # mask_blue = cv.inRange(hsv, self.blue_lower, self.blue_upper)
        
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_CLOSE, self.kernel, iterations=2)
        # mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, self.kernel, iterations=1)
        # mask_blue = cv.morphologyEx(mask_blue, cv.MORPH_CLOSE, self.kernel, iterations=2)
                                    
        red_circle = self._largest_circle(mask_red)
        
        if red_circle is None:
            return DetectionResult(
                found=False,
                debug={"mask": mask_red}
            )
        
        rx, ry, rr, _ = red_circle

        cx = rx
        cy = ry

        x_ref = 0.5 * w
        y_ref = h - 1

        dx = cx - x_ref
        dy = y_ref - cy
        if dy < 1.0:
            dy = 1.0

        angle_deg = math.degrees(math.atan2(dx, dy))

        pickup_ready = (
            abs(angle_deg) <= self.cfg.BULLSEYE_ANGLE_TOL_DEG
            and cy >= self.cfg.BULLSEYE_PICKUP_Y
        )

        debug = {}
        if getattr(self.cfg, "DEBUG_SHOW", False):
            dbg = ctx.frame.copy()
            cv.circle(dbg, (int(rx), int(ry)), int(rr), (0, 0, 255), 2)
            cv.circle(dbg, (int(cx), int(cy)), 4, (0, 255, 0), -1)
            cv.line(dbg, (int(x_ref), int(y_ref)), (int(cx), int(cy)), (0, 255, 255), 2)
            cv.line(dbg, (0, int(self.cfg.BULLSEYE_PICKUP_Y)), (w - 1, int(self.cfg.BULLSEYE_PICKUP_Y)), (255, 255, 0), 1)
            debug["frame"] = dbg
            debug["mask"] = mask_red

        return DetectionResult(
            found=True,
            center=(int(round(cx)), int(round(cy))),
            offset_px=dx,
            angle_deg=angle_deg,
            area=red_circle[3],
            pickup_ready=pickup_ready,
            debug=debug
        )
        
class SafezoneDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.green_lower = np.array(cfg.GREEN_LOWER, dtype=np.uint8)
        self.green_upper = np.array(cfg.GREEN_UPPER, dtype=np.uint8)
        
        k = int(cfg.MORPH_K)
        self.kernel = np.ones((k, k), dtype=np.uint8)
    
    def detect(self, ctx: FrameContext) -> DetectionResult: # copilot generated, not tested
        mask = cv.inRange(ctx.hsv, self.green_lower, self.green_upper)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, self.kernel, iterations=2)
        
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return DetectionResult(found=False)
        
        c = max(contours, key=cv.contourArea)
        area = cv.contourArea(c)
        if area < self.cfg.MIN_SAFEZONE_AREA:
            return DetectionResult(found=False)
        
        M = cv.moments(c)
        if M["m00"] == 0:
            return DetectionResult(found=False)
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return DetectionResult(
            found=True,
            center=(cx, cy),
            area=area,
            debug={"mask": mask} if self.cfg.DEBUG_SHOW else None
        )
        

            
            