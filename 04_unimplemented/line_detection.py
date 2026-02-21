import cv2 as cv
import numpy as np
import math

def wrap_deg(a):
    """Wrap angle to (-180, 180)."""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def detect_red_line():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 30)

    # Red in HSV (two ranges because red wraps hue)
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    # Tuning knobs
    MIN_MASK_AREA = 800          # ignore tiny red blobs
    CANNY1, CANNY2 = 50, 150     # edge detection thresholds
    HOUGH_THRESH = 30            # votes needed
    MIN_LINE_LEN = 30            # pixels
    MAX_LINE_GAP = 10            # pixels

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv.resize(frame, (480, 480))
        h, w = frame.shape[:2]
        cx_img = w // 2

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(hsv, red_lower1, red_upper1)
        mask2 = cv.inRange(hsv, red_lower2, red_upper2)
        mask = cv.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)

        angle_deg = None
        offset_px = None

        # Quick area gate so we don't run Hough on garbage
        if cv.countNonZero(mask) > MIN_MASK_AREA:
            # Edge detection on the mask (better than on raw frame for this use-case)
            edges = cv.Canny(mask, CANNY1, CANNY2)

            # Probabilistic Hough -> line segments
            lines = cv.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi / 180,
                threshold=HOUGH_THRESH,
                minLineLength=MIN_LINE_LEN,
                maxLineGap=MAX_LINE_GAP
            )

            if lines is not None and len(lines) > 0:
                # Compute a length-weighted average direction
                # We'll express "heading error" as:
                #   0 deg = vertical line (straight ahead)
                #   + = line tilts to the right, - = tilts to the left
                sum_w = 0.0
                sum_sin = 0.0
                sum_cos = 0.0

                # Also estimate lateral offset by averaging x-intercept at a chosen y (near bottom)
                y_ref = int(0.85 * h)
                sum_xref = 0.0
                sum_wxref = 0.0

                MAX_ABS_DEG_FROM_VERTICAL = 90.0  # keep only lines within ±45° of vertical

                for (x1, y1, x2, y2) in lines[:, 0, :]:
                    dx = x2 - x1
                    dy = y2 - y1
                    length = math.hypot(dx, dy)
                    if length < 1e-6:
                        continue

                    # Angle relative to vertical (0 = vertical). Range [-180,180)
                    ang_from_x = math.degrees(math.atan2(dy, dx))
                    ang_from_vert = wrap_deg(ang_from_x - 90.0)

                    # ---- FILTER: reject anything beyond ±45° from vertical (i.e., too horizontal/diagonal)
                    if abs(ang_from_vert) > MAX_ABS_DEG_FROM_VERTICAL:
                        continue

                    # Weight by segment length
                    wgt = length
                    sum_w += wgt

                    rad = math.radians(ang_from_vert)
                    sum_cos += wgt * math.cos(rad)
                    sum_sin += wgt * math.sin(rad)

                    # Offset calc at y_ref (same as before)
                    if abs(dy) > 1e-6:
                        t = (y_ref - y1) / dy
                        x_at_y = x1 + t * dx
                        if -0.25 <= t <= 1.25:
                            sum_xref += wgt * x_at_y
                            sum_wxref += wgt

                    # Debug draw ONLY accepted segments
                    cv.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if sum_w > 0:
                    angle_deg = math.degrees(math.atan2(sum_sin, sum_cos))

                if sum_wxref > 0:
                    x_ref = sum_xref / sum_wxref
                    offset_px = int(round(x_ref - cx_img))

                    # draw reference point + centerline
                    cv.circle(frame, (int(round(x_ref)), y_ref), 6, (255, 255, 255), -1)
                    cv.line(frame, (cx_img, 0), (cx_img, h - 1), (255, 255, 255), 1)

                # Show y_ref line
                cv.line(frame, (0, y_ref), (w - 1, y_ref), (255, 255, 255), 1)

            else:
                edges = None
        else:
            edges = None

        # HUD text
        if angle_deg is not None:
            cv.putText(frame, f"Heading error (deg): {angle_deg:+.1f}",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if offset_px is not None:
                cv.putText(frame, f"Offset @ y={int(0.85*h)} (px): {offset_px:+d}",
                           (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            cv.putText(frame, "No red line detected",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv.imshow("mask", mask)
        if edges is not None:
            cv.imshow("edges", edges)
        cv.imshow("Red Line Heading (Hough)", frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    detect_red_line()