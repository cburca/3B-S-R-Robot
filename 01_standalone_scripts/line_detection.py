import cv2 as cv
import numpy as np
import math


def wrap_deg(a):
    a = (a + 180.0) % 360.0 - 180.0
    return a


def detect_red_line():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv.CAP_PROP_FPS, 30)

    red_lower1 = np.array([0, 80, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 80, 100])
    red_upper2 = np.array([180, 255, 255])

    MIN_MASK_AREA = 400
    CANNY1, CANNY2 = 25, 175
    HOUGH_THRESH = 50
    MIN_LINE_LEN = 40
    MAX_LINE_GAP = 10

    MAX_ABS_DEG_FROM_VERTICAL = 45.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h0, w0 = frame.shape[:2]
        if w0 != 640 or h0 != 480:
            frame = cv.resize(frame, (640, 480))

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
        edges = None

        if cv.countNonZero(mask) > MIN_MASK_AREA:
            edges = cv.Canny(mask, CANNY1, CANNY2)

            lines = cv.HoughLinesP(
                edges,
                rho=1,
                theta=np.pi / 180,
                threshold=HOUGH_THRESH,
                minLineLength=MIN_LINE_LEN,
                maxLineGap=MAX_LINE_GAP
            )

            if lines is not None and len(lines) > 0:
                sum_w = 0.0
                sum_sin = 0.0
                sum_cos = 0.0

                y_ref = int(0.85 * h)
                sum_xref = 0.0
                sum_wxref = 0.0

                for (x1, y1, x2, y2) in lines[:, 0, :]:
                    if y2 < y1:
                        x1, y1, x2, y2 = x2, y2, x1, y1

                    dx = x2 - x1
                    dy = y2 - y1
                    length = math.hypot(dx, dy)
                    if length < 1e-6:
                        continue

                    ang_from_vert = math.degrees(math.atan2(dx, dy))
                    ang_from_vert = wrap_deg(ang_from_vert)

                    if abs(ang_from_vert) > MAX_ABS_DEG_FROM_VERTICAL:
                        continue

                    wgt = length
                    sum_w += wgt
                    rad = math.radians(ang_from_vert)
                    sum_cos += wgt * math.cos(rad)
                    sum_sin += wgt * math.sin(rad)

                    if abs(dy) > 1e-6:
                        t = (y_ref - y1) / dy
                        x_at_y = x1 + t * dx
                        if -0.25 <= t <= 1.25:
                            sum_xref += wgt * x_at_y
                            sum_wxref += wgt

                    cv.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if sum_w > 0:
                    angle_deg = math.degrees(math.atan2(sum_sin, sum_cos))

                if sum_wxref > 0:
                    x_ref = sum_xref / sum_wxref
                    offset_px = int(round(x_ref - cx_img))

                    cv.circle(frame, (int(round(x_ref)), y_ref), 6, (255, 255, 255), -1)
                    cv.line(frame, (cx_img, 0), (cx_img, h - 1), (255, 255, 255), 1)

                cv.line(frame, (0, y_ref), (w - 1, y_ref), (255, 255, 255), 1)

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