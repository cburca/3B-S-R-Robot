import cv2 as cv
import numpy as np
from config import Config
import vision.vision_system 
from vision.vision_system import FrameContext, BullseyeDetector, LineDetector, DetectionResult, blue_detector

if __name__ == "__main__":
    cfg = Config()
    detector = BullseyeDetector(cfg)
    # detector = LineDetector(cfg)
    blue_scan = blue_detector(cfg)
    cap = cv.VideoCapture(0)  # default cam, Maybe make COM4

    if not cap.isOpened():
        print("can't open camera.")
        raise SystemExit

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame.")
                break
            
            ctx = FrameContext(frame)
            result = detector.detect(ctx)
            blue_found = blue_scan.detect_blue(ctx)

            print(
                f"valid={result.found}, angle={result.angle_deg}, "
                f"offset={result.offset_px}, blue_found={blue_found}"
            )

            debug = result.debug

            if "frame" in debug:
                cv.imshow("frame", debug["frame"])
            if "mask" in debug:
                cv.imshow("mask", debug["mask"])
            if "edges" in debug:
                cv.imshow("edges", debug["edges"])

            if cv.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv.destroyAllWindows()