import cv2 as cv
import numpy as np
from config import Config
from vision.red_line import RedLineDetector

if __name__ == "__main__":
    cfg = Config()
    detector = RedLineDetector(cfg)
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

            angle_deg, offset_px, valid, debug = detector.process(frame)

            print(f"valid={valid}, angle={angle_deg}, offset={offset_px}")

            if "frame" in debug:
                cv.imshow("frame", debug["frame"])
            if "mask" in debug:
                cv.imshow("mask", debug["mask"])
            if debug.get("edges") is not None:
                cv.imshow("edges", debug["edges"])

            if cv.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cap.release()
        cv.destroyAllWindows()