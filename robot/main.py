import time
import math
import cv2 as cv

from config import Config
from vision.red_line import RedLineDetector
from control.outer_heading_pd import HeadingPD
from control.mixer import DiffDriveMixer
from hardware.usb_serial import USBSerial
from hardware.encoder_speed import EncoderSpeedEstimator

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
    cfg = Config()

    # camera
    cap = cv.VideoCapture(cfg.CAM_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
    cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

    # modules
    vision = RedLineDetector(cfg)
    mixer = DiffDriveMixer(cfg.r, cfg.L)

    # Outer PD works in radians, so we’ll convert theta_deg->rad before stepping
    outer = HeadingPD(cfg.KP_THETA, cfg.KD_THETA, dt=cfg.DT_OUTER, u_limit=cfg.U_YAW_LIMIT)


    io = USBSerial(cfg.SERIAL_PORT, baudrate=cfg.BAUD_RATE)
    io.connect(handshake=True)
    # speed_est = EncoderSpeedEstimator(cfg.ENCODER_CPR, dt=cfg.DT_INNER, alpha=0.35)

    # inital states
    yaw_cmd = 0.0         # normalized “uθ” in [-1,1]
    v_cmd = 0.0           # m/s
    theta_ref_deg = 0.0   # follow vertical

    # timing
    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()

    # All of this below might be defunct

    try:
        while True:
            now = time.perf_counter()

            # outer
            if now >= t_next_outer:
                ret, frame = cap.read()
                if ret:
                    theta_deg, offset_px, valid, dbg = vision.process(frame)

                    if valid:
                        theta_err_rad = math.radians(theta_ref_deg - theta_deg)

                        yaw_cmd = outer.step(0.0, -theta_err_rad)  # err = 0 - (-err) = err

                        v_cmd = cfg.vmax * (1.0 - cfg.KV * abs(yaw_cmd))
                        v_cmd = clamp(v_cmd, cfg.V_MIN, cfg.vmax)
                    else:
                        yaw_cmd = 0.0
                        v_cmd = 0.0

                    # debug display (optional)
                    if cfg.DEBUG_SHOW and dbg:
                        cv.imshow("mask", dbg["mask"])
                        if dbg["edges"] is not None:
                            cv.imshow("edges", dbg["edges"])
                        cv.imshow("vision", dbg["frame"])
                        if cv.waitKey(1) & 0xFF == ord('q'):
                            break

                t_next_outer += cfg.DT_OUTER

    finally:
        try:
            io.set_pwm(0, 0)
            io.close()
        except Exception:
            pass
        cap.release()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()