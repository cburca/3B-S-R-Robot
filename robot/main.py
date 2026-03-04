import time
import math
import cv2 as cv

from config import Config
from vision.red_line import RedLineDetector
from control.outer_heading_pd import HeadingPD
from control.mixer import DiffDriveMixer
from hardware.usb_serial import USBSerial


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def omega_to_ticks_per_sec(omega_rad_s, cpr):
    return omega_rad_s * (cpr / (2.0 * math.pi))


def send_vel(io, l_tps, r_tps):
    if hasattr(io, "set_vel"):
        io.set_vel(l_tps, r_tps)
        return
    if hasattr(io, "set_velocity"):
        io.set_velocity(l_tps, r_tps)
        return
    if hasattr(io, "write"):
        io.write(f"V {l_tps:.3f} {r_tps:.3f}\n")
        return
    raise AttributeError("USBSerial has no set_vel/set_velocity/write method")


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
    io.connect()

    # inital states
    yaw_cmd = 0.0         # yaw rate command (rad/s)
    v_cmd = 0.0           # m/s
    theta_ref_deg = 0.0   # follow vertical

    # timing
    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()

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
                        yaw_cmd = outer.step(0.0, -theta_err_rad)

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

                while t_next_outer <= now:
                    t_next_outer += cfg.DT_OUTER

            # inner
            if now >= t_next_inner:
                w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)
                l_cps = omega_to_ticks_per_sec(w_l, cfg.ENCODER_CPR)
                r_cps = omega_to_ticks_per_sec(w_r, cfg.ENCODER_CPR)
                send_vel(io, l_cps, r_cps)

                while t_next_inner <= now:
                    t_next_inner += cfg.DT_INNER

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