import time
import math
import cv2 as cv

from config import Config
from vision.red_line import RedLineDetector
from control.outer_heading_pd import HeadingPD
from control.wheel_pi import WheelPI # maybe not needed...
from control.mixer import DiffDriveMixer
from hardware.arduino_i2c import ArduinoIO
from hardware.encoder_speed import EncoderSpeedEstimator

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def main():
    cfg = Config()

    # --- Camera ---
    cap = cv.VideoCapture(cfg.CAM_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
    cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

    # --- Modules ---
    vision = RedLineDetector(cfg)
    mixer = DiffDriveMixer(cfg.r, cfg.L)

    # Outer PD works in radians, so we’ll convert theta_deg->rad before stepping
    outer = HeadingPD(cfg.KP_THETA, cfg.KD_THETA, dt=cfg.DT_OUTER, u_limit=cfg.U_YAW_LIMIT)

    pi_L = WheelPI(cfg.KP_W, cfg.KI_W, dt=cfg.DT_INNER, u_limit=cfg.U_PWM_LIMIT)
    pi_R = WheelPI(cfg.KP_W, cfg.KI_W, dt=cfg.DT_INNER, u_limit=cfg.U_PWM_LIMIT)

    io = ArduinoIO(cfg.I2C_BUS, cfg.ARDUINO_ADDR, reg_pwm=cfg.REG_PWM, reg_enc=cfg.REG_ENC)
    speed_est = EncoderSpeedEstimator(cfg.ENCODER_CPR, dt=cfg.DT_INNER, alpha=0.35)

    # --- State shared between loops ---
    yaw_cmd = 0.0         # normalized “uθ” in [-1,1]
    v_cmd = 0.0           # m/s
    theta_ref_deg = 0.0   # follow vertical

    # timing
    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()

            # ---------- OUTER LOOP (camera rate ~30 Hz) ----------
            if now >= t_next_outer:
                ret, frame = cap.read()
                if ret:
                    theta_deg, offset_px, valid, dbg = vision.process(frame)

                    if valid:
                        # Convert deg -> rad for PD consistency with report derivation :contentReference[oaicite:9]{index=9}
                        theta_err_rad = math.radians(theta_ref_deg - theta_deg)

                        # outer PD expects "measurement", so feed equivalent:
                        # step(theta_ref, theta_meas) -> err = ref - meas
                        # We can just set theta_meas_rad = -theta_err_rad with ref=0, or implement directly:
                        # easiest: treat theta_deg as measured and do conversion inside:
                        yaw_cmd = outer.step(0.0, -theta_err_rad)  # err = 0 - (-err) = err

                        # speed schedule from report idea: v = vmax (1 - kv |yaw_cmd|)
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

            # ---------- INNER LOOP (fast, encoder-based PI) ----------
            if now >= t_next_inner:
                enc = io.read_encoders()
                wL_meas, wR_meas = speed_est.update(enc)

                # Convert normalized yaw_cmd uθ -> yaw-rate command ωcmd (rad/s)
                # If you assume yaw_cmd scales to achievable yaw rate via kinematics,
                # simplest consistent mapping is: uθ * wmax -> differential wheel component, but
                # your report uses uθ into θdot = k uθ (Eq. 4), i.e. uθ behaves like yaw-rate actuation. :contentReference[oaicite:10]{index=10}
                #
                # For implementation: treat yaw_cmd as ωcmd directly after scaling:
                omega_cmd = yaw_cmd * (cfg.r * cfg.wmax / cfg.L) * 2.0  # conservative scaling; tune
                # You can also just set omega_cmd = yaw_cmd and tune gains; keep sign consistent.

                wL_ref, wR_ref = mixer.wheel_speed_setpoints(v_cmd, omega_cmd)

                uL = pi_L.step(wL_ref, wL_meas)   # normalized [-1,1]
                uR = pi_R.step(wR_ref, wR_meas)

                # Send to Arduino as int16 scaled
                io.set_pwm(int(uL * cfg.CMD_SCALE), int(uR * cfg.CMD_SCALE))

                t_next_inner += cfg.DT_INNER

            # small sleep to avoid 100% CPU
            time.sleep(0.0005)

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