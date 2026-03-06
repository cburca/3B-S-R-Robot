import os
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
    raise AttributeError("Serial interface has no supported velocity command method")


def hard_stop(io):
    try:
        send_vel(io, 0.0, 0.0)
    except Exception:
        pass

    if hasattr(io, "write"):
        try:
            io.write("STOP\n")
        except Exception:
            pass


def wait_for_port(port, timeout=15.0, poll_dt=0.25):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if os.path.exists(port):
            return True
        time.sleep(poll_dt)
    return False


def open_serial_with_retry(cfg):
    last_err = None

    for _ in range(cfg.SERIAL_CONNECT_RETRIES):
        try:
            if not wait_for_port(cfg.SERIAL_PORT, timeout=cfg.SERIAL_WAIT_TIMEOUT):
                raise RuntimeError(f"Serial port not found: {cfg.SERIAL_PORT}")

            io = USBSerial(
                port=cfg.SERIAL_PORT,
                baud=cfg.BAUD,
                timeout=cfg.SERIAL_TIMEOUT,
                handshake=cfg.SERIAL_HANDSHAKE,
            )

            time.sleep(cfg.ARDUINO_RESET_DELAY)
            hard_stop(io)
            return io

        except Exception as e:
            last_err = e
            time.sleep(cfg.SERIAL_RETRY_DELAY)

    raise RuntimeError(f"Failed to open serial after retries: {last_err}")


def open_camera_with_retry(cfg):
    last_err = None

    for _ in range(cfg.CAM_CONNECT_RETRIES):
        cap = cv.VideoCapture(cfg.CAM_INDEX)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
        cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                return cap

        cap.release()
        last_err = RuntimeError("Camera failed to open or return a frame")
        time.sleep(cfg.CAM_RETRY_DELAY)

    raise RuntimeError(f"Failed to open camera after retries: {last_err}")


def run_once(cfg):
    cap = None
    io = None

    try:
        io = open_serial_with_retry(cfg)
        cap = open_camera_with_retry(cfg)

        detector = RedLineDetector(cfg)
        heading = HeadingPD(cfg.KP_HEADING, cfg.KD_HEADING, dt=1.0 / cfg.LOOP_HZ)
        mixer = DiffDriveMixer(cfg.WHEEL_RADIUS_M, cfg.TRACK_WIDTH_M)

        period = 1.0 / cfg.LOOP_HZ
        t_start = time.time()
        t_prev = t_start

        while True:
            now = time.time()

            if cfg.RUN_TIME_S > 0.0 and (now - t_start) >= cfg.RUN_TIME_S:
                hard_stop(io)
                break

            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera read failed")

            angle_deg, offset_px, valid, _ = detector.process(frame)

            if valid:
                yaw_cmd = heading.update(angle_deg)
                yaw_cmd = clamp(yaw_cmd, -cfg.YAW_RATE_MAX, cfg.YAW_RATE_MAX)

                v_cmd = cfg.V_MAX * (1.0 - cfg.KV * abs(yaw_cmd))
                v_cmd = clamp(v_cmd, cfg.V_MIN, cfg.V_MAX)
            else:
                yaw_cmd = 0.0
                v_cmd = 0.0

            w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)

            l_tps = omega_to_ticks_per_sec(w_l, cfg.ENC_CPR)
            r_tps = omega_to_ticks_per_sec(w_r, cfg.ENC_CPR)

            l_tps = clamp(l_tps, -cfg.MAX_TPS, cfg.MAX_TPS)
            r_tps = clamp(r_tps, -cfg.MAX_TPS, cfg.MAX_TPS)

            send_vel(io, l_tps, r_tps)

            if cfg.SHOW_DEBUG:
                cv.imshow("frame", frame)
                if cv.waitKey(1) & 0xFF == 27:
                    hard_stop(io)
                    break

            elapsed = time.time() - now
            sleep_t = period - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)

            t_prev = now

    finally:
        if io is not None:
            hard_stop(io)
            time.sleep(0.05)
            hard_stop(io)

            if hasattr(io, "close"):
                try:
                    io.close()
                except Exception:
                    pass

        if cap is not None:
            cap.release()

        cv.destroyAllWindows()


def main():
    cfg = Config()

    while True:
        try:
            run_once(cfg)
            break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[main] restart after error: {e}")
            time.sleep(cfg.MAIN_RETRY_DELAY)


if __name__ == "__main__":
    main()