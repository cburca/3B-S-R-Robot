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


def cfg_get(cfg, *names, default=None):
    for name in names:
        if hasattr(cfg, name):
            return getattr(cfg, name)
    return default


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
    raise AttributeError("No supported velocity send method on serial interface")


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


def make_serial(cfg):
    port = cfg_get(cfg, "SERIAL_PORT", default="/dev/ttyACM0")
    baud = cfg_get(cfg, "BAUD", "BAUD_RATE", default=115200)
    timeout = cfg_get(cfg, "SERIAL_TIMEOUT", default=0.1)
    handshake = cfg_get(cfg, "SERIAL_HANDSHAKE", default=True)

    try:
        return USBSerial(port=port, baud=baud, timeout=timeout, handshake=handshake)
    except TypeError:
        pass

    try:
        return USBSerial(port, baud, timeout, handshake)
    except TypeError:
        pass

    try:
        return USBSerial(port, baud, handshake=handshake)
    except TypeError:
        pass

    return USBSerial(port, baud)


def open_serial_with_retry(cfg):
    port = cfg_get(cfg, "SERIAL_PORT", default="/dev/ttyACM0")
    wait_timeout = cfg_get(cfg, "SERIAL_WAIT_TIMEOUT", default=15.0)
    connect_retries = int(cfg_get(cfg, "SERIAL_CONNECT_RETRIES", default=10))
    retry_delay = cfg_get(cfg, "SERIAL_RETRY_DELAY", default=1.0)
    reset_delay = cfg_get(cfg, "ARDUINO_RESET_DELAY", default=2.0)

    last_err = None

    for _ in range(connect_retries):
        try:
            if not wait_for_port(port, timeout=wait_timeout):
                raise RuntimeError(f"Serial port not found: {port}")

            io = make_serial(cfg)
            time.sleep(reset_delay)
            hard_stop(io)
            return io

        except Exception as e:
            last_err = e
            time.sleep(retry_delay)

    raise RuntimeError(f"Failed to open serial: {last_err}")


def open_camera_with_retry(cfg):
    cam_index = cfg_get(cfg, "CAM_INDEX", default=0)
    cam_w = cfg_get(cfg, "CAM_W", default=640)
    cam_h = cfg_get(cfg, "CAM_H", default=480)
    cam_fps = cfg_get(cfg, "CAM_FPS", default=30.0)
    retries = int(cfg_get(cfg, "CAM_CONNECT_RETRIES", default=10))
    retry_delay = cfg_get(cfg, "CAM_RETRY_DELAY", default=1.0)

    last_err = None

    for _ in range(retries):
        cap = cv.VideoCapture(cam_index)
        cap.set(cv.CAP_PROP_FRAME_WIDTH, cam_w)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, cam_h)
        cap.set(cv.CAP_PROP_FPS, cam_fps)

        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                return cap

        cap.release()
        last_err = RuntimeError("Camera failed to open or return frames")
        time.sleep(retry_delay)

    raise RuntimeError(f"Failed to open camera: {last_err}")


def make_heading_controller(cfg):
    try:
        return HeadingPD(cfg)
    except TypeError:
        pass

    kp = cfg_get(cfg, "KP_HEADING", default=0.0)
    kd = cfg_get(cfg, "KD_HEADING", default=0.0)
    dt = 1.0 / cfg_get(cfg, "LOOP_HZ", default=20.0)

    try:
        return HeadingPD(kp, kd, dt=dt)
    except TypeError:
        pass

    return HeadingPD(kp, kd)


def detector_run(detector, frame):
    for name in ("process", "detect", "run", "update"):
        if hasattr(detector, name):
            return getattr(detector, name)(frame)
    raise AttributeError("RedLineDetector has no supported frame-processing method")


def heading_update(ctrl, angle_deg):
    for name in ("update", "step", "compute"):
        if hasattr(ctrl, name):
            return getattr(ctrl, name)(angle_deg)
    raise AttributeError("HeadingPD has no supported update method")


def run_once(cfg):
    io = None
    cap = None

    try:
        io = open_serial_with_retry(cfg)
        cap = open_camera_with_retry(cfg)

        detector = RedLineDetector(cfg)

        heading = make_heading_controller(cfg)

        wheel_radius = cfg_get(cfg, "WHEEL_RADIUS_M", "WHEEL_RADIUS", default=0.03)
        track_width = cfg_get(cfg, "TRACK_WIDTH_M", "TRACK_WIDTH", "WHEEL_BASE_M", default=0.14)
        mixer = DiffDriveMixer(wheel_radius, track_width)

        loop_hz = cfg_get(cfg, "LOOP_HZ", default=20.0)
        period = 1.0 / loop_hz

        run_time_s = cfg_get(cfg, "RUN_TIME_S", default=0.0)
        yaw_rate_max = cfg_get(cfg, "YAW_RATE_MAX", default=2.0)
        v_max = cfg_get(cfg, "V_MAX", "vmax", default=0.25)
        v_min = cfg_get(cfg, "V_MIN", default=0.0)
        kv = cfg_get(cfg, "KV", default=0.25)
        enc_cpr = cfg_get(cfg, "ENC_CPR", "CPR", default=2797)
        max_tps = cfg_get(cfg, "MAX_TPS", default=2000.0)
        show_debug = bool(cfg_get(cfg, "SHOW_DEBUG", default=True))

        t_start = time.time()

        while True:
            t_loop = time.time()

            if run_time_s > 0.0 and (t_loop - t_start) >= run_time_s:
                hard_stop(io)
                break

            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera read failed")

            result = detector_run(detector, frame)

            if len(result) >= 4:
                angle_deg, offset_px, valid, debug = result[:4]
            elif len(result) == 3:
                angle_deg, offset_px, valid = result
                debug = {}
            else:
                raise RuntimeError("Unexpected detector return format")

            if valid:
                yaw_cmd = heading_update(heading, angle_deg)
                yaw_cmd = clamp(yaw_cmd, -yaw_rate_max, yaw_rate_max)

                v_cmd = v_max * (1.0 - kv * abs(yaw_cmd))
                v_cmd = clamp(v_cmd, v_min, v_max)
            else:
                yaw_cmd = 0.0
                v_cmd = 0.0

            w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)

            l_tps = omega_to_ticks_per_sec(w_l, enc_cpr)
            r_tps = omega_to_ticks_per_sec(w_r, enc_cpr)

            l_tps = clamp(l_tps, -max_tps, max_tps)
            r_tps = clamp(r_tps, -max_tps, max_tps)

            send_vel(io, l_tps, r_tps)

            if show_debug:
                cv.imshow("frame", frame)
                if cv.waitKey(1) & 0xFF == 27:
                    hard_stop(io)
                    break

            dt = time.time() - t_loop
            sleep_t = period - dt
            if sleep_t > 0.0:
                time.sleep(sleep_t)

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
    retry_delay = cfg_get(cfg, "MAIN_RETRY_DELAY", default=2.0)

    while True:
        try:
            run_once(cfg)
            break
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[main] restart after error: {e}")
            time.sleep(retry_delay)


if __name__ == "__main__":
    main()