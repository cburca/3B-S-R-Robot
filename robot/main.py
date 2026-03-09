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


def slew(prev, target, rate_up, rate_down, dt):
    dv = target - prev
    lim = (rate_up if dv > 0.0 else rate_down) * dt
    if dv > lim:
        return prev + lim
    if dv < -lim:
        return prev - lim
    return target


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

    port = getattr(cfg, "SERIAL_PORT", "")
    if isinstance(port, str) and port and not port.startswith("/") and "tty" in port:
        cfg.SERIAL_PORT = "/" + port

    v_slew_up = float(getattr(cfg, "V_SLEW_UP", 0.25))
    v_slew_down = float(getattr(cfg, "V_SLEW_DOWN", 0.75))
    yaw_slew = float(getattr(cfg, "YAW_SLEW", 0.0))
    w_lim = float(getattr(cfg, "WHEEL_OMEGA_LIMIT", getattr(cfg, "wmax", cfg.vmax / cfg.r)))

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

    io = USBSerial(cfg.SERIAL_PORT, baudrate=cfg.BAUD_RATE, timeout=0.10)
    io.connect()

    time.sleep(10.0)  # Arduino resets on port open

    try:
        io.ser.reset_input_buffer()
        io.ser.reset_output_buffer()
    except Exception:
        pass

    io.write("SIX\n")
    t0 = time.time()
    ok = False
    while time.time() - t0 < 5.0:
        r = io.read()
        if r == "SEVEN":
            ok = True
            break
    if not ok:
        raise RuntimeError("Handshake failed")

    io.write("E 1\n")
    io.read()  # optional: consume "OK E ..."

    # inital states
    halted = False
    halted_prev = False
    
    yaw_cmd = 0.0         # yaw rate command (rad/s)
    v_cmd = 0.0           # m/s
    theta_ref_deg = 0.0   # follow vertical

    # timing
    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()
    
    t_start = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            if now - t_start >= cfg.MAX_RUN_S:
                break
            
            # outer
            if now >= t_next_outer:
                ret, frame = cap.read()
                if not ret:
                    valid = False
                    theta_deg = None
                    offset_px = None
                    dbg = None
                else:
                    theta_deg, offset_px, valid, dbg = vision.process(frame)

                    if not valid:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0
                
                    else:
                        halted = False

                        theta_err_rad = math.radians(theta_ref_deg - theta_deg)
                        yaw_target = outer.step(0.0, -theta_err_rad)

                        if yaw_slew > 0.0:
                            yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
                        else:
                            yaw_cmd = yaw_target

                        speed_scale = 1.0 / (1.0 + cfg.KV * abs(yaw_cmd))
                        v_target = cfg.vmax * speed_scale
                        v_target = clamp(v_target, cfg.V_MIN, cfg.vmax)
                        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)

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
                if halted:
                    if not halted_prev:
                        io.write("S\n")
                    send_vel(io, 0.0, 0.0)
                else:
                    w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)

                    w_peak = max(abs(w_l), abs(w_r))
                    if w_peak > w_lim and w_peak > 1e-6:
                        s = w_lim / w_peak
                        w_l *= s
                        w_r *= s

                    l_cps = w_l
                    r_cps = w_r
                    send_vel(io, l_cps, r_cps)

                halted_prev = halted

                ack = io.read()
                if ack and ack.startswith("ERR"):
                    print("ARDUINO:", ack)

                while t_next_inner <= now:
                    t_next_inner += cfg.DT_INNER

    finally:
        try:
            io.write("S\n")
            time.sleep(0.2)
            io.close()
        except Exception:
            pass
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    main()