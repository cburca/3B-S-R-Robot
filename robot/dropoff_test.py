import time
import math
import cv2 as cv
import numpy as np

from config import Config
from vision.vision_system import (
    FrameContext,
    LineDetector,
    SafezoneDetector,
)
from control.outer_heading_pd import HeadingPD
from control.mixer import DiffDriveMixer
from control.state import RobotStateMachine, State
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
    if hasattr(io, "write"):
        io.write(f"V {l_tps:.3f} {r_tps:.3f}\n")


def hard_stop(io):
    try:
        send_vel(io, 0.0, 0.0)
    except Exception:
        pass
    if hasattr(io, "write"):
        try:
            io.write("S\n")
        except Exception:
            pass


def run_detector(detector, ctx):
    if hasattr(detector, "detect"):
        return detector.detect(ctx)
    return None


def update_line_follow(
    cfg,
    outer,
    line_valid,
    line_theta_deg,
    yaw_cmd,
    v_cmd,
    line_lost_since,
    now,
    v_slew_up,
    v_slew_down,
    yaw_slew,
):
    if line_valid and line_theta_deg is not None:
        line_lost_since = None
        halted = False

        theta_rad = math.radians(line_theta_deg)
        yaw_target = outer.step(0.0, theta_rad)

        if yaw_slew > 0.0:
            yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
        else:
            yaw_cmd = yaw_target

        # Velocity scaling
        speed_scale = 1.0 / (1.0 + getattr(cfg, "KV", 0.0) * abs(yaw_cmd))
        v_target = cfg.vmax * speed_scale
        v_target = clamp(v_target, getattr(cfg, "V_MIN", 0.01), cfg.vmax)
        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)

        return halted, yaw_cmd, v_cmd, line_lost_since, 1

    if line_lost_since is None:
        line_lost_since = now

    if now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
        return True, 0.0, 0.0, line_lost_since, 0

    return False, yaw_cmd, v_cmd, line_lost_since, 0


def main():
    cfg = Config()

    # RPi 5 serial port fix
    if (
        isinstance(cfg.SERIAL_PORT, str)
        and cfg.SERIAL_PORT
        and not cfg.SERIAL_PORT.startswith("/")
        and "tty" in cfg.SERIAL_PORT
    ):
        cfg.SERIAL_PORT = "/" + cfg.SERIAL_PORT

    # Headless check
    import os
    has_display = "DISPLAY" in os.environ or (os.name == 'nt')
    if not has_display:
        cfg.DEBUG_SHOW = False

    # Start directly in FIND_SAFETY
    sm = RobotStateMachine(state=State.FIND_SAFETY)
    
    # Test flow: FIND_SAFETY -> DEPOSITION -> RETURN -> DONE
    TEST_FLOW = {
        State.FIND_SAFETY: State.DEPOSITION,
        State.DEPOSITION: State.RETURN,
        State.RETURN: State.DONE
    }

    v_slew_up = float(cfg.V_SLEW_UP)
    v_slew_down = float(cfg.V_SLEW_DOWN)
    yaw_slew = float(cfg.YAW_SLEW)
    w_lim = float(getattr(cfg, "WHEEL_OMEGA_LIMIT", 10.0))

    cap = cv.VideoCapture(cfg.CAM_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
    cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

    io = USBSerial(cfg.SERIAL_PORT, baudrate=cfg.BAUD, timeout=cfg.SERIAL_TIMEOUT)
    try:
        io.connect()
    except Exception as e:
        print(f"Serial connection failed: {e}")
        return

    time.sleep(2.0)
    if cfg.SERIAL_HANDSHAKE:
        io.write("SIX\n")
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < 5.0:
            line = io.read()
            if line and "SEVEN" in line:
                print("Handshake OK")
                break
            time.sleep(0.1)

    line_vision = LineDetector(cfg)
    safezone_vision = SafezoneDetector(cfg)
    
    outer = HeadingPD(
        kp=cfg.KP_THETA,
        kd=cfg.KD_THETA,
        dt=cfg.DT_OUTER,
        u_limit=cfg.U_YAW_LIMIT
    )
    mixer = DiffDriveMixer(r=cfg.r, L=cfg.L)

    v_cmd = 0.0
    yaw_cmd = 0.0
    line_lost_since = None
    
    dropoff_sent = False
    last_ack = None
    halted = False
    halted_prev = False

    t0 = time.monotonic()
    t_next_outer = t0
    t_next_inner = t0

    print(f"Starting focused drop-off test: {sm.state}")

    try:
        while sm.state != State.DONE and (time.monotonic() - t0 < cfg.MAX_RUN_S):
            now = time.monotonic()

            if now >= t_next_outer:
                ret, frame = cap.read()
                ctx = FrameContext(frame) if ret else None

                line_valid = False
                line_theta_deg = None
                safety_found = False

                if ctx is not None:
                    # Line following is active in FIND_SAFETY and RETURN
                    if sm.state in {State.FIND_SAFETY, State.RETURN}:
                        line_result = run_detector(line_vision, ctx)
                        if line_result:
                            line_theta_deg = getattr(line_result, "angle_deg", None)
                            line_valid = bool(getattr(line_result, "found", False))

                    # Safezone detection is active in FIND_SAFETY
                    if sm.state == State.FIND_SAFETY:
                        safety_result = run_detector(safezone_vision, ctx)
                        safety_found = bool(getattr(safety_result, "found", False))

                    # Debug visualization
                    if cfg.DEBUG_SHOW:
                        dbg_frame = frame.copy()
                        # Overlay detector debug if available
                        if sm.state == State.FIND_SAFETY and safety_found:
                             if safety_result.debug.get("frame") is not None:
                                 dbg_frame = safety_result.debug["frame"]
                        elif sm.state in {State.FIND_SAFETY, State.RETURN}:
                            if line_result and line_result.debug.get("frame") is not None:
                                dbg_frame = line_result.debug["frame"]
                        
                        cv.putText(dbg_frame, f"State: {sm.state.name}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv.imshow("Drop-off Test", dbg_frame)
                        if cv.waitKey(1) & 0xFF == ord('q'):
                            sm.transition_to(State.DONE)

                # State Transitions and Control
                if sm.state == State.FIND_SAFETY:
                    if safety_found:
                        print("Safezone detected! Transitioning to DEPOSITION.")
                        halted = True
                        v_cmd, yaw_cmd = 0.0, 0.0
                        sm.transition_to(TEST_FLOW[sm.state])
                    else:
                        halted, yaw_cmd, v_cmd, line_lost_since, _ = update_line_follow(
                            cfg, outer, line_valid, line_theta_deg, yaw_cmd, v_cmd, line_lost_since, now, v_slew_up, v_slew_down, yaw_slew
                        )

                elif sm.state == State.DEPOSITION:
                    halted = True
                    v_cmd, yaw_cmd = 0.0, 0.0
                    if not dropoff_sent:
                        print("Sending drop-off command: D")
                        io.write("S\n")
                        io.write("D\n")
                        dropoff_sent = True
                    
                    if last_ack:
                        if "DROP_DONE" in last_ack:
                            print("Drop-off complete! Resuming line following (RETURN)...")
                            sm.transition_to(TEST_FLOW[sm.state])
                        elif "ERR" in last_ack:
                            print(f"Drop-off error: {last_ack}")
                            sm.transition_to(State.DONE)

                elif sm.state == State.RETURN:
                    halted, yaw_cmd, v_cmd, line_lost_since, _ = update_line_follow(
                        cfg, outer, line_valid, line_theta_deg, yaw_cmd, v_cmd, line_lost_since, now, v_slew_up, v_slew_down, yaw_slew
                    )
                    if halted:
                        print("Reached end of line. Stopping.")
                        sm.transition_to(State.DONE)

                t_next_outer += cfg.DT_OUTER

            if now >= t_next_inner:
                # Motor control ONLY when not doing an autonomous Arduino maneuver
                if sm.state != State.DEPOSITION:
                    if halted:
                        if not halted_prev: io.write("S\n")
                        send_vel(io, 0.0, 0.0)
                    else:
                        w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)
                        
                        w_peak = max(abs(w_l), abs(w_r))
                        if w_peak > w_lim and w_peak > 1e-6:
                            s = w_lim / w_peak
                            w_l *= s
                            w_r *= s
                        
                        l_cps = w_l * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        r_cps = w_r * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        send_vel(io, l_cps, r_cps)
                    halted_prev = halted
                else:
                    halted_prev = True
                
                ack = io.read()
                if ack: last_ack = ack.strip()
                
                t_next_inner += cfg.DT_INNER
            
            time.sleep(0.001)

    finally:
        hard_stop(io)
        io.close()
        cap.release()
        cv.destroyAllWindows()
        print("Test finished.")

if __name__ == "__main__":
    main()
