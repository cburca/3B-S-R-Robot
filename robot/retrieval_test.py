import time
import math
import cv2 as cv
import numpy as np

from config import Config
from vision.vision_system import (
    FrameContext,
    LineDetector,
    BullseyeDetector,
    blue_detector,
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


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


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

        # Velocity scaling from main.py
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

    # Start directly in RETRIEVAL
    sm = RobotStateMachine(state=State.RETRIEVAL)
    
    # After RETRIEVAL, go to SEARCHING (line following) as requested
    TEST_FLOW = {
        State.RETRIEVAL: State.SEARCHING,
        State.SEARCHING: State.DONE
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
    
    # Corrected Initializations
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
    
    pickup_sent = False
    pending_turn_angle = 180.0 
    last_ack = None
    halted = False
    halted_prev = False

    t0 = time.monotonic()
    t_next_outer = t0
    t_next_inner = t0

    print(f"Starting focused retrieval test: {sm.state}")

    try:
        while sm.state != State.DONE and (time.monotonic() - t0 < cfg.MAX_RUN_S):
            now = time.monotonic()

            if now >= t_next_outer:
                ret, frame = cap.read()
                ctx = FrameContext(frame) if ret else None

                line_valid = False
                line_theta_deg = None

                if ctx is not None:
                    if sm.state == State.SEARCHING:
                        line_result = run_detector(line_vision, ctx)
                        if line_result:
                            line_theta_deg = getattr(line_result, "angle_deg", None)
                            line_valid = bool(getattr(line_result, "found", False))

                    # Debug visualization
                    if cfg.DEBUG_SHOW and ctx is not None:
                        dbg_frame = frame.copy()
                        if sm.state == State.SEARCHING and line_result and line_result.debug.get("frame") is not None:
                            dbg_frame = line_result.debug["frame"]
                        
                        cv.putText(dbg_frame, f"State: {sm.state.name}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv.imshow("Retrieval Test", dbg_frame)
                        if cv.waitKey(1) & 0xFF == ord('q'):
                            sm.transition_to(State.DONE)

                # State Transitions and Control
                if sm.state == State.RETRIEVAL:
                    halted = True
                    v_cmd, yaw_cmd = 0.0, 0.0
                    if not pickup_sent:
                        print(f"Sending pickup command: T {pending_turn_angle:.2f}")
                        io.write("S\n")
                        io.write(f"T {pending_turn_angle:.2f}\n")
                        pickup_sent = True
                    
                    if last_ack:
                        if "PICK_DONE " in last_ack:
                            print("Pickup complete! Resuming line following (SEARCHING)...")
                            outer.reset() # reset PD internal state to zero (prev_err = 0)
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                            line_lost_since = None
                            halted = False
                            sm.transition_to(TEST_FLOW[sm.state])
                        elif "ERR" in last_ack:
                            print(f"Retrieval error: {last_ack}")
                            sm.transition_to(State.DONE)

                elif sm.state == State.SEARCHING:
                    halted, yaw_cmd, v_cmd, line_lost_since, _ = update_line_follow(
                        cfg, outer, line_valid, line_theta_deg, yaw_cmd, v_cmd, line_lost_since, now, v_slew_up, v_slew_down, yaw_slew
                    )
                    print(f"Halted: {halted}")
                    print(f"Line theta degree: {line_theta_deg}")
                    print(f"Line valid: {line_valid}")
                    print(f"Yaw command: {yaw_cmd}")
                    print(f"Velocity command: {v_cmd}")
                    if halted:
                        print("Reached end of line or line lost.")
                        sm.transition_to(State.DONE)

                t_next_outer += cfg.DT_OUTER

            if now >= t_next_inner:
                # Motor control ONLY when not doing an autonomous Arduino maneuver
                if sm.state != State.RETRIEVAL:
                    if halted:
                        if not halted_prev: io.write("S\n")
                        send_vel(io, 0.0, 0.0)
                    else:
                        w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)
                        
                        # Apply angular velocity limit from main.py
                        w_peak = max(abs(w_l), abs(w_r))
                        if w_peak > w_lim and w_peak > 1e-6:
                            s = w_lim / w_peak
                            w_l *= s
                            w_r *= s
                        
                        l_cps = w_l * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        r_cps = w_r * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        send_vel(io, l_cps, r_cps)
                    halted_prev = halted
                
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
