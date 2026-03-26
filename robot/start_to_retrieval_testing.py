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


def run_blue_detector(detector, ctx):
    if hasattr(detector, "detect_blue"):
        return detector.detect_blue(ctx)
    return False


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

        v_target = cfg.vmax
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

    # Check if we have a display (important for headless RPi)
    import os
    has_display = "DISPLAY" in os.environ or (os.name == 'nt')
    if not has_display:
        print("No display detected, disabling cv.imshow")
        cfg.DEBUG_SHOW = False

    # Force starting state for this test script
    sm = RobotStateMachine(state=State.SEARCHING)
    
    # Override nominal flow for this test
    TEST_FLOW = {
        State.SEARCHING: State.BULLSEYE_LINEUP,
        State.BULLSEYE_LINEUP: State.RETRIEVAL,
        State.RETRIEVAL: State.RETURN,
        State.RETURN: State.DONE,
    }

    v_slew_up = float(cfg.V_SLEW_UP)
    v_slew_down = float(cfg.V_SLEW_DOWN)
    yaw_slew = float(cfg.YAW_SLEW)
    w_lim = float(cfg.WHEEL_OMEGA_LIMIT)

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

    time.sleep(2.0) # wait for arduino reset
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
    bullseye_vision = BullseyeDetector(cfg)
    blue_scan = blue_detector(cfg)
    
    outer = HeadingPD(cfg)
    mixer = DiffDriveMixer(cfg)

    v_cmd = 0.0
    yaw_cmd = 0.0
    line_lost_since = None
    bullseye_lost_since = None
    bullseye_lost_timeout = cfg.BULLSEYE_LOST_TIMEOUT
    bullseye_align_kp = cfg.BULLSEYE_ALIGN_KP

    pickup_sent = False
    pending_turn_angle = 180.0
    last_ack = None
    halted = False
    halted_prev = False

    t0 = time.monotonic()
    t_next_outer = t0
    t_next_inner = t0

    print(f"Starting retrieval test: {sm.state}")

    try:
        while sm.state != State.DONE and (time.monotonic() - t0 < cfg.MAX_RUN_S):
            now = time.monotonic()

            if now >= t_next_outer:
                ret, frame = cap.read()
                ctx = FrameContext(frame) if ret else None

                line_valid = False
                line_theta_deg = None
                bullseye_found = False
                bullseye_center = None
                bullseye_angle_deg = None
                pickup_ready = False

                if ctx is not None:
                    if sm.state in {State.SEARCHING, State.RETURN}:
                        line_result = run_detector(line_vision, ctx)
                        if line_result:
                            line_theta_deg = line_result.angle_deg
                            line_valid = line_result.found

                    if sm.state == State.SEARCHING:
                        bullseye_found = run_blue_detector(blue_scan, ctx)

                    elif sm.state == State.BULLSEYE_LINEUP:
                        bullseye_result = run_detector(bullseye_vision, ctx)
                        if bullseye_result:
                            bullseye_found = bullseye_result.found
                            bullseye_center = bullseye_result.center
                            bullseye_angle_deg = bullseye_result.angle_deg
                            pickup_ready = bullseye_result.pickup_ready

                    # Debug visualization
                    if cfg.DEBUG_SHOW and ctx is not None:
                        dbg_frame = frame.copy()
                        if sm.state == State.BULLSEYE_LINEUP and bullseye_result and bullseye_result.debug.get("frame") is not None:
                            dbg_frame = bullseye_result.debug["frame"]
                        elif line_result and line_result.debug.get("frame") is not None:
                            dbg_frame = line_result.debug["frame"]
                        
                        cv.putText(dbg_frame, f"State: {sm.state.name}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv.imshow("Retrieval Test", dbg_frame)
                        if cv.waitKey(1) & 0xFF == ord('q'):
                            sm.transition_to(State.DONE)

                # State Transitions and Control
                if sm.state == State.SEARCHING:
                    if bullseye_found:
                        print("Bullseye (blue) detected! Switching to LINEUP")
                        halted = True
                        v_cmd, yaw_cmd = 0.0, 0.0
                        sm.transition_to(TEST_FLOW[sm.state])
                    else:
                        halted, yaw_cmd, v_cmd, line_lost_since, _ = update_line_follow(
                            cfg, outer, line_valid, line_theta_deg, yaw_cmd, v_cmd, line_lost_since, now, v_slew_up, v_slew_down, yaw_slew
                        )

                elif sm.state == State.BULLSEYE_LINEUP:
                    if bullseye_found:
                        bullseye_lost_since = None
                        halted = False
                        
                        cy = bullseye_center[1] if bullseye_center else 0
                        dy = cfg.BULLSEYE_PICKUP_Y - cy
                        
                        angle_ok = abs(bullseye_angle_deg) <= cfg.BULLSEYE_ANGLE_TOL_DEG if bullseye_angle_deg is not None else False
                        dy_ok = dy <= 0

                        if pickup_ready and angle_ok and dy_ok:
                            print("Pickup ready! Transitioning to RETRIEVAL")
                            final_angle = bullseye_angle_deg if bullseye_angle_deg is not None else 0.0
                            pending_turn_angle = wrap_deg(180.0 - final_angle)
                            halted = True
                            v_cmd, yaw_cmd = 0.0, 0.0
                            sm.transition_to(TEST_FLOW[sm.state])
                        else:
                            # Alignment logic
                            if bullseye_angle_deg is not None and not angle_ok:
                                yaw_cmd = clamp(bullseye_align_kp * bullseye_angle_deg, -cfg.U_YAW_LIMIT, cfg.U_YAW_LIMIT)
                            else:
                                yaw_cmd = 0.0
                            
                            if dy > 0:
                                v_cmd = clamp(cfg.BULLSEYE_DY_KP * dy, cfg.BULLSEYE_CREEP_MIN, cfg.BULLSEYE_CREEP_MAX)
                            else:
                                v_cmd = 0.0
                    else:
                        if bullseye_lost_since is None: bullseye_lost_since = now
                        if now - bullseye_lost_since >= bullseye_lost_timeout:
                            print("Bullseye lost during lineup. Faulting.")
                            sm.transition_to(State.FAULT)

                elif sm.state == State.RETRIEVAL:
                    halted = True
                    v_cmd, yaw_cmd = 0.0, 0.0
                    if not pickup_sent:
                        print(f"Sending pickup command: T {pending_turn_angle:.2f}")
                        io.write("S\n")
                        io.write(f"T {pending_turn_angle:.2f}\n")
                        pickup_sent = True
                    
                    if last_ack:
                        if "PICK_DONE" in last_ack:
                            print("Pickup complete! Returning...")
                            sm.transition_to(TEST_FLOW[sm.state])
                        elif "ERR" in last_ack:
                            print(f"Retrieval error: {last_ack}")
                            sm.transition_to(State.FAULT)

                elif sm.state == State.RETURN:
                    halted, yaw_cmd, v_cmd, line_lost_since, _ = update_line_follow(
                        cfg, outer, line_valid, line_theta_deg, yaw_cmd, v_cmd, line_lost_since, now, v_slew_up, v_slew_down, yaw_slew
                    )
                    if halted:
                        print("Reached end of return line or line lost.")
                        sm.transition_to(State.DONE)

                t_next_outer += cfg.DT_OUTER

            if now >= t_next_inner:
                if sm.state != State.RETRIEVAL:
                    if halted:
                        if not halted_prev: io.write("S\n")
                        send_vel(io, 0.0, 0.0)
                    else:
                        w_l, w_r = mixer.wheel_speed_setpoints(v_cmd, yaw_cmd)
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
