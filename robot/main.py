import time
import math
import cv2 as cv

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


def run_detector(detector, ctx):
    if hasattr(detector, "detect"):
        return detector.detect(ctx)
    if hasattr(detector, "process"):
        return detector.process(ctx)
    raise AttributeError(
        f"{detector.__class__.__name__} has neither detect(ctx) nor process(ctx)"
    )


def run_blue_detector(detector, ctx):
    if hasattr(detector, "detect"):
        result = detector.detect(ctx)
    elif hasattr(detector, "detect_blue"):
        result = detector.detect_blue(ctx)
    else:
        raise AttributeError(
            f"{detector.__class__.__name__} has neither detect(ctx) nor detect_blue(ctx)"
        )

    if isinstance(result, bool):
        return result

    return bool(getattr(result, "found", False))


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

        speed_scale = 1.0 / (1.0 + cfg.KV * abs(yaw_cmd))
        v_target = cfg.vmax * speed_scale
        v_target = clamp(v_target, cfg.V_MIN, cfg.vmax)
        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)

        return halted, yaw_cmd, v_cmd, line_lost_since, 1

    if line_lost_since is None:
        line_lost_since = now

    if now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
        return True, 0.0, 0.0, line_lost_since, 0

    return False, yaw_cmd, v_cmd, line_lost_since, 0


def main():
    cfg = Config()

    if (
        isinstance(cfg.SERIAL_PORT, str)
        and cfg.SERIAL_PORT
        and not cfg.SERIAL_PORT.startswith("/")
        and "tty" in cfg.SERIAL_PORT
    ):
        cfg.SERIAL_PORT = "/" + cfg.SERIAL_PORT

    v_slew_up = float(cfg.V_SLEW_UP)
    v_slew_down = float(cfg.V_SLEW_DOWN)
    yaw_slew = float(cfg.YAW_SLEW)
    w_lim = float(cfg.WHEEL_OMEGA_LIMIT)
    max_run_s = float(getattr(cfg, "MAX_RUN_S", getattr(cfg, "RUN_TIME_S", 20.0)))

    print("max angular vel is " + str(w_lim))

    cap = cv.VideoCapture(cfg.CAM_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
    cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {cfg.CAM_INDEX}")

    line_vision = LineDetector(cfg)
    bullseye_vision = BullseyeDetector(cfg)
    blue_scan = blue_detector(cfg)
    safezone_vision = SafezoneDetector(cfg)

    mixer = DiffDriveMixer(cfg.r, cfg.L)
    outer = HeadingPD(
        cfg.KP_THETA,
        cfg.KD_THETA,
        dt=cfg.DT_OUTER,
        u_limit=cfg.U_YAW_LIMIT,
    )

    io = USBSerial(cfg.SERIAL_PORT, baudrate=cfg.BAUD, timeout=cfg.SERIAL_TIMEOUT)
    io.connect()

    time.sleep(2.0)

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
            print("handshake completed")
            break
    if not ok:
        raise RuntimeError("Handshake failed")

    io.write("E 1\n")
    io.read()

    halted = False
    halted_prev = False

    yaw_cmd = 0.0
    v_cmd = 0.0

    sm = RobotStateMachine()
    last_state = sm.state
    last_ack = None

    line_lost_since = None
    bullseye_lost_since = None

    pickup_sent = False
    dropoff_sent = False
    pending_turn_angle = 0.0

    start_delay_s = getattr(cfg, "START_DELAY_S", 1.0)
    bullseye_lost_timeout = getattr(cfg, "BULLSEYE_LOST_TIMEOUT", 0.4)
    bullseye_align_kp = getattr(cfg, "BULLSEYE_ALIGN_KP", 0.01)

    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()
    t_start = time.perf_counter()

    stats_start = t_start
    camera_query_count = 0
    pd_update_count = 0
    usb_send_count = 0
    
    # near your other setup vars
    post_pick_settle_s = getattr(cfg, "POST_PICK_SETTLE_S", 0.35)
    resume_find_safety_at = None

    try:
        while True:
            now = time.perf_counter()
            if now - t_start >= max_run_s:
                break

            if now >= t_next_outer:
                camera_query_count += 1

                if sm.state != last_state:
                    print("STATE:", sm.state.name)

                    if sm.state != State.RETRIEVAL:
                        pickup_sent = False
                    if sm.state != State.DEPOSITION:
                        dropoff_sent = False

                    bullseye_lost_since = None
                    last_state = sm.state

                ret, frame = cap.read()
                ctx = FrameContext(frame) if ret else None

                line_valid = False
                line_theta_deg = None

                bullseye_found = False
                bullseye_offset_px = None
                bullseye_angle_deg = None
                pickup_ready = False
                bullseye_center = None

                safety_found = False

                if ctx is not None:
                    if sm.state in {State.SEARCHING, State.FIND_SAFETY, State.RETURN}:
                        line_result = run_detector(line_vision, ctx)
                        line_theta_deg = getattr(line_result, "angle_deg", None)
                        line_valid = bool(getattr(line_result, "found", False))

                    if sm.state == State.SEARCHING:
                        bullseye_found = run_blue_detector(blue_scan, ctx)

                    elif sm.state == State.BULLSEYE_LINEUP:
                        bullseye_result = run_detector(bullseye_vision, ctx)
                        bullseye_found = bool(getattr(bullseye_result, "found", False))
                        bullseye_offset_px = getattr(bullseye_result, "offset_px", None)
                        bullseye_angle_deg = getattr(bullseye_result, "angle_deg", None)
                        pickup_ready = bool(getattr(bullseye_result, "pickup_ready", False))
                        bullseye_center = getattr(bullseye_result, "center", None)

                    elif sm.state == State.FIND_SAFETY:
                        safety_result = run_detector(safezone_vision, ctx)
                        safety_found = bool(getattr(safety_result, "found", False))

                if sm.state == State.WAIT_TO_START:
                    halted = True
                    yaw_cmd = 0.0
                    v_cmd = 0.0

                    if sm.time_in_state() >= start_delay_s:
                        sm.next()

                elif sm.state == State.SEARCHING:
                    if bullseye_found:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0
                        line_lost_since = None
                        sm.next()
                    else:
                        halted, yaw_cmd, v_cmd, line_lost_since, pd_inc = update_line_follow(
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
                        )
                        pd_update_count += pd_inc

                elif sm.state == State.BULLSEYE_LINEUP:
                    if bullseye_found:
                        bullseye_lost_since = None
                        halted = False

                        cy = bullseye_center[1] if bullseye_center is not None else None
                        dy = cfg.BULLSEYE_PICKUP_Y - cy if cy is not None else None

                        angle_ok = (
                            bullseye_angle_deg is not None
                            and abs(bullseye_angle_deg) <= cfg.BULLSEYE_ANGLE_TOL_DEG
                        )
                        dy_ok = (dy is not None and dy <= 0)

                        if pickup_ready and angle_ok and dy_ok:
                            final_bullseye_angle_deg = (
                                bullseye_angle_deg if bullseye_angle_deg is not None else 0.0
                            )
                            pending_turn_angle = wrap_deg(180.0 - final_bullseye_angle_deg)

                            halted = True
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                            
                            if not pickup_sent:
                                last_ack = None
                                io.write("S\n")
                                io.write(f"T {pending_turn_angle:.2f}\n")
                                pickup_sent = True

                            while not last_ack:
                                ack = io.read()
                                if ack:
                                    last_ack = ack.strip()

                            if last_ack.startswith("PICK_DONE"):
                                print("pickup acknowledged:", last_ack)
                                for _ in range(20):
                                    cap.read()

                                halted = True
                                yaw_cmd = 0.0
                                v_cmd = 0.0
                                line_lost_since = None
                                bullseye_lost_since = None

                                if hasattr(outer, "reset"):
                                    outer.reset()
                                resume_find_safety_at = now + post_pick_settle_s
                                sm.transition_to(State.FIND_SAFETY)
                            elif last_ack.startswith("ERR PICK") or last_ack.startswith("ERR T"):
                                print("pickup failed:", last_ack)
                                sm.transstion_to(State.FAULT)
                        else:
                            if bullseye_angle_deg is not None and not angle_ok:
                                yaw_target = clamp(
                                    -bullseye_align_kp * bullseye_angle_deg,
                                    -cfg.U_YAW_LIMIT,
                                    cfg.U_YAW_LIMIT,
                                )

                                if yaw_slew > 0.0:
                                    yaw_cmd = slew(
                                        yaw_cmd,
                                        yaw_target,
                                        yaw_slew,
                                        yaw_slew,
                                        cfg.DT_OUTER,
                                    )
                                else:
                                    yaw_cmd = yaw_target
                            else:
                                yaw_cmd = 0.0

                            if dy is not None and dy > 0:
                                v_target = clamp(
                                    cfg.BULLSEYE_DY_KP * dy,
                                    cfg.BULLSEYE_CREEP_MIN,
                                    cfg.BULLSEYE_CREEP_MAX,
                                )
                                v_cmd = slew(
                                    v_cmd,
                                    v_target,
                                    v_slew_up,
                                    v_slew_down,
                                    cfg.DT_OUTER,
                                )
                            else:
                                v_cmd = 0.0
                    else:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0

                        if bullseye_lost_since is None:
                            bullseye_lost_since = now

                        if now - bullseye_lost_since >= bullseye_lost_timeout:
                            sm.transition_to(State.DONE)

                # elif sm.state == State.RETRIEVAL:
                #     halted = True
                #     yaw_cmd = 0.0
                #     v_cmd = 0.0

                #     if not pickup_sent:
                #         last_ack = None
                #         io.write("S\n")
                #         io.write(f"T {pending_turn_angle:.2f}\n")
                #         pickup_sent = True

                #     if last_ack:
                #         if last_ack.startswith("PICK_DONE"):
                #             print("pickup acknowledged:", last_ack)
                #             sm.next()
                #         elif last_ack.startswith("ERR PICK") or last_ack.startswith("ERR T"):
                #             print("pickup failed:", last_ack)
                #             sm.transition_to(State.FAULT)

                elif sm.state == State.FIND_SAFETY:
                    if resume_find_safety_at is not None and now < resume_find_safety_at:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0

                    elif safety_found:
                        resume_find_safety_at = None
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0
                        sm.next()

                    else:
                        resume_find_safety_at = None
                        halted, yaw_cmd, v_cmd, line_lost_since, pd_inc = update_line_follow(
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
                        )
                        pd_update_count += pd_inc
                        
                        print("after pickup:", sm.state, halted, line_valid, line_theta_deg, yaw_cmd, v_cmd)

                        if line_lost_since is not None and now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
                            print("Line lost while finding safety. Ending run.")
                            sm.transition_to(State.DONE)

                elif sm.state == State.DEPOSITION:
                    halted = True
                    yaw_cmd = 0.0
                    v_cmd = 0.0

                    if not dropoff_sent:
                        last_ack = None
                        io.write("S\n")
                        io.write("D\n")
                        dropoff_sent = True

                    if last_ack:
                        if last_ack.startswith("DROP_DONE"):
                            print("dropoff acknowledged")
                            sm.next()
                        elif last_ack.startswith("ERR DROP") or last_ack.startswith("ERR D"):
                            print("dropoff failed:", last_ack)
                            sm.transition_to(State.FAULT)

                elif sm.state == State.RETURN:
                    halted, yaw_cmd, v_cmd, line_lost_since, pd_inc = update_line_follow(
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
                    )
                    pd_update_count += pd_inc

                    if halted:
                        sm.next()

                elif sm.state in {State.DONE, State.FAULT}:
                    halted = True
                    yaw_cmd = 0.0
                    v_cmd = 0.0
                    hard_stop(io)
                    break
                else:
                    halted = True
                    yaw_cmd = 0.0
                    v_cmd = 0.0

                while t_next_outer <= now:
                    t_next_outer += cfg.DT_OUTER

            if now >= t_next_inner:
                action_active = sm.state in {State.RETRIEVAL, State.DEPOSITION}

                if not action_active:
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

                        l_cps = w_l * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        r_cps = w_r * (cfg.ENCODER_CPR / (2.0 * math.pi))
                        send_vel(io, l_cps, r_cps)
                        usb_send_count += 1

                    halted_prev = halted
                else:
                    halted_prev = True

                ack = io.read()
                if ack:
                    last_ack = ack.strip()
                    if last_ack.startswith("ERR"):
                        print("ARDUINO:", last_ack)

                while t_next_inner <= now:
                    t_next_inner += cfg.DT_INNER

    finally:
        total_runtime = time.perf_counter() - stats_start

        if total_runtime > 0.0:
            avg_camera_hz = camera_query_count / total_runtime
            avg_pd_hz = pd_update_count / total_runtime
            avg_usb_hz = usb_send_count / total_runtime

            print("\n=== Runtime Frequency Summary ===")
            print("Total runtime [s]: " + str(total_runtime))
            print("Camera queries: " + str(camera_query_count) + " | avg Hz: " + str(avg_camera_hz))
            print("PD updates: " + str(pd_update_count) + " | avg Hz: " + str(avg_pd_hz))
            print("USB sends: " + str(usb_send_count) + " | avg Hz: " + str(avg_usb_hz))

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