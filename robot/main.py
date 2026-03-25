import io
import os
import time
import math
import cv2 as cv

from config import Config
from vision.vision_system import LineDetector, BullseyeDetector, blue_detector, SafezoneDetector # NEW SAFEZONE Functionality
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


def hard_stop(io): # might get rid of this
    try:
        send_vel(io, 0.0, 0.0)
    except Exception:
        pass

    if hasattr(io, "write"):
        try:
            io.write("STOP\n")
        except Exception:
            pass


def main():
    cfg = Config()

    if isinstance(cfg.SERIAL_PORT, str) and cfg.SERIAL_PORT and not cfg.SERIAL_PORT.startswith("/") and "tty" in cfg.SERIAL_PORT:
        cfg.SERIAL_PORT = "/" + cfg.SERIAL_PORT

    v_slew_up = float(cfg.V_SLEW_UP)
    v_slew_down = float(cfg.V_SLEW_DOWN)
    yaw_slew = float(cfg.YAW_SLEW)
    w_lim = float(cfg.WHEEL_OMEGA_LIMIT)

    print("max angular vel is " + str(w_lim))

    cap = cv.VideoCapture(cfg.CAM_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cfg.CAM_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.CAM_H)
    cap.set(cv.CAP_PROP_FPS, cfg.CAM_FPS)

    line_vision = LineDetector(cfg)
    bullseye_vision = BullseyeDetector(cfg)
    blue_scan = blue_detector(cfg)
    safezone_vision = SafezoneDetector(cfg)         # NEW SAFEZONE Functionality
    mixer = DiffDriveMixer(cfg.r, cfg.L)
    outer = HeadingPD(cfg.KP_THETA, cfg.KD_THETA, dt=cfg.DT_OUTER, u_limit=cfg.U_YAW_LIMIT)

    io = USBSerial(cfg.SERIAL_PORT, baudrate=cfg.BAUD, timeout=cfg.SERIAL_TIMEOUT)
    io.connect()

    time.sleep(2.0)

    try:
        io.ser.reset_input_buffer()
        io.ser.reset_output_buffer()
    except Exception:
        pass

    # Handshake
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

    # Motion / command state
    halted = False
    halted_prev = False

    yaw_cmd = 0.0
    v_cmd = 0.2
    theta_ref_rad = 0.0
    theta_rad = 0.0

    # Mission state
    sm = RobotStateMachine()
    last_state = sm.state
    last_ack = None

    # Detection / timeout helpers
    line_lost_since = None
    bullseye_lost_since = None

    # One-shot action flags
    pickup_sent = False

    # Tunables
    start_delay_s = getattr(cfg, "START_DELAY_S", 1.0)
    bullseye_lost_timeout = getattr(cfg, "BULLSEYE_LOST_TIMEOUT", 0.4)
    bullseye_align_kp = getattr(cfg, "BULLSEYE_ALIGN_KP", 0.01)
    bullseye_forward_angle_gate_deg = getattr(cfg, "BULLSEYE_FORWARD_ANGLE_GATE_DEG", 8.0)
    bullseye_creep_speed = getattr(cfg, "BULLSEYE_CREEP_SPEED", 0.05)

    t_next_inner = time.perf_counter()
    t_next_outer = time.perf_counter()
    t_start = time.perf_counter()

    stats_start = t_start
    camera_query_count = 0
    pd_update_count = 0
    usb_send_count = 0

    try:
        while True:
            now = time.perf_counter()
            if now - t_start >= cfg.MAX_RUN_S:
                break

            if now >= t_next_outer:
                camera_query_count += 1

                if sm.state != last_state:
                    print("STATE:", sm.state.name)
                    if sm.state != State.RETRIEVAL:
                        pickup_sent = False
                    bullseye_lost_since = None
                    last_state = sm.state

                ret, frame = cap.read()

                line_valid = False
                line_theta_deg = None
                line_offset_px = None
                line_dbg = None

                bullseye_found = False
                bullseye_offset_px = None
                bullseye_angle_deg = None
                pickup_ready = False
                bull_dbg = None
                final_bullseye_angle_deg = 0.0
                
                turn_angle = None

                dbg = None

                # Vision results extraction
                if ret:
                    if sm.state == State.SEARCHING:
                        line_result = line_vision.process(frame)
                        line_theta_deg = line_result.angle_deg
                        line_offset_px = line_result.offset_px
                        line_valid = line_result.found
                        bullseye_found = blue_scan.detect_blue(frame)

                    elif sm.state == State.BULLSEYE_LINEUP:
                        # replace these next lines with your actual bullseye call/result extraction
                        result = bullseye_vision.process(frame)
                        bullseye_found = result.found
                        bullseye_offset_px = result.offset_px
                        bullseye_angle_deg = result.angle_deg
                        pickup_ready = getattr(result, "pickup_ready", False)

                    elif sm.state == State.FIND_SAFETY:
                        line_result = line_vision.process(frame)
                        line_theta_deg = line_result.angle_deg
                        line_offset_px = line_result.offset_px
                        line_valid = line_result.found
                        line_dbg = line_result.debug
                        safety_result = safezone_vision.detect(frame)
                        safety_found = safety_result.found # placeholder for now, will be separate vision class eventually
                        
                        dbg = line_result.debug
                        
                    elif sm.state == State.RETURN:
                        line_result = line_vision.process(frame)
                        line_theta_deg = line_result.angle_deg
                        line_offset_px = line_result.offset_px
                        line_valid = line_result.found
                        line_dbg = line_result.debug
                        
                        dbg = line_dbg

                    else:
                        dbg = None

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

                    elif line_valid:
                        line_lost_since = None
                        halted = False

                        theta_rad = math.radians(line_theta_deg)
                        theta_ref_rad = 0.0

                        yaw_target = outer.step(theta_ref_rad, theta_rad)
                        pd_update_count += 1

                        if yaw_slew > 0.0:
                            yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
                        else:
                            yaw_cmd = yaw_target

                        speed_scale = 1.0 / (1.0 + cfg.KV * abs(yaw_cmd))
                        v_target = cfg.vmax * speed_scale
                        v_target = clamp(v_target, cfg.V_MIN, cfg.vmax)
                        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)

                    else:
                        if line_lost_since is None:
                            line_lost_since = now

                        if now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
                            halted = True
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                        else:
                            halted = False
                elif sm.state == State.BULLSEYE_LINEUP: # change once I see fixed turn code
                    if bullseye_found:
                        bullseye_lost_since = None
                        halted = False

                        cy = result.center[1] if result.center is not None else None
                        dy = cfg.BULLSEYE_PICKUP_Y - cy if cy is not None else None

                        angle_ok = (
                            bullseye_angle_deg is not None
                            and abs(bullseye_angle_deg) <= cfg.BULLSEYE_ANGLE_TOL_DEG
                        )
                        dy_ok = (dy is not None and dy <= 0)

                        if pickup_ready and angle_ok and dy_ok: # kinds redundant to check pickup_ready here since it should only be true if angle and dy are good, but just in case
                            final_bullseye_angle_deg = bullseye_angle_deg if bullseye_angle_deg is not None else 0.0
                            halted = True
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                            if turn_angle > 180.0: # exact directions TBD
                                turn_angle -= 360.0
                            io.write(f"TURN {turn_angle:.2f}\n") 
                            
                            sm.next()
                        else:
                            # turn only if angle is outside tolerance
                            if not angle_ok: # not sure how this will behave, maybe use gains from PD
                                yaw_target = clamp(
                                    bullseye_align_kp * bullseye_angle_deg,
                                    -cfg.U_YAW_LIMIT,
                                    cfg.U_YAW_LIMIT,
                                )

                                if yaw_slew > 0.0:
                                    yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
                                else:
                                    yaw_cmd = yaw_target
                            else:
                                yaw_cmd = 0.0

                            # move forward until dy reaches zero
                            if dy is not None and dy > 0:
                                v_target = clamp(
                                    cfg.BULLSEYE_DY_KP * dy,
                                    cfg.BULLSEYE_CREEP_MIN,
                                    cfg.BULLSEYE_CREEP_MAX,
                                )
                                v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)
                            else:
                                v_cmd = 0.0

                    else:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0

                        if bullseye_lost_since is None:
                            bullseye_lost_since = now

                        if now - bullseye_lost_since >= bullseye_lost_timeout:
                            sm.transition_to(State.SEARCHING)
                elif sm.state == State.RETRIEVAL: # purely picking system
                    halted = True
                    yaw_cmd = 0.0
                    v_cmd = 0.0

                    if not pickup_sent:
                        io.write("T " + str(turn_angle) + "\n")
                        pickup_sent = True

                    pickup_done = (last_ack == "PICK_DONE")
                    if pickup_done:
                        print("pickup acknowledged")
                        # later: sm.next() when FIND_SAFETY is integrated
                elif sm.state == State.FIND_SAFETY:
                    if safety_found:
                        halted = True
                        yaw_cmd = 0.0
                        v_cmd = 0.0
                        sm.next()
                    elif line_valid:
                        line_lost_since = None
                        halted = False

                        theta_rad = math.radians(line_theta_deg)
                        theta_ref_rad = 0.0

                        yaw_target = outer.step(theta_ref_rad, theta_rad)
                        pd_update_count += 1

                        if yaw_slew > 0.0:
                            yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
                        else:
                            yaw_cmd = yaw_target

                        speed_scale = 1.0 / (1.0 + cfg.KV * abs(yaw_cmd))
                        v_target = cfg.vmax * speed_scale
                        v_target = clamp(v_target, cfg.V_MIN, cfg.vmax)
                        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)

                    else:
                        if line_lost_since is None:
                            line_lost_since = now

                        if now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
                            halted = True
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                        else:
                            halted = False
                elif sm.state == State.DEPOSITION:
                    if not dropoff_sent:
                        io.write("T " + str(turn_angle) + "\n")
                        dropoff_sent = True
                elif sm.state == State.RETURN:
                    if line_valid:
                        line_lost_since = None
                        halted = False

                        theta_rad = math.radians(line_theta_deg)
                        theta_ref_rad = 0.0

                        yaw_target = outer.step(theta_ref_rad, theta_rad)
                        pd_update_count += 1

                        if yaw_slew > 0.0:
                            yaw_cmd = slew(yaw_cmd, yaw_target, yaw_slew, yaw_slew, cfg.DT_OUTER)
                        else:
                            yaw_cmd = yaw_target

                        speed_scale = 1.0 / (1.0 + cfg.KV * abs(yaw_cmd))
                        v_target = cfg.vmax * speed_scale
                        v_target = clamp(v_target, cfg.V_MIN, cfg.vmax)
                        v_cmd = slew(v_cmd, v_target, v_slew_up, v_slew_down, cfg.DT_OUTER)
                    else:
                        if line_lost_since is None:
                            line_lost_since = now

                        if now - line_lost_since >= cfg.LINE_LOST_TIMEOUT:
                            halted = True
                            yaw_cmd = 0.0
                            v_cmd = 0.0
                            sm.next()
                        else:
                            halted = False
                    
                else: # ideally never reaches here, there will be a done and fault state eventually
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
                    halted_prev = True   # optional safety so next halt re-sends stop cleanly

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