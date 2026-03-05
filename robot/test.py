# robot/test.py

# python3 robot/test.py --port COM4 --cmd ticks --cpr 2797 --omega 6 --duration 10 --handshake
import time
import math
import argparse

from hardware.usb_serial import USBSerial


def ts():
    return time.strftime("%H:%M:%S")


def log(msg):
    print(f"[{ts()}] {msg}", flush=True)


def bytes_hex(b: bytes, max_len=64):
    if b is None:
        return "None"
    if len(b) > max_len:
        b = b[:max_len]
        return " ".join(f"{x:02X}" for x in b) + " ..."
    return " ".join(f"{x:02X}" for x in b)


def ser_readline_raw(io: USBSerial) -> bytes:
    if not io.ser or not io.ser.is_open:
        return b""
    try:
        return io.ser.readline()
    except Exception as e:
        log(f"RX readline exception: {repr(e)}")
        return b""


def ser_in_waiting(io: USBSerial) -> int:
    try:
        return int(getattr(io.ser, "in_waiting", 0))
    except Exception:
        return 0


def handshake(io: USBSerial, timeout_s: float = 5.0) -> bool:
    log("Handshake: TX 'SIX\\n'")
    io.write("SIX\n")

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        iw = ser_in_waiting(io)
        raw = ser_readline_raw(io)
        if raw:
            txt = raw.decode(errors="ignore").strip()
            log(f"Handshake: RX txt='{txt}' raw_hex={bytes_hex(raw)} in_waiting={iw}")
            if txt == "SEVEN":
                log("Handshake: OK")
                return True
        else:
            time.sleep(0.05)

    log("Handshake: FAIL (timeout)")
    return False


def make_cmd(cmd_mode: str, wl: float, wr: float, cpr: float) -> str:
    if cmd_mode == "rad":
        return f"W {wl:.6f} {wr:.6f}\n"
    if cmd_mode == "ticks":
        l_tps = wl * (cpr / (2.0 * math.pi))
        r_tps = wr * (cpr / (2.0 * math.pi))
        return f"V {l_tps:.3f} {r_tps:.3f}\n"
    raise ValueError(f"Unknown cmd_mode: {cmd_mode}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=0.10)
    ap.add_argument("--boot_delay", type=float, default=2.0)

    ap.add_argument("--omega", type=float, default=5.0, help="wheel rad/s forward")
    ap.add_argument("--duration", type=float, default=5.0, help="seconds to drive")
    ap.add_argument("--hz", type=float, default=20.0, help="command rate (Hz)")

    ap.add_argument("--cmd", choices=["rad", "ticks"], default="rad")
    ap.add_argument("--cpr", type=float, default=2797.0)

    ap.add_argument("--handshake", action="store_true")

    ap.add_argument("--left_sign", type=float, default=1.0)
    ap.add_argument("--right_sign", type=float, default=1.0)

    ap.add_argument("--print_period", type=float, default=1.0)

    args = ap.parse_args()

    log(f"Config: port={args.port} baud={args.baud} timeout={args.timeout} boot_delay={args.boot_delay}")
    log(f"Motion: omega={args.omega} rad/s duration={args.duration}s hz={args.hz} cmd={args.cmd}")

    io = USBSerial(args.port, baudrate=args.baud, timeout=args.timeout)

    log("Serial: connect()")
    try:
        io.connect()
    except Exception as e:
        log(f"Serial: CONNECT FAIL: {repr(e)}")
        log("If this is PermissionError(13) on Windows: close Arduino Serial Monitor / PlatformIO monitor and replug USB.")
        return

    try:
        log(f"Serial: open OK (is_open={io.ser.is_open}, port={getattr(io.ser, 'port', None)})")
    except Exception:
        log("Serial: open OK")

    if args.boot_delay > 0:
        log(f"Serial: boot delay {args.boot_delay:.2f}s (Arduino may reset on open)")
        time.sleep(args.boot_delay)

    try:
        io.ser.reset_input_buffer()
        io.ser.reset_output_buffer()
        log("Serial: buffers flushed")
    except Exception as e:
        log(f"Serial: flush failed: {repr(e)}")

    pre = ser_readline_raw(io)
    if pre:
        log(f"Serial: pre-RX txt='{pre.decode(errors='ignore').strip()}' raw_hex={bytes_hex(pre)}")
    else:
        log("Serial: pre-RX (none)")

    if args.handshake:
        ok = handshake(io)
        if not ok:
            log("Exiting due to handshake failure")
            io.close()
            return

    wl = args.left_sign * args.omega
    wr = args.right_sign * args.omega

    period = 1.0 / max(args.hz, 1e-6)
    t_end = time.time() + max(args.duration, 0.0)

    sent = 0
    tx_fail = 0
    rx_ok = 0
    rx_empty = 0
    last_rx_txt = ""
    last_rx_hex = ""
    last_print = time.time()

    log("Starting command loop")
    try:
        while time.time() < t_end:
            cmd = make_cmd(args.cmd, wl, wr, args.cpr)

            try:
                io.write(cmd)
            except Exception as e:
                tx_fail += 1
                log(f"TX FAIL: {repr(e)} cmd='{cmd.strip()}'")
            sent += 1

            iw = ser_in_waiting(io)
            raw = ser_readline_raw(io)

            if raw:
                rx_ok += 1
                last_rx_txt = raw.decode(errors="ignore").strip()
                last_rx_hex = bytes_hex(raw)
            else:
                rx_empty += 1

            now = time.time()
            if now - last_print >= args.print_period:
                log(
                    f"Status: sent={sent} tx_fail={tx_fail} rx_ok={rx_ok} rx_empty={rx_empty} "
                    f"in_waiting={iw} last_rx='{last_rx_txt}' last_rx_hex={last_rx_hex}"
                )
                last_print = now

            time.sleep(period)

    except KeyboardInterrupt:
        log("Interrupted")

    log("Sending stop")
    try:
        io.write(make_cmd(args.cmd, 0.0, 0.0, args.cpr))
    except Exception as e:
        log(f"Stop TX failed: {repr(e)}")

    io.close()
    log("Done")


if __name__ == "__main__":
    main()