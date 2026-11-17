import ctypes
import msvcrt
import time

import serial
import vgamepad as vg

PORT = "COM9"
BAUD = 921600
TIMEOUT_S = 0.25
STATUS_EVERY_S = 0.5

steer_range = 90
scale = 1.0


BRAKE_INVERT = True
BRAKE_BIT = 4

THROTTLE_BIT = 5

BUTTON_MAP = {
    0: vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    1: vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    2: vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
    3: vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    4: vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    5: vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
}

HELP = """Keys:
  z  zero the steering point
  c  recalibrate gyro (hold still, ~3 s)
  a  toggle accelerometer drift correction
  i  invert direction
  +/-  adjust angle scale 0.5%
  [ ]  steering range -/+ 30 deg
  q  quit
"""


def parse(line):
    try:
        fields = line.strip().split(",")
        bits = int(fields[1]) if len(fields) > 1 else 0
        brake = int(fields[2]) if len(fields) > 2 else 0
        throttle = int(fields[3]) if len(fields) > 3 else 0
        return int(fields[0]) / 100.0, bits, brake, throttle
    except (ValueError, IndexError):
        return None


def to_axis_int(angle):
    v = max(-1.0, min(1.0, angle / steer_range))
    return int(round(v * 32767))


def send(ser, text):
    ser.write((text + "\n").encode())


def handle_key(ser, ch):
    global scale, steer_range
    if ch in ("z", "c", "a", "i"):
        send(ser, ch)
    elif ch in ("+", "="):
        scale = min(1.99, scale * 1.005)
        send(ser, "s %.4f" % scale)
    elif ch in ("-", "_"):
        scale = max(0.51, scale / 1.005)
        send(ser, "s %.4f" % scale)
    elif ch == "[":
        steer_range = max(30.0, steer_range - 30.0)
    elif ch == "]":
        steer_range = min(900.0, steer_range + 30.0)
    elif ch == "q":
        return True
    return False


def setup_realtime():
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass
    try:
        k32 = ctypes.windll.kernel32
        k32.SetPriorityClass(k32.GetCurrentProcess(), 0x00000080)
    except Exception:
        pass


def release_all(pad):
    for btn in BUTTON_MAP.values():
        pad.release_button(button=btn)


def main():
    setup_realtime()
    pad = vg.VX360Gamepad()
    ser = serial.Serial(PORT, BAUD, timeout=0.002)
    buf = b""
    now = time.perf_counter()
    last_rx = now
    last_show = 0.0
    prev_buttons = 0
    prev_axis = 0
    braking = False
    throttle = 0
    centered = True
    angle = 0.0
    status = 0

    print(HELP)
    try:
        while True:
            while msvcrt.kbhit():
                if handle_key(ser, msvcrt.getwch()):
                    return

            data = ser.read(max(1, ser.in_waiting))
            now = time.perf_counter()

            if data:
                buf += data
                if len(buf) > 4096:
                    buf = buf[-256:]
                if b"\n" in buf:
                    parts = buf.split(b"\n")
                    buf = parts[-1]
                    result = None
                    for raw in reversed(parts[:-1]):
                        result = parse(raw.decode(errors="ignore"))
                        if result:
                            break
                    if result:
                        angle, bits, brake_raw, throttle = result
                        status = bits
                        brake_val = (255 - brake_raw) if BRAKE_INVERT else brake_raw
                        braking = brake_val >= 128
                        buttons = bits & 0x3F
                        if braking:
                            buttons |= 1 << BRAKE_BIT
                        if throttle >= 128:
                            buttons |= 1 << THROTTLE_BIT
                        last_rx = now
                        centered = False

                        changed = False
                        axis = to_axis_int(angle)
                        if axis != prev_axis:
                            pad.left_joystick(x_value=axis, y_value=0)
                            prev_axis = axis
                            changed = True
                        if buttons != prev_buttons:
                            for bit, btn in BUTTON_MAP.items():
                                if (buttons >> bit) & 1:
                                    pad.press_button(button=btn)
                                else:
                                    pad.release_button(button=btn)
                            prev_buttons = buttons
                            changed = True
                        if changed:
                            pad.update()

            if not centered and now - last_rx > TIMEOUT_S:
                pad.left_joystick(x_value=0, y_value=0)
                release_all(pad)
                pad.update()
                prev_axis = 0
                prev_buttons = 0
                braking = False
                throttle = 0
                centered = True

            if now - last_show > STATUS_EVERY_S:
                last_show = now
                print(
                    "\rangle %8.2f  accel %-3s  trust %d  inv %d  scale %.3f  range %d  brake(LB) %-3s  thr(RB) %-3s   "
                    % (
                        angle,
                        "ON" if status & 0x4000 else "off",
                        1 if status & 0x8000 else 0,
                        1 if status & 0x2000 else 0,
                        scale,
                        steer_range,
                        "ON" if braking else "off",
                        "ON" if throttle >= 128 else "off",
                    ),
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        pad.left_joystick(x_value=0, y_value=0)
        release_all(pad)
        pad.update()
        ser.close()
        try:
            ctypes.windll.winmm.timeEndPeriod(1)
        except Exception:
            pass


main()