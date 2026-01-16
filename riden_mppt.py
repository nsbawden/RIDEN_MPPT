import time
import serial
import sys
import subprocess
from datetime import datetime

# -------------------------------------------------------------------------------------------------
# RIDEN RD6024 "SOFTWARE MPPT" CONTROLLER (Modbus RTU over USB-Serial / CH340)
#
# RUN
# py riden_mppt.py [target volts] [com port] [slave] [--bg]
#
# TYPICAL (defaults as well)
# py riden_mppt.py 33 COM4 1
#
# BACKGROUND MODE (for Task Scheduler)
# py riden_mppt.py 33 COM4 1 --bg
#
# OVERVIEW
# - We treat the RD6024 as a programmable, current-limited DC/DC converter.
# - In a PV->battery setup, the RD6024 is effectively the "smart load" attached to the PV array.
# - The dominant failure mode is PV collapse under clouds:
#     - PV input voltage VIN suddenly drops.
#     - If we keep demanding current, the RD6024 drags VIN down toward battery volts + ~2V
#       (your observed "stuck collapse state").
#
# DESIGN PHILOSOPHY WE LANDED ON
# 1) Stability beats textbook MPPT.
#    - Classic P&O MPPT hill-climbs power. It can work, but it can also get "lost" during clouds
#      because the PV knee/cliff is abrupt and the RD6024 will happily keep pulling current.
#    - Our approach is "constant PV voltage control": hold VIN near a chosen knee.
#      This *prevents collapse* and recovers quickly.
#
# 2) ISET is the throttle.
#    - We control only the current limit (ISET).
#    - VSET is your battery charging goal (absorb/float/etc.) and can be managed separately.
#
# 3) Robustness matters (real-world Modbus).
#    - CRC validation, strict frame reads, retries.
#    - "Bad Modbus read" happens; we retry and keep running.
#
# 4) Clouds require asymmetry and adaptive steps.
#    - Backing off must be faster/stronger than creeping up.
#    - Far from target: take big steps.
#    - Near target: small steps to avoid hunting.
#
# REGISTER MAP WE FOUND (your RD6024; Slave ID=1; FC=03 holding regs)
# Base block at 0x0008 (8 regs: 0x0008..0x000F):
#   0x0008  VSET   raw=V*100     (set output voltage)
#   0x0009  ISET   raw=A*100     (set output current limit)
#   0x000A  OUTV   raw=V*100     (measured output voltage)
#   0x000B  IOUT   raw=A*100     (measured output current)
#   0x000C  ???    unknown/status (often 0 in our snapshots)
#   0x000D  POUT   raw=W*100     (measured output power)
#   0x000E  VIN    raw=V*100     (measured input voltage, PV voltage in our use-case)
#   0x000F  ???    not output on/off (ignore for ON)
#
# Output ON/OFF flags:
#   0x0011  ON1  1 when output ON, 0 when output OFF
#   0x0012  ON2  1 when output ON, 0 when output OFF
# We write both because the device toggles both when the panel Output button is used.
# -------------------------------------------------------------------------------------------------

DEFAULT_TARGET_VIN = 33.0  # volts; good knee you found for your 36V panels

# -------------------------------------------------------------------------------------------------
# STATE NAMES (change in ONE place)
# -------------------------------------------------------------------------------------------------
STATE_BATT_FULL = "BATT F"
STATE_CONST_V = "CONST V"
STATE_MAX_R = "MAX R"


def modbus_crc16(data: bytes) -> int:
    # Standard Modbus RTU CRC-16 (poly 0xA001, init 0xFFFF).
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _crc_ok(frame: bytes) -> bool:
    # Modbus RTU CRC: low byte first.
    if len(frame) < 4:
        return False
    crc_rx = frame[-2] | (frame[-1] << 8)
    return crc_rx == modbus_crc16(frame[:-2])


def _read_exact(ser, n: int, timeout_s: float) -> bytes:
    # Read exactly n bytes or time out.
    end = time.time() + timeout_s
    buf = bytearray()
    while len(buf) < n and time.time() < end:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.005)
    return bytes(buf)


def _modbus_read_regs(
    ser,
    slave: int,
    func: int,
    start: int,
    qty: int,
    tries: int = 3,
    timeout_s: float = 0.35,
) -> list[int]:
    # Generic Modbus RTU read (FC=03 or FC=04). We use FC=03 for this RD6024 map.
    pdu = bytes([slave, func, (start >> 8) & 0xFF, start & 0xFF, (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    for _ in range(tries):
        ser.reset_input_buffer()  # clear junk so old bytes can't corrupt this parse
        ser.write(frame)
        ser.flush()

        hdr = _read_exact(ser, 3, timeout_s)  # [slave][func][bytecount]
        if len(hdr) < 3:
            continue
        if hdr[0] != slave:
            continue

        # Exception response: func | 0x80 (5 bytes total). Retry on exception/time artifacts.
        if hdr[1] == (func | 0x80):
            _read_exact(ser, 2, timeout_s)  # exception_code + CRC (best-effort drain)
            continue

        if hdr[1] != func:
            continue

        bytecount = hdr[2]
        if bytecount != 2 * qty:
            _read_exact(ser, bytecount + 2, timeout_s)  # drain + CRC
            continue

        data_plus_crc = _read_exact(ser, bytecount + 2, timeout_s)
        rx = hdr + data_plus_crc
        if len(rx) != (3 + bytecount + 2):
            continue
        if not _crc_ok(rx):
            continue

        regs: list[int] = []
        for i in range(qty):
            hi = rx[3 + 2 * i]
            lo = rx[3 + 2 * i + 1]
            regs.append((hi << 8) | lo)
        return regs

    raise RuntimeError(f"Modbus read failed func=0x{func:02X} start=0x{start:04X} qty={qty}")


def _modbus_write_reg(
    ser,
    slave: int,
    addr: int,
    value: int,
    tries: int = 3,
    timeout_s: float = 0.35,
) -> None:
    # Modbus RTU "Write Single Register" (FC=06).
    value &= 0xFFFF
    pdu = bytes([slave, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    for _ in range(tries):
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()

        rx = _read_exact(ser, 8, timeout_s)  # normal echo is 8 bytes
        if len(rx) != 8:
            continue
        if not _crc_ok(rx):
            continue
        if rx[0] != slave:
            continue

        if rx[1] == (0x06 | 0x80):
            raise RuntimeError(f"Modbus exception write code=0x{rx[2]:02X}")

        if rx[:6] == pdu:  # device echoes our request PDU on success
            return

    raise RuntimeError(f"Modbus write failed addr=0x{addr:04X} value={value}")


# -------------------------------------------------------------------------------------------------
# REGISTER CONSTANTS (confirmed on your RD6024)
# -------------------------------------------------------------------------------------------------

REG_BASE = 0x0008
REG_VSET = 0x0008
REG_ISET = 0x0009
REG_OUTV = 0x000A
REG_IOUT = 0x000B
REG_POUT = 0x000D
REG_VIN = 0x000E

REG_ON1 = 0x0011
REG_ON2 = 0x0012


def read_status(ser, slave: int):
    # Read the core 8-register block.
    r = _modbus_read_regs(ser, slave, 0x03, REG_BASE, 8)

    vset = r[0] / 100.0
    iset = r[1] / 100.0
    outv = r[2] / 100.0
    iout = r[3] / 100.0
    pout = r[5] / 100.0
    vin = r[6] / 100.0

    # Output ON state: 0x0011, 0x0012.
    f = _modbus_read_regs(ser, slave, 0x03, REG_ON1, 2)
    on = int((f[0] != 0) or (f[1] != 0))

    return vset, iset, outv, iout, pout, vin, on


def set_vset(ser, slave: int, volts: float) -> None:
    _modbus_write_reg(ser, slave, REG_VSET, int(round(volts * 100.0)))


def set_iset(ser, slave: int, amps: float) -> None:
    _modbus_write_reg(ser, slave, REG_ISET, int(round(amps * 100.0)))


def set_output(ser, slave: int, enabled: bool) -> None:
    v = 1 if enabled else 0
    _modbus_write_reg(ser, slave, REG_ON1, v)
    _modbus_write_reg(ser, slave, REG_ON2, v)


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _quant_amps(a: float) -> float:
    # RD6024 current resolution is 0.01A, so quantize to avoid float-chatter.
    return round(a + 1e-9, 2)


def _quant_volts(v: float) -> float:
    # RD6024 voltage resolution is 0.01V.
    return round(v + 1e-9, 2)


def _shutdown_windows_now() -> None:
    # Force close apps to guarantee power off (you’re OK with manual restart in the morning).
    # If you do NOT want to force-close apps, remove the "/f".
    # subprocess.run(["shutdown", "/s", "/f", "/t", "0"], check=False)
    subprocess.run(["shutdown", "/h"], check=False)


def mppt_loop(vtarget: float, port: str, slave: int, background_mode: bool) -> None:
    # ---------------------------------------------------------------------------------------------
    # CONSTANT-PV-VOLTAGE CONTROL LOOP (cloud-resistant "software MPPT")
    # ---------------------------------------------------------------------------------------------

    SHUTDOWN_AFTER_HOUR = 17  # 5pm
    SHUTDOWN_WATTS = 15.0     # approx laptop draw threshold
    SHUTDOWN_HOLD_S = 300     # seconds below threshold before shutdown (5 min)

    VSET_DROP_FULL = 0.30  # volts to reduce VSET when in BATT F state

    VIN_BAND = 0.05  # volts; deadband around target to avoid constant chatter
    HARD_DROP = 5.0  # volts; collapse trigger (VIN far below target => emergency backoff)

    I_MIN = 0.01
    I_MAX = 24.0  # amps; your chosen maximum current pull
    I_OVER = 0.5  # how much ISET can exceed IOUT
    I_UNATTENDED = 15.0  # Best amps for RIDEN to use if MPPT controller is inactive (laptop off)

    LOOP_DELAY = 0.5
    FAST_DELAY = 0.2

    STARTUP_DELAY_S = 8.0  # background wake: give USB/CH340 time to enumerate

    # -------------------------------------------------------------------------------------------------
    # 5-LEVEL STEP DIVISIONS (instead of 3)
    # -------------------------------------------------------------------------------------------------

    V1 = 1.00  # fine/near boundary
    V2 = 1.50  # near/mid boundary
    V3 = 1.75  # mid/far boundary
    V4 = 2.00  # far/huge boundary

    HUGE_STEP_UP = 1.75
    HUGE_STEP_DN = 1.75
    FAR_STEP_UP = 0.20
    FAR_STEP_DN = 0.20
    MID_STEP_UP = 0.075
    MID_STEP_DN = 0.075
    NEAR_STEP_UP = 0.05
    NEAR_STEP_DN = 0.05
    FINE_STEP_UP = 0.01
    FINE_STEP_DN = 0.01

    # -------------------------------------------------------------------------------------------------
    # CHARGE/MODE DETECTION (with hysteresis so mode does not flap on 0.01-0.03V noise)
    # -------------------------------------------------------------------------------------------------
    FULL_I = 3.5  # Current out when battery is full

    CONSTV_ENTER_EPS = 0.05  # enter CONST V when OUTV >= VSET - 0.03
    CONSTV_EXIT_EPS = 0.08   # exit CONST V only when OUTV <= VSET - 0.08

    mode_is_constv = False  # sticky state

    if background_mode:
        time.sleep(STARTUP_DELAY_S)

    with serial.Serial(port, 115200, timeout=0.05, write_timeout=0.25) as ser:
        # Capture RIDEN starting state and restore on exit (Ctrl-C / exceptions included).
        start_vset = None
        start_iset = None
        start_on = None

        try:
            try:
                vset0, iset0, outv0, iout0, pout0, vin0, on0 = read_status(ser, slave)
                start_vset = vset0
                start_iset = iset0
                start_on = bool(on0)
            except Exception as e:
                print(f"ERR initial read_status: {e}")

            # We run with output enabled, but we will restore original ON state on exit.
            try:
                set_output(ser, slave, True)
            except Exception as e:
                print(f"ERR set_output: {e}")

            target_vin = vtarget
            shutdown_start = None
            vset_base = None
            vset_reduced = False

            iset_cmd = None  # last command we intended (kept stable even if readback jitters)

            print(f"START TARGET_VIN={target_vin:0.2f}V (default={DEFAULT_TARGET_VIN:0.2f}V)")

            while True:
                try:
                    vset, iset, outv, iout, pout, vin, on = read_status(ser, slave)
                except Exception as e:
                    print(f"ERR read_status: {e}")
                    time.sleep(0.5)
                    continue

                if not on:
                    time.sleep(1.0)
                    continue

                if vset_base is None:
                    vset_base = vset

                if iset_cmd is None:
                    iset_cmd = iset

                # Sticky CONST V detection with hysteresis.
                if mode_is_constv:
                    if outv <= (vset - CONSTV_EXIT_EPS):
                        mode_is_constv = False
                else:
                    if outv >= (vset - CONSTV_ENTER_EPS):
                        mode_is_constv = True

                if mode_is_constv and (iout < FULL_I):
                    status_now = STATE_BATT_FULL
                elif mode_is_constv:
                    status_now = STATE_CONST_V
                else:
                    status_now = STATE_MAX_R

                # SHUTDOWN LAPTOP AT END OF THE DAY (only in background mode)
                if background_mode:
                    now = datetime.now()
                    after_sunset = now.hour >= SHUTDOWN_AFTER_HOUR

                    if after_sunset and (pout < SHUTDOWN_WATTS):
                        if shutdown_start is None:
                            shutdown_start = time.time()
                    else:
                        shutdown_start = None

                    if shutdown_start is not None and (time.time() - shutdown_start) >= SHUTDOWN_HOLD_S:
                        try:
                            set_iset(ser, slave, _quant_amps(clamp(I_UNATTENDED, I_MIN, I_MAX)))  # set RIDEN for best without controller
                        except Exception:
                            pass
                        _shutdown_windows_now()
                        return

                # HARD CLOUD COLLAPSE:
                if vin < (target_vin - HARD_DROP):
                    new_iset = _quant_amps(clamp(iset * 0.7, I_MIN, I_MAX))
                    try:
                        set_iset(ser, slave, new_iset)
                        iset_cmd = new_iset
                    except Exception as e:
                        print(f"ERR set_iset: {e}")
                        time.sleep(0.5)
                        continue

                    # VSET DROP/RESTORE FEATURE
                    if status_now == STATE_BATT_FULL:
                        if not vset_reduced:
                            vset_base = vset
                            vset_full = _quant_volts(max(0.0, vset_base - VSET_DROP_FULL))
                            try:
                                set_vset(ser, slave, vset_full)
                                vset_reduced = True
                            except Exception:
                                pass
                    else:
                        if vset_reduced:
                            try:
                                set_vset(ser, slave, _quant_volts(vset_base))
                            except Exception:
                                pass
                            vset_reduced = False
                            vset_base = vset
                        else:
                            vset_base = vset

                    print(
                        f"TGT={target_vin:5.2f}  "
                        f"FAST DROP  VIN={vin:5.2f}  ISET {iset:5.2f}->{new_iset:5.2f}  "
                        f"STAT={status_now}"
                    )
                    time.sleep(FAST_DELAY)
                    continue

                # -------------------------------------------------------------------------------------------------
                # NORMAL CONTROL (5-level steps)
                # -------------------------------------------------------------------------------------------------
                err = vin - target_vin
                abs_err = -err if err < 0.0 else err
                iceil = _quant_amps(clamp(iout + I_OVER, I_MIN, I_MAX))

                if mode_is_constv:
                    step_up = FINE_STEP_UP
                    step_dn = FINE_STEP_DN
                    band = "FINE"
                else:
                    if abs_err > V4:
                        step_up = HUGE_STEP_UP
                        step_dn = HUGE_STEP_DN
                        band = "HUGE"
                    elif abs_err > V3:
                        step_up = FAR_STEP_UP
                        step_dn = FAR_STEP_DN
                        band = "FAR "
                    elif abs_err > V2:
                        step_up = MID_STEP_UP
                        step_dn = MID_STEP_DN
                        band = "MID "
                    elif abs_err > V1:
                        step_up = NEAR_STEP_UP
                        step_dn = NEAR_STEP_DN
                        band = "NEAR"
                    else:
                        step_up = FINE_STEP_UP
                        step_dn = FINE_STEP_DN
                        band = "FINE"

                step_up = _quant_amps(step_up)
                step_dn = _quant_amps(step_dn)

                if err > VIN_BAND:
                    new_iset = iset_cmd + step_up
                elif err < -VIN_BAND:
                    new_iset = iset_cmd - step_dn
                else:
                    new_iset = iset_cmd

                new_iset = _quant_amps(clamp(new_iset, I_MIN, I_MAX))

                if new_iset > iceil:
                    new_iset = iceil

                if new_iset != iset_cmd:
                    try:
                        set_iset(ser, slave, new_iset)
                        iset_cmd = new_iset
                    except Exception as e:
                        print(f"ERR set_iset: {e}")
                        time.sleep(0.5)
                        continue

                # VSET DROP/RESTORE FEATURE
                if status_now == STATE_BATT_FULL:
                    if not vset_reduced:
                        vset_base = vset
                        vset_full = _quant_volts(max(0.0, vset_base - VSET_DROP_FULL))
                        try:
                            set_vset(ser, slave, vset_full)
                            vset_reduced = True
                        except Exception as e:
                            print(f"ERR set_vset: {e}")
                else:
                    if vset_reduced:
                        try:
                            set_vset(ser, slave, _quant_volts(vset_base))
                        except Exception as e:
                            print(f"ERR set_vset: {e}")
                        vset_reduced = False
                        vset_base = vset
                    else:
                        vset_base = vset

                print(
                    f"VSET={vset:5.2f}V "
                    f"VOUT={outv:5.2f}V "
                    f"IOUT={iout:5.2f}A  | "
                    f"PWR={pout:5.1f}W  "
                    f"VIN={vin:5.2f}  "
                    f"ISET={iset_cmd:5.2f}  "
                    f"STEP={step_up:2.2f}/{step_dn:2.2f}  "
                    f"{band}  "
                    f"{status_now}"
                )

                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            # Ctrl-C: normal exit path; restore below in finally.
            return
        finally:
            # Restore RIDEN values captured at program start (best-effort).
            # Note: do not print success logs; only errors.
            try:
                if start_vset is not None:
                    set_vset(ser, slave, _quant_volts(start_vset))
            except Exception as e:
                print(f"ERR restore VSET: {e}")

            try:
                if start_iset is not None:
                    # set_iset(ser, slave, _quant_amps(start_iset))
                    set_iset(ser, slave, _quant_amps(5.0))
            except Exception as e:
                print(f"ERR restore ISET: {e}")

            try:
                if start_on is not None:
                    set_output(ser, slave, bool(start_on))
            except Exception as e:
                print(f"ERR restore OUTPUT: {e}")


if __name__ == "__main__":
    bg = "--bg" in sys.argv

    vtarget = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TARGET_VIN
    port = sys.argv[2] if len(sys.argv) > 2 else "COM4"
    slave = int(sys.argv[3]) if len(sys.argv) > 3 else 1

    mppt_loop(vtarget, port, slave, bg)
