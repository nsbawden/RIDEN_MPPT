import time
import serial
import sys
import subprocess
from datetime import datetime

# -------------------------------------------------------------------------------------------------
# RIDEN RD6024 "SOFTWARE MPPT" CONTROLLER (Modbus RTU over USB-Serial / CH340)
#
# RUN
# py riden_mppt.py [target volts] [com port] [slave]
#
# TYPICAL (defaults as well)
# py riden_mppt.py 33 COM4 1
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
# IMPORTANT LESSON FROM YOUR LOG
# - When batteries were full, the auto finder "succeeded" incorrectly by returning ~Voc-ish VIN (≈35.9V).
# - That happened because the "batteries full / CV-limited" detection was too weak.
# - Fix: strong CV-limited detection + "must observe meaningful VIN response" before accepting a knee.
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
    subprocess.run(["shutdown", "/s", "/t", "0"], check=False)


def mppt_loop(vtarget: float, port: str, slave: int) -> None:
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

    FAR_V = 1.0
    MID_V = 0.5

    FAR_STEP_UP = 0.50
    FAR_STEP_DN = 1.00
    MID_STEP_UP = 0.05
    MID_STEP_DN = 0.05
    FINE_STEP_UP = 0.01
    FINE_STEP_DN = 0.01

    # -------------------------------------------------------------------------------------------------
    # CHARGE/MODE DETECTION (with hysteresis so mode does not flap on 0.01-0.03V noise)
    # -------------------------------------------------------------------------------------------------
    FULL_I = 3.5 # Current out when battery is full

    CONSTV_ENTER_EPS = 0.03  # enter CONST V when OUTV >= VSET - 0.03
    CONSTV_EXIT_EPS = 0.08   # exit CONST V only when OUTV <= VSET - 0.08

    mode_is_constv = False  # sticky state

    # -------------------------------------------------------------------------------------------------
    # SUN "SLOPE" PROBE (ONLY valid in MAX R)
    #
    # Step ISET DOWN (safe) and watch VIN rise + IOUT fall.
    # SL = |ΔIOUT / ΔVIN| in A/V (higher => stronger sun).
    # -------------------------------------------------------------------------------------------------

    PROBE_ENABLE = False
    PROBE_PERIOD_S = 2.0
    PROBE_POLL_S = 0.2

    PROBE_DI_MIN = 0.05
    PROBE_DI_MAX = 2.00
    PROBE_DI = 0.20

    PROBE_DV_MIN = 0.03
    PROBE_DV_MAX = 0.18
    PROBE_IOUT_MIN = 0.10

    SL_ALPHA = 0.20

    # SL scaling (MID and FINE only; FAR is not scaled)
    SL_REF = 0.5        # with SL ~4..5, scale ~8..10
    SL_SCALE_MIN = 1.0
    SL_SCALE_MAX = 3.0

    sl = 0.0
    last_probe_t = 0.0

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

            iset_cmd = None  # last command we *intended* (used to restore after probe)

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

                # SHUTDOWN LAPTOP AT END OF THE DAY
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

                    iceil = _quant_amps(clamp(iout + I_OVER, I_MIN, I_MAX))

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
                        f"SL={sl:5.2f}  "
                        f"STAT={status_now}"
                    )
                    time.sleep(FAST_DELAY)
                    continue

                # -------------------------------------------------------------------------------------------------
                # PROBE (ONLY in MAX R; never in CONST V)
                # -------------------------------------------------------------------------------------------------
                if PROBE_ENABLE and iout > 1.0 and (status_now == STATE_MAX_R) and ((time.time() - last_probe_t) >= PROBE_PERIOD_S):
                    if iset_cmd > (I_MIN + PROBE_DI_MIN):
                        iset0 = iset_cmd

                        di = PROBE_DI
                        if di > (iset0 - I_MIN):
                            di = iset0 - I_MIN
                        di = _quant_amps(clamp(di, PROBE_DI_MIN, PROBE_DI_MAX))

                        iset_down = _quant_amps(clamp(iset0 - di, I_MIN, I_MAX))
                        try:
                            set_iset(ser, slave, iset_down)
                        except Exception as e:
                            print(f"ERR set_iset (probe down): {e}")
                            time.sleep(0.5)
                            continue

                        time.sleep(PROBE_POLL_S)

                        try:
                            vset_p1, iset_p1, outv_p1, iout_p1, pout_p1, vin_p1, on_p1 = read_status(ser, slave)
                        except Exception as e:
                            print(f"ERR read_status (probe down): {e}")
                            time.sleep(0.5)
                            continue

                        try:
                            set_iset(ser, slave, iset0)
                        except Exception as e:
                            print(f"ERR set_iset (probe restore): {e}")
                            time.sleep(0.5)
                            continue

                        time.sleep(PROBE_POLL_S)

                        try:
                            vset_p2, iset_p2, outv_p2, iout_p2, pout_p2, vin_p2, on_p2 = read_status(ser, slave)
                        except Exception as e:
                            print(f"ERR read_status (probe restore): {e}")
                            time.sleep(0.5)
                            continue

                        iset_cmd = iset0
                        last_probe_t = time.time()

                        dv = vin_p2 - vin_p1
                        diout = iout_p2 - iout_p1

                        if dv < 0.0:
                            dv = -dv
                        if diout < 0.0:
                            diout = -diout

                        if (dv > 0.0001) and (diout >= PROBE_IOUT_MIN):
                            s_now = diout / dv
                            sl += (s_now - sl) * SL_ALPHA

                        dv_probe = vin_p2 - vin_p1
                        if dv_probe < 0.0:
                            dv_probe = -dv_probe

                        if dv_probe < PROBE_DV_MIN:
                            PROBE_DI = clamp(PROBE_DI * 1.5, PROBE_DI_MIN, PROBE_DI_MAX)
                        elif dv_probe > PROBE_DV_MAX:
                            PROBE_DI = clamp(PROBE_DI * 0.67, PROBE_DI_MIN, PROBE_DI_MAX)

                # -------------------------------------------------------------------------------------------------
                # NORMAL CONTROL (SL scaling only in MAX R)
                # -------------------------------------------------------------------------------------------------
                err = vin - target_vin
                abs_err = -err if err < 0.0 else err
                iceil = _quant_amps(clamp(iout + I_OVER, I_MIN, I_MAX))

                # if status_now == STATE_MAX_R:
                #     sl_scale = sl / SL_REF if SL_REF > 0.0 else 1.0
                #     sl_scale = clamp(sl_scale, SL_SCALE_MIN, SL_SCALE_MAX)
                # else:
                #     sl_scale = 1.0
                    
                sl_scale = 1.0

                if abs_err > FAR_V:
                    step_up = FAR_STEP_UP
                    step_dn = FAR_STEP_DN
                elif abs_err > MID_V:
                    step_up = MID_STEP_UP * sl_scale
                    step_dn = MID_STEP_DN * sl_scale
                else:
                    step_up = FINE_STEP_UP * sl_scale
                    step_dn = FINE_STEP_DN * sl_scale

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
                    # f"TGT={target_vin:5.2f} "
                    f"VSET={vset:5.2f}V "
                    f"VOUT={outv:5.2f}V "
                    f"IOUT={iout:5.2f}A  | "
                    f"PWR={pout:5.1f}W  "
                    f"VIN={vin:5.2f}  "
                    f"ISET={iset_cmd:5.2f}  "
                    # f"SL={sl:5.2f}*{sl_scale:4.2f}  "
                    f"SU={step_up:2.2f}  "
                    f"SD={step_dn:2.2f}  "
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
                    set_iset(ser, slave, _quant_amps(start_iset))
            except Exception as e:
                print(f"ERR restore ISET: {e}")

            try:
                if start_on is not None:
                    set_output(ser, slave, bool(start_on))
            except Exception as e:
                print(f"ERR restore OUTPUT: {e}")


if __name__ == "__main__":
    vtarget = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TARGET_VIN
    port = sys.argv[2] if len(sys.argv) > 2 else "COM4"
    slave = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    mppt_loop(vtarget, port, slave)
