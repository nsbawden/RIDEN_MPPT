import time
import serial
import sys
import subprocess
import os
from datetime import datetime
import ctypes
import ctypes.wintypes

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

DEFAULT_TARGET_VIN = 33.0  # volts; user provided target is treated as the fixed target when SAG is disabled
V_UNATTENDED = 15.0  # Best set volts for RIDEN to use if MPPT controller is inactive (laptop off)
I_UNATTENDED = 15.0  # Best amps for RIDEN to use if MPPT controller is inactive (laptop off)
START_VSET = 15.0 # Initial vset
START_ISET = 0.01 # Initial iset

# -------------------------------------------------------------------------------------------------
# STATE NAMES (change in ONE place)
# -------------------------------------------------------------------------------------------------
STATE_BATT_FULL = "BATT F"
STATE_CONST_V = "CONST V"
STATE_MAX_R = "MAX R"

# -------------------------------------------------------------------------------------------------
# SINGLE-INSTANCE CONTROL (FG takes over from BG)
# -------------------------------------------------------------------------------------------------

KERNEL32 = ctypes.WinDLL("kernel32", use_last_error=True)

_CreateMutexW = KERNEL32.CreateMutexW
_CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.wintypes.BOOL, ctypes.wintypes.LPCWSTR)
_CreateMutexW.restype = ctypes.wintypes.HANDLE

_OpenMutexW = KERNEL32.OpenMutexW
_OpenMutexW.argtypes = (ctypes.wintypes.DWORD, ctypes.wintypes.BOOL, ctypes.wintypes.LPCWSTR)
_OpenMutexW.restype = ctypes.wintypes.HANDLE

_ReleaseMutex = KERNEL32.ReleaseMutex
_ReleaseMutex.argtypes = (ctypes.wintypes.HANDLE,)
_ReleaseMutex.restype = ctypes.wintypes.BOOL

_CloseHandle = KERNEL32.CloseHandle
_CloseHandle.argtypes = (ctypes.wintypes.HANDLE,)
_CloseHandle.restype = ctypes.wintypes.BOOL

_GetLastError = KERNEL32.GetLastError

ERROR_ALREADY_EXISTS = 183
MUTEX_MODIFY_STATE = 0x0001

# "Global\" allows cross-session visibility (Task Scheduler vs interactive logon).
# If that ever causes permission issues, change to "Local\".
MPPT_MUTEX_NAME = "Global\\RIDEN_MPPT_OWNER_MUTEX"


def mppt_take_ownership_or_exit(background_mode: bool) -> bool:
    # Background behavior: if someone else already owns it, exit quietly.
    # Foreground behavior: if background owns it, force it to release so FG can proceed.
    #
    # Note: we do not log successes; only errors.

    h_owner = _CreateMutexW(None, False, MPPT_MUTEX_NAME)
    if not h_owner:
        print("ERR CreateMutexW failed")
        return False

    already_exists = (_GetLastError() == ERROR_ALREADY_EXISTS)
    if not already_exists:
        # First instance; keep the handle open for lifetime of process.
        return True

    if background_mode:
        # BG should not steal; exit.
        try:
            _CloseHandle(h_owner)
        except Exception:
            pass
        return False

    # FG: attempt to force release by opening the existing mutex and releasing it.
    # This is slightly "aggressive", but it's what you want: FG wins.
    h_existing = _OpenMutexW(MUTEX_MODIFY_STATE, False, MPPT_MUTEX_NAME)
    if not h_existing:
        # If we can't open it, we still proceed (FG) because CreateMutex succeeded.
        # Worst case: BG continues too; not ideal, but avoids blocking.
        print("ERR OpenMutexW failed")
        return True

    try:
        # If BG owns it, this will release it (ownership semantics are kernel-managed).
        _ReleaseMutex(h_existing)
    except Exception:
        pass

    try:
        _CloseHandle(h_existing)
    except Exception:
        pass

    return True


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

        # Read first 2 bytes to determine response type.
        hdr = _read_exact(ser, 2, timeout_s)  # [slave][func]
        if len(hdr) != 2:
            continue
        if hdr[0] != slave:
            continue

        func = hdr[1]

        if func == (0x06 | 0x80):
            # Exception response is 5 bytes total: slave, func|0x80, exc_code, crc_lo, crc_hi
            tail = _read_exact(ser, 3, timeout_s)
            rx = hdr + tail
            if len(rx) != 5:
                continue
            if not _crc_ok(rx):
                continue
            exc = rx[2]
            raise RuntimeError(f"Modbus exception write code=0x{exc:02X}")

        if func != 0x06:
            continue

        # Normal response is 8 bytes total: slave, func, addr_hi, addr_lo, val_hi, val_lo, crc_lo, crc_hi
        tail = _read_exact(ser, 6, timeout_s)
        rx = hdr + tail
        if len(rx) != 8:
            continue
        if not _crc_ok(rx):
            continue

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

def _mppt_exit(ser, slave: int) -> None:
    # Restore RIDEN values.
    vset = V_UNATTENDED
    iset = I_UNATTENDED
    
    now = datetime.now()
    after_noon = now.hour >= 12
    
    if (after_noon): vset = 14.4
    
    try:
        set_vset(ser, slave, _quant_volts(vset))
    except Exception as e:
        print(f"ERR restore VSET: {e}")

    try:
        set_iset(ser, slave, _quant_amps(iset))
    except Exception as e:
        print(f"ERR restore ISET: {e}")



# -------------------------------------------------------------------------------------------------
# CSV LOGGING (single definition point)
# -------------------------------------------------------------------------------------------------

def _csv_escape(s: str) -> str:
    if s is None:
        return ""
    if any(c in s for c in [",", "\"", "\n", "\r"]):
        return "\"" + s.replace("\"", "\"\"") + "\""
    return s


def _log_file_name(now: datetime) -> str:
    # MPPT_LOG_YYYY_MM_DD.csv
    return f"MPPT_LOG_{now.year:04d}_{now.month:02d}_{now.day:02d}.csv"


# Edit the log by changing ONLY this list (fields) and the mapping in _log_get_value().
LOG_FIELDS = [
    "ts_local",
    "bg",
    "status",
    "band",
    "target_vin",
    "vin",
    "vset",
    "outv",
    "iout",
    "pout",
]


def _log_get_value(field: str, ctx: dict) -> str:
    # Single mapping point for all fields. Add/remove fields here only.
    # ctx contains only current "in use" values.
    v = ctx.get(field, "")
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return f"{v:.2f}"
    return str(v)


def _log_write_if_due(last_log_t: float, log_delay_s: float, ctx: dict) -> float:
    # One place that:
    #  - chooses daily file name
    #  - writes header if needed
    #  - writes one row
    if (time.time() - last_log_t) < log_delay_s:
        return last_log_t

    now = datetime.now()
    path = _log_file_name(now)

    try:
        need_header = (not os.path.exists(path)) or (os.path.getsize(path) == 0)
    except Exception:
        need_header = True

    try:
        with open(path, "a", encoding="utf-8", newline="") as f:
            if need_header:
                f.write(",".join(_csv_escape(x) for x in LOG_FIELDS) + "\n")
            row = [_log_get_value(k, ctx) for k in LOG_FIELDS]
            f.write(",".join(_csv_escape(x) for x in row) + "\n")
            f.flush()
    except Exception as e:
        print(f"ERR log write: {e}")

    return time.time()


def mppt_loop(vtarget: float, port: str, slave: int, background_mode: bool) -> None:
    # ---------------------------------------------------------------------------------------------
    # CONSTANT-PV-VOLTAGE CONTROL LOOP (cloud-resistant "software MPPT")
    # ---------------------------------------------------------------------------------------------

    SHUTDOWN_AFTER_HOUR = 17  # 5pm
    SHUTDOWN_WATTS = 45.0     # approx laptop draw threshold
    SHUTDOWN_HOLD_S = 300     # seconds below threshold before shutdown (5 min)

    VSET_DROP_BATT_FULL = 0.60  # volts to reduce VSET when in BATT F state

    VIN_BAND = 0.00  # volts; deadband around target to avoid constant chatter
    HARD_DROP = 5.0  # volts; collapse trigger (VIN far below target => emergency backoff)

    I_MIN = 0.01
    I_MAX = 24.0  # amps; your chosen maximum current pull
    I_OVER = 0.5  # how much ISET can exceed IOUT

    LOOP_DELAY = 0.20  # seconds; always fast (single loop rate)
    LOG_DELAY = 5.0    # seconds

    STARTUP_DELAY_S = 8.0  # background wake: give USB/CH340 time to enumerate

    # -------------------------------------------------------------------------------------------------
    # HCC-OFFSET TARGET CONTROL
    # -------------------------------------------------------------------------------------------------
    HCC_STEP_V = 0.10              # volts per adjustment; smaller = slower/smoother knee search
    HCC_OFFSET_MIN_V = -2.00       # max aggressive drift (T_USER - 2V); limits how close to knee we probe
    HCC_OFFSET_MAX_V =  2.00       # max conservative drift (T_USER + 2V); limits how far we back off
    HCC_QUIET_S = 90.0             # seconds with no HCC events before drifting more aggressive
    HCC_DECAY_STABLE_EPS = 0.50    # VIN must be within this of target before allowing aggressive drift

    # -------------------------------------------------------------------------------------------------
    # SAFE POST-HCC RECOVERY ACCELERATION (NO JUMPS)
    # -------------------------------------------------------------------------------------------------
    # After an HCC collapse, VIN often rebounds very high (unloaded PV). We can climb ISET faster
    # for a short time, but still monotonically and still respecting band logic.
    RECOVERY_WINDOW_S = 6.0   # accelerate only briefly after leaving HCC
    RECOVERY_ERR_V = 1.50     # only when VIN is > target + this
    RECOVERY_GAIN = 2.0       # multiply step_up by this during recovery window (bounded below)
    RECOVERY_MIN_POUT_W = 100.0  # only allow boosted recovery when PV is actually delivering power

    # -------------------------------------------------------------------------------------------------
    # 5-LEVEL STEP DIVISIONS
    # -------------------------------------------------------------------------------------------------
    V1 = 0.25  # fine/near boundary
    V2 = 0.50  # near/mid boundary
    V3 = 1.00  # mid/far boundary
    V4 = 1.50  # far/huge boundary

    HUGE_STEP_UP = 0.80
    HUGE_STEP_DN = 0.80
    FAR_STEP_UP = 0.20
    FAR_STEP_DN = 0.20
    MID_STEP_UP = 0.10
    MID_STEP_DN = 0.10
    NEAR_STEP_UP = 0.05
    NEAR_STEP_DN = 0.05
    FINE_STEP_UP = 0.01
    FINE_STEP_DN = 0.01

    # -------------------------------------------------------------------------------------------------
    # CHARGE/MODE DETECTION (with hysteresis)
    # -------------------------------------------------------------------------------------------------
    FULL_I = 4.00                 # Current out when battery is full (laptop etc. loads)
    REDUCED_DISABLE_I = FULL_I + 8.0

    CONSTV_ENTER_EPS = 0.05
    CONSTV_EXIT_EPS = 0.10

    # "High current" is used only for your VSET drop gating (unchanged behavior).
    HIGH_CURRENT_I = FULL_I + 8.0  # 12A default

    is_v_limited = False  # sticky state

    if background_mode:
        time.sleep(STARTUP_DELAY_S)

    last_log_t = 0.0

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

            # Start with fixed vset.
            try:
                vset = START_VSET
                set_vset(ser, slave, _quant_volts(vset))
                time.sleep(0.1)
            except Exception as e:
                print(f"ERR initializing VSET: {e}")

            # Start with fixed iset (very low).
            try:
                iset = START_ISET
                set_iset(ser, slave, _quant_amps(iset))  # amps, not volts
                time.sleep(0.1)
            except Exception as e:
                print(f"ERR initializing ISET: {e}")

            # Base target is user-provided.
            T_USER = float(vtarget)

            # HCC offset state.
            hcc_offset_v = 0.0
            last_no_hcc_t = time.time()  # time anchor for "quiet" stability probe-down
            doing_hcc = False               # True while VIN is in "HCC zone"
            first_hcc_ignored = False    # ignore the first HCC episode only

            # Track exit time to enable brief post-HCC acceleration.
            last_hcc_exit_t = -1e9

            # Start with fixed target derived from base + offset.
            target_vin = _quant_volts(T_USER + hcc_offset_v)

            shutdown_start = None
            vset_base = None
            vset_reduced = False

            iset_cmd = None  # last command we intended (kept stable even if readback jitters)

            print(
                f"START T_USER={T_USER:0.2f}V  "
                f"(start TARGET_VIN={target_vin:0.2f}V)  LOOP={LOOP_DELAY:0.2f}s"
            )

            # -------------------------------------------------------------------------------------------------
            # MAIN LOOP
            # -------------------------------------------------------------------------------------------------
            while True:
                try:
                    vset, iset, outv, iout, pout, vin, on = read_status(ser, slave)
                except Exception as e:
                    print(f"ERR read_status: {e}")
                    time.sleep(0.5)
                    continue

                if vset_base is None:
                    vset_base = vset

                if iset_cmd is None:
                    iset_cmd = iset

                # Sticky CONST V detection with hysteresis.
                if is_v_limited:
                    if outv <= (vset - CONSTV_EXIT_EPS):
                        is_v_limited = False
                else:
                    if outv >= (vset - CONSTV_ENTER_EPS):
                        is_v_limited = True

                # Classification flags.
                is_soaking = bool(is_v_limited and (iout < FULL_I))
                is_high_current = bool(iout >= HIGH_CURRENT_I)
                is_max_r = bool(not is_v_limited)

                if is_soaking:
                    status_now = STATE_BATT_FULL
                elif is_v_limited:
                    status_now = STATE_CONST_V
                else:
                    status_now = STATE_MAX_R

                # -------------------------------------------------------------------------------------------------
                # AUTO-SHUTDOWN
                # -------------------------------------------------------------------------------------------------
                now = datetime.now()
                after_sunset = now.hour >= SHUTDOWN_AFTER_HOUR

                if after_sunset and (pout < SHUTDOWN_WATTS):
                    if shutdown_start is None:
                        shutdown_start = time.time()
                else:
                    shutdown_start = None

                if shutdown_start is not None and (time.time() - shutdown_start) >= SHUTDOWN_HOLD_S:
                    _mppt_exit(ser, slave)
                    _shutdown_windows_now()
                    return

                # -------------------------------------------------------------------------------------------------
                # COMPUTE CURRENT TARGET FROM BASE + OFFSET WHEN NOT V LIMITED
                # -------------------------------------------------------------------------------------------------
                # IMPORTANT:
                # - We do NOT drift/adjust hcc_offset_v while in CONST V.
                # - While v-limited, we keep showing the last target_vin, but HCC drift logic is frozen.
                if not is_v_limited:
                    target_vin = _quant_volts(T_USER + hcc_offset_v)
                else:
                    last_no_hcc_t = time.time()  # keep HCC quiet timer fresh during CONST V

                # -------------------------------------------------------------------------------------------------
                # HCC (HARD CLOUD COLLAPSE) EPISODE DETECTION (edge-triggered)
                # -------------------------------------------------------------------------------------------------
                # IMPORTANT:
                # - Disable HCC episode detection while in CONST V, because we are no longer controlling VIN.
                # - This prevents hcc_offset_v from drifting while voltage-limited.
                if not is_v_limited:
                    hcc_trigger = bool(vin < (target_vin - HARD_DROP))
                else:
                    hcc_trigger = False

                if hcc_trigger and (not doing_hcc):
                    # Entering a new HCC episode (one fall off the cliff).
                    doing_hcc = True
                    last_no_hcc_t = time.time()  # reset quiet timer on episode start

                    if first_hcc_ignored:
                        hcc_offset_v = clamp(hcc_offset_v + HCC_STEP_V, HCC_OFFSET_MIN_V, HCC_OFFSET_MAX_V)
                    else:
                        first_hcc_ignored = True  # ignore the first episode only (startup normalization)

                    # Recompute target after offset bump (so prints and subsequent logic match).
                    target_vin = _quant_volts(T_USER + hcc_offset_v)

                if (not hcc_trigger) and doing_hcc:
                    # Exited the HCC episode.
                    doing_hcc = False
                    last_no_hcc_t = time.time()  # start quiet timer once we're back out
                    last_hcc_exit_t = time.time()

                # -------------------------------------------------------------------------------------------------
                # STABLE "PROBE DOWN" (bidirectional behavior)
                # -------------------------------------------------------------------------------------------------
                # IMPORTANT:
                # - Freeze probe-down while in CONST V (same reason: we are not controlling VIN there).
                now_t0 = time.time()
                if (not is_v_limited) and (not doing_hcc) and ((now_t0 - last_no_hcc_t) >= HCC_QUIET_S):
                    if vin >= (target_vin - HCC_DECAY_STABLE_EPS):
                        hcc_offset_v = clamp(hcc_offset_v - HCC_STEP_V, HCC_OFFSET_MIN_V, HCC_OFFSET_MAX_V)
                        last_no_hcc_t = now_t0
                        target_vin = _quant_volts(T_USER + hcc_offset_v)

                # -------------------------------------------------------------------------------------------------
                # CONTROL ERROR (VIN relative to target)
                # -------------------------------------------------------------------------------------------------
                band = ""
                step_used = 0.0
                new_iset = float(iset_cmd)
                iceil = _quant_amps(clamp(iout + I_OVER, I_MIN, I_MAX))
                err_vin = vin - target_vin
                abs_err_vin = -err_vin if err_vin < 0.0 else err_vin

                if hcc_trigger:
                    # -------------------------------------------------------------------------------------------------
                    # HARD CLOUD COLLAPSE:
                    # -------------------------------------------------------------------------------------------------
                    band = "FAST"
                    step_used = 0.0

                    new_iset = _quant_amps(clamp(iset_cmd * 0.7, I_MIN, I_MAX))

                    try:
                        set_iset(ser, slave, new_iset)
                        iset_cmd = new_iset
                    except Exception as e:
                        print(f"ERR set_iset: {e}")
                        time.sleep(0.5)
                        continue

                    print(
                        f"FAST RECOVER TGT={target_vin:5.2f} HCC={hcc_offset_v:+0.2f} "
                        f"IOUT={iout:5.2f}A "
                        f"PWR={pout:5.1f}W "
                        f"VIN={vin:5.2f} ISET {iset_cmd:5.2f}->{new_iset:5.2f} "
                        f"{status_now}"
                    )

                else:
                    # -------------------------------------------------------------------------------------------------
                    # NORMAL CONTROL
                    # -------------------------------------------------------------------------------------------------
                    if abs_err_vin > V4:
                        step_up = HUGE_STEP_UP
                        step_dn = HUGE_STEP_DN
                        band = "HUGE"
                    elif abs_err_vin > V3:
                        step_up = FAR_STEP_UP
                        step_dn = FAR_STEP_DN
                        band = "FAR "
                    elif abs_err_vin > V2:
                        step_up = MID_STEP_UP
                        step_dn = MID_STEP_DN
                        band = "MID "
                    elif abs_err_vin > V1:
                        step_up = NEAR_STEP_UP
                        step_dn = NEAR_STEP_DN
                        band = "NEAR"
                    else:
                        step_up = FINE_STEP_UP
                        step_dn = FINE_STEP_DN
                        band = "FINE"

                    # SAFE post-HCC acceleration (still monotonic; no jumps)
                    # Only for a short window after leaving HCC, and only when VIN is well above target.
                    if (time.time() - last_hcc_exit_t) <= RECOVERY_WINDOW_S:
                        if (err_vin > RECOVERY_ERR_V) and (pout >= RECOVERY_MIN_POUT_W):
                            step_up = min(step_up * RECOVERY_GAIN, HUGE_STEP_UP)

                    step_up = _quant_amps(step_up)
                    step_dn = _quant_amps(step_dn)

                    if err_vin > VIN_BAND:
                        new_iset = float(iset_cmd) + step_up
                        step_used = step_up
                    elif err_vin < -VIN_BAND:
                        new_iset = float(iset_cmd) - step_dn
                        step_used = -step_dn
                    else:
                        new_iset = float(iset_cmd)
                        step_used = 0.0

                    new_iset = _quant_amps(clamp(new_iset, I_MIN, I_MAX))

                    # Never command much more than what is actually flowing (+ margin) when voltage-limited.
                    if new_iset > iceil and is_v_limited:
                        new_iset = iceil
                        step_used = 9.99

                    if new_iset != iset_cmd:
                        try:
                            set_iset(ser, slave, new_iset)
                            iset_cmd = new_iset
                        except Exception as e:
                            print(f"ERR set_iset: {e}")
                            time.sleep(0.5)
                            continue

                    # VSET DROP/RESTORE FEATURE (unchanged logic, still current-gated)
                    if not is_high_current:
                        if is_soaking:
                            if not vset_reduced:
                                vset_base = vset
                                vset_full = _quant_volts(max(0.0, vset_base - VSET_DROP_BATT_FULL))
                                try:
                                    set_vset(ser, slave, vset_full)
                                    vset_reduced = True
                                except Exception as e:
                                    print(f"ERR set_vset: {e}")
                        else:
                            # Huge hysterisis required to come out of vset_reduced
                            if vset_reduced and iout > REDUCED_DISABLE_I:
                                try:
                                    set_vset(ser, slave, _quant_volts(vset_base))
                                except Exception as e:
                                    print(f"ERR set_vset: {e}")
                                vset_reduced = False
                                vset_base = vset
                            else:
                                vset_base = vset

                    hcc_tag = ""
                    if not is_v_limited:
                        hcc_tag = f" HCC={hcc_offset_v:+0.2f}"

                    print(
                        f"VSET={vset:5.2f}V "
                        f"OUT {outv:5.2f}V {iout:5.2f}A | "
                        f"PWR={pout:5.1f}W "
                        f"VIN={vin:5.2f} "
                        f"TGT={target_vin:5.2f} "
                        f"ISET={iset_cmd:5.2f} "
                        f"{step_used:+5.2f} "
                        f"{'[' + band + ']' if is_v_limited else ' ' + band + ' '} "
                        f"{status_now}"
                        f"{hcc_tag}"
                    )

                # -------------------------------------------------------------------------------------------------
                # LOGGING (ONE PLACE: end of loop)
                # -------------------------------------------------------------------------------------------------
                ctx = {
                    "ts_local": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "bg": 1 if background_mode else 0,
                    "status": status_now,
                    "band": band,
                    "target_vin": float(target_vin),
                    "vin": float(vin),
                    "vset": float(vset),
                    "outv": float(outv),
                    "iout": float(iout),
                    "pout": float(pout),
                }

                last_log_t = _log_write_if_due(last_log_t, LOG_DELAY, ctx)

                time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            return
        finally:
            _mppt_exit(ser, slave)


if __name__ == "__main__":
    bg = "--bg" in sys.argv

    if not mppt_take_ownership_or_exit(bg):
        sys.exit(0)

    vtarget = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_TARGET_VIN
    port = sys.argv[2] if len(sys.argv) > 2 else "COM4"
    slave = int(sys.argv[3]) if len(sys.argv) > 3 else 1

    mppt_loop(vtarget, port, slave, bg)
