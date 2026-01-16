import time
import serial
import sys


# ------------------ Modbus helpers ------------------

def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _txrx(ser: serial.Serial, frame: bytes, rx_len: int, delay_s: float = 0.05) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(delay_s)
    return ser.read(rx_len)


def read_holding_regs(ser: serial.Serial, slave: int, start: int, qty: int) -> list[int]:
    pdu = bytes([slave, 0x03,
                 (start >> 8) & 0xFF, start & 0xFF,
                 (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    rx = _txrx(ser, frame, 5 + 2 * qty + 2)
    if len(rx) < 5 or rx[1] != 0x03:
        raise RuntimeError("Bad Modbus read")

    regs = []
    for i in range(qty):
        regs.append((rx[3 + 2*i] << 8) | rx[3 + 2*i + 1])
    return regs


def write_single_reg(ser: serial.Serial, slave: int, addr: int, value: int) -> None:
    value &= 0xFFFF
    pdu = bytes([slave, 0x06,
                 (addr >> 8) & 0xFF, addr & 0xFF,
                 (value >> 8) & 0xFF, value & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    rx = _txrx(ser, frame, 8)
    if len(rx) != 8 or rx[:6] != pdu:
        raise RuntimeError("Bad Modbus write")


# ------------------ RD6024 register map ------------------

REG_BASE   = 0x0008
REG_VSET   = 0x0008
REG_ISET   = 0x0009
REG_OUTV   = 0x000A
REG_IOUT   = 0x000B
REG_POUT   = 0x000D
REG_VIN    = 0x000E

REG_ON1    = 0x0011
REG_ON2    = 0x0012


# ------------------ High-level helpers ------------------

def read_status(ser, slave):
    r = read_holding_regs(ser, slave, REG_BASE, 8)

    vset = r[0] / 100.0
    iset = r[1] / 100.0
    outv = r[2] / 100.0
    iout = r[3] / 100.0
    pout = r[5] / 100.0
    vin  = r[6] / 100.0

    f = read_holding_regs(ser, slave, REG_ON1, 2)
    on = int((f[0] != 0) or (f[1] != 0))

    return vset, iset, outv, iout, pout, vin, on


def set_iset(ser, slave, amps):
    write_single_reg(ser, slave, REG_ISET, int(round(amps * 100.0)))


def set_output(ser, slave, enabled):
    v = 1 if enabled else 0
    write_single_reg(ser, slave, REG_ON1, v)
    write_single_reg(ser, slave, REG_ON2, v)


# ------------------ MPPT logic ------------------

def mppt_loop(port="COM4", slave=1):
    TARGET_PANEL_V = 30.0      # volts – adjust for your array
    I_STEP         = 0.05      # amps per step
    I_MIN          = 0.0
    I_MAX          = 18.0      # your chosen hard ceiling
    LOOP_DELAY     = 0.5       # seconds

    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.25) as ser:

        set_output(ser, slave, True)

        direction = 1
        last_power = 0.0

        while True:
            vset, iset, outv, iout, pout, vin, on = read_status(ser, slave)

            if not on:
                time.sleep(1.0)
                continue

            # --- panel voltage clamp logic ---
            if vin < TARGET_PANEL_V:
                direction = -1
            elif vin > TARGET_PANEL_V + 0.5:
                direction = 1

            # --- perturb & observe on real power ---
            if pout < last_power:
                direction = -direction

            new_iset = iset + direction * I_STEP
            if new_iset < I_MIN:
                new_iset = I_MIN
            if new_iset > I_MAX:
                new_iset = I_MAX

            set_iset(ser, slave, new_iset)
            last_power = pout

            print(
                f"VIN={vin:5.2f}  "
                f"OUT={outv:5.2f}V {iout:5.2f}A  "
                f"P={pout:6.1f}W  "
                f"ISET={new_iset:5.2f}"
            )

            time.sleep(LOOP_DELAY)


# ------------------ Entry point ------------------

if __name__ == "__main__":
    port  = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    mppt_loop(port, slave)
