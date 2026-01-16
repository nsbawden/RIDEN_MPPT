import sys
import time
import serial


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
    pdu = bytes([slave, 0x03, (start >> 8) & 0xFF, start & 0xFF, (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    rx = _txrx(ser, frame, 5 + 2 * qty + 2)
    if len(rx) < 5 or rx[0] != slave or rx[1] != 0x03 or rx[2] != 2 * qty:
        raise RuntimeError(f"Bad read response: {rx.hex(' ')}")

    regs: list[int] = []
    for i in range(qty):
        regs.append((rx[3 + 2 * i] << 8) | rx[3 + 2 * i + 1])
    return regs


def write_single_reg(ser: serial.Serial, slave: int, addr: int, value: int) -> None:
    value &= 0xFFFF
    pdu = bytes([slave, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    rx = _txrx(ser, frame, 8)
    if len(rx) != 8 or rx[:6] != pdu:
        raise RuntimeError(f"Bad write response: {rx.hex(' ')}")


def read_status(ser: serial.Serial, slave: int = 1) -> dict:
    base = 0x0008
    r = read_holding_regs(ser, slave, base, 8)

    vset = r[0] / 100.0
    iset = r[1] / 100.0
    outv = r[2] / 100.0
    iout = r[3] / 100.0
    pout = r[5] / 100.0
    vin = r[6] / 100.0

    f = read_holding_regs(ser, slave, 0x0011, 2)
    on = int((f[0] != 0) or (f[1] != 0))

    return {"vset": vset, "iset": iset, "outv": outv, "iout": iout, "pout": pout, "vin": vin, "on": on}


def set_vset(ser: serial.Serial, slave: int, volts: float) -> None:
    write_single_reg(ser, slave, 0x0008, int(round(volts * 100.0)))


def set_iset(ser: serial.Serial, slave: int, amps: float) -> None:
    write_single_reg(ser, slave, 0x0009, int(round(amps * 100.0)))


def set_output(ser: serial.Serial, slave: int, enabled: bool) -> None:
    v = 1 if enabled else 0
    write_single_reg(ser, slave, 0x0011, v)
    write_single_reg(ser, slave, 0x0012, v)


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    cmd = sys.argv[3].lower() if len(sys.argv) > 3 else "read"

    with serial.Serial(
        port=port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.25,
        write_timeout=0.25,
    ) as ser:
        if cmd == "read":
            s = read_status(ser, slave)
            print(
                f"VIN={s['vin']:.2f}V  VSET={s['vset']:.2f}V  ISET={s['iset']:.2f}A  "
                f"OUTV={s['outv']:.2f}V  IOUT={s['iout']:.2f}A  POUT={s['pout']:.2f}W  ON={s['on']}"
            )
            return 0

        if cmd == "watch":
            while True:
                s = read_status(ser, slave)
                print(
                    f"VIN={s['vin']:.2f}  OUT={s['outv']:.2f}V {s['iout']:.2f}A  P={s['pout']:.2f}W  "
                    f"SET={s['vset']:.2f}V {s['iset']:.2f}A  ON={s['on']}"
                )
                time.sleep(0.5)
            return 0

        if cmd == "vset":
            if len(sys.argv) < 5:
                raise RuntimeError("Usage: vset <volts>")
            set_vset(ser, slave, float(sys.argv[4]))
            return 0

        if cmd == "iset":
            if len(sys.argv) < 5:
                raise RuntimeError("Usage: iset <amps>")
            set_iset(ser, slave, float(sys.argv[4]))
            return 0

        if cmd == "on":
            set_output(ser, slave, True)
            return 0

        if cmd == "off":
            set_output(ser, slave, False)
            return 0

        raise RuntimeError("Commands: read | watch | vset <V> | iset <A> | on | off")


if __name__ == "__main__":
    raise SystemExit(main())
