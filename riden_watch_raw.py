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

def read_holding_regs(ser: serial.Serial, slave: int, start: int, qty: int) -> list[int]:
    pdu = bytes([slave, 0x03, (start >> 8) & 0xFF, start & 0xFF, (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)

    rx = ser.read(5 + 2 * qty + 2)
    if len(rx) < 5 or rx[0] != slave or rx[1] != 0x03 or rx[2] != 2 * qty:
        raise RuntimeError(f"Bad response: {rx.hex(' ')}")

    regs = []
    for i in range(qty):
        regs.append((rx[3 + 2*i] << 8) | rx[3 + 2*i + 1])
    return regs

def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    base = 0x0008

    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.25) as ser:
        prev = None
        while True:
            r = read_holding_regs(ser, slave, base, 8)
            # Print raw + scaled for the obvious ones
            vset = r[0] / 100.0
            iset = r[1] / 100.0
            outv = r[2] / 100.0
            iout = r[3] / 100.0
            pout = r[5] / 100.0
            vin  = r[6] / 100.0

            diff = ""
            if prev is not None:
                changes = [i for i in range(8) if r[i] != prev[i]]
                if changes:
                    diff = "  CHG:" + ",".join(str(i) for i in changes)

            print(
                f"RAW: " + " ".join(f"{x:5d}" for x in r) +
                f" | VIN={vin:5.2f} OUT={outv:5.2f}V {iout:5.2f}A P={pout:6.2f} "
                f"SET={vset:5.2f}V {iset:5.2f}A{diff}"
            )

            prev = r
            time.sleep(0.5)

if __name__ == "__main__":
    raise SystemExit(main())
