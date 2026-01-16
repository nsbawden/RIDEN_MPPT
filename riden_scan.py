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

def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def read_regs(ser: serial.Serial, slave: int, start: int, qty: int) -> list[int] | None:
    pdu = bytes([slave, 0x03, (start >> 8) & 0xFF, start & 0xFF, (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)

    rx = ser.read(5 + 2 * qty + 2)  # header + data + crc
    if len(rx) < 5:
        return None
    if rx[0] != slave or rx[1] != 0x03:
        return None
    bytecount = rx[2]
    if bytecount != 2 * qty:
        return None

    regs = []
    for i in range(qty):
        hi = rx[3 + 2*i]
        lo = rx[3 + 2*i + 1]
        regs.append((hi << 8) | lo)
    return regs

def looks_like_voltage(x: int) -> bool:
    # Common RD scales: V*100, so 0..6500 => 0..65.00 V
    return 0 <= x <= 6500

def looks_like_current(x: int) -> bool:
    # Common RD scales: A*1000, so 0..30000 => 0..30.000 A
    return 0 <= x <= 30000

def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.25) as ser:
        print(f"Scanning {port} slave {slave}...")

        # Scan 0x0000..0x007F in blocks of 8
        for start in range(0x0000, 0x0080, 8):
            regs = read_regs(ser, slave, start, 8)
            if regs is None:
                continue

            # Flag blocks that contain plausible V/I-like numbers
            v_hits = [i for i, r in enumerate(regs) if looks_like_voltage(r)]
            i_hits = [i for i, r in enumerate(regs) if looks_like_current(r)]
            if v_hits or i_hits:
                print(f"{start:04X}: " + " ".join(f"{r:5d}" for r in regs))
        return 0

if __name__ == "__main__":
    raise SystemExit(main())
