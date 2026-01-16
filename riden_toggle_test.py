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

def read_holding_regs(ser, slave, start, qty):
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

def write_single_reg(ser, slave, addr, value):
    value &= 0xFFFF
    pdu = bytes([slave, 0x06, (addr >> 8) & 0xFF, addr & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)

    rx = ser.read(8)
    if len(rx) != 8 or rx[:6] != pdu:
        raise RuntimeError(f"Bad write echo: {rx.hex(' ')}")

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    base = 0x0008

    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.25) as ser:
        before = read_holding_regs(ser, slave, base, 8)
        print("BEFORE:", before)

        # Candidate ON/OFF control registers to try (safe single-register writes)
        candidates = [
            0x000F,  # common in some maps
            0x0010,  # sometimes used as a flag area
            0x0048,  # your scan shows small integers here
            0x0042,  # also small/flaggy in your scan region
        ]

        for addr in candidates:
            for val in (0, 1):
                try:
                    write_single_reg(ser, slave, addr, val)
                    after = read_holding_regs(ser, slave, base, 8)
                    changed = [i for i in range(8) if after[i] != before[i]]
                    print(f"Wrote {val} to {addr:#06x}; CHG idx={changed}; AFTER={after}")
                    before = after
                except Exception as e:
                    print(f"Write {val} to {addr:#06x} failed: {e}")

if __name__ == "__main__":
    main()
