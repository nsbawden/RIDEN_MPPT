import sys, time, serial

def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def read_regs(ser, slave, start, qty):
    pdu = bytes([slave, 0x03, (start >> 8) & 0xFF, start & 0xFF, (qty >> 8) & 0xFF, qty & 0xFF])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)
    rx = ser.read(5 + 2*qty + 2)
    if len(rx) < 5 or rx[0] != slave or rx[1] != 0x03 or rx[2] != 2*qty:
        raise RuntimeError(f"Bad response: {rx.hex(' ')}")
    regs = []
    for i in range(qty):
        regs.append((rx[3+2*i] << 8) | rx[3+2*i+1])
    return regs

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    start = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0x0000
    qty = int(sys.argv[4], 0) if len(sys.argv) > 4 else 0x0040  # 64 regs

    with serial.Serial(port, 115200, timeout=0.25, write_timeout=0.25) as ser:
        regs = read_regs(ser, slave, start, qty)
        for i, v in enumerate(regs):
            addr = start + i
            print(f"{addr:04X}: {v:5d}")

if __name__ == "__main__":
    main()
