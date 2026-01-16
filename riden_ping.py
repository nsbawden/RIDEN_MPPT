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

def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    slave = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    # Modbus RTU: [slave][func=0x03][start_hi][start_lo][qty_hi][qty_lo][crc_lo][crc_hi]
    pdu = bytes([slave, 0x03, 0x00, 0x00, 0x00, 0x02])
    crc = modbus_crc16(pdu)
    frame = pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    try:
        with serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.25,
            write_timeout=0.25,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            print(f"TX ({len(frame)}): {hexdump(frame)}")
            ser.write(frame)
            ser.flush()

            time.sleep(0.05)  # give device time to respond
            rx = ser.read(256)

            if not rx:
                print("RX: (no response)")
                return 2

            print(f"RX ({len(rx)}): {hexdump(rx)}")

            # Typical response: [slave][0x03][bytecount=4][r0_hi][r0_lo][r1_hi][r1_lo][crc_lo][crc_hi]
            if len(rx) >= 9 and rx[0] == slave and rx[1] == 0x03 and rx[2] == 4:
                r0 = (rx[3] << 8) | rx[4]
                r1 = (rx[5] << 8) | rx[6]
                print(f"Parsed: reg0={r0} reg1={r1}")
            return 0

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1

if __name__ == "__main__":
    raise SystemExit(main())
