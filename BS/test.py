import serial
import time
import re

def send_cmd(ser, cmd):
    if not cmd.endswith('\r\n'):
        cmd += '\r\n'
    ser.write(cmd.encode())
    time.sleep(0.2)
    response = ser.read_all().decode(errors='ignore')
    return response

def parse_gga(line):
    parts = line.strip().split(",")
    if len(parts) > 6:
        try:
            fix_type = int(parts[6])
            return fix_type
        except ValueError:
            return -1
    return -1

def main():
    print("=== UM980 RTK BASE SETUP ===")
    port = input("COM-port (eks COM3 eller /dev/ttyUSB0): ").strip()
    baud = input("Baudrate [default: 115200]: ").strip()
    if not baud:
        baud = 115200
    else:
        baud = int(baud)

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.5)
    except Exception as e:
        print(f"❌ Kunne ikke åpne port {port}: {e}")
        return

    print("🛰️ Setter base i survey-in modus (60s)...")
    print(send_cmd(ser, "MODE BASE TIME 60"))

    print("⏳ Venter noen sekunder før vi aktiverer GGA...")
    time.sleep(2)

    print("🗺️ Aktiverer GGA utdata...")
    print(send_cmd(ser, "gpgga com1 1"))

    print("📡 Leser GGA-meldinger og venter på RTK Fix (fix type 4 eller 7)...\n")

    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                fix = parse_gga(line)
                print(f"{line}  → Fix type: {fix}", end='\r')

                if fix in (4, 7):
                    print(f"\n✅ RTK Fix oppnådd! Fix type: {fix}")
                    break
    except KeyboardInterrupt:
        print("\n⛔ Avbrutt av bruker.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
