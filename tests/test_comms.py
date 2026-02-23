import serial
import struct
import time

# Configuration
PORT = 'COM1'  # Change to your HC-05 COM port
BAUD = 9600

def parse_packet(packet):
    if len(packet) != 7 or packet[0] != 0xAA:
        return None
    
    throttle, pitch, roll, yaw, grip, checksum = struct.unpack('bbbbbB', packet[1:])
    
    # Verify checksum
    calc_checksum = throttle ^ pitch ^ roll ^ yaw ^ grip
    if (calc_checksum & 0xFF) == checksum:
        return {
            'throttle': throttle,
            'pitch': pitch,
            'roll': roll,
            'yaw': yaw,
            'grip': grip
        }
    return None

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Listening on {PORT}...")
        
        while True:
            if ser.in_waiting >= 7:
                byte = ser.read(1)
                if byte == b'\xaa':
                    data = byte + ser.read(6)
                    result = parse_packet(data)
                    if result:
                        print(f"Received: {result}")
                    else:
                        print("Checksum error or invalid packet")
            time.sleep(0.01)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
