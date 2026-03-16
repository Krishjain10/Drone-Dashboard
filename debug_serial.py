import serial
import time
import sys

# Configuration
PORTS = ['COM25', 'COM22']
BAUD_RATES = [57600, 115200]

def test_port(port, baud):
    print(f"Testing {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        start_time = time.time()
        byte_count = 0
        
        while time.time() - start_time < 5:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                byte_count += len(data)
                # Print first few bytes in hex to identify protocol
                if byte_count < 50: 
                    print(f"Received: {data.hex().upper()}")
                    if b'\xfe' in data or b'\xfd' in data:
                        print("  -> MAVLink magic byte detected!")
            time.sleep(0.01)
            
        print(f"Total bytes received in 5s: {byte_count}")
        ser.close()
        return byte_count > 0
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("Diagnosis Tool")
    for port in PORTS:
        for baud in BAUD_RATES:
            print("-" * 20)
            if test_port(port, baud):
                print(f"✅ Data received on {port} @ {baud}")
            else:
                print(f"❌ No data on {port} @ {baud}")
