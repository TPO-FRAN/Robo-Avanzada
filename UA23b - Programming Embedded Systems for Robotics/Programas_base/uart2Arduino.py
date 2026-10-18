import serial
import time

def main():
    print("=" * 60)
    print("ARDUINO LED BLINK CONTROL - WITH ERROR HANDLING")
    print("=" * 60)
    print("Commands:")
    print("  Enter a number (10-10000ms) to set blink interval")
    print("  Type 'q' to quit")
    print("-" * 60)
    
    try:
        with serial.Serial("/dev/ttyACM0", 9600, timeout=2) as arduino:
            time.sleep(2)
            print(f"Connected to {arduino.port}")
            print("-" * 60)
            
            arduino.reset_input_buffer()
            
            while True:
                # Read any waiting data FIRST
                while arduino.in_waiting > 0:
                    line = arduino.readline().decode().strip()
                    if line:
                        print(f"[Arduino] {line}")
                
                # Get user input
                user_input = input("\nEnter interval (10-10000ms): ").strip()
                
                if user_input.lower() == 'q':
                    print("Goodbye!")
                    break
                
                if user_input.isdigit():
                    interval = int(user_input)
                    
                    # Check range BEFORE sending (to avoid waiting)
                    if interval < 10:
                        print("[Warning] Interval too short! Minimum is 10ms")
                        continue
                    elif interval > 10000:
                        print("[Warning] Interval too long! Maximum is 10000ms")
                        continue
                    
                    # Send to Arduino
                    arduino.write(f"{interval}\n".encode())
                    print(f"[SENT] Interval: {interval}ms")
                    
                    # Wait for response with timeout
                    start_time = time.time()
                    response_received = False
                    
                    while time.time() - start_time < 0.5:  # 500ms timeout
                        if arduino.in_waiting > 0:
                            response = arduino.readline().decode().strip()
                            if response:
                                print(f"[Arduino] {response}")
                                response_received = True
                                break
                        time.sleep(0.01)
                    
                    if not response_received:
                        print("[WARNING] No response from Arduino (timeout)")
                        
                else:
                    print("Error: Please enter a number or 'q' to quit")
                    
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
    finally:
        print("Communication ended")

if __name__ == "__main__":
    main()
