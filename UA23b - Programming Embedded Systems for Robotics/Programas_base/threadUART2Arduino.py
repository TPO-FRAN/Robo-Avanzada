import serial
import time
import threading

# ============================================
# GLOBAL VARIABLES
# ============================================
# stop_event: A signal that tells the thread when to stop running
#             Think of it like a "stop button" for the background thread
stop_event = threading.Event()

# last_response: Stores the most recent message from Arduino
#                The thread writes to this, the main program reads from it
last_response = ""

# response_available: A flag that indicates if a new response is waiting
#                     True = new data arrived, False = no new data
response_available = False

# response_lock: A lock that prevents two threads from accessing shared data
#                at the same time. Without this, data could become corrupted!
response_lock = threading.Lock()


# ============================================
# THREAD FUNCTION: Receives data from Arduino
# ============================================
def receive_from_arduino(arduino, stop_event):
    """
    ================================================================
    WHAT IS THIS THREAD FOR?
    ================================================================
    This thread runs in the BACKGROUND to continuously receive data
    from Arduino. It works independently from the main program.
    
    WHY DO WE NEED A THREAD?
    -------------------------
    PROBLEM: Without a thread, the program would freeze when waiting
             for user input or waiting for Arduino data.
    
    SOLUTION: The thread runs in the background, constantly checking
              for Arduino data while the main program handles user input.
    
    HOW THE THREAD WORKS:
    ----------------------
    1. Runs in an infinite loop (until told to stop)
    2. Each loop: Checks if Arduino sent any data
    3. If data exists: Reads it and stores it safely
    4. If no data: Sleeps for 10ms to prevent CPU overload
    5. Repeats steps 2-4 continuously
    
    REAL-WORLD ANALOGY:
    --------------------
    This is like having a dedicated assistant who constantly watches
    for messages while you focus on other tasks. The assistant:
    - Watches for messages continuously (background thread)
    - Notifies you when a message arrives (sets response_available)
    - Stores the message for you (last_response)
    - You check whenever you're ready (main program)
    
    ================================================================
    """
    global last_response, response_available
    
    # The thread runs until it's told to stop
    # stop_event.is_set() becomes True when main program wants to exit
    while not stop_event.is_set():
        try:
            # STEP 1: Check if Arduino has sent any data
            # in_waiting is the number of bytes waiting to be read
            # If > 0, data is available!
            if arduino.in_waiting > 0:
                
                # STEP 2: Read one line of data from Arduino
                # readline() reads until newline character
                # decode() converts bytes to string
                # strip() removes extra whitespace
                line = arduino.readline().decode().strip()
                
                # STEP 3: If we got valid data, store it safely
                if line:
                    # CRITICAL: Use the lock before modifying shared variables
                    # This prevents the main thread from reading while we're writing
                    with response_lock:
                        # Store the response for the main thread to read
                        last_response = line
                        # Signal that a new response is available
                        response_available = True
                        
        except:
            # If an error occurs, we ignore it and continue
            # This prevents the thread from crashing
            pass
        
        # STEP 4: Sleep for 10ms to prevent CPU overload
        # Without this, the loop would run millions of times per second
        # 10ms = 0.01 seconds, so this runs 100 times per second
        # This is efficient and responsive
        time.sleep(0.01)


# ============================================
# MAIN PROGRAM
# ============================================
def main():
    """
    ================================================================
    MAIN PROGRAM: User Interface and Control
    ================================================================
    This is the "main" thread that:
    - Shows the user interface
    - Gets user input
    - Sends commands to Arduino
    - Displays responses from the background thread
    
    HOW IT INTERACTS WITH THE WORKER THREAD:
    ----------------------------------------
    1. Main thread: Creates and starts the worker thread
    2. Worker thread: Runs in background, receiving data
    3. Main thread: Sends commands and checks for responses
    4. Main thread: Uses lock to safely read data from worker
    5. Main thread: Stops worker thread when user types 'q'
    
    ================================================================
    """
    global last_response, response_available
    
    # Display the user interface
    print("=" * 60)
    print("ARDUINO BLINK CONTROL - WITH THREAD")
    print("=" * 60)
    print("Commands:")
    print("  Enter a number (10-10000ms) to set blink interval")
    print("  Type 'q' to quit")
    print("-" * 60)
    
    try:
        # Connect to Arduino
        # 'with' statement automatically closes the connection when done
        with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
            # Wait 2 seconds for Arduino to reset after connection
            time.sleep(2)
            print(f"Connected to {arduino.port}")
            print("-" * 60)
            
            # Clear any old data that might be in the buffer
            arduino.reset_input_buffer()
            
            # ============================================
            # CREATE AND START THE WORKER THREAD
            # ============================================
            # STEP 1: CREATE the thread object
            # - target: The function to run in the thread
            # - args: Arguments to pass to the function
            # - daemon=True: Thread stops when main program ends
            #   (This is like "auto-cleanup" for the thread)
            receive_thread = threading.Thread(
                target=receive_from_arduino,
                args=(arduino, stop_event),
                daemon=True
            )
            
            # STEP 2: START the thread
            # The thread begins running receive_from_arduino() in the background
            # The main program continues immediately (doesn't wait)
            receive_thread.start()
            print("Receive thread started (running in background)")
            print("-" * 60)
            # ============================================
            
            # ============================================
            # MAIN PROGRAM LOOP
            # ============================================
            while True:
                # ----------------------------------------------------
                # STEP 1: Check if Arduino sent any messages
                # ----------------------------------------------------
                # This is where the main thread "receives" data from
                # the worker thread. The worker thread puts data in
                # shared variables, and the main thread reads them.
                with response_lock:
                    if response_available:
                        # Display the response from Arduino
                        print(f"[Arduino] {last_response}")
                        # Clear the flag so we don't display it again
                        response_available = False
                        # Clear the stored response to save memory
                        last_response = ""
                
                # ----------------------------------------------------
                # STEP 2: Get user input
                # ----------------------------------------------------
                # This is where the main thread "blocks" (waits)
                # Notice: The worker thread continues running during this!
                # The user can take as long as they want to type, and
                # the background thread keeps checking for Arduino data.
                user_input = input("\nEnter interval (10-10000ms): ").strip()
                
                # ----------------------------------------------------
                # STEP 3: Process user command
                # ----------------------------------------------------
                
                # Check if user wants to quit
                if user_input.lower() == 'q':
                    print("Goodbye!")
                    break
                
                # Check if input is a valid number
                if user_input.isdigit():
                    interval = int(user_input)
                    
                    # Validate range
                    if interval < 10:
                        print("Warning: Too short! Minimum is 10ms")
                        continue
                    elif interval > 10000:
                        print("Warning: Too long! Maximum is 10000ms")
                        continue
                    
                    # ----------------------------------------------------
                    # STEP 4: Clear old response (if any)
                    # ----------------------------------------------------
                    # Before sending a new command, clear any old response
                    # This prevents showing an old response for a new command
                    with response_lock:
                        response_available = False
                        last_response = ""
                    
                    # ----------------------------------------------------
                    # STEP 5: Send command to Arduino
                    # ----------------------------------------------------
                    # Write the command to the serial port
                    # The Arduino will process it and send a response
                    arduino.write(f"{interval}\n".encode())
                    arduino.flush()  # Ensure data is sent immediately
                    print(f"[SENT] Interval: {interval}ms")
                    
                    # ----------------------------------------------------
                    # STEP 6: Wait for response with timeout
                    # ----------------------------------------------------
                    # We need to wait for Arduino to respond before
                    # allowing the user to send another command.
                    # This prevents "response confusion" where responses
                    # from different commands get mixed up.
                    #
                    # HOW THE WAIT WORKS:
                    # 1. Record the start time
                    # 2. Loop until: a) response arrives, or b) 1 second passes
                    # 3. Check response_available flag (set by worker thread)
                    # 4. If response arrives: display it and exit loop
                    # 5. If timeout: show warning and continue
                    #
                    # This is like saying: "I'll wait up to 1 second
                    # for the assistant (worker thread) to bring me
                    # a response. If they don't bring it in time,
                    # I'll assume something went wrong."
                    response_received = False
                    start_time = time.time()
                    while time.time() - start_time < 1.0:
                        # Check if worker thread received a response
                        with response_lock:
                            if response_available:
                                # Display the response
                                print(f"[Arduino] {last_response}")
                                # Clear the flag
                                response_available = False
                                last_response = ""
                                response_received = True
                                break
                        # Wait 50ms before checking again
                        # This prevents the loop from using too much CPU
                        time.sleep(0.05)
                    
                    # If we exited the loop without getting a response
                    # (response_available never became True)
                    if not response_received:
                        print("[WARNING] No response from Arduino (timeout)")
                    
                else:
                    print("Error: Enter a number or 'q' to quit")
                    
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
    finally:
        # ============================================
        # CLEANUP: Signal the thread to stop
        # ============================================
        # When the main program ends, we need to tell the worker
        # thread to stop. We do this by setting the stop_event.
        #
        # HOW THIS WORKS:
        # 1. Main: stop_event.set()  → Sets the "stop" signal
        # 2. Thread: while not stop_event.is_set(): → Loop exits
        # 3. Thread: Function ends → Thread stops
        # 4. daemon=True ensures it stops with main program
        #
        # This is like saying: "Assistant, you can stop working now."
        print("\nStopping thread...")
        stop_event.set()
        print("Communication ended. Goodbye!")


# ============================================
# PROGRAM ENTRY POINT
# ============================================
# This ensures the program only runs when executed directly
# (not when imported as a module by another program)
if __name__ == "__main__":
    main()
