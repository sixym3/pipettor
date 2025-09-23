#!/usr/bin/env python3
import serial
import time
import sys
import threading

# Configuration
SERIAL_PORT = '/tmp/ttyUR'  # Default serial port
BAUD_RATE = 115200         # Standard baud rate
TIMEOUT = 1                # Serial timeout in seconds

def read_responses(ser):
    """Continuously read responses from Arduino"""
    while True:
        try:
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                if response:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] Arduino: {response}")
        except Exception as e:
            # Silent exit when main thread closes serial port
            break

def send_command(ser, command):
    """Send a command with debug information"""
    timestamp = time.strftime("%H:%M:%S")
    
    # Add newline for Arduino line parsing
    if not command.endswith('\n'):
        command += '\n'
    
    # Convert to bytes and send
    cmd_bytes = command.encode('utf-8')
    print(f"[{timestamp}] Sending: {repr(command.strip())} ({len(cmd_bytes)} bytes)")
    
    try:
        bytes_written = ser.write(cmd_bytes)
        ser.flush()  # Ensure data is sent
        return True
    except Exception as e:
        print(f"[{timestamp}] Error sending: {e}")
        return False

def main():
    print("Arduino Serial Communication Terminal")
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        print(f"Connected successfully!")
        print("\nInstructions:")
        print("- Press Enter (empty) to send 'V' (toggle LED)")
        print("- Type any command and press Enter to send it")
        print("- Type 'exit' or 'quit' to close the program")
        print("- Press Ctrl+C to force quit")
        print("-" * 50)
        
        # Start response reader thread
        reader_thread = threading.Thread(target=read_responses, args=(ser,), daemon=True)
        reader_thread.start()
        
        # Give Arduino time to initialize
        time.sleep(2)
        
        # Interactive command loop
        while True:
            try:
                # Get user input
                user_input = input().strip()
                
                # Check for exit commands
                if user_input.lower() in ['exit', 'quit']:
                    print("Exiting...")
                    break
                
                # If empty, send 'V', otherwise send what was typed
                if user_input == "":
                    command = "V"
                else:
                    command = user_input
                
                # Send the command
                send_command(ser, command)
                
            except EOFError:
                # Handle Ctrl+D
                print("\nExiting...")
                break
                
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        print("Make sure socat is running and the virtual port exists")
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

def cli_main():
    """Entry point for ros2 run command"""
    main()

if __name__ == "__main__":
    main()