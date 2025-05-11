#!/usr/bin/env python3
"""
Repeat stepper commands via serial port, with support for batch commands.

Use:
    python serial_sequence_repeater1.1.py --port COM9 --baud 115200

--- Stepper Control ---
Commands:
  A idx val   -> set acceleration of motor[idx]
  T idx val   -> set fast speed of motor[idx]
  L idx val   -> set slow speed of motor[idx]
  P idx deg   -> set target position (degrees) of motor[idx]
  D idx       -> dump status of motor[idx]
  ALL         -> dump all statuses

Both startup_commands and commands lists may contain single or multiple commands
in the form "P 2 200 P 1 200 P 0 -200". Multiple commands in one string will
be parsed and sent in sequence each loop iteration.

Repeat stepper commands via serial port, with DONE acknowledgment.


"""

#!/usr/bin/env python3

import argparse
import time
import sys
from serial import Serial, SerialException
import serial.tools.list_ports

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")

def send_batch_and_wait_done(ser, cmd_group):
    tokens = cmd_group.split()
    i = 0
    while i < len(tokens):
        tk = tokens[i].upper()
        if tk == "ALL":
            ser.write(b"ALL\n")
            print("Sent -> ALL")
            i += 1
        elif tk in {"A", "T", "L", "D", "P"}:
            if tk == "D":
                if i + 1 < len(tokens):
                    sub = f"D {tokens[i+1]}"
                    i += 2
                else:
                    print("Malformed D command")
                    break
            else:
                if i + 2 < len(tokens):
                    sub = f"{tk} {tokens[i+1]} {tokens[i+2]}"
                    i += 3
                else:
                    print(f"Malformed {tk} command")
                    break
            ser.write((sub + "\n").encode())
            print(f"Sent -> {sub}")
        else:
            print(f"Skipping unknown token: {tokens[i]}")
            i += 1

    # Wait for DONE signal from microcontroller
    buffer = ""
    print("Waiting for DONE...", end="", flush=True)
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"\n[Board] {line}")
            buffer += line
            if "DONE" in line:
                print("✔ DONE received.")
                break

def main():
    parser = argparse.ArgumentParser(description="Stepper command batch sender with DONE wait.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", "-p", help="Serial port, e.g., COM3 or /dev/ttyUSB0")
    group.add_argument("--list", "-l", action="store_true", help="List serial ports and exit")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--delay", "-d", type=float, default=1.0, help="Optional delay after DONE (s)")
    args = parser.parse_args()

    if args.list:
        list_ports()
        sys.exit(0)

    try:
        ser = Serial(args.port, args.baud, timeout=1)
        print(f"Opened port {args.port} at {args.baud} baud")
        time.sleep(2)
    except SerialException as e:
        print(f"Error opening {args.port}: {e}")
        sys.exit(1)

    # Initial setup commands
    startup_commands = [
        "A 2 1000",
        "T 2 2000",
        "L 2 200",
        "P 2 0 P 1 0 P 0 0",
    ]

    # Repeating batches
    commands = [
        "P 2 0",
        "P 2 0",
        "P 2 -15000",
        "P 2 -15000",
    ]

    try:
        print("=== Sending Startup Commands ===")
        for batch in startup_commands:
            send_batch_and_wait_done(ser, batch)
            time.sleep(args.delay)

        print("\n=== Starting Repeating Commands ===")
        while True:
            for idx, batch in enumerate(commands):
                send_batch_and_wait_done(ser, batch)
                time.sleep(args.delay)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
