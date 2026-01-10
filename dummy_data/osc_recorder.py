#!/usr/bin/env python3
"""
OSC Message Recorder
Records OSC messages for 1 minute and saves them to a JSON file for replay in TouchDesigner.
"""

import json
import time
import sys
from datetime import datetime

try:
    from pythonosc import dispatcher
    from pythonosc import osc_server
except ImportError:
    print("Installing python-osc...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-osc"])
    from pythonosc import dispatcher
    from pythonosc import osc_server

import threading

# Configuration
OSC_PORT = 9001
RECORD_DURATION = 60  # seconds

# Storage for recorded messages
recorded_messages = []
start_time = None


def osc_handler(address, *args):
    """Handler for all incoming OSC messages"""
    global start_time, recorded_messages

    if start_time is None:
        start_time = time.time()

    timestamp = time.time() - start_time

    # Convert args to a serializable format
    serializable_args = []
    for arg in args:
        if isinstance(arg, (int, float, str, bool)):
            serializable_args.append(arg)
        elif isinstance(arg, bytes):
            serializable_args.append(arg.decode('utf-8', errors='replace'))
        else:
            serializable_args.append(str(arg))

    message = {
        "timestamp": timestamp,
        "address": address,
        "args": serializable_args
    }
    recorded_messages.append(message)

    # Print progress
    msg_count = len(recorded_messages)
    if msg_count % 100 == 0:
        print(f"  Recorded {msg_count} messages... ({timestamp:.1f}s)")


def main():
    global start_time, recorded_messages

    print("=" * 50)
    print("OSC Message Recorder")
    print("=" * 50)
    print(f"Port: {OSC_PORT}")
    print(f"Duration: {RECORD_DURATION} seconds")
    print("=" * 50)

    # Set up dispatcher to handle all addresses
    disp = dispatcher.Dispatcher()
    disp.set_default_handler(osc_handler)

    # Create server
    try:
        server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", OSC_PORT), disp)
    except OSError as e:
        print(f"ERROR: Could not bind to port {OSC_PORT}: {e}")
        print("Make sure no other application is using this port.")
        sys.exit(1)

    print(f"\nListening for OSC messages on port {OSC_PORT}...")
    print("Recording will start when first message is received.")
    print("Press Ctrl+C to stop early.\n")

    # Start server in a thread
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    # Wait for first message or timeout
    wait_start = time.time()
    while start_time is None:
        if time.time() - wait_start > 120:  # 2 minute timeout waiting for first message
            print("ERROR: No OSC messages received within 2 minutes. Exiting.")
            server.shutdown()
            sys.exit(1)
        time.sleep(0.1)

    print(f"First message received! Recording for {RECORD_DURATION} seconds...")

    # Record for specified duration
    try:
        while time.time() - start_time < RECORD_DURATION:
            remaining = RECORD_DURATION - (time.time() - start_time)
            print(f"\r  Time remaining: {remaining:.1f}s | Messages: {len(recorded_messages)}    ", end="", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nRecording stopped by user.")

    # Stop server
    server.shutdown()

    print(f"\n\nRecording complete!")
    print(f"Total messages recorded: {len(recorded_messages)}")

    if len(recorded_messages) == 0:
        print("No messages were recorded. Exiting.")
        sys.exit(1)

    # Generate filename with timestamp
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"osc_recording_{timestamp_str}.json"

    # Prepare output data
    output_data = {
        "metadata": {
            "recorded_at": datetime.now().isoformat(),
            "duration_seconds": recorded_messages[-1]["timestamp"] if recorded_messages else 0,
            "message_count": len(recorded_messages),
            "port": OSC_PORT,
            "unique_addresses": list(set(m["address"] for m in recorded_messages))
        },
        "messages": recorded_messages
    }

    # Save to file
    with open(filename, 'w') as f:
        json.dump(output_data, f, indent=2)

    print(f"\nSaved to: {filename}")
    print(f"\nUnique OSC addresses found:")
    for addr in output_data["metadata"]["unique_addresses"]:
        count = sum(1 for m in recorded_messages if m["address"] == addr)
        print(f"  {addr}: {count} messages")

    return filename


if __name__ == "__main__":
    main()
