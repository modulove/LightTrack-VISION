#!/usr/bin/env python3
"""
OSC Message Player
Replays recorded OSC messages from a JSON file.
Can be used standalone or imported into TouchDesigner.
"""

import json
import time
import sys
import argparse

try:
    from pythonosc import udp_client
except ImportError:
    print("Installing python-osc...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-osc"])
    from pythonosc import udp_client


def load_recording(filename):
    """Load a recording from JSON file"""
    with open(filename, 'r') as f:
        data = json.load(f)
    return data


def play_recording(filename, target_ip="127.0.0.1", target_port=9001, speed=1.0, loop=False):
    """
    Play back a recorded OSC session.

    Args:
        filename: Path to the JSON recording file
        target_ip: IP address to send OSC messages to
        target_port: Port to send OSC messages to
        speed: Playback speed multiplier (1.0 = realtime, 2.0 = double speed)
        loop: Whether to loop the playback
    """
    print("=" * 50)
    print("OSC Message Player")
    print("=" * 50)

    # Load recording
    data = load_recording(filename)
    messages = data["messages"]
    metadata = data["metadata"]

    print(f"File: {filename}")
    print(f"Duration: {metadata['duration_seconds']:.1f}s")
    print(f"Messages: {metadata['message_count']}")
    print(f"Target: {target_ip}:{target_port}")
    print(f"Speed: {speed}x")
    print(f"Loop: {loop}")
    print("=" * 50)

    # Create OSC client
    client = udp_client.SimpleUDPClient(target_ip, target_port)

    try:
        while True:
            print("\nStarting playback...")
            start_time = time.time()

            for i, msg in enumerate(messages):
                # Calculate when this message should be sent
                target_time = start_time + (msg["timestamp"] / speed)

                # Wait until it's time to send
                now = time.time()
                if target_time > now:
                    time.sleep(target_time - now)

                # Send the message
                client.send_message(msg["address"], msg["args"])

                # Progress update
                if i % 100 == 0:
                    elapsed = time.time() - start_time
                    print(f"\r  Progress: {i}/{len(messages)} messages | {elapsed:.1f}s elapsed    ", end="", flush=True)

            print(f"\n\nPlayback complete!")

            if not loop:
                break
            else:
                print("Looping...")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nPlayback stopped by user.")


def main():
    parser = argparse.ArgumentParser(description="Replay recorded OSC messages")
    parser.add_argument("filename", help="JSON recording file to play")
    parser.add_argument("--ip", default="127.0.0.1", help="Target IP address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9001, help="Target port (default: 9001)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--loop", action="store_true", help="Loop playback")

    args = parser.parse_args()

    play_recording(args.filename, args.ip, args.port, args.speed, args.loop)


if __name__ == "__main__":
    main()
