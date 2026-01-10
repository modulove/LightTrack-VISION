# OSC Recording Data

This folder contains recorded OSC data from the LightTrack-VISION radar system for testing and development purposes.

## Files

- `osc_recorder.py` - Script to record OSC messages (requires `python-osc`)
- `osc_player.py` - Script to replay recorded OSC messages
- `osc_recording_*.json` - Recorded OSC data files

## Recording

To record new OSC data:

```bash
py -3 osc_recorder.py
```

The script listens on port 9001 and records for 60 seconds after the first message is received.

## Playback

To replay recorded data (for TouchDesigner testing):

```bash
py -3 osc_player.py osc_recording_YYYYMMDD_HHMMSS.json
```

Options:
- `--ip` - Target IP (default: 127.0.0.1)
- `--port` - Target port (default: 9001)
- `--speed` - Playback speed multiplier (default: 1.0)
- `--loop` - Loop playback continuously

Example:
```bash
py -3 osc_player.py osc_recording_20260110_210825.json --port 9001 --loop
```

## OSC Message Format

The recorded data contains radar tracking messages:

| Address | Description |
|---------|-------------|
| `/radar/count` | Number of detected targets |
| `/radar/N/present` | Target N presence (0/1) |
| `/radar/N/x` | Target N X position |
| `/radar/N/y` | Target N Y position |
| `/radar/N/distance` | Target N distance |
| `/radar/N/speed` | Target N speed |

Where N = 1, 2, 3 for up to 3 tracked targets.

## Requirements

```bash
pip install python-osc
```
