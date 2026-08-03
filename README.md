# Volz Servo Controller

Standalone test app for cycling a Volz DA-15N servo (DA 15-N.30.3.SC1500.U.LT.115)
between two preset positions at a set speed, over a USB-to-RS-485 adapter
(FTDI FT232R or similar).

## Requirements

- Python 3.10+ with `pyserial` (`pip install -r requirements.txt`)
- USB-RS485 adapter connected to the servo bus
- Close the Volz VISIO app first (Windows) - it holds the COM port open

### Linux setup

Port auto-detection works the same as on Windows - `volz_servo.py` scans
connected serial devices for an FTDI/RS-485 adapter and picks it automatically
(e.g. `/dev/ttyUSB0`), no `--port` needed in the common case.

One extra one-time step: your user needs access to the serial device, which
on most distros means membership in the `dialout` group:

```
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for the group change to take effect. Without
this, opening the port fails with a permission error.

## Usage

On Windows, double-click `RunServoCycle.bat`. On Linux, run
`./run_servo_cycle.sh` (or `python3 volz_servo.py ...` directly). From a
terminal on either platform:

```
python3 volz_servo.py                          # cycle +42.4 <-> -45 deg at 30 deg/s forever
python3 volz_servo.py --speed 10 --cycles 3    # 10 deg/s, 3 round trips
python3 volz_servo.py --dwell 2                # pause 2 s at each end
python3 volz_servo.py --period 30              # one full cycle every 30 s
python3 volz_servo.py -l                       # also log per-cycle stats to VolzTest_<date>_<time>.csv
python3 volz_servo.py --check                  # comms check only, no motion
python3 volz_servo.py --port /dev/ttyUSB0 --id 2   # explicit port / servo ID (COM5 on Windows)
```

Stop any time with Ctrl+C.

The console shows commanded vs. actual position, supply current, and motor
temperature live. With `-l`, each cycle (one A-B-A round trip including
dwells) is logged with measured start/end position, commanded positions,
motor/PCB temperature, and average/peak supply current.

## Protocol notes

Implements the Volz DA26 RS485 Communication Protocol Spec V1.16 (shared by
the DA-15N): 6-byte frames `[cmd, id, arg1, arg2, crc_hi, crc_lo]`,
CRC-16 poly 0x8005 init 0xFFFF, 115200 baud 8N1.

- Position: 12-bit value, 0 deg = 0x0800, 19.2 counts/deg, valid range
  +/-45 deg (0x04A1..0x0B60). On the wire the value is split as bits 11-7
  in arg1 and bits 6-0 in arg2.
- Commands used: 0xDD new position, 0x92 read position, 0xB0 current,
  0xB1 voltage, 0xC0 temperature.
- Note: this servo does NOT respond to the newer 0xDC "extended position"
  command used by some public implementations (e.g. ArduPilot) - it
  silently ignores it.
- The position command has no speed parameter; speed control is done by
  ramping the commanded position at 50 Hz.

`VISIO-V1_6a/` is the Volz factory test application (proprietary, for
internal use only).
