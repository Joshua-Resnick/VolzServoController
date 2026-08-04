#!/usr/bin/env python3
"""Cycle a Volz DA-15N servo between two preset positions at a set speed.

Implements the Volz RS-485 protocol (DA26 RS485 Protocol Spec V1.16, which
the DA-15N shares): 6-byte frames [cmd, id, arg1, arg2, crc_hi, crc_lo],
CRC-16 poly 0x8005 init 0xFFFF, 115200 baud 8N1, over a USB-to-RS-485
adapter (FTDI FT232R or similar).

Position encoding: 12-bit value, 0 deg = 0x0800, 19.2 counts/deg,
+45 deg = 0x0B60, -45 deg = 0x04A1. On the wire the 12-bit value is split
as bits 11-7 in Argument 1 and bits 6-0 in Argument 2 (MSBs zero).

The position command has no built-in speed parameter, so speed is achieved
by ramping the commanded position at UPDATE_HZ. Current is sampled every
tick and temperature once per second; both are shown live and, with -l,
summarized per cycle in a log file.

Usage:
    python volz_servo.py                 # cycle 42.4 <-> -45 at 30 deg/s forever
    python volz_servo.py --speed 10      # slower
    python volz_servo.py --cycles 3      # stop after 3 round trips
    python volz_servo.py -l              # also write VolzTest_<date>_<time>.csv
    python volz_servo.py --check         # comms check only, no motion
    python volz_servo.py --port COM5     # explicit COM port

    # Progressively ramp a delay from a min to a max over N cycles, then hold
    # at max. Dwell (pause at each end) and period (time between cycle
    # starts) each ramp independently; give both -min/-max to enable one.
    python volz_servo.py --dwell-min 1 --dwell-max 10 --dwell-ramp-cycles 20
    python volz_servo.py --period-min 5 --period-max 60 --period-ramp-cycles 30

Stop any time with Ctrl+C. Close the Volz VISIO app first - it holds the
COM port open.
"""

import argparse
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports

# --- Volz protocol constants (DA26 RS485 Protocol Spec V1.16) ---
CMD_NEW_POSITION = 0xDD
RSP_NEW_POSITION = 0x44
CMD_READ_POSITION = 0x92
RSP_READ_POSITION = 0x62
CMD_READ_CURRENT = 0xB0
RSP_READ_CURRENT = 0x30
CMD_READ_VOLTAGE = 0xB1
RSP_READ_VOLTAGE = 0x31
CMD_READ_TEMPERATURE = 0xC0
RSP_READ_TEMPERATURE = 0x10

POS_CENTER = 0x0800   # 0 deg
POS_MAX = 0x0B60      # +45 deg
POS_MIN = 0x04A1      # -45 deg
COUNTS_PER_DEG = 19.2

UPDATE_HZ = 50.0      # position streaming rate for the speed ramp
DWELL_SAMPLE_S = 0.1  # current sampling interval while dwelling at an endpoint
LOG_INTERVAL_S = 0.1  # position/current/temperature log sampling interval


def crc16(data: bytes) -> int:
    """CRC-16, poly 0x8005, init 0xFFFF, over the first 4 frame bytes."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd: int, servo_id: int, arg1: int, arg2: int) -> bytes:
    body = bytes([cmd, servo_id, arg1, arg2])
    crc = crc16(body)
    return body + bytes([crc >> 8, crc & 0xFF])


def deg_to_args(deg: float) -> tuple[int, int]:
    """Angle -> 12-bit position value -> (arg1, arg2) wire encoding."""
    val = round(POS_CENTER + deg * COUNTS_PER_DEG)
    val = max(POS_MIN, min(POS_MAX, val))
    return (val >> 7) & 0x1F, val & 0x7F


def args_to_deg(arg1: int, arg2: int) -> float:
    val = ((arg1 & 0x1F) << 7) | (arg2 & 0x7F)
    return (val - POS_CENTER) / COUNTS_PER_DEG


class VolzServo:
    def __init__(self, port: str, baud: int, servo_id: int):
        self.servo_id = servo_id
        self.ser = serial.Serial(port=port, baudrate=baud, bytesize=8,
                                 parity=serial.PARITY_NONE, stopbits=1,
                                 timeout=0.02)

    def transact(self, cmd: int, rsp_code: int, arg1: int = 0, arg2: int = 0) -> bytes | None:
        """Send a command, return the validated 6-byte response or None."""
        self.ser.reset_input_buffer()
        self.ser.write(build_frame(cmd, self.servo_id, arg1, arg2))
        rsp = self.ser.read(6)
        if (len(rsp) != 6 or rsp[0] != rsp_code
                or crc16(rsp[:4]) != (rsp[4] << 8 | rsp[5])):
            return None
        return rsp

    def set_position(self, deg: float) -> float | None:
        """Command a position; return the servo's reported current position."""
        rsp = self.transact(CMD_NEW_POSITION, RSP_NEW_POSITION, *deg_to_args(deg))
        return None if rsp is None else args_to_deg(rsp[2], rsp[3])

    def read_position(self) -> float | None:
        """Read the actual position - causes no motion."""
        rsp = self.transact(CMD_READ_POSITION, RSP_READ_POSITION)
        return None if rsp is None else args_to_deg(rsp[2], rsp[3])

    def read_current(self) -> float | None:
        """Primary supply current in A (20 mA resolution)."""
        rsp = self.transact(CMD_READ_CURRENT, RSP_READ_CURRENT)
        return None if rsp is None else rsp[2] * 0.02

    def read_temperature(self) -> tuple[int | None, int | None] | None:
        """(motor, PCB) temperature in degC. 0x00 raw = no sensor -> None entry."""
        rsp = self.transact(CMD_READ_TEMPERATURE, RSP_READ_TEMPERATURE)
        if rsp is None:
            return None
        return (rsp[2] - 50 if rsp[2] else None,
                rsp[3] - 50 if rsp[3] else None)

    def read_voltage(self) -> float | None:
        rsp = self.transact(CMD_READ_VOLTAGE, RSP_READ_VOLTAGE)
        return None if rsp is None else rsp[2] * 0.2


    def close(self):
        self.ser.close()


def find_port() -> str | None:
    """Auto-detect the USB-RS485 adapter (prefers FTDI devices)."""
    ports = list(list_ports.comports())
    for p in ports:
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("ft232", "ftdi", "usb serial", "rs485", "rs-485")):
            return p.device
    if len(ports) == 1:
        return ports[0].device
    return None


class Telemetry:
    """Collects current samples, caches a slow-polled temperature, and

    optionally writes a log (every LOG_INTERVAL_S seconds) of position/
    current/temperature regardless of
    whether the servo is being actively moved. Each logged row is tagged with
    the caller-set `cycle`/`phase` so cycle-level stats can be recovered by
    grouping the raw rows afterward.
    """

    def __init__(self, servo: VolzServo, logfile=None):
        self.servo = servo
        self.samples: list[float] = []
        self.motor_temp: int | None = None
        self.pcb_temp: int | None = None
        self._next_temp_poll = 0.0
        self.logfile = logfile
        self._next_log_tick = 0.0
        self.cycle = 0
        self.phase = "start"
        self.poll_temp()

    def sample(self, position: float | None = None):
        """Take one current reading. `position`, if known to the caller (e.g.
        mid-move), is reused for the log instead of an extra read."""
        amps = self.servo.read_current()
        if amps is not None:
            self.samples.append(amps)
        now = time.perf_counter()
        if now >= self._next_temp_poll:
            self._next_temp_poll = now + 1.0
            self.poll_temp()
        if self.logfile is not None and now >= self._next_log_tick:
            self._next_log_tick = now + LOG_INTERVAL_S
            if position is None:
                position = self.servo.read_position()
            self.logfile.write(f"{datetime.now():%Y-%m-%d %H:%M:%S},{self.cycle},{self.phase},"
                                f"{fmt(position, '.2f')},{fmt(amps, '.3f')},"
                                f"{fmt(self.motor_temp, 'd')},{fmt(self.pcb_temp, 'd')}\n")
            self.logfile.flush()
        return amps

    def poll_temp(self):
        temps = self.servo.read_temperature()
        if temps is not None:
            self.motor_temp, self.pcb_temp = temps

    def reset_samples(self):
        self.samples = []

    @property
    def avg_current(self) -> float | None:
        return sum(self.samples) / len(self.samples) if self.samples else None

    @property
    def peak_current(self) -> float | None:
        return max(self.samples) if self.samples else None

    def status_str(self, amps: float | None) -> str:
        cur = f"{amps:5.2f} A" if amps is not None else "  ?  A"
        tmp = f"motor {self.motor_temp:3d} C" if self.motor_temp is not None else "motor  ? C"
        return f"{cur}   {tmp}"


def move_to(servo: VolzServo, telem: Telemetry, start: float, target: float,
            speed: float) -> float:
    """Ramp the commanded position from start to target at speed (deg/s)."""
    step = speed / UPDATE_HZ
    direction = 1.0 if target > start else -1.0
    setpoint = start
    period = 1.0 / UPDATE_HZ
    next_tick = time.perf_counter()
    while True:
        setpoint += direction * step
        done = (setpoint - target) * direction >= 0
        if done:
            setpoint = target
        actual = servo.set_position(setpoint)
        amps = telem.sample(actual)
        if actual is not None:
            print(f"\r  cmd {setpoint:+7.2f} deg   actual {actual:+7.2f} deg   "
                  f"{telem.status_str(amps)}   ", end="", flush=True)
        if done:
            print()
            return target
        next_tick += period
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)


def dwell(servo: VolzServo, telem: Telemetry, seconds: float):
    """Hold position for `seconds`, sampling current while waiting."""
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        telem.sample()
        time.sleep(min(DWELL_SAMPLE_S, max(0.0, end - time.perf_counter())))


def fmt(value, spec: str) -> str:
    return format(value, spec) if value is not None else ""


def ramp_value(cycle: int, vmin: float, vmax: float, ramp_cycles: int) -> float:
    """Linear ramp from vmin (cycle 1) to vmax (cycle `ramp_cycles`), holding at vmax after."""
    if ramp_cycles <= 1:
        return vmax
    t = min(1.0, (cycle - 1) / (ramp_cycles - 1))
    return vmin + (vmax - vmin) * t


def main():
    ap = argparse.ArgumentParser(description="Cycle a Volz DA-15N between two positions.")
    ap.add_argument("--pos-a", type=float, default=42.4, help="first position, deg (default 42.4)")
    ap.add_argument("--pos-b", type=float, default=-45.0, help="second position, deg (default -45)")
    ap.add_argument("--speed", type=float, default=30.0, help="move speed, deg/s (default 30)")
    ap.add_argument("--dwell", type=float, default=1.0, dest="dwell_s",
                    help="pause at each end, s (default 1); ignored if --dwell-min/--dwell-max are given")
    ap.add_argument("--dwell-min", type=float, default=None,
                    help="starting dwell time, s, for a progressive ramp (requires --dwell-max)")
    ap.add_argument("--dwell-max", type=float, default=None,
                    help="ending dwell time, s, for a progressive ramp (requires --dwell-min)")
    ap.add_argument("--dwell-ramp-cycles", type=int, default=20,
                    help="cycles to ramp dwell from min to max, then hold at max (default 20)")
    ap.add_argument("--cycles", type=int, default=0, help="round trips to run, 0 = forever (default 0)")
    ap.add_argument("--period", type=float, default=0.0,
                    help="seconds between the start of each cycle, 0 = back-to-back (default 0); "
                         "ignored if --period-min/--period-max are given")
    ap.add_argument("--period-min", type=float, default=None,
                    help="starting period, s, for a progressive ramp (requires --period-max)")
    ap.add_argument("--period-max", type=float, default=None,
                    help="ending period, s, for a progressive ramp (requires --period-min)")
    ap.add_argument("--period-ramp-cycles", type=int, default=20,
                    help="cycles to ramp period from min to max, then hold at max (default 20)")
    ap.add_argument("--port", default=None, help="COM port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (default 115200)")
    ap.add_argument("--id", type=int, default=1, dest="servo_id", help="servo ID (default 1)")
    ap.add_argument("-l", "--log", action="store_true",
                    help="write per-cycle log to VolzTest_<date>_<time>.csv")
    ap.add_argument("--check", action="store_true", help="comms check only, no motion")
    args = ap.parse_args()

    if (args.dwell_min is None) != (args.dwell_max is None):
        ap.error("--dwell-min and --dwell-max must be given together")
    if (args.period_min is None) != (args.period_max is None):
        ap.error("--period-min and --period-max must be given together")
    dwell_ramp = args.dwell_min is not None
    period_ramp = args.period_min is not None

    port = args.port or find_port()
    if port is None:
        print("No USB-RS485 adapter found. Available ports:")
        for p in list_ports.comports():
            print(f"  {p.device}: {p.description}")
        print("Specify one with --port COMx")
        sys.exit(1)

    try:
        servo = VolzServo(port, args.baud, args.servo_id)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("If the Volz VISIO app is running, close it - it holds the port open.")
        sys.exit(1)

    print(f"Connected to {port} @ {args.baud} baud, servo ID {args.servo_id}")

    position = servo.read_position()
    if position is None:
        print("ERROR: no response from servo. Check wiring/ID/baud, and that the")
        print("Volz VISIO app is closed.")
        servo.close()
        sys.exit(1)

    volts = servo.read_voltage()
    amps = servo.read_current()
    temps = servo.read_temperature() or (None, None)
    print(f"Servo responding: position {position:+.2f} deg"
          + (f", supply {volts:.1f} V" if volts is not None else "")
          + (f", current {amps:.2f} A" if amps is not None else "")
          + (f", motor {temps[0]} C" if temps[0] is not None else "")
          + (f", PCB {temps[1]} C" if temps[1] is not None else ""))

    if args.check:
        servo.close()
        sys.exit(0)

    start_time = datetime.now()
    logfile = None
    if args.log:
        logname = start_time.strftime("VolzTest_%Y-%m-%d_%H-%M-%S.csv")
        logfile = open(logname, "w", encoding="utf-8", newline="")
        logfile.write(f"# Volz servo test started {start_time:%Y-%m-%d %H:%M:%S}\n")
        logfile.write(f"# Port {port}, servo ID {args.servo_id}, baud {args.baud}, "
                      f"sampling every {LOG_INTERVAL_S:g}s\n")
        dwell_desc = (f"dwell ramp {args.dwell_min:g}->{args.dwell_max:g} s over "
                      f"{args.dwell_ramp_cycles} cycles" if dwell_ramp else f"dwell {args.dwell_s:g} s")
        logfile.write(f"# Commanded positions {args.pos_a:+.2f} / {args.pos_b:+.2f} deg, "
                      f"speed {args.speed:g} deg/s, {dwell_desc}\n")
        logfile.write("time,cycle,phase,position_deg,current_A,motor_temp_C,pcb_temp_C\n")
        logfile.flush()
        print(f"Logging to {logname}")

    if period_ramp:
        period_note = f", period ramp {args.period_min:g}->{args.period_max:g}s over {args.period_ramp_cycles} cycles"
    elif args.period > 0:
        period_note = f", one cycle every {args.period:g}s"
    else:
        period_note = ""
    dwell_note = (f"dwell ramp {args.dwell_min:g}->{args.dwell_max:g}s over {args.dwell_ramp_cycles} cycles"
                  if dwell_ramp else f"dwell {args.dwell_s:g}s")
    print(f"Cycling {args.pos_a:+.1f} <-> {args.pos_b:+.1f} deg at {args.speed:g} deg/s, {dwell_note}"
          f" ({'forever' if args.cycles == 0 else f'{args.cycles} cycles'}){period_note} - Ctrl+C to stop")

    telem = Telemetry(servo, logfile=logfile)
    try:
        # Ramp from wherever the servo actually is to the start position.
        telem.cycle, telem.phase = 0, "start"
        current = move_to(servo, telem, position, args.pos_a, args.speed)
        cycle = 0
        while args.cycles == 0 or cycle < args.cycles:
            cycle += 1
            telem.cycle = cycle
            cycle_start_perf = time.perf_counter()
            start_pos = servo.read_position()
            telem.reset_samples()

            dwell_s = (ramp_value(cycle, args.dwell_min, args.dwell_max, args.dwell_ramp_cycles)
                       if dwell_ramp else args.dwell_s)

            telem.phase = "dwell_a"
            dwell(servo, telem, dwell_s)
            print(f"Cycle {cycle}: -> {args.pos_b:+.1f} deg" + (f" (dwell {dwell_s:.2f}s)" if dwell_ramp else ""))
            telem.phase = "to_b"
            current = move_to(servo, telem, current, args.pos_b, args.speed)
            telem.phase = "dwell_b"
            dwell(servo, telem, dwell_s)
            print(f"Cycle {cycle}: -> {args.pos_a:+.1f} deg")
            telem.phase = "to_a"
            current = move_to(servo, telem, current, args.pos_a, args.speed)

            end_pos = servo.read_position()
            telem.poll_temp()
            summary = (f"Cycle {cycle} done: start {fmt(start_pos, '+.2f')} deg, "
                       f"end {fmt(end_pos, '+.2f')} deg, "
                       f"avg {fmt(telem.avg_current, '.2f')} A, "
                       f"peak {fmt(telem.peak_current, '.2f')} A, "
                       f"motor {telem.motor_temp} C, PCB {telem.pcb_temp} C")
            print(summary)

            period = (ramp_value(cycle, args.period_min, args.period_max, args.period_ramp_cycles)
                      if period_ramp else args.period)
            if period > 0:
                telem.phase = "idle"
                remaining = period - (time.perf_counter() - cycle_start_perf)
                if remaining > 0:
                    print(f"Idle {remaining:.1f}s until next cycle (period {period:.2f}s)")
                    dwell(servo, telem, remaining)
                else:
                    print(f"Warning: cycle took longer than the {period:.2f}s period "
                          f"({time.perf_counter() - cycle_start_perf:.1f}s)")
        print("Done.")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if logfile:
            logfile.close()
        servo.close()


if __name__ == "__main__":
    main()
