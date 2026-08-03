#!/usr/bin/env bash
# Cycles the Volz DA-15N between +42.4 and -45 deg at 30 deg/s.
# Edit the numbers below to change positions or speed (deg/s).
# Add -l to the line below to write a VolzTest_<date>_<time>.csv log file.
cd "$(dirname "$0")"
python3 volz_servo.py --pos-a 42.4 --pos-b -45 --speed 30
