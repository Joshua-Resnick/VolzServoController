@echo off
rem Cycles the Volz DA-15N between +42.4 and -45 deg at 30 deg/s.
rem Edit the numbers below to change positions or speed (deg/s).
rem Add -l to the line below to write a VolzTest_<date>_<time>.csv log file.
cd /d "%~dp0"
python volz_servo.py --pos-a 42.4 --pos-b -45 --speed 30 -l
pause
