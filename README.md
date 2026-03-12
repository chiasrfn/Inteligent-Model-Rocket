# Inteligent-Model-Rocket
Flight Control &amp; Apogee Detection

The Inteligent-Model-Rocket is an experimental sounding rocket developed as a final project for an exam of the Automation Engineering course (January 2024) at Sapienza University of Rome. The primary objective of the mission was to design and build a vehicle capable of autonomously reaching Apogee, detecting it through sensor fusion, and executing a safe recovery via automated parachute deployment.This project integrates high-frequency data acquisition, real-time state estimation via Kalman Filtering, and custom-built power electronics for pyrotechnic ignition.

Repository structure:

nimbus2024/: The core flight software. It manages the main mission logic (Sequential Functional Chart), sensor data fusion (IMU + Barometer), and the autonomous recovery trigger.

ground_test/: Specialized firmware used for static testing of the ejection charges and the parachute deployment mechanism.

launch_test.ino: Ground-based simulation code designed to verify the reliability of the Apogee detection algorithms.

test_hz_imu/: A utility script used to benchmark and optimize the sampling frequency of the sensors (up to 560Hz for the controller).

calibrazione_sensori/: A collection of scripts and data used for the calibration of the BNO085 IMU and BMP390 barometer to minimize measurement bias during high-acceleration flight.

Key Technical Specs:
1) Microcontroller: ESP-32.
2) Navigation: 6-DOF Sensor Fusion with Kalman Filter.
3) Telemetry: Real-time monitoring via Wi-Fi.
4) Recovery: Dual-deployment ready pyrotechnic channels



Launch video: https://drive.google.com/drive/folders/1Z4qFGuhKrDvThmkjfc8BpkpILZe07uZE?usp=sharing
