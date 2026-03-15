# Intelligent-Model-Rocket
**Flight Control and Apogee Detection**

---

## Project Overview
The Intelligent-Model-Rocket is an experimental sounding rocket developed as a final project for the Automation Engineering course (January 2024) at **Sapienza University of Rome**. 

The primary mission objective was to design and construct a vehicle capable of:
* Autonomously reaching Apogee.
* Detecting the peak altitude through multi-sensor data fusion.
* Executing a safe recovery via an automated parachute deployment system.

This project integrates high-frequency data acquisition, real-time state estimation via Kalman Filtering, and custom-built power electronics for pyrotechnic ignition.

---

## Repository Structure

* `code/`: Contains all flight and test firmware:
    * `space_rocket.ino`: **Core Flight Software.** Manages the main mission logic (Sequential Functional Chart), sensor data fusion (IMU + Barometer), and the autonomous recovery trigger.
    * `ground_test.ino`: Specialized firmware used for static testing of ejection charges and the parachute deployment mechanism.
    * `launch_test.ino`: Ground-based simulation code designed to verify the reliability of Apogee detection algorithms.
    * `sensor_test.ino`: Utility script used to benchmark and optimize sampling frequency (up to 560Hz for the controller).
* `docs/`: Contains the full technical documentation:
    * `Technical_Report.pdf`: A comprehensive document detailing the rocket's design, mathematical models, and flight analysis (Available in Italian).
* `sensor_calibration/`: A collection of scripts and data used to calibrate the BNO085 IMU and BMP390 barometer, minimizing measurement bias during high-acceleration phases.
* `images/`: A selection of the most significant photos and plots from the development, assembly, and launch phases.
---

## Technical Specifications

* **Microcontroller**: ESP-32
* **Navigation**: 6-DOF Sensor Fusion with Kalman Filter
* **Telemetry**: Real-time monitoring via Wi-Fi
* **Recovery System**: Dual-deployment ready pyrotechnic channels
* **Sampling Rate**: Up to 560Hz

---

## Documentation and Media

* **Technical Report**: Detailed analysis of the rocket's design, sensor fusion algorithms, and test results (Available in Italian in the `docs/` folder).
* **Launch Video**: [Watch the flight test here](https://drive.google.com/drive/folders/1Z4qFGuhKrDvThmkjfc8BpkpILZe07uZE?usp=sharing).
---

## Academic Context
* **Institution**: Sapienza University of Rome
* **Course**: Automation Engineering
* **Date**: January 2024
