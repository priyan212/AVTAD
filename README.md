# AVTAD — Active Visual Tracking & Defense Mechanism

AVTAD (Active Visual Tracking And Defense) is a research/prototype repository that combines computer-vision-based detection with physical actuation (pan-tilt, laser, omni-wheel vehicle) to detect, track, and react to aerial targets (e.g., drones). The repo contains detection models, control scripts, Arduino sketches, demo videos, and a MATLAB report used for analysis.

---

## Table of contents

* [Features](#features)
* [Repository structure](#repository-structure)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Configuration](#configuration)
* [How to run](#how-to-run)
* [Hardware setup & wiring notes](#hardware-setup--wiring-notes)
* [Models included](#models-included)
* [Viewing demos & reports](#viewing-demos--reports)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)

---

## Features

* Real-time visual detection and tracking using provided model weights.
* Pan-tilt control to aim a laser or camera at detected targets.
* Omni-wheel vehicle demo for mobile tracking/manoeuvring.
* Multiple pretrained model weights (varying sizes/accuracy) included for quick testing.
* MATLAB-based analysis/reporting included.

---

## Repository structure

> (Top-level files and their purpose)

* `AVTAD_BASE.py` — Main detection/control script (base station logic). Edit the configuration at the top to point to your model, camera, and serial port.
* `AVTAD_MOUNT.py` — Mount/pan-tilt-focused script (aiming logic for servos/laser).
* `LARGE.pt`, `MEDIUM.pt`, `LIGHT.pt` — Pretrained model weights (drop-in `.pt` files for the detection pipeline).
* `PAN_TILT_LASER.ino` — Arduino sketch for pan-tilt laser control.
* `OMNI_WHEEL_SERIAL.ino` — Arduino sketch for omni-wheel vehicle motion via serial commands.
* `*.mp4` (e.g. `DETECTION_RADAR_DEMO.mp4`, `LASER_PAN_TILT.mp4`, `OMNI_WHEEL_VEHICLE_DEMO.mp4`, `OPTIMIZED_VEHICLE.mp4`, `VideoWithDrone.mp4`) — Demo videos showcasing detection, tracking, and vehicle movement.
* `MATLAB_REPORT.html`, `MATLAB_GRAPH.png`, `REPORT_ORGANISED.html` — Analysis, visualizations, and reports created using MATLAB.
* `WORK_FLOW.png` — High-level workflow diagram for the system.



---

## Prerequisites

* **Hardware**: USB webcam (or compatible camera), Arduino (Uno/Mega/any board with required pins), pan-tilt assembly and servos, motor drivers for omni-wheel vehicle, power supply for motors.
* **Software**:

  * Python 3.8+ (3.9/3.10 recommended)
  * `pip` for Python package management
  * Arduino IDE for uploading `.ino` sketches

**Typical Python libraries used** (install below):
`torch`, `opencv-python`, `numpy`, `pyserial`, `matplotlib`, `imutils` (or equivalent). If you prefer, create a `requirements.txt` and install from that.

---

## Installation

1. Clone the repository:

```bash
git clone https://github.com/priyan212/AVTAD.git
cd AVTAD
```

2. (Recommended) Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate   # Linux / macOS
venv\Scripts\activate.bat  # Windows (PowerShell/CMD)
```

3. Install dependencies. If a `requirements.txt` is added later, use that. Otherwise install common packages:

```bash
pip install torch opencv-python numpy pyserial matplotlib imutils
```

> **Note**: Install a GPU-enabled version of `torch` if you plan to use CUDA acceleration. See PyTorch's official site for platform-specific install commands.

---

## Configuration

Open `AVTAD_BASE.py` and `AVTAD_MOUNT.py` in a text editor. Near the top you should find configuration variables — common ones to look for and edit before the first run:

* `MODEL_PATH` or `WEIGHTS` — path to the model (e.g. `LIGHT.pt`).
* `CAMERA_INDEX` or `CAMERA_ID` — integer index for OpenCV camera (0 is default for first webcam).
* `SERIAL_PORT` — serial device for Arduino (e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`, or `COM3` on Windows).
* `BAUD_RATE` — serial speed (match value used in Arduino sketches).
* Detection thresholds (e.g. `CONFIDENCE_THRESHOLD`, `NMS_THRESHOLD`)

If the scripts accept CLI args, you can pass these at runtime; otherwise edit the variables in the source.

---

## How to run

### 1) Run the base detection + control pipeline (PC side)

```bash
python3 AVTAD_BASE.py
```

* This script runs the detection model on the configured camera stream and sends actuation commands to the Arduino via serial when targets are detected.

### 2) Run mount (pan-tilt only) mode

```bash
python3 AVTAD_MOUNT.py
```

* Use this to test and tune the pan-tilt aiming logic without the full vehicle stack.

### 3) Upload Arduino sketches

* Open `PAN_TILT_LASER.ino` in Arduino IDE and upload to the board attached to the pan-tilt assembly.
* Open `OMNI_WHEEL_SERIAL.ino` and upload to the vehicle controller board for omni-wheel demos.

> Refer to the comments at the top of the `.ino` files for pin mappings and default baud rates. Make sure `SERIAL_PORT` and `BAUD_RATE` in the Python scripts match the Arduino settings.

---

## Models included

* `LIGHT.pt` — smallest/fastest model (lower accuracy, lower latency).
* `MEDIUM.pt` — middle ground between speed and accuracy.
* `LARGE.pt` — largest/heaviest model (higher accuracy, more compute required).

**Tip**: Start with `LIGHT.pt` for real-time demos on CPU, then try `MEDIUM`/`LARGE` on GPU for better detection.

If you want to replace these with your own trained model, place your `.pt` file in the repo and update `MODEL_PATH`.

---

## Viewing demos & analysis

* Demo videos are included as MP4 files — open them with any media player to see recorded runs.
* MATLAB report and graphs are included in `MATLAB_REPORT.html` and `MATLAB_GRAPH.png` for analysis and offline viewing.

---

## Troubleshooting

* **No camera detected**: check `CAMERA_INDEX`, ensure camera not used by another application.
* **Serial port errors**: ensure correct port (`/dev/tty*` / `COM*`) and that no other application is holding the port. Match `BAUD_RATE` in Python and Arduino sketch.
* **Slow detection**: try `LIGHT.pt` or run with GPU-enabled Torch. Reduce frame size passed to the model for faster inference.
* **Servo jitter / mis-aim**: tune PID or damping code in `PAN_TILT_LASER.ino`/Python mount script; check mechanical looseness.

---

## Contributing

Contributions, issues, and feature requests are welcome. If you want changes to this README or code improvements, please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/xyz`)
3. Commit your changes and push
4. Open a Pull Request describing your changes

---
