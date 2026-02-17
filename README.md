# LegSim

LegSim is a Python-based robotic leg simulation and animation toolkit.

It allows you to:

- Simulate a mechanical robot leg with linkage geometry
- Solve inverse kinematics interactively
- Save animation frames
- Auto-play motion sequences
- Generate ready-to-upload PlatformIO projects
- Export servo animations for:
  - Arduino Uno
  - ESP32
  - ESP32-S3

This project is designed for education, prototyping, and real hardware testing.

---

# 📦 Project Overview

LegSim consists of three main components:

## 1️⃣ NormalLeg.py
A simple 2-segment leg using analytic inverse kinematics.

- Real-time mouse-controlled IK
- Displays servo angles
- Educational example of 2D leg kinematics

---

## 2️⃣ RoboLeg.py
A more advanced leg model with:

- Lever mechanics
- Piston-based geometry
- Brute-force inverse kinematics
- Visualization using pygame

This simulates a mechanically realistic linkage system instead of a simple 2-link arm.

---

## 3️⃣ RoboLegAnimatorForESP32ESP32S3andArduino.py

🚀 This is the newest and most powerful tool in the repository.

It provides:

- Tkinter-based GUI
- Real-time IK solving
- Saveable animation frames
- Automatic playback
- PlatformIO project generator
- Arduino / ESP32 / ESP32-S3 support

This tool bridges simulation and real hardware.

---

# 🧠 How the Animator Works

### Geometry Simulation

The system models:

- Upper leg (L1)
- Lower leg (L2)
- Servo-driven piston mechanism
- Lever arm (HEBEL)
- Attachment geometry

Servo 2 does NOT directly control an angle.
Instead, it modifies piston length:

piston_length = STANGE_BASIS + HEBEL * sin(angle)


This creates realistic mechanical linkage motion.

---

# 🖥 GUI Features

The Tkinter app provides:

✔ Mouse-based inverse kinematics  
✔ Real-time servo angle display  
✔ Save servo positions  
✔ Clear saved positions  
✔ Playback saved animation  
✔ Step through frames  
✔ Export PlatformIO project  

Controls:

- Drag mouse → Solve IK
- "SPEICHERN" → Save current servo position
- "WIEDERGABE" → Auto-play saved positions
- "NÄCHSTE" → Step to next frame
- "EXPORT CODE" → Generate hardware project

---

# 🔧 PlatformIO Export System

The animator can automatically generate a complete PlatformIO project.

Supported boards:

- Arduino Uno
- ESP32
- ESP32-S3

The generator creates:

- platformio.ini
- src/main.cpp
- README.md
- .gitignore

The generated firmware:

- Stores animation frames in flash
- Plays animation in loop
- Allows serial control

---

# 🎮 Serial Commands (On Hardware)

After uploading firmware:

| Command | Action |
|---------|--------|
| s | Start animation |
| x | Stop animation |
| n | Next frame |
| r | Reset to beginning |

---

# 📡 Hardware Requirements

- 2 Servo motors
- Arduino Uno OR ESP32 OR ESP32-S3
- External power supply recommended for servos
- USB cable for programming

Servo pins are configurable during export.

---

# 🛠 Installation

## Requirements

Python 3.8+

For simulation:

pip install pygame


Tkinter is included in standard Python installations.

---

# ▶ Usage

## Run Basic Leg

python NormalLeg.py


## Run Mechanical Leg

python RoboLeg.py


## Run Animator + Hardware Export Tool

python RoboLegAnimatorForESP32ESP32S3andArduino.py


---

# 📂 File Overview

| File | Purpose |
|------|----------|
| NormalLeg.py | Basic 2-segment inverse kinematics |
| RoboLeg.py | Mechanical linkage simulation |
| RoboLegAnimatorForESP32ESP32S3andArduino.py | GUI animation + PlatformIO exporter |
| LICENSE | MIT License |

---

# 🎯 Project Goals

This project demonstrates:

- Inverse kinematics (analytic + brute force)
- Mechanical linkage simulation
- Servo animation sequencing
- Embedded firmware generation
- PlatformIO project automation
- Bridging Python simulation with real hardware

---

# ⚠ Notes

- This is an educational robotics project.
- IK is solved via brute-force search (not optimized).
- Servo limits are configurable inside the script.
- Always power servos externally when using ESP32 or Arduino.

---

# 📜 License

This project is released under the MIT License.

You are free to modify, distribute, and use it for personal or commercial projects.

---

# 🚀 Future Ideas

- 3D leg simulation
- Gait generation
- Trajectory interpolation
- Real-time serial live control
- WiFi / BLE control for ESP32
- Web-based UI

---

LegSim connects simulation to real robotics hardware in a simple and educational way.
