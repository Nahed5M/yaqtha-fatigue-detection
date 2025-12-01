## Project Overview
- Uses OpenCV + MediaPipe FaceMesh to analyze real-time eye landmarks  
- Calculates EAR (Eye Aspect Ratio) to detect fatigue levels  
- Displays an on-screen alert when fatigue is detected  
- Sends a command to Arduino to trigger:
 1- Red LED  
2- Buzzer alert  
- Normal state is shown through a green LED
  
---

## Features
- Real-time face and eye landmark detection  
- EAR-based fatigue calculation  
- Communication between Python and Arduino using Serial interface
- LED indicators for status  
- Buzzer alarm for fatigue  
- Fully modular (Software + Hardware layers)

---

## System Architecture
**Input Layer**  
Camera captures the driver's face in real-time.

**Processing Layer (Python – Computer Vision)**  
- Detect facial landmarks using MediaPipe FaceMesh  
- Calculate EAR (Eye Aspect Ratio)  
- Evaluate fatigue based on threshold  
- Generate status output (Normal / Fatigued)

**Communication Layer (Serial Interface)**  
- Python sends command ("G" for normal, "R" for fatigue) to Arduino via serial port

**Hardware Action Layer (Arduino Unit)**  
- "G" activates Green LED (Normal state)  
- "R" activates Red LED + Buzzer (Fatigue alert)

---

## Software (Python)
File: `fatigue_app.py`

Python script responsible for:
- Capturing camera feed  
- Detecting face landmarks  
- Calculating EAR  
- Determining fatigue  
- Sending signals (R/G) to Arduino  

---

## Hardware (Arduino)
File: `fatigue_gate.ino`

Arduino sketch performs:
- Reading serial commands  
- Activating Green/Red LEDs  
- Triggering buzzer on fatigue  

---

## Technologies Used
- Python 3  
- OpenCV  
- MediaPipe FaceMesh  
- NumPy  
- PySerial  
- Arduino Uno 

---

## How to Run
### 1) Install dependencies:
```bash
pip install opencv-python mediapipe numpy pyserial
```
### 2) Run Python script:
```bash
python fatigue_app.py
```
### 3) Upload Arduino code:
Open `fatigue_gate.ino` in Arduino IDE and upload it.

---
## Author

**Amnah Alkhater**  
Lead Developer and Technical Owner of the Yaqtha Fatigue Detection Project

LinkedIn: https://www.linkedin.com/in/amnahalkhater
