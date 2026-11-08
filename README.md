# 5-DOF High-Torque Robotic Arm - Live 3D Simulation & Web Controller

Built specifically for the **3x 150 kg Servos + 2x MG996R Servos + ESP8266** robotic arm.

---

## 🚀 Quick Start (Localhost 3D Simulator)

### Method 1: 1-Click Launch (Recommended)
1. Double-click **`start_simulator.bat`** inside this folder.
2. It will automatically start the local Node.js server on `http://localhost:3000` and open it in your browser.

### Method 2: Manual Node.js Terminal
```bash
node server.js
```
Then navigate in Google Chrome or Microsoft Edge to:
👉 **`http://localhost:3000`**

### Method 3: Direct Browser Open (No Server Needed)
Double-click `public/index.html` to open it directly in Chrome or Edge.

---

## ⚡ Connecting to the Real Physical Arm

The interface offers **two zero-friction connection modes**:

### Mode 1: Direct USB Cable (Web Serial API) - ZERO Setup!
1. Plug your ESP8266 into your computer's USB port with a data cable.
2. In the web interface, click the **"USB Serial"** tab in the top header.
3. Click **"Connect USB Port"**, select your ESP8266's COM port in the browser popup, and click Connect.
4. **Done!** Every slider move and 3D angle now streams to the physical arm at 115200 baud with <5ms latency.

### Mode 2: Wi-Fi WebSocket (Wireless Control)
1. Enter your Wi-Fi credentials in `ESP8266_RobotArm/ESP8266_RobotArm.ino`.
2. Flash the sketch to the ESP8266 and open the Serial Monitor to get its IP (e.g. `192.168.1.50`).
3. In the web interface header, enter the IP and click **"Connect"**.
4. The system communicates bidirectionally over WebSockets (port 81) with live round-trip latency telemetry.

---

## 🔌 Hardware Wiring & Power Architecture

> [!WARNING]
> **CRITICAL POWER REQUIREMENT**: 
> The three 150 kg.cm servos have massive stall current draw (up to 8A - 10A each). 
> **NEVER** power the servos from the ESP8266 3.3V or 5V (Vin) pins! 
> You **MUST** use an external high-current power supply (6.0V - 7.4V, **at least 10A to 15A continuous** rating).

### Wiring Schematic (Direct ESP8266 GPIOs)

```
 [ External High-Current Power Supply ]
 (6.0V - 7.4V / 10A - 20A)
     (+) VCC ---------------------------> To ALL 5 Servos Red (+ power wire)
     (-) GND -------+-------------------> To ALL 5 Servos Brown/Black (- ground wire)
                    |
                    | (MANDATORY COMMON GROUND)
                    v
 [ ESP8266 NodeMCU / Wemos D1 Mini ]
     GND -----------+
     D1 (GPIO 5)  ----------------------> Servo 1 (Base Pitch / Shoulder - 150kg) Signal (Orange/Yellow)
     D2 (GPIO 4)  ----------------------> Servo 2 (Elbow Pitch - 150kg) Signal (Orange/Yellow)
     D5 (GPIO 14) ----------------------> Servo 3 (Wrist Pitch - 150kg) Signal (Orange/Yellow)
     D6 (GPIO 12) ----------------------> Servo 4 (Wrist Roll/Yaw - MG996R) Signal (Orange/Yellow)
     D7 (GPIO 13) ----------------------> Servo 5 (Gripper Claw - MG996R) Signal (Orange/Yellow)
```

---

## 🎮 Web Controller Features

1. **Authentic 3D Visualizer**:
   - Modeled to match the cylindrical PVC base, pink anodized 150kg servo bodies, dual steel U-brackets, long conduit tubes, and 2-jaw gripper claw.
   - Orbit controls: Left-click drag to rotate, right-click drag to pan, scroll wheel to zoom.
2. **Forward Kinematics (FK)**:
   - High-precision sliders (0° - 180°), fine step buttons (`-5`, `-1`, `+1`, `+5`), angle inversion checkboxes.
3. **Inverse Kinematics (IK)**:
   - Cartesian coordinate sliders (X, Y, Z, and Wrist Pitch) with live analytical 2-link/3-link kinematic solver.
4. **Keyframe Motion Sequencer**:
   - Save poses with duration delays, play automated pick-and-place routines, loop continuously, and export/import routines as JSON files.
5. **S-Curve Velocity Limiting**:
   - Firmware automatically ramps angles smoothly at a controlled speed (default: 40°/s) to protect the 150kg metal gears and eliminate inertial whip.
6. **Emergency Stop (E-STOP)**:
   - Prominent red button that halts any active movement or sequence immediately.

