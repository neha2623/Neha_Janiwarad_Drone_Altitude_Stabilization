# Adaptive Drone Altitude Stabilization System

A MATLAB-based drone altitude control system using Adaptive PID control for real-time stabilization and disturbance rejection.

---

# Features

- Adaptive PID Controller
- Real-Time Drone Stabilization
- Wind Disturbance Rejection
- Live Drone Animation
- Dynamic Gain Adaptation
- Root Locus Analysis
- Bode Stability Analysis
- Performance Metrics Evaluation

---

# System Model

The drone altitude dynamics are modeled using the transfer function:

G(s) = 1 / (s² + 2s + 5)

The controller stabilizes the drone altitude using closed-loop feedback control.

---

# Control Strategy

The system uses an Adaptive PID Controller where controller gains automatically increase when disturbance or tracking error increases.

This improves:
- disturbance rejection
- response speed
- flight stability

---

# Simulation Results

## 1. Live Adaptive Drone Stabilization
<img width="1301" height="652" alt="image" src="https://github.com/user-attachments/assets/34c769e1-f936-4464-b188-be3f79d9d58a" />
Real-time drone stabilization under wind disturbance conditions.

---

## 2. Adaptive Gain Scheduling

- Dynamic Kp graph changing during disturbance
<img width="697" height="462" alt="image" src="https://github.com/user-attachments/assets/16149ecb-9b6d-4a7f-b59d-1d8de1108864" />

Controller gains automatically adapt during disturbances for faster stabilization.

---

## 3. Closed Loop Step Response

<img width="1600" height="635" alt="image" src="https://github.com/user-attachments/assets/7d5b06f6-8bf7-4539-8bf2-5431e902d43c" />
- Closed-loop step response graph

Stable altitude tracking with low overshoot and fast settling time.

---

## 4. Stability Analysis

<img width="1600" height="663" alt="image" src="https://github.com/user-attachments/assets/fd33665a-520c-4f6f-8e45-743aa8ce7527" />

- Bode plot OR Root Locus plot


Frequency-domain stability analysis of the drone control system.

---

## 5. Final Performance Metrics


<img width="1600" height="664" alt="image" src="https://github.com/user-attachments/assets/af3521fd-d698-484a-95e2-3b2db94347b9" />

<img width="1400" height="859" alt="image" src="https://github.com/user-attachments/assets/67a260f6-0a66-421d-af34-9b1a19e3c0c8" />


---

# Performance Metrics

The following metrics are evaluated:

- Overshoot
- Settling Time
- Steady-State Error

These metrics help analyze:
- system stability
- response speed
- disturbance rejection capability

---

# Technologies Used

- MATLAB
- Control System Toolbox

---

# Future Improvements

- Simulink integration
- 3D drone visualization
- Autonomous trajectory tracking
- Multi-axis drone dynamics

---

# Conclusion

This project demonstrates how adaptive feedback control can stabilize a drone altitude system under external disturbances while maintaining stable and robust flight performance.

The project combines classical control systems concepts with real-time visualization for an intuitive UAV stabilization platform.
