# Neha_Janiwarad_Drone_Altitude_Stabilization
Control Systems Hackathon - BNMIT

This project was developed for the Problem Statement 1: Drone Altitude Stabilization provided by RoboStrata Technologies. It fulfills all deliverables including the transfer function model, tuned parameters, step response plots, and robustness analysis.

##Key Engineering Features
1.Adaptive PID Control: Dynamic gain scheduling ($K_p$ and $K_d$) that adjusts based on the magnitude of the error to balance speed and stability.
2.Sensor Fusion (Complementary Filter): Combines noisy altitude readings with high-frequency velocity data to provide a smooth, reliable state estimate.
3.Real-World Constraints:
Actuator Saturation: Models physical motor thrust limits.
Integral Anti-Windup: Prevents overshoot during prolonged disturbances (clamping logic).
Motor Dynamics: Simulates the time-constant delay of physical brushless motors.
4.System Health Monitoring: Tracks battery discharge and simulates "In-Flight Faults" (e.g., 35% loss in motor efficiency).
5.Environmental Stress Testing: Includes a "Storm" scenario with harmonic wind oscillation and sudden high-force gusts.
