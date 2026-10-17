# Two-Wheeled Self-Balancing (TWSB) Mobile Inverted Pendulum
**Multi-Agent Underactuated Platform — Phase I Closure**

[![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20Simulink%20%7C%20MATLAB-blue)]()
[![CAD](https://img.shields.io/badge/CAD-SolidWorks%202024-informational)]()
[![Control](https://img.shields.io/badge/Control-DLQR%20+%20Integrator%20%7C%20Cascaded%20PID-orange)]()
[![TRL](https://img.shields.io/badge/TRL-4%20(Lab%20Validated)-success)]()

This repository contains the complete Phase I hardware designs, multi-tier simulation models, control synthesis scripts, and real-time firmware for the **Two-Wheeled Self-Balancing (TWSB) Mobile Inverted Pendulum**. Developed under the i-Hub Foundation for Cobotics (IHFC) Ready program (Cohort 9), this platform is engineered as an agile, zero-radius-turn human-machine companion tailored for unstructured workspace logistics in Indian MSMEs.

---

## 📂 Repository Structure

```text
├── CAD/
│   └── Robot_Assemblies_and_Parts_SolidWorks2024.zip  # Full SolidWorks 2024 part files and tier assemblies
├── MATLAB/
│   ├── New_twmr_Neo_Linear_StateFeedback.m            # 6x6 Torque-input state-space & pole placement script
│   ├── LQR_Accel.m                                    # 6x6 Acceleration-input state-space with LQR controller
│   ├── LWR_Accel_int.m                                # 7x7 Augmented acceleration model with integral state (LQI)
│   └── discrete_ILQR_accel.m                          # Discrete-time version of the 7x7 LQI control script
├── Simulink/
│   ├── Simscape_Model_and_Data.zip                    # .slx model, workspace parameter data files, and URDF export
└── ESP32_Firmware/
    └── TWSB_Control_Firmware.ino                      # Standalone 100Hz real-time ESP32 C++ firmware script

