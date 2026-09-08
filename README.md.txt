# STM32 Active Aerodynamics Controller

> 🚧 **Work in Progress**

Embedded active aerodynamics controller developed as part of an Engineering Thesis in Automation and Robotics at AGH University of Krakow.

The system uses an STM32G474RE microcontroller to acquire vehicle data through CAN / OBD-II and control an active rear wing using a servo actuator.

## Current features

- STM32G474RE / NUCLEO-G474RE
- CAN / OBD-II communication
- Vehicle speed acquisition
- Driving-state detection
- Active aerodynamic state control
- PWM servo control
- Configurable speed thresholds and hysteresis

## Planned development

- IMU integration
- Lateral acceleration detection
- Improved CAN communication
- Algorithm tuning
- Mechanical wing prototype
- Vehicle testing

## Status

The project is currently under active development and the control algorithm and hardware architecture may change.

## Disclaimer

This is an experimental engineering prototype and is not intended to be used as a production or safety-certified automotive system.
