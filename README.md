# STM32 Active Aerodynamics System

> **Status: In development**

Engineering thesis project focused on the design and implementation of an embedded active aerodynamic rear-wing control system.

The system uses vehicle data acquired through the OBD-II CAN bus together with inertial measurements from an IMU to automatically select the position of an aerodynamic wing driven by a servo motor.

---

## Project Goal

The goal of the project is to develop a prototype active aerodynamic system capable of automatically changing the position of a rear wing depending on the current vehicle state.

The system currently supports three wing states:

- **NORMAL** – default aerodynamic position providing a moderate angle of attack
- **OPEN** – low-drag position used at higher vehicle speeds
- **AIR_BRAKE** – high-drag position activated during strong braking

The system also uses lateral acceleration measurements to detect cornering. During a detected corner, the wing is forced into the NORMAL position and AIR_BRAKE activation is blocked.

---

## Hardware

The current prototype is based on:

- **STM32 NUCLEO-G474RE**
- **STM32G474RE microcontroller**
- **SN65HVD230 CAN transceiver**
- **LSM6DSO 6-axis IMU**
- Servo motor used for wing actuation
- OBD-II connection to the vehicle CAN bus
- External 5 V servo power supply

The STM32 board, CAN transceiver and servo power supply use a common ground.

---

## System Overview

The system combines information obtained from the vehicle CAN bus with measurements from the inertial sensor.

The general data flow is:


Vehicle
   |
   | OBD-II / CAN
   v
SN65HVD230
   |
   v
STM32G474RE
   |
   +----------------------+
   |                      |
   |                      v
   |               Control Algorithm
   |                      |
   |                      v
   |                  Servo PWM
   |                      |
   |                      v
   |              Aerodynamic Wing
   |
   v
LSM6DSO IMU


The control algorithm currently uses:

- vehicle speed
- calculated longitudinal acceleration/deceleration
- lateral acceleration measured by the IMU
- configurable speed thresholds
- hysteresis
- consecutive-sample validation
- post-braking state hold

---

## CAN / OBD-II Communication

Vehicle data is obtained through the OBD-II connector using CAN communication.

The STM32 uses the integrated **FDCAN peripheral configured in Classic CAN mode**.

The physical CAN interface is provided by the **SN65HVD230 CAN transceiver**.

Current OBD-II connections:


OBD-II pin 6   -> CANH
OBD-II pin 14  -> CANL
OBD-II pin 5   -> GND


The system sends standard OBD-II requests using the functional request identifier:


0x7DF


ECU responses are expected in the range:


0x7E8 - 0x7EF


---

## Vehicle Speed

Vehicle speed is currently the primary OBD-II parameter used by the control algorithm.

Standard OBD-II request:


Mode: 01
PID:  0D


Request data:


02 01 0D 00 00 00 00 00


Typical positive response:


03 41 0D A ...


where:


A = vehicle speed in km/h


The current implementation sends the vehicle speed request periodically every:


500 ms


---

## IMU

The system uses an **LSM6DSO** inertial measurement unit.

The sensor contains:

- 3-axis accelerometer
- 3-axis gyroscope

Currently only the accelerometer is used by the control algorithm.

Communication with the STM32 is performed using:


I2C


The firmware automatically checks both possible LSM6DSO I2C addresses:


0x6A
0x6B


The device is verified using the `WHO_AM_I` register.

Expected value:


0x6C


---

## Accelerometer Configuration

Current accelerometer configuration:


Measurement range: ±2 g
Output data rate:  104 Hz


The accelerometer values are internally represented in milli-g:


1000 mg = 1 g


The lateral axis can be changed in the firmware depending on the final physical orientation of the sensor.

---

## Corner Detection

The IMU is used to detect lateral acceleration during cornering.

The currently selected lateral acceleration axis is:


Y


The firmware applies a simple first-order low-pass filter:


filtered = filtered + (raw - filtered) / 4


This reduces the influence of short acceleration spikes caused by:

- vibration
- road irregularities
- mechanical shocks

Corner detection uses two different thresholds in order to provide hysteresis.

Current configuration:


Corner ON threshold:  200 mg
Corner OFF threshold: 120 mg


Multiple consecutive samples are required before the corner state changes.

Current configuration:


Corner detection: 3 consecutive samples
Corner clearing:  5 consecutive samples


This prevents a single acceleration spike from immediately changing the wing position.

---

## Control Algorithm

The control system determines the requested aerodynamic state using vehicle speed, calculated longitudinal acceleration and lateral acceleration.

The available states are:


NORMAL
OPEN
AIR_BRAKE


### NORMAL

NORMAL is the default wing position.

The system returns to NORMAL when:

- vehicle speed is low
- OPEN conditions are not satisfied
- braking ends
- a corner is detected
- AIR_BRAKE is blocked

NORMAL is also the initial wing position after system startup.

### OPEN

OPEN represents the low-drag wing configuration.

The purpose of this state is to reduce aerodynamic resistance during higher-speed driving when additional aerodynamic drag is not required.

Current speed hysteresis:


OPEN activation threshold:   55 km/h
OPEN deactivation threshold: 45 km/h


The difference between the activation and deactivation thresholds prevents rapid state switching around a single speed value.

OPEN is not activated immediately after crossing the speed threshold.

The vehicle must remain without significant deceleration for several consecutive speed samples.

Current configuration:


OPEN stable samples: 3


This helps prevent unstable switching caused by the limited resolution of the OBD-II vehicle speed PID.

### AIR_BRAKE

AIR_BRAKE represents the maximum-drag wing position.

The state is activated when sufficiently strong vehicle deceleration is detected.

Current minimum vehicle speed for AIR_BRAKE:


40 km/h


Longitudinal acceleration is estimated using consecutive vehicle speed measurements:


delta_v = current_speed - previous_speed


and:


acceleration_rate = delta_v / delta_time


The internal acceleration rate is represented using:


0.1 km/h/s


The calculated value is also converted approximately to g-force for diagnostic output.

AIR_BRAKE is only allowed when the vehicle is not currently detected as cornering.

---

## Post-Braking Behaviour

AIR_BRAKE is not intentionally held after braking is no longer detected.

When braking ends:


AIR_BRAKE -> NORMAL


The NORMAL state is then maintained for several speed samples before OPEN can be activated again.

Current configuration:


POST_BRAKE_NORMAL_SAMPLES = 3


With a vehicle speed sampling period of approximately 500 ms, this corresponds to approximately:


1.5 s


This prevents an immediate transition:


AIR_BRAKE -> OPEN


after braking.

---

## Cornering Priority

Corner detection has higher priority than OPEN and AIR_BRAKE.

When a corner is detected:


OPEN      -> NORMAL
AIR_BRAKE -> NORMAL
NORMAL    -> NORMAL


While the corner state is active:


AIR_BRAKE activation is blocked


The purpose is to avoid introducing a large aerodynamic state change during cornering.

---

## Servo Control

The aerodynamic wing is actuated using a servo motor.

The servo is controlled using hardware PWM generated by:


TIM2


Current PWM frequency:


50 Hz


The PWM period is therefore:


20 ms


Current prototype pulse widths:


OPEN       = 1500 us
NORMAL     = 1700 us
AIR_BRAKE  = 2000 us


These values are temporary and will be calibrated after the mechanical wing prototype is completed.

The final values depend on:

- mechanical linkage
- servo mounting position
- available servo travel
- required aerodynamic angles

PWM generation is performed by the STM32 hardware timer, so the CPU does not generate servo pulses in software.

---

## Software Structure

The firmware is being refactored from a single large `main.c` file into separate functional modules.

Current project structure:


Core/
├── Inc/
│   ├── main.h
│   ├── app_types.h
│   ├── debug_utils.h
│   ├── servo.h
│   ├── imu.h
│   ├── obd.h
│   └── wing_control.h
│
└── Src/
    ├── main.c
    ├── debug_utils.c
    ├── servo.c
    ├── imu.c
    ├── obd.c
    └── wing_control.c


### `main.c`

Responsible for:

- STM32 initialization
- CubeMX-generated peripheral configuration
- main application loop
- coordination between application modules

### `obd.c`

Responsible for:

- CAN configuration used by the application
- OBD-II request transmission
- CAN RX processing
- OBD-II response parsing

### `imu.c`

Responsible for:

- LSM6DSO initialization
- I2C communication
- accelerometer data conversion
- signal filtering
- corner detection

### `wing_control.c`

Responsible for:

- vehicle motion state determination
- OPEN logic
- AIR_BRAKE logic
- speed hysteresis
- post-braking behaviour
- integration of vehicle speed and corner information

### `servo.c`

Responsible for:

- servo PWM control
- mapping wing states to servo pulse widths
- updating servo position when the wing state changes

### `debug_utils.c`

Responsible for:

- diagnostic formatting functions
- conversion of internal values to readable serial output

---

## STM32CubeMX Generated Code

Peripheral initialization remains managed by STM32CubeMX.

Functions such as:

c
MX_GPIO_Init();
MX_FDCAN1_Init();
MX_TIM2_Init();
MX_I2C1_Init();


remain in the CubeMX-generated part of the project.

Application modules use the already configured hardware peripherals instead of duplicating their initialization.

This provides a clear separation between:


hardware configuration


and:


application logic


---

## Current Firmware Architecture

The current firmware is intentionally kept relatively simple while the mechanical prototype and control algorithm are still being validated.

Current implementation:


CAN receive        -> polling
OBD request timing -> HAL_GetTick()
IMU sampling       -> HAL_GetTick()
I2C communication  -> blocking HAL functions
Servo control      -> hardware PWM
UART debug output  -> printf()


This architecture simplifies early testing and debugging.

The final firmware architecture is planned to become more event-driven after the functional behaviour of the prototype is validated.

---

## Planned Firmware Improvements

Future firmware development includes:

- replacing CAN receive polling with FDCAN receive interrupts
- introducing a CAN RX software queue
- removing unnecessary blocking delays
- replacing blocking I2C communication with DMA
- using the LSM6DSO Data Ready interrupt
- separating diagnostic logging from control logic
- adding CAN communication timeout handling
- adding IMU communication timeout handling
- implementing fail-safe behaviour
- adding an independent watchdog
- improving error handling
- final calibration of control thresholds

---

## Mechanical Prototype

A physical wing prototype is currently being developed.

The prototype will be used to determine:

- final servo mounting geometry
- wing rotation range
- servo pulse limits
- aerodynamic state angles
- IMU mounting orientation
- mechanical response of the system

The current servo positions and IMU thresholds are therefore considered development values.

---

## Current Development Status

Currently implemented:

- STM32G474RE project configuration
- Classic CAN communication
- SN65HVD230 CAN interface
- OBD-II request transmission
- OBD-II vehicle speed reception
- vehicle speed parsing
- longitudinal acceleration estimation
- three-state wing control algorithm
- speed hysteresis
- braking detection
- post-braking NORMAL hold
- servo PWM control
- LSM6DSO communication over I2C
- accelerometer data acquisition
- lateral acceleration filtering
- corner detection
- AIR_BRAKE inhibition during cornering
- automatic return to NORMAL during cornering
- UART diagnostic output
- initial firmware modularization

Currently under development:

- physical aerodynamic wing prototype
- final servo calibration
- real-world IMU threshold calibration
- further vehicle testing
- final embedded firmware architecture

---

## Planned Development

The next major stages are:

1. Complete the physical wing prototype.
2. Determine the final servo mounting geometry.
3. Calibrate OPEN, NORMAL and AIR_BRAKE positions.
4. Determine the final IMU mounting orientation.
5. Calibrate corner detection thresholds.
6. Perform vehicle tests.
7. Validate the wing control algorithm.
8. Replace CAN polling with interrupt-based reception.
9. Introduce I2C DMA and IMU Data Ready interrupts.
10. Implement communication timeouts and fail-safe behaviour.
11. Add watchdog protection.
12. Perform final system validation.

---

## Development Environment

The project is developed using:

- **STM32CubeIDE**
- **STM32CubeMX**
- **STM32 HAL**
- **C**
- **Git**
- **GitHub**

Target platform:


STM32 NUCLEO-G474RE
STM32G474RE
ARM Cortex-M4


---

## Project Status

This project is actively under development as part of an engineering thesis.

The repository represents the current development state and may contain temporary configuration values used during prototype testing.

---

## Disclaimer

This system is an academic and experimental prototype.

It is not a production-ready automotive control system and has not been designed or certified according to automotive functional safety standards.