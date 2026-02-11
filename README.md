# Microcontroller-Based DC Motor Speed Control with Encoder Utilizing 3-Phase AC

## Project Overview

This project implements a precise DC motor speed control system using a microcontroller-based approach with encoder feedback and a three-phase half-wave controlled rectifier circuit. The system achieves real-time speed regulation through closed-loop control, making it suitable for applications in robotics, industrial automation, and electric vehicle systems.

## Technical Specifications

### Hardware Components
- Microcontroller: STM32F103C8T6 (ARM Cortex-M3)
- DC Motor: 12V, 3000 RPM rated
- Encoder: Incremental rotary encoder (360 pulses per revolution)
- Power Stage: Three-phase half-wave controlled rectifier
- Thyristors: SCR-based switching circuit
- Transformer: 220V AC to 18V AC (3-phase)

### Control Features
- Closed-loop speed control with encoder feedback
- Real-time firing angle regulation (0-180 degrees)
- PID controller implementation for stable operation
- Load compensation and disturbance rejection
- Serial communication interface for monitoring

## Repository Structure

```
.
├── README.md
├── docs/
│   ├── PROJECT_REPORT.md
│   ├── HARDWARE_DESIGN.md
│   ├── SOFTWARE_ARCHITECTURE.md
│   └── TESTING_RESULTS.md
├── src/
│   ├── main.c
│   ├── motor_control.c
│   ├── motor_control.h
│   ├── encoder.c
│   ├── encoder.h
│   ├── pid_controller.c
│   ├── pid_controller.h
│   ├── thyristor_driver.c
│   └── thyristor_driver.h
├── schematics/
│   ├── circuit_diagram.pdf
│   └── pcb_layout.pdf
├── results/
│   ├── speed_response.png
│   ├── load_test.png
│   └── efficiency_curve.png
├── simulation/
│   └── proteus_model.pdsprj
└── LICENSE

```

## System Architecture

### Control Loop
1. Encoder reads actual motor speed
2. Speed error calculated (setpoint - actual)
3. PID controller computes firing angle
4. Thyristor driver adjusts phase angle
5. Rectifier output voltage controls motor speed

### Key Algorithms
- Encoder pulse counting with Timer/Counter peripheral
- Digital PID controller with anti-windup
- Zero-crossing detection for thyristor firing
- UART-based real-time data logging

## Installation and Setup

### Prerequisites
- STM32CubeIDE or Keil MDK
- STM32 ST-LINK Utility
- ARM GCC Toolchain
- Serial terminal (PuTTY, Tera Term)

### Hardware Setup
1. Connect three-phase AC supply to rectifier input
2. Wire thyristor gates to microcontroller PWM outputs
3. Connect encoder channels A and B to microcontroller interrupt pins
4. Connect motor terminals to rectifier output
5. Establish UART connection for monitoring

### Software Compilation
```bash
# Clone repository
git clone https://github.com/ahmedbilal9/Microcontroller-based-DC-Motor-Speed-control-with-encoder-utilizing-3-phase-AC.git
cd Microcontroller-based-DC-Motor-Speed-control-with-encoder-utilizing-3-phase-AC

# Open project in STM32CubeIDE
# Build project (Ctrl+B)
# Flash to microcontroller via ST-LINK
```

### Configuration Parameters
Edit `motor_control.h` to adjust:
- Target speed setpoint (RPM)
- PID gains (Kp, Ki, Kd)
- Encoder resolution
- Firing angle limits

## Performance Results

### Speed Control Accuracy
- Steady-state error: Less than 2%
- Rise time: 0.8 seconds
- Overshoot: Less than 5%
- Settling time: 1.5 seconds

### Load Response
- Load disturbance rejection within 1 second
- Speed drop under 50% rated load: Less than 3%
- Recovery time: 0.6 seconds

### Efficiency
- Overall system efficiency: 82-87%
- Power factor: 0.85 (with rectifier)

## Testing Procedures

Detailed testing documentation available in `docs/TESTING_RESULTS.md`:
- No-load speed verification
- Step response analysis
- Load impact testing
- Long-term stability evaluation
- Thermal performance assessment

## Safety Considerations

### Electrical Safety
- High voltage AC input requires proper insulation
- Thyristor heat sinking mandatory for continuous operation
- Fuse protection on AC input recommended
- Ground all metal enclosures

### Operational Safety
- Do not exceed motor rated voltage
- Monitor motor temperature during extended operation
- Encoder wiring should avoid high-voltage traces
- Emergency stop circuit recommended

## Applications

### Industrial Automation
- Conveyor belt speed control
- Packaging machinery
- Assembly line equipment

### Robotics
- Mobile robot drive systems
- Robotic arm joint control
- Autonomous vehicle propulsion

### Electric Vehicles
- Low-speed vehicle traction control
- Auxiliary system drives
- Regenerative braking systems

## Future Enhancements

- Implement sensorless speed estimation for redundancy
- Add current limiting for motor protection
- Develop GUI-based tuning interface
- Integrate CAN bus communication
- Implement adaptive PID tuning
- Add fault diagnostics and logging

## Technical Documentation

Comprehensive documentation available in `docs/` directory:
- `PROJECT_REPORT.md`: Complete project description and methodology
- `HARDWARE_DESIGN.md`: Circuit design and component selection
- `SOFTWARE_ARCHITECTURE.md`: Code structure and algorithm details
- `TESTING_RESULTS.md`: Experimental data and analysis

## References

1. STM32F103 Reference Manual - STMicroelectronics
2. "Power Electronics: Converters, Applications, and Design" - Mohan, Undeland, Robbins
3. "Feedback Control of Dynamic Systems" - Franklin, Powell, Emami-Naeini
4. IEEE Standards for Motor Control Systems

## Contributing

Contributions are welcome. Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Document code changes thoroughly
4. Submit pull request with detailed description

## License

This project is licensed under the MIT License. See LICENSE file for details.

## Author

Ahmed Bilal
Electrical Engineering Student | Embedded Systems & Power Electronics

- GitHub: https://github.com/ahmedbilal9
- LinkedIn: https://linkedin.com/in/ahmedbilal9
- Email: ahmedbilalned@gmail.com

## Acknowledgments

- NED University of Engineering and Technology
- Department of Electrical Engineering
- Laboratory supervisors and technical staff

---

Last Updated: 2026-02-11 14:10:45