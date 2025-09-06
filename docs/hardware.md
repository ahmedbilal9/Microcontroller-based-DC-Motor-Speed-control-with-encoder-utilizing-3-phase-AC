# Hardware Components & Connectivity

### Hardware Connectivity
- DC motor is connected to the output of the voltage regulator (Three-phase half-wave controlled rectifier).
- Encoder outputs are connected to STM32, which is then connected to a PC for RPM measurement.
- Speed is controlled by varying the firing angle.

### Hardware Components
- **Constant DC Supply Circuit:**
  - Diodes
  - Screw connectors
  - Vero board

- **Variable DC Converter (Three-phase half-wave controlled rectifier):**
  - Thyristor
  - Transformers
  - Vero board
  - Screw connectors
  - MOSFET
  - Jumper wires

- **Measurement Unit:**
  - Encoder (built-in with motor)
  - STM32
  - Jumper wires
