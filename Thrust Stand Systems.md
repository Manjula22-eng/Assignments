Electronic System Design for Thrust Stand:
Power Supply Considerations:
Battery: 4S-6S LiPo (14.8V to 25.2 nominal, up to 60V max)
ESC: Compatible with ESP32 signal (PWM control)

Measurement Calculations & Part Selection:
Battery Voltage Measurement:
Part: Voltage divider with ADC input (ESP32 ADC can handle 3.3V max)

Voltage Divider Calculation:
Assume
Input Voltage: 60V Max
ESP32 ADC reference Voltage: 3.3V

Resistor Selection:
R1 = 100k ohm, R2 = 5.6K ohm
Output Voltage:
Vout = Vin * R2 / R1 + R2
Vout = 60v * 5.6k/ 105.6k = Approximately 3.2 V
This Keeps the ADC input within range.

Current Measurement:
Part: ACS712-60A (Half Effect Sensor)
Calculation:
Sensitivity: 66m V/A
Maximum Current: 60A
Output Voltage at 60A:
Vout = 2.5 + (60A * 0.066V/A) = 6.46 V
Needs Voltage divider to scale within ESP32 ADC Range (0-3.3V).

Thrust Measurement:
Part: HX711 with 5KG Load cell
Load Cell Calculation:
Full scale output: 1m V/V
Excitation Voltage: 5V
Max Output Voltage:
1m V/V * 5V = 5mV
HX711 amplifies this for ESP32 ADC reading.

RPM Measurement:
Part: IR Photo diode + Encoder Disc
Calculation:
Max RPM: 100,000 RPM
Encoder Disc with 1slot
Frequency Calculation:
f = RPM * Slots / 60
f = 100,000 * 1 / 60 = 1666.67 Hz
ESP32 reads pulses via Interrupts.

Temperature Measurement:
Part: MAX6675 + K-Type Thermo couple
Range: Up to 150 degrees
Resolution: + (or) – 0.25 degrees
Interface: SPI

Key Components & Interfaces:
Battery (4S-6S LiPo): Provides power.
ESP32: Main controller, reads sensor data, processes it, and logs/display results.
Voltage Sensor (Divider): Converts 60V max to ESP32 ADC range (0–3.3V).
Current Sensor (ACS712-60A): Measures current and sends analog output to ESP32 ADC.
Thrust Sensor (Load Cell + HX711): Measures thrust up to 5kg.
RPM Sensor (IR Photodiode): Detects rotor speed and sends pulse signals to ESP32.
Temperature Sensor (MAX6675 + K-Type Thermocouple): Measures temperature up to 150°C.
ESC (Electronic Speed Controller): Controls motor speed via PWM from ESP32.
Display (OLED): Shows live data readings.
SD Card Module: Logs real-time data for analysis.

CONCLUSION:
This design ensures proper measurement of Voltage, current, thrust, RPM, and temperature using an ESP32 for data logging and control.
