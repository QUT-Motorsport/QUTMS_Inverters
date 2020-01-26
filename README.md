![QUTMS Banner](https://github.com/QUT-Motorsport/QUTMS_Master/blob/master/src/qutmsBanner.jpg?raw=true)
# Inverters
Custom inverter and motor control system designed for Plettenberg Nova 15 brushless sensored motors. Each inverter communicates to the chassis controller over CANBUS using a line dedicated to inverters.

# Features
- ATmega64M1 microcontroller with integrated power stage controller and CANBUS interface
- Isolated motor encoder input and decoding
- Isolated high frequency gate drivers for the 12 high current mosfets
- Input current measurement and 2/3 output phase current measurement
- Integrated capacitor bleed resistor
- Cooling block temperature measurement
