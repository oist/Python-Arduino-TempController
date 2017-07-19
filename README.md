# Python-Arduino-TempController
An open electronics temperature controller based on Arduino I/O and Python GUI.

##Summary
Simple logic, fuczzy logic, and PID control program resides on the Arduino which also reports thermocouple data. The PC python GUI sends command of setpoint and on/off command to Arduino.
The heater are now configured for transparent ITO heater. But since the code are open, you can modify the heater to peltier element etc as well as the temperature sensor.

##Dependencies
Python
PyQT4
PyQTgraph
lxml
numpy
