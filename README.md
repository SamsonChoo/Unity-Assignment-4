# Unity-Assignment-4

This project will allow the user to send commands from Unity to actuators connected to Arduino.

This project includes an Arduino file and a Unity project respectively.

To run the code, please ensure the following:
1. Your firewall is turned off.
2. Your VMWare Adapter is turned off.
3. Your computer and Arduino is connected to the same Wi-Fi(which requires no log-in).

Setup:
1. Arduino
	line 26: key in Wi-Fi name
	line 27: key in Wi-Fi password
	line 30: key in computer IP address
	
2. Unity
	OSCConnection: Key in Arduino IP address into Target IP Address of OSC Out (Obtained from serial monitor of Arduino IDE)
	Your OSC IN IP Address should be same as the computer IP address which you keyed into the Arduino code
	
How to use:
Upload the Arduino code, run the Unity code, press the buttons to control the actuators.
