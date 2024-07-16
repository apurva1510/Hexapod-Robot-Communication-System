Hexapod Robot Communication System

This repository contains the communication system developed for a proof-of-concept Insect Robot, which is capable of crawling, avoiding obstacles, and autonomously heading towards a docking station upon receiving specific commands. The project leverages Bluetooth communication to control the Hexapod Robot using an Android device.

- Project Overview
  For my final year project, I was responsible for developing and implementing the communication system used to control the Hexapod Robot. Key tasks included researching the development environment, selecting the communication protocol, and assisting in the design decisions for the robot. The system comprises an Android app that acts as the client and a Raspberry Pi that serves as the server.

- Key Features
  - Remote Controller App: Developed using Java on Android Studio, the app includes UI elements like a joystick, toggles, and buttons to control various robot functions.
    - Directional Control: Basic demo app with four directional buttons.
    - Joystick Module: Allows movement and rotation of the robot in the XY plane.
    - Control Toggles and Buttons: Features for robot power on/off, different GAIT modes, homing, and emergency stop.
      
  - Bluetooth Communication: Commands are sent from the Android app to the Raspberry Pi using Bluetooth.
    - Client-Side: Android app sends out commands based on UI interactions.
    - Server-Side: Python script on the Raspberry Pi handles the incoming commands and integrates them into the robot's movement and homing mechanisms.
      
- Technologies Used
    - Android App Development: Java, Android Studio.
    - Bluetooth Programming: Bluetooth-Socket programming in Python.
    - Server-Client Architecture: Python scripting on Raspberry Pi.
      
- Learning Outcomes: This project provided hands-on experience with:
  - Bluetooth-Socket programming.
  - Android app development using Java.
  - Server-client relationship in communication systems.
