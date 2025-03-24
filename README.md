# AutonomousLaneFollowingRobot
Vision based lane following robot (Arduino Nicla vision and ESP32 microcontroller)



This project is based on two subsystems, 
- Computer Vision based lane detection module
    - Arduino Nicla Vision with embedded camera, and processor
        - Vision library: OpenMV, programming in Micropython
        - Download openMV IDE from https://openmv.io/pages/download?gad_source=1&gclid=Cj0KCQjwhYS_BhD2ARIsAJTMMQarojCgbvqHUTITU58kk-nWdyTioWGosjfcoChAJUAWUw5wDYQKkoMaAjNtEALw_wcB
        - Niclavision.py has the runtime program for the image processing and lane detection module.
            - Capture image in real time
            - process the images (Thresholding, binary conversion, noise reduction)
            - Image perspective correction and bird eye view generation
            - Lane detection using regression algorithm
            - Outputs lane direction, and lateral lane position in image frame to the robot controller via UART interface.

            
- Robot controller with differential drive. 
    - Alvik Robot with differential drives, and a ESP32 microcontroller for robot controls
    - Recieves information via UART interface from Vision Module.
    - Implements a PID controller
    - Ziegler-Nichols method for controller tuning and optimisation
    - Robot controls using the program Alvik.py

Requirements: 
- Micropython
- OpenMV IDE
- Arduino IDE for Micropython
- Alvik Robot
- Nicla Vision module (OpenMV)
- ESLOV to I2C connector (for I2C and/or UART communication).
  
