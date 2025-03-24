#Shishir Pokhrel, University of Siegen, shishir.pokhrel@student.uni-siegen.de

#References:
    # Arduino Tutorials
    # Arduino Forums
    # Open-source community

from arduino_alvik import ArduinoAlvik
from machine import UART
from time import sleep_ms, sleep
import sys
import struct

# ----------------------------------------------------------------------------------
#This code runs on ArduinoAlvik (ESP board). It needs to be uploaded there. 
# Follow the instruction on the Alvik homepage for firmware and softwrae installation. 
# Open a new file, and paste this code, or upload this code on the device. 
# Once the code is uploaded to the device, it can be run at startup by calling the code on the main.py file. 
# ----------------------------------------------------------------------------------

# Initialize ArduinoAlvik
alvik = ArduinoAlvik()
alvik.begin()

# gains
THETA_GAIN = 2.0
RHO_GAIN = -1.0 # Rho is negative becaus eimage forms in lower right quadrant, but our perspective is on top right. 
P_GAIN = 12
I_GAIN = 8
D_GAIN = 1.5
I_MAX = 10
I_MIN = 0
IMAGE_UPDATE_INTERVAL = 100  # ms
BASE_SPEED = -22
# ----------------------------------------------------------------------------------
uart = UART(2, tx='A5', rx='A4', baudrate=115200)      # TX = A5, RX = A4
print("UART Initialized. Testing communication...") # Print in the terminal.

def receive_data():
    if uart.any():  # Check if there's data 
        data = uart.read()  # Read  available data
        received_data = data
        if data:
            theta, rho_error = struct.unpack('<ff', received_data)
#            print("Received:", data.decode('UTF-8').strip())
            print("Received:", theta, rho_error) # print the recieved data in terminal
            print("Received:", data.strip())
        else:
            print("No data received.")


# ----------------------------------------------------------------------------------
# Wait for user touch to start This is the tick button. It takes 50 ms to respond due to sleep. 
while alvik.get_touch_ok():
    sleep_ms(50)
while not alvik.get_touch_ok():
    sleep_ms(50)
# ----------------------------------------------------------------------------------
# Main control loop
try:
    while True:
        while not alvik.get_touch_cancel():
            # Receive new error values from Nicla Vision
            result = receive_data()
            if result:  # If valid data received
                t, r = result

                # prepare values
                new_result = (t * THETA_GAIN) + (r * RHO_GAIN)
                delta_result = new_result - old_result
                old_result = new_result

                # PID components
                p_output = new_result # proportional component
                i_output = max(min(i_output + new_result, I_MAX), I_MIN) # integral component
                d_output = (delta_result * 1000) / \
                    IMAGE_UPDATE_INTERVAL  # Fixed 100msinterval
                pid_output = (P_GAIN * p_output) + \
                    (I_GAIN * i_output) + (D_GAIN * d_output) # PID; add the three components

                # Calculate steering adjustment
                steering_adjustment = max(min(int(pid_output), 90), -90)/1000 #divide to account for time unit ms-> s. 
                # Do not steer more than 90 or less than -90 degrees

                # Update motor speeds based on steering
                if steering_adjustment > 0:  # Turn right
                    left_speed = BASE_SPEED + steering_adjustment # to turn right add to the left wheel and subtract from the right wheel
                    right_speed = BASE_SPEED - steering_adjustment
                elif steering_adjustment < 0:  # Turn left
                    left_speed = BASE_SPEED + steering_adjustment
                    right_speed = BASE_SPEED - steering_adjustment
                else:  # Go straight
                    left_speed = BASE_SPEED
                    right_speed = BASE_SPEED

                # Set wheel speeds
                alvik.set_wheels_speed(left_speed, right_speed) 

                # print informaiton on the terminal for debugging. 
                print(f"t: {t:.6f}, r: {r:.6f}, pid_output: {
                      pid_output:.2f}, steering: {steering_adjustment}")
                print(f"Left Speed: {left_speed}, Right Speed: {right_speed}")

                # Wait for the next cycle
                sleep_ms(IMAGE_UPDATE_INTERVAL)
            else:
                print("Waiting for UART data")

            # Stop the robot after processing all errors
            alvik.brake()

        #  touch_cancel is pressed
        while not alvik.get_touch_ok():
            alvik.left_led.set_color(0, 0, 1)
            alvik.right_led.set_color(0, 0, 1)
            alvik.brake()
            sleep_ms(100)
# ----------------------------------------------------------------------------------
except KeyboardInterrupt:
    # Stop the robot when interrupted, this can hapeen if there is no information recieved from the nicla vision module. 
    alvik.brake()
    alvik.stop()
    sys.exit()
# ----------------------------------------------------------------------------------
