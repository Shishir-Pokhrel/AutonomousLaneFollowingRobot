#Shishir Pokhrel, University of Siegen, shishir.pokhrel@student.uni-siegen.de

#References:
    # OpenMV tutorials
    # Arduino Forums
    # Open-source community

import sensor, imu
import time
import math
from machine import UART
from time import sleep
import struct

# Initialize UART
uart = UART(4, baudrate=115200) # send the values with this rate, note that same baudrate is necessary on the alvik side as well. 
print("|---------------UART: Nicla Device-----------------|")

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # take the images in grayscale format. Alternatively if necessary can take in sensor.RGB as well.
sensor.set_framesize(sensor.QQVGA) # choose an image frame size, 320x240
sensor.set_framerate(80) # set Nominal frame rate for the camera; This value will reduce when there is more to process (upto 25fps is good) 
sensor.skip_frames(time=2000) # initially skip 2 seconds for the camera to adjust and image stabilisation.
GRAYSCALE_THRESHOLD = (0, 50) # Threshold lower values, because lane lines are black. Value range from 0-255, 255 is white


# Define the regression line algorithm. Coordinate system has four quadrants.
      # - Quadrant 1: 0° <= theta < 90° and rho >= 0
      # - Quadrant 2: 90° <= theta < 180° and rho >= 0
      # - Quadrant 3: 180° <= theta < 270° and rho < 0 (unused in this implementation)
      # - Quadrant 4: 270° <= theta < 360° and rho < 0

def line_to_theta_and_rho(line):
    if line.rho() < 0:  # Q 3 and 4
        if line.theta() < 90:  # Q3
            return (math.sin(math.radians(line.theta())),
                    math.cos(math.radians(line.theta() + 180)) * -line.rho()) # obtain data as (sin(theta),rcos(theta))
        else:  # Quadrant 4
            return (math.sin(math.radians(line.theta() - 180)),
                    math.cos(math.radians(line.theta() + 180)) * -line.rho())
    else:  # Q 1 and 2
        if line.theta() < 90:  # Q1
            if line.theta() < 45:
                return (math.sin(math.radians(180 - line.theta())),
                        math.cos(math.radians(line.theta())) * line.rho())
            else:
                return (math.sin(math.radians(line.theta() - 180)),
                        math.cos(math.radians(line.theta())) * line.rho())
        else:  # Q 2
            return (math.sin(math.radians(180 - line.theta())),
                    math.cos(math.radians(line.theta())) * line.rho())


def line_to_theta_and_rho_error(line, img):
    t, r = line_to_theta_and_rho(line) # t and r are the outputs from regression fit
    return (t, r - (img.width() // 2)) # Divide the rho by 2 because we want the line ideally to be in the middle of the screen.


# Image corrections 
b= 0.1 # Translation of 0.1 mm 
cam_rot = imu.pitch()-90.0 # Define pitch of the camera, this is obtained from the imu sensor. The image is rotated with this angle later.
cornerse = [(20, 20), (140, 20), (160, 120), (0, 120)] # These are points to be used for perspective correction. (forms a trapezoid)

while True:

    ty = math.sin(cam_rot) * b # Translationx
    tx = math.cos(cam_rot) * b # Translationy

    #use one or the other image correction. 

    img = sensor.snapshot().binary([GRAYSCALE_THRESHOLD]).histeq().rotation_corr(cam_rot, 0, 0,tx,ty) # Image rotation correction with cam_rot angle
    #img = sensor.snapshot().binary([GRAYSCALE_THRESHOLD]).histeq().rotation_corr(corners=cornerse) # alternatively, perspective correction with corner points

    # fit a regression line on the white elements.
    # In grayscale thresholding, we threshold black lines. In binary image they are depicted white. 
    # fit a regression along these (threshold value of 253-255)
    # robustness is false meaning least square method is used, if true Thiel-Sen method is used where outliers are mitigated
    line = img.get_regression([(253, 255)], robust=False) 
    if line:
        img.draw_line(line.line(), color=(255, 100, 150)) # If a line is fit, draw it on the image
        error = line_to_theta_and_rho_error(line, img) # get the parameters from the line, 
        theta = error[0]    # pack the values from error for theta and rho to prepare to send to Alvik.
        rho_error = error[1]

        try:
            data = struct.pack('<ff', theta, rho_error)  # send as Little endian floats, least significant is sent first.

            #Send data over UART
            uart.write(data)
            print(f"Sent Data: Theta = {theta}, Rho Error = {rho_error}") 

            #Print for debugging
            print("Sent Bytes:", " ".join(f"{byte:02X}" for byte in data))

            sleep(0.01)  # Wait before sending again (optional, this must be less than sleep in alvik)
        except Exception as e:
            print(f"Error: {e}")#print system error and reason for it.


