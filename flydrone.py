"""
Simple script for take off and control with arrow keys
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

from pynput.keyboard import Key, Listener

#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udpout:192.168.42.1:14550')

#-- Setup the commanded flying speed
roll = 0
yaw = 0
pitch = 0
thrust = 0

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED_NOGPS")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   #while True:
   #   v_alt = vehicle.location.global_relative_frame.alt
   #   print(">> Altitude = %.1f m"%v_alt)
   #   if v_alt >= altitude - 1.0:
   #       print("Target altitude reached")
   #       break
   #   time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, d_roll, d_pitch, d_yaw, d_thrust):
    
    global roll
    global pitch
    global yaw
    global thrust

    roll = roll + d_roll
    pitch = pitch + d_pitch
    yaw = yaw + d_yaw
    thrust = thrust + d_thrust

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0b00000000,
        (1,0,0,0),
        roll,pitch,yaw,
        thrust
    )

    # msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #         0,
    #         0, 0,
    #         mavutil.mavlink.MAV_FRAME_BODY_NED,
    #         0b0000111111000111, #-- BITMASK -> Consider only the velocities
    #         0, 0, 0,        #-- POSITION
    #         vx, vy, vz,     #-- VELOCITY
    #         0, 0, 0,        #-- ACCELERATIONS
    #         0, 0)

    vehicle.send_mavlink(msg)
    vehicle.flush()
    print("Complete")

def on_press(key):
    try:
	    dummy = key.char
    except:
        if key == Key.up:
            print("thrust up")
            set_velocity_body(vehicle, 0, 0, 0, 50)
        elif key == Key.down:
            print("thrust down")
            set_velocity_body(vehicle, 0, 0, 0, -50)    
        return
    if key.char == 'w':
        print("pitch forward")
        set_velocity_body(vehicle, 0, -100, 0, 0)
    elif key.char == 's':
        print("pitch down")
        set_velocity_body(vehicle, 0 , 100, 0, 0)
    elif key.char == 'a':
        print("roll left")
        set_velocity_body(vehicle, -100, 0, 0, 0)
    elif key.char == 'd':
        print("roll right")
        set_velocity_body(vehicle, 100, 0, 0, 0)

def on_release(key):
    print('{0} release'.format(
        key))
    if key == Key.esc:
        # Stop listener
        return False    

#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)

# Collect events until released
with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
