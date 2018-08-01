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
gnd_speed = 200 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   #while not vehicle.is_armable:
   #   print("waiting to be armable")
   #   time.sleep(1)

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
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_attitude_target(
        0, 0, 0,
        0b00000000,
        (1,0,0,0),
        0,5,0,
        0
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
    
    
#-- Key event function
#def key(event):
    #if event.char == event.keysym: #-- standard keys
    #    if event.keysym == 'r':
    #        print("r pressed >> Set the vehicle to RTL")
    #        vehicle.mode = VehicleMode("RTL")
            
    #else: #-- non standard keys
     #   if event.keysym == 'w':
     #       print("Up key pressed")
      #      set_velocity_body(vehicle, gnd_speed, 0, 0)
       # elif event.keysym == 's':
       #     set_velocity_body(vehicle,-gnd_speed, 0, 0)
       # elif event.keysym == 'a':
       #     set_velocity_body(vehicle, 0, -gnd_speed, 0)
        #elif event.keysym == 'd':
         #   set_velocity_body(vehicle, 0, gnd_speed, 0)

def on_press(key):
    try:
	dummy = key.char
    except:
	return
    if key.char == 'w':
        print("Up key pressed")
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif key.char == 's':
        print("down key pressed")
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    elif key.char == 'a':
        print("left key pressed")
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif key.char == 'd':
        print("right key pressed")
        set_velocity_body(vehicle, 0, gnd_speed, 0)

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
