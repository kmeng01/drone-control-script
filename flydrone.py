"""
Simple script for take off and control with arrow keys
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

from pynput.keyboard import Key, Listener
from pynput.mouse import Listener as Listener2, Button
import math

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udpout:192.168.42.1:14550')
# vehicle = 0

#-- Setup the commanded flying speed
thrust = 0

#-- Define arm and takeoff
def init_motors():

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED_NOGPS")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Fly now!")
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_thrust(vehicle, d_thrust):
    global thrust

    thrust = min(max(0,thrust + d_thrust),10)
    print(thrust)

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        #0b11111101,
	0b00000000,
        (1,0,0,0),
        0,0,0,
        thrust
    )

    for _ in range(3): 
        vehicle.send_mavlink(msg)
        vehicle.flush()

def set_euler(vehicle, roll, pitch, yaw, reset):
    global thrust
    print(roll, pitch, yaw, thrust)

    x = roll
    y = pitch
    z = yaw
    w = 1

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        #0b00000010,
	0b00000000,
        (w,x,y,z),
        5,5,5,
        thrust
    )

    epochs = 3
    if reset:
        epochs = 20

    for _ in range(epochs): 
        vehicle.send_mavlink(msg)
        vehicle.flush()

# Mouse wheel
def on_click(x, y, button, pressed):
    print('{0} at {1}'.format(
        'Pressed' if pressed else 'Released',
        (x, y)))
    if not pressed and button == Button.right:
        return False

def on_scroll(x, y, dx, dy):
    print('Scrolled {0}'.format(
        (dx, dy)))
    if dy < 0:
        print("thrust down")
        set_thrust(vehicle, -1)
    elif dy > 0:
        print("thrust up")
        set_thrust(vehicle, 1)

# Keyboard
def on_press(key):
    try:
	    dummy = key.char
    except:
        return
    if key.char == 'w':
        print("pitch forward")
        set_euler(vehicle, 0, -10, 0, False)
    elif key.char == 's':
        print("pitch down")
        set_euler(vehicle, 0 , 10, 0, False)
    elif key.char == 'a':
        print("roll left")
        set_euler(vehicle, -10, 0, 0, False)
    elif key.char == 'd':
        print("roll right")
        set_euler(vehicle, 10, 0, 0, False)

def on_release(key):
    if key == Key.esc:
        return False
    try:
	    dummy = key.char
    except:
        return
    if key.char == 'w' or key.char == 'a' or key.char == 's' or key.char == 'd':
        print("reset euler angles")
        set_euler(vehicle, 0, 0, 0, True)

#---- MAIN FUNCTION
#- Takeoff
init_motors()

# Collect events until released
with Listener(on_press=on_press,on_release=on_release) as listener:
    with Listener2(on_scroll=on_scroll, on_click=on_click) as listener:
        listener.join()
