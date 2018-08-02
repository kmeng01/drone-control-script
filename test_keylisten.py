from pynput.keyboard import Key, Listener
from pynput.mouse import Listener as Listener2

# Mouse wheel
def on_click(x, y, button, pressed):
    print('{0} at {1}'.format(
        'Pressed' if pressed else 'Released',
        (x, y)))
    if not pressed:
        # Stop listener
        return False

def on_scroll(x, y, dx, dy):
    print('Scrolled {0}'.format(
        (dx, dy)))

# Keyboard
def on_press(key):
    print('{0} pressed'.format(
        key))

def on_release(key):
    print('{0} release'.format(
        key))
    if key == Key.esc:
        # Stop listener
        return False

with Listener(on_press=on_press,on_release=on_release) as listener:
    with Listener2(on_scroll=on_scroll, on_click=on_click) as listener:
        listener.join()