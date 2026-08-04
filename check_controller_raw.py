#!/usr/bin/env python3
"""
Isolates whether pygame is actually receiving controller input, completely
independent of gamecontroller.py's own logic. No CAN, no robot - purely
checks the controller. Press buttons/move sticks for ~15 seconds; if
nothing here changes from zero, the problem is at the pygame/OS level,
not in gamecontroller.py's code.
"""
import pygame
import time

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(f"pygame sees {count} joystick(s).")
if count == 0:
    print("No joystick detected at all - this is a pygame/OS-level problem, "
          "not something in gamecontroller.py.")
    raise SystemExit

js = pygame.joystick.Joystick(0)
js.init()
print(f"Using: {js.get_name()}")
print(f"Axes: {js.get_numaxes()}  Buttons: {js.get_numbuttons()}  Hats: {js.get_numhats()}")
print("\nMove sticks / press buttons now - watching for 15 seconds...\n")

start = time.time()
last_print = 0
while time.time() - start < 15:
    pygame.event.pump()
    axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
    buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(i) for i in range(js.get_numhats())]
    if time.time() - last_print > 0.3:
        print(f"axes={axes} buttons={buttons} hats={hats}")
        last_print = time.time()
    time.sleep(0.02)

print("\nDone. If everything above stayed at 0/False the whole time even "
      "while you pressed things, pygame isn't receiving real input from "
      "this controller right now.")
