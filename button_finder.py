#!/usr/bin/env python3
"""
Prints the index of any button that's currently pressed, live, so we
can find which index actually corresponds to Triangle (or any other
button) on this specific controller/driver setup - rather than
assuming standard PS-controller ordering, which this DualSense/SDL
combination has already proven not to reliably follow (see the d-pad
SDL compatibility issue that moved the arm gesture off the d-pad
originally).

Read-only - does not touch CAN, does not import gamecontroller.py,
cannot move the robot. Just watches the controller.

Usage: python3 button_finder.py
Press Ctrl+C to stop.
"""
import pygame
import time

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller: {joystick.get_name()}")
print(f"Button count: {joystick.get_numbuttons()}")
print("Press any button - I'll print which index(es) go active.\n")
print("Press Ctrl+C to stop.\n")

prev_states = [False] * joystick.get_numbuttons()

try:
    while True:
        pygame.event.pump()
        for i in range(joystick.get_numbuttons()):
            state = bool(joystick.get_button(i))
            if state != prev_states[i]:
                print(f"Button index {i}: {'PRESSED' if state else 'released'}")
                prev_states[i] = state
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nStopped.")
