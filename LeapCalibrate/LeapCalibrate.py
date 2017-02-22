################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap
import sys
import thread
import time

limits_dict = {'max_x': -500.0, 'min_x': 500.0,
               'max_y': -500.0, 'min_y': 500.0,
               'max_z': -500.0, 'min_z': 500.0}


class SampleListener(Leap.Listener):
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

    def on_init(self, controller):
        print "Leap Initialized."

    def on_connect(self, controller):
        print "Leap Connected."

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Leap Disconnected."

    def on_exit(self, controller):
        print "Leap Exited."

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # Get hands
        for hand in frame.hands:
            limits_dict['max_x'] = max(float(hand.palm_position[0]), limits_dict['max_x'])
            limits_dict['max_y'] = max(float(hand.palm_position[1]), limits_dict['max_y'])
            limits_dict['max_z'] = max(float(hand.palm_position[2]), limits_dict['max_z'])
            limits_dict['min_x'] = min(float(hand.palm_position[0]), limits_dict['min_x'])
            limits_dict['min_y'] = min(float(hand.palm_position[1]), limits_dict['min_y'])
            limits_dict['min_z'] = min(float(hand.palm_position[2]), limits_dict['min_z'])

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"


def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    print "Place hand over center of Leap Motion. \n Press enter to begin calibration coutndown."
    sys.stdin.readline()

    print "Starting calibration in 3..."
    time.sleep(1)
    print "Starting calibration in 2..."
    time.sleep(1)
    print "Starting calibration in 1..."
    time.sleep(1)
    print "Calibration started!"

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed

    print "Press enter to end calibration"
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Write positions to file
        try:
            limits_strings = {"mx": str(limits_dict['max_x']) + "\n", "lx": str(limits_dict['min_x']) + "\n",
                              "my": str(limits_dict['max_y']) + "\n", "ly": str(limits_dict['min_y']) + "\n",
                              "mz": str(limits_dict['max_z']) + "\n", "lz": str(limits_dict['min_z']) + "\n"}

            print "Most X: " + limits_strings["mx"]
            print "Least X: " + limits_strings["lx"]
            print "Most Y: " + limits_strings["my"]
            print "Least Y: " + limits_strings["ly"]
            print "Most Z: " + limits_strings["mz"]
            print "Least Z: " + limits_strings["lz"]

            # Open file
            a_file = open('test_limits.txt', 'w+')

            print "Writing to file..."
            # Clear file
            a_file.truncate(0)

            # Write new contents to file
            a_file.write(limits_strings['mx'])
            a_file.write(limits_strings['lx'])
            a_file.write(limits_strings['my'])
            a_file.write(limits_strings['ly'])
            a_file.write(limits_strings['mz'])
            a_file.write(limits_strings['lz'])

            print "Write finished, closing calibration."
            a_file.close()

        except IOError:
            "Cannot access file! Calibration not written."

        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
