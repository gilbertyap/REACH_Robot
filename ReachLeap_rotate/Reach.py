################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import os, sys, inspect, thread, time
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
arch_dir = '../lib/x64' if sys.maxsize > 2**32 else '../lib/x86'

sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

import Leap
import serial
from servoLimit import changex
from servoLimit import changey
from servoLimit import changez
from servoLimit import changec

ser = serial.Serial('COM1', 115200, timeout=1)

class SampleListener(Leap.Listener):
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

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
    cont = False
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']

    # Initiate averages and counter (to starting position)
    average = {"x_avg": 90.0, "y_avg": 100.0, "z_avg": 90.0, "claw_avg": 85.0}
    frame_counter = 0
    average_factor = 5

    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Wait until the controller is connected
    while controller.is_connected is False:
        time.sleep(0.01)

    # Run the program indefinitely
    while cont is False and controller.is_connected is True:
        # For every average_factor frames, average out the coordinates
        while frame_counter < average_factor:
            frame = controller.frame()
            thumb_x = 0.0
            thumb_z = 0.0
            index_x = 0.0
            index_y = 0.0
            index_z = 0.0

            print "Frame id: %d, hands: %d" % (frame.id, len(frame.hands))

            if frame.hands > 0:
                # Figure out which hand has been there the longest
                oldest_hand_time = 0.0
                oldest_hand_id = 0

                for hand in frame.hands:
                    if hand.time_visible > oldest_hand_time:
                        oldest_hand_time = hand.time_visible
                        oldest_hand_id = hand.id

                # Only track the hand with the id of the hand with the longest time visible
                track_hand = frame.hand(oldest_hand_id)
                for finger in track_hand.fingers:
                    bone = finger.bone(3)
                    if finger_names[finger.type] == "Thumb":
                        thumb_x = bone.next_joint[0]
                        # thumb_y = bone.next_joint[1]
                        thumb_z = bone.next_joint[2]

                    if finger_names[finger.type] == "Index":
                        index_x = bone.next_joint[0]
                        index_y = bone.next_joint[1]
                        index_z = bone.next_joint[2]
                        lower_index_x = bone.prev_joint[0]
                        lower_index_z = bone.prev_joint[2]

                    bone = finger.bone(2)
                    if finger_names[finger.type] == "Index":
                        lower_index_x = bone.prev_joint[0]
                        lower_index_z = bone.prev_joint[2]

                    print " id %d, position: %s" % (hand.id, hand.palm_position)

                average["x_avg"] += changex(index_x, track_hand.wrist_position[0], index_z, track_hand.wrist_position[2])
                average["y_avg"] += changey(index_y, track_hand.wrist_position[1], index_z, track_hand.wrist_position[2])
                average["z_avg"] += changez(track_hand.palm_position[2])
                average["claw_avg"] += changec(thumb_x, lower_index_x, thumb_z, lower_index_z)

                time.sleep(0.01)
                frame_counter += 1

            # Average out the values, effectively a low pass filter
            average["x_avg"] = int(average["x_avg"] / average_factor)
            average["y_avg"] = int(average["y_avg"] / average_factor)
            average["z_avg"] = int(average["z_avg"] / average_factor)
            average["claw_avg"] = int(average["claw_avg"] / average_factor)

        # Construct and send message
        write_values = [240,  # Start ID
                        241, chr(average["x_avg"]),  # base
                        242, chr(average["y_avg"]),  # height
                        243, chr(average["z_avg"]),  # reach
                        244, chr(average["claw_avg"]),  # claw
                        245]  # End ID

        # Send messages over serial
        print write_values
        ser.write(write_values)

        # Keep this process running until Enter is pressed
        print "Press Enter to quit..."
        try:
            sys.stdin.readline()
        except KeyboardInterrupt:
            pass
        finally:
            cont = True

    # Move arm to default position
    ser.write(chr(254))

    time.sleep(2)
    # Remove the sample listener when done
    controller.remove_listener(listener)


if __name__ == "__main__":
    main()
