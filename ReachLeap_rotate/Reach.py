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
import serial
from servoLimit import changex
from servoLimit import changey
from servoLimit import changez
from servoLimit import changec


ser = serial.Serial('COM3', 19200, timeout=0)


class SampleListener(Leap.Listener):
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        thumb_x = 0.0
        #thumb_y = 0.0
        thumb_z = 0.0

        index_x = 0.0
        index_y = 0.0
        index_z = 0.0

        #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
        #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers))

        # Get hands
        for hand in frame.hands:
            if hand.is_right:
                handType = "Left hand" if hand.is_left else "Right hand"

                for finger in hand.fingers:
                    distal_bone = finger.bone(3)

                    if self.finger_names[finger.type] == "Thumb":
                        thumb_x = distal_bone.next_joint[0]
                        thumb_y = distal_bone.next_joint[1]
                        thumb_z = distal_bone.next_joint[2]

                    if self.finger_names[finger.type] == "Index":
                        index_x = distal_bone.next_joint[0]
                        index_y = distal_bone.next_joint[1]
                        index_z = distal_bone.next_joint[2]
                        lower_index_x = distal_bone.prev_joint[0]
                        lower_index_y = distal_bone.prev_joint[1]
                        lower_index_z = distal_bone.prev_joint[2]

                    prox_bone = finger.bone(1)
                    if self.finger_names[finger.type] == "Index":
                        lower_index_x = prox_bone.prev_joint[0]
                        lower_index_y = prox_bone.prev_joint[1]
                        lower_index_z = prox_bone.prev_joint[2]


                # print "  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position)

                ser.write(chr(240))  # Start of message

                ser.write(chr(241))  # Horizontal Angle change
                ser.write(chr(changex(index_x, hand.wrist_position[0], index_z, hand.wrist_position[2])))  # x position - left/right

                ser.write(chr(242))  # Vertical Angle change
                ser.write(chr(changey(index_y, hand.wrist_position[1], index_z, hand.wrist_position[2])))  # y position - up/down

                ser.write(chr(243))  # Forward/Backward reach
                ser.write(chr(changez(hand.palm_position[2])))  # z position - forward/backward

                ser.write(chr(244))  # Fingers Distance
                ser.write(chr(changec(thumb_x, lower_index_x, thumb_z, lower_index_z)))

                ser.write(chr(245))  # End of Group

                time.sleep(0.01)

                if hand.grab_strength > 0.95:
                    print('Sleeping...')
                    time.sleep(3)
                    print('Restarting...')

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
    # Set the servo limits
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed

    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Move arm to default position
        ser.write(chr(254))

        time.sleep(2)
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
