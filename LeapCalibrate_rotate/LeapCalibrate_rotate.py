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
import math


limits_dict = {'max_h_angle': -200.0,
               'min_h_angle': 200.0,
               'max_v_angle': -200.0,
               'min_v_angle': 200.0,
               'max_claw': -100.0,
               'min_claw':100.0}


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

        thumb_x = 0.0
        # thumb_y = 0.0
        thumb_z = 0.0

        index_x = 0.0
        index_y = 0.0
        index_z = 0.0

        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
            frame.id, frame.timestamp, len(frame.hands), len(frame.fingers))

        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            for finger in hand.fingers:
                bone = finger.bone(3)

                if self.finger_names[finger.type] == "Thumb":
                    thumb_x = bone.next_joint[0]
                    # thumb_y = bone.next_joint[1]
                    thumb_z = bone.next_joint[2]

                if self.finger_names[finger.type] == "Index":
                    index_x = bone.next_joint[0]
                    index_y = bone.next_joint[1]
                    index_z = bone.next_joint[2]
                    lower_index_x = bone.prev_joint[0]
                    # lower_index_y = bone.prev_joint[1]
                    lower_index_z = bone.prev_joint[2]

                bone = finger.bone(2)
                if self.finger_names[finger.type] == "Index":
                    lower_index_x = bone.prev_joint[0]
                    # lower_index_y = bone.prev_joint[1]
                    lower_index_z = bone.prev_joint[2]

            wrist_x = hand.wrist_position[0]
            wrist_y = hand.wrist_position[1]
            wrist_z = hand.wrist_position[2]

            flip = -1.0 if index_z >= wrist_z else 1.0

            #print "  %s, id %d, position: %s" % (handType, hand.id, hand.palm_position)

            horizontal_angle = math.atan(-(wrist_x - index_x) / (-flip * (index_z - wrist_z))) * (180.0 / math.pi)
            vertical_angle = math.atan((index_y - wrist_y) / (-flip * (index_z - wrist_z))) * (180 / math.pi)
            fingerXDist = abs(thumb_x - lower_index_x)
            fingerZDist = abs(thumb_z - lower_index_z)
            clawDist = math.sqrt(abs((fingerXDist ** 2) + (fingerZDist ** 2)))

            limits_dict['max_h_angle'] = max(horizontal_angle, limits_dict['max_h_angle'])
            limits_dict['min_h_angle'] = min(horizontal_angle, limits_dict['min_h_angle'])
            limits_dict['max_v_angle'] = max(vertical_angle, limits_dict['max_v_angle'])
            limits_dict['min_v_angle'] = min(vertical_angle, limits_dict['min_v_angle'])
            limits_dict['max_claw'] = max(clawDist, limits_dict['max_claw'])
            limits_dict['min_claw'] = min(clawDist, limits_dict['min_claw'])

            time.sleep(0.06)
            # time.sleep(.8)

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
            limits_strings = {"mha": str(limits_dict['max_h_angle']) + "\n", "lha": str(limits_dict['min_h_angle']) + "\n",
                              "mva": str(limits_dict['max_v_angle']) + "\n", "lva": str(limits_dict['min_v_angle']) + "\n",
                              "mcd": str(limits_dict['max_claw']) + "\n", "lcd": str(limits_dict['min_claw']) + "\n"}

            print "Most Horizontal Angle: " + limits_strings["mha"]
            print "Least Horizontal Angle: " + limits_strings["lha"]
            print "Most Vertical Angle: " + limits_strings["mva"]
            print "Least Vertical Angle: " + limits_strings["lva"]
            print "Most Claw Distance: " + limits_strings["mcd"]
            print "Least Claw Distance: " + limits_strings["lcd"]

            # Open file
            a_file = open('rotate_limits.txt', 'w+')

            print "Writing to file..."
            # Clear file
            a_file.truncate(0)

            # Write new contents to file
            a_file.write(limits_strings['mha'])
            a_file.write(limits_strings['lha'])
            a_file.write(limits_strings['mva'])
            a_file.write(limits_strings['lva'])
            a_file.write(limits_strings['mcd'])
            a_file.write(limits_strings['lcd'])

            print "Write finished, closing calibration."
            a_file.close()

        except IOError:
            "Cannot access file! Calibration not written."

        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
