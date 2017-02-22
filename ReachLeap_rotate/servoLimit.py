# Use this file to limit the motion of the servos
# Linearly interpolate between the lowest value of the Leap that you want to use
# and the highest value that you want to use
import math

# NEED TO GET ANGLE VERSION OF THESE MEASUREMENTS, OR MIGHT NOT NEED THESE

default_limits = {"xt": 150.0, "xb": -150.0,
                  "yt": 225.0, "yb": 0.0,
                  "zt": 75.0, "zb": -75.0,
                  "ct": 75.0, "cb": 45.0}

current_dict = default_limits

try:
    a_file = open('test_limits.txt', 'r')
#    current_dict['xt'] = float(a_file.readline())
#    current_dict['xb'] = float(a_file.readline())
#    current_dict['yt'] = float(a_file.readline())
#    current_dict['yb'] = float(a_file.readline())
    current_dict['zt'] = float(a_file.readline())
    current_dict['zb'] = float(a_file.readline())
    a_file.close()
except IOError:
    print "Problems accessing file! Default limits used"

# Servo limits
servoXTLimit = 180.0  # Base servo top angle limit
servoXDefault = 90.0
servoXBLimit = 0.0  # Base servo bottom angle limit

servoYTLimit = 130.0  # Height servo top angle limit
servoYDefault = 90.0
servoYBLimit = 20.0  # Height servo bottom angle limit

palmZTLimit = current_dict['zt']  # top limit
palmZBLimit = current_dict['zb']  # bottom limit
palmZDelta = palmZTLimit - palmZBLimit  # Difference between the above
servoZTLimit = 145.0  # Reach servo top angle limit
servoZBLimit = 60.0  # Reach servo bottom angle limit

servoCTLimit = 85.0  # Claw servo top angle limit
servoCDefault = 85.0
servoCBLimit = 10.0  # Claw servo bottom angle limit

# Use Linear Interpolation to restrict movement


def changex(finger_x, wrist_x, finger_z, wrist_z):
    flip = -1.0 if finger_z >= wrist_z else 1.0

    horizontal_angle = math.atan(-(wrist_x - finger_x)/(-flip*(finger_z - wrist_z)))*(180.0/math.pi)
    print(horizontal_angle)
    angleXTLimit = servoXTLimit - servoXDefault
    angleXBLimit = servoXDefault - servoXBLimit

    if horizontal_angle > 0:
        if horizontal_angle >= angleXTLimit:
            return int(servoXTLimit)
        else:
            return int(servoXDefault - horizontal_angle)
    else:
        if abs(horizontal_angle) >= angleXBLimit:
            return int(servoXBLimit)
        else:
            return int(servoXDefault + abs(horizontal_angle))


def changey(finger_y, wrist_y, finger_z, wrist_z):
    flip = -1.0 if finger_z >= wrist_z else 1.0
    vertical_angle = math.atan((finger_y-wrist_y)/(-flip*(finger_z-wrist_z)))*(180/math.pi)
    #print(vertical_angle)
    angleYTLimit = servoYTLimit - servoYDefault
    angleYBLimit = servoYDefault - servoYBLimit

    if vertical_angle > 0:
        if vertical_angle >= angleYTLimit:
            return int(servoYTLimit)
        else:
            return int(servoYDefault + vertical_angle)
    else:
        if abs(vertical_angle) >= angleYBLimit:
            return int(servoYBLimit)
        else:
            return int(servoYDefault - abs(vertical_angle))


def changez(palmz):
    if palmz < palmZBLimit:
        return int(servoZTLimit)
    elif palmz > palmZTLimit:
        return int(servoZBLimit)
    else:
        return int(abs((((palmz - palmZBLimit) / palmZDelta) * servoZBLimit)
                       + ((1.0 - abs((palmz - palmZBLimit) / palmZDelta)) * servoZTLimit)))


def changec(finger1_x, finger2_x, finger1_z, finger2_z):
    fingerXDist = abs(finger1_x - finger2_x)
    fingerZDist = abs(finger1_z - finger2_z)
    fingerDist = math.sqrt(abs((fingerXDist ** 2) + (fingerZDist ** 2)))
    fingersCDelta = current_dict['ct'] - current_dict['cb']
    #print fingerDist

    if fingerDist <= current_dict['cb']:
        return int(servoCTLimit)
    elif fingerDist > current_dict['ct']:
        return int(servoCBLimit)
    else:
        return int(abs((((fingerDist - current_dict['cb']) / fingersCDelta) * servoCBLimit)
                       + ((1.0 - abs((fingerDist - current_dict['cb']) / fingersCDelta)) * servoCTLimit)))

