# Use this file to limit the motion of the servos
# Linearly interpolate between the lowest value of the Leap that you want to use
# and the highest value that you want to use
import math

# sizes

default_limits = {"xt": 150.0, "xb": -150.0,
                  "yt": 225.0, "yb": 0.0,
                  "zt": 125.0, "zb": -125.0,
                  "ct": 75.0, "cb": 45.0}

current_dict = default_limits

try:
    a_file = open('test_limits.txt', 'r')
    current_dict['xt'] = float(a_file.readline())
    current_dict['xb'] = float(a_file.readline())
    current_dict['yt'] = float(a_file.readline())
    current_dict['yb'] = float(a_file.readline())
    current_dict['zt'] = float(a_file.readline())
    current_dict['zb'] = float(a_file.readline())
    current_dict['ct'] = float(a_file.readline())
    current_dict['cb'] = float(a_file.readline())
    a_file.close()
except IOError:
    print "Problems accessing file! Default limits used"

# Servo limits
servoXTLimit = 180.0  # Base servo top angle limit
servoXBLimit = 0.0  # Base servo bottom angle limit

servoYTLimit = 130.0  # Height servo top angle limit
servoYBLimit = 20.0  # Height servo bottom angle limit

servoZTLimit = 145.0  # Reach servo top angle limit
servoZBLimit = 60.0  # Reach servo bottom angle limit

servoCTLimit = 85.0  # Claw servo top angle limit
servoCBLimit = 10.0  # Claw servo bottom angle limit

# Use Linear Interpolation to restrict movement


def changex(palmx):
    palmXDelta = current_dict['xt'] - current_dict['xb']

    if palmx < current_dict['xb']:
        return int(servoXTLimit)
    elif palmx > current_dict['xt']:
        return int(servoXBLimit)
    else:
        return int(abs((((palmx - current_dict['xb']) / palmXDelta) * servoXBLimit)
                       + ((1.0 - abs((palmx - current_dict['xb']) / palmXDelta)) * servoXTLimit)))


def changey(palmy):
    palmYDelta = current_dict['yt'] - current_dict['yb']

    if palmy < current_dict['yb']:
        return int(servoYBLimit)
    elif palmy > current_dict['yt']:
        return int(servoYTLimit)
    else:
        return int(abs((((palmy - current_dict['yb']) / palmYDelta) * servoYTLimit)
                       + ((1.0 - abs((palmy - current_dict['yb']) / palmYDelta)) * servoYBLimit)))


def changez(palmz):
    palmZDelta = current_dict['zt'] - current_dict['zb']

    if palmz < current_dict['zb']:
        return int(servoZTLimit)
    elif palmz > current_dict['zt']:
        return int(servoZBLimit)
    else:
        return int(abs((((palmz - current_dict['zb']) / palmZDelta) * servoZBLimit)
                       + ((1.0 - abs((palmz - current_dict['zb']) / palmZDelta)) * servoZTLimit)))


def changec(finger1_x, finger2_x, finger1_z, finger2_z):
    fingerXDist = abs(finger1_x - finger2_x)
    fingerZDist = abs(finger1_z - finger2_z)
    fingerDist = math.sqrt(abs((fingerXDist ** 2) + (fingerZDist ** 2)))
    fingersCDelta = current_dict['ct'] - current_dict['cb']

    if fingerDist < current_dict['cb']:
        return int(servoCTLimit)
    elif fingerDist > current_dict['cb']:
        return int(servoCBLimit)
    else:
        return int(abs((((fingerDist - current_dict['cb']) / fingersCDelta) * servoCBLimit)
                       + ((1.0 - abs((fingerDist - current_dict['cb']) / fingersCDelta)) * servoCTLimit)))

