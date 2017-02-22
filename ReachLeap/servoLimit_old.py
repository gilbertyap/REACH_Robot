# Use this file to limit the motion of the servos
# Linearly interpolate between the lowest value of the Leap that you want to use
# and the highest value that you want to use
import math

# sizes

l_dict = {"xt": 150.0, "xb": -150.0,
          "yt": 225.0, "yb": 0.0,
          "zt": 125.0, "zb": -125.0,
          "ct": 75.0, "cb": 45.0}

current_dict = l_dict

palmXTLimit = current_dict['xt']   # top limit
palmXBLimit = current_dict['xb']  # bottom limit
palmXDelta = palmXTLimit - palmXBLimit  # Difference between the above
servoXTLimit = 180.0  # Base servo top angle limit
servoXBLimit = 0.0  # Base servo bottom angle limit

palmYTLimit = current_dict['yt']  # top limit
palmYBLimit = current_dict['yb']  # bottom limit
palmYDelta = palmYTLimit - palmYBLimit  # Difference between the above
servoYTLimit = 130.0  # Height servo top angle limit
servoYBLimit = 20.0  # Height servo bottom angle limit

palmZTLimit = current_dict['zt']  # top limit
palmZBLimit = current_dict['zb']  # bottom limit
palmZDelta = palmZTLimit - palmZBLimit  # Difference between the above
servoZTLimit = 145.0  # Reach servo top angle limit
servoZBLimit = 60.0  # Reach servo bottom angle limit

fingersCTLimit = current_dict['ct']  # top limit
fingersCBLimit = current_dict['cb']  # bottom limit
fingersCDelta = fingersCTLimit - fingersCBLimit  # Difference between the above
servoCTLimit = 85.0  # Claw servo top angle limit
servoCBLimit = 10.0  # Claw servo bottom angle limit

# Use Linear Interpolation to restrict movement


def setlimits(current_dict):
    a_file = open('test_limits', 'r')

    current_dict['xt'] = float(a_file.readline())
    current_dict['xb'] = float(a_file.readline())
    current_dict['yt'] = float(a_file.readline())
    current_dict['yb'] = float(a_file.readline())
    current_dict['zt'] = float(a_file.readline())
    current_dict['zb'] = float(a_file.readline())
    current_dict['ct'] = float(a_file.readline())
    current_dict['cb'] = float(a_file.readline())


# Use Linear Interpolation to restrict movement


def changex(palmx):
    palmXDelta = current_dict['xt']  - current_dict['xb']

    if palmx < palmXBLimit:
        return int(servoXTLimit)
    elif palmx > palmXTLimit:
        return int(servoXBLimit)
    else:
        return int(abs((((palmx-palmXBLimit) / palmXDelta) * servoXBLimit)
                       + ((1.0 - abs((palmx-palmXBLimit) / palmXDelta)) * servoXTLimit)))


def changey(palmy):
    if palmy < palmYBLimit:
        return int(servoYBLimit)
    elif palmy > palmYTLimit:
        return int(servoYTLimit)
    else:
        return int(abs((((palmy - palmYBLimit) / palmYDelta) * servoYTLimit)
                       + ((1.0 - abs((palmy - palmYBLimit) / palmYDelta)) * servoYBLimit)))


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
    fingerDist = math.sqrt(abs((fingerXDist**2)+(fingerZDist**2)))

    if fingerDist < fingersCBLimit:
        return int(servoCTLimit)
    elif fingerDist > fingersCTLimit:
        return int(servoCBLimit)
    else:
        return int(abs((((fingerDist - fingersCBLimit) / fingersCDelta) * servoCBLimit)
                       + ((1.0 - abs((fingerDist - fingersCBLimit) / fingersCDelta)) * servoCTLimit)))