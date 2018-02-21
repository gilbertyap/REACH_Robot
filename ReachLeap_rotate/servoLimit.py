# Use this file to limit the motion of the servos
# Linearly interpolate between the lowest value of the Leap that you want to use
# and the highest value that you want to use
import math

# TODO NEED TO GET ANGLE VERSION OF THESE MEASUREMENTS

default_limits = {"xt": 150.0, "xb": -150.0,
                  "yt": 225.0, "yb": 0.0,
                  "zt": 75.0, "zb": -75.0,
                  "ct": 75.0, "cb": 45.0}

# Servo limits
# x - base, y - height, z - reach, c - claw
servo_limits = {"xt": 180.0, "xb": 0.0,
                "yt": 130.0, "yb": 20.0,
                "zt": 145.0, "zb": 60.0,
                "ct": 85, "cb": 10.0}

current_dict = default_limits

try:
    a_file = open('test_limits.txt', 'r')
#   current_dict['xt'] = float(a_file.readline())
#   current_dict['xb'] = float(a_file.readline())
#   current_dict['yt'] = float(a_file.readline())
#   current_dict['yb'] = float(a_file.readline())
    current_dict['zt'] = float(a_file.readline())
    current_dict['zb'] = float(a_file.readline())
    a_file.close()
except IOError:
    print "Problems accessing file! Default limits used"

servoXDefault = 90.0
servoYDefault = 90.0
palmZTLimit = current_dict['zt']  # top limit
palmZBLimit = current_dict['zb']  # bottom limit
palmZDelta = palmZTLimit - palmZBLimit  # Difference between the above


# Changes x coordinate into value for the servo motor
def changex(finger_x, wrist_x, finger_z, wrist_z):
    # Determine whether or not to flip value (due to rotation)
    flip = -1.0 if finger_z >= wrist_z else 1.0

    # Find the angle between the finger and wrist, establish top and bottom limits
    horizontal_angle = math.atan(-(wrist_x - finger_x)/(-flip*(finger_z - wrist_z)))*(180.0/math.pi)
    angleXTLimit = servo_limits["xt"] - servoXDefault
    angleXBLimit = servoXDefault - servo_limits["xb"]

    # Determine if the value found is beyond the limits
    # if not, move the position relative to the default
    if horizontal_angle > 0:
        if horizontal_angle >= angleXTLimit:
            return int(servo_limits["xt"])
        else:
            return int(servoXDefault - horizontal_angle)
    else:
        if abs(horizontal_angle) >= angleXBLimit:
            return int(servo_limits["xb"])
        else:
            return int(servoXDefault + abs(horizontal_angle))


# Changes y coordinate into value for the servo motor
def changey(finger_y, wrist_y, finger_z, wrist_z):
    flip = -1.0 if finger_z >= wrist_z else 1.0
    vertical_angle = math.atan((finger_y-wrist_y)/(-flip*(finger_z-wrist_z)))*(180/math.pi)
    angleYTLimit = servo_limits["yt"] - servoYDefault
    angleYBLimit = servoYDefault - servo_limits["yb"]

    if vertical_angle > 0:
        if vertical_angle >= angleYTLimit:
            return int(servo_limits["yt"])
        else:
            return int(servoYDefault + vertical_angle)
    else:
        if abs(vertical_angle) >= angleYBLimit:
            return int(servo_limits["yb"])
        else:
            return int(servoYDefault - abs(vertical_angle))


# Changes z coordinate into value for the servo motor
def changez(palmz):
    if palmz < palmZBLimit:
        return int(servo_limits["zt"])
    elif palmz > palmZTLimit:
        return int(servo_limits["zb"])
    else:
        # The final value that is returned is the percentage the servo's
        # bottom limit plus a percentage of the servo's top limit
        # This is done by multiplying the limit by a ratio of the difference from the limit divided by the midpoint
        return int(abs((((palmz - palmZBLimit) / palmZDelta) * servo_limits["zb"])
                       + ((1.0 - abs((palmz - palmZBLimit) / palmZDelta)) * servo_limits["zt"])))


# Changes claw coordinate into value for the servo motor
def changec(finger1_x, finger2_x, finger1_z, finger2_z):
    fingerXDist = abs(finger1_x - finger2_x)
    fingerZDist = abs(finger1_z - finger2_z)
    fingerDist = math.sqrt(abs((fingerXDist ** 2) + (fingerZDist ** 2)))
    fingersCDelta = current_dict['ct'] - current_dict['cb']

    if fingerDist <= current_dict['cb']:
        return int(servo_limits["ct"])
    elif fingerDist > current_dict['ct']:
        return int(servo_limits["cb"])
    else:
        return int(abs((((fingerDist - current_dict['cb']) / fingersCDelta) * servo_limits["cb"])
                       + ((1.0 - abs((fingerDist - current_dict['cb']) / fingersCDelta)) * servo_limits["ct"])))


# Generic LERP function, value must be [0,1]
def lerp(value, bot_limit, top_limit):
    if value > top_limit:
        return top_limit
    elif value < bot_limit:
        return bot_limit
    else:
        return ((1 - value)*bot_limit) + (value * top_limit)
