# https://ardupilot.org/dev/docs/mavlink-rcinput.html
# By default, channels 1-4 are used for flight controls (i.e. roll, pitch, throttle, and yaw).
# The default flight mode channel is 8 for Plane and Rover and 5 for Copter.
# pwm range: 1000-2000(but most time is 1100-1900), 1500 means mid-position(0)
rc_copter_map = {
    'roll' : 1,
    'pitch' : 2,
    'throttle' : 3,
    'yaw' : 4,
    'mode' : 5
}

# https://ardupilot.org/plane/docs/common-rc-transmitter-flight-mode-configuration.html
# for mode, 0-1230 mode1, 1231-1360 mode2, 1361-1490 mode3, 1491-1620 mode4, 1621-1759 mode5, 1750+ mode6
rc_plane_map = {
    'roll' : 1,
    'pitch' : 2,
    'throttle' : 3,
    'yaw' : 4,
    'mode' : 5
}

# https://ardupilot.org/rover/docs/common-radio-control-calibration.html
rc_rover_map = {
    'steer' : 1,
    'throttle' : 3,
    'mode' : 8
}

cmd_map={
    1 : 'paramset',
    2 : 'cmdlong',
    3 : 'modeset',
    4 : 'randomfly'
}