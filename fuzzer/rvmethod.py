import time
import os
import random
from pymavlink import mavutil, mavwp
import re
'''
Some RV [through mavlink] method
including : arm throttle and takeoff, parameter set, mode change, load mission, etc.
'''

def gps_data(master):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat
        lon = msg.lon
        alt = msg.relative_alt
        return lat, lon, alt

def change_mode(master, modename):
    # Choose a mode and check whether the mode is available
    mode = modename
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        mode = 'AUTO'
        #if unknown mode ,then use 'auto' mode

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    return modename
#PX4 does not use the string pattern name mapping like ArduPilot, but uses the so-called "custom_mode"
def change_mode_px4(master, mode_name):
    px4_modes = {
        "MANUAL": 1,
        "ALTCTL": 2,
        "POSCTL": 3,
        "AUTO.MISSION": 4,
        "AUTO.LOITER": 5,
        "AUTO.RTL": 6,
        "OFFBOARD": 6, #ERROR [simulator_mavlink] poll timeout 0, 22
        "AUTO.TAKEOFF": 10
    }
    if mode_name not in px4_modes:
        raise ValueError(f"[!] Unsupported PX4 mode: {mode_name}")

    custom_mode = px4_modes[mode_name]

    master.set_mode(
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode
    )
    print(f"#### PX4 flight mode set to: {mode_name} ####")

def set_rc_channel_pwm(master, channel_id, pwm=1500):
    '''
    ref : https://www.ardusub.com/developers/pymavlink.html and https://www.ardusub.com/developers/rc-input-and-output.html
    channel_id: 1-pitch, 2-roll, 3-throttle, 4-yaw, 5-forward?, 6-lateral,
    7-camera pan, 8-camera tilt, 9-lights 1 level, 10-lights 2 level, 11-video switch
    Channel pwm value 1100-1900. mid:1500
    '''

    if channel_id < 1 or channel_id >18:
        #print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[channel_id - 1] = pwm

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values)  # RC channel list, in microseconds.


def arm_takeoff(master, alt=30):
    #arm_command: 0 disarm, 1 arm
    arm_command = 1
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)
    #msg = master.recv_match(type='COMMAND_ACK', blocking=True) #FIXME: this line??? seems useless?
    #print(msg)
    time.sleep(1)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    #msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #print(msg)
    #time.sleep(5)

def arm_takeoff_px4(master):
    # load mission + set mode AUTO.MISSION + arm throttle->takeoff
    loadmission_px4(master)

    change_mode_px4(master, 'AUTO.MISSION')

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print(f"[Mission & Takeoff] Vehicle armed and takeoff: 10 meters. Start mission.")
    # The alt should be set in px4_mission.txt

def loadmission(master, n=1):
    # add a parameter 'n', means that the n-th task file is selected and loaded.
    loader = mavwp.MAVWPLoader()
    loader.target_system = master.target_system
    loader.target_component = master.target_component
    # pre-define some missions
    missionname = f'missiondata/case{n}.txt'
    if not os.path.isfile(missionname):
        print(f"{missionname} does not exist. Using default mission: missiondata/case1.txt")
        missionname = 'missiondata/case1.txt'
    loader.load(missionname)  # load mission

    timeout = 6 #fix 300
    master.waypoint_clear_all_send()
    master.waypoint_count_send(loader.count())
    try:
        # looping to send each waypoint information
        for i in range(loader.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=timeout)
            master.mav.send(loader.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))
        mission_ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout) #FIXME: blocking?
        print(mission_ack_msg)
    except TimeoutError:
        print('upload mission timeout')
    change_mode(master,'AUTO')


def loadmission_px4(master, n=1):
    mission_file = f'missiondata/px4_mission{n}.txt'
    if not os.path.isfile(mission_file):
        print(f"{mission_file} does not exist. Using default mission: missiondata/case1.txt")
        mission_file = 'missiondata/px4_mission1.txt'
    with open(mission_file, 'r') as f:
        lines = f.readlines()

    if not lines[0].startswith("QGC WPL 110"):
        raise ValueError("Invalid mission file: must start with 'QGC WPL 110'")

    mission_items = []
    for line in lines[1:]:
        #parts = line.strip().split('\t')
        parts = re.split(r'\s+', line.strip())
        if len(parts) < 12:
            continue

        seq = int(parts[0])
        current = int(parts[1])
        frame = int(parts[2])
        command = int(parts[3])
        param1 = float(parts[4])
        param2 = float(parts[5])
        param3 = float(parts[6])
        param4 = float(parts[7])
        x = int(float(parts[8]) * 1e7) # lat
        y = int(float(parts[9]) * 1e7) # lon
        z = float(parts[10])
        autocontinue = int(parts[11])

        mission_items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=master.target_system,
            target_component=master.target_component,
            seq=seq,
            frame=frame,
            command=command,
            current=current,
            autocontinue=autocontinue,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            x=x,
            y=y,
            z=z,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        ))

    # Clear old tasks
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(mission_items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    for item in mission_items:
        # req = master.recv_match(type='MISSION_REQUEST_INT', blocking=True, timeout=5)
        # if req is None:
        #     raise RuntimeError("Timeout: No MISSION_REQUEST_INT received")
        master.mav.send(item)
    print(f"[Mission] Sent mission file: {mission_file}")



def paramset(master, paramid, paramvalue):
    '''paramid should be encode('utf-8') '''
    # usage :  paramset(self.master, paramid=b'xxx', paramvalue=int/float?)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        paramid,
        float(paramvalue),
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )

    # master.param_set_send(param, value) # use mavutil is ok?
    # msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    # print(msg)

def paramset_px4(master, paramid, paramvalue):
    master.mav.param_set_send(master.target_system, master.target_component,
                              paramid,
                              float(paramvalue),
                              mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

def randomfly(master, wp_cnt):
    # usage: wpcnt=0, randomfly(master, wpcnt) , wpcnt += 1
    wp_add_alt = random.randint(50, 100)

    lat = -35 + random.uniform(0, 2) #: change lat&lon
    lon = 148 + random.uniform(0, 2)
    master.mav.mission_item_send(master.target_system, master.target_component,
                                 wp_cnt,
                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 2, 0, 0, 0, 0, 0,
                                 lat, lon, wp_add_alt)
    print("Sent change waypoint command (lat:%f, lon:%f, alt:%f)" % (lat, lon, wp_add_alt))
    time.sleep(10)

def get_mission_wp_data(master):
    # t1 = threading.Thread(target=throttle_th, args=())
    # t1.daemon = True
    # t1.start()
    while True:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True)
        if msg:
            current_wp = msg.seq
            print(f"Current waypoint: {current_wp}")

            master.mav.mission_request_send(
                master.target_system,
                master.target_component,
                current_wp
            )

            wp_msg = master.recv_match(type='MISSION_ITEM', blocking=True)
            #FIXME: error!!!!!!!!!!!!!! always waiting...

            if wp_msg:
                print(f"Waypoint {wp_msg.seq}: lat={wp_msg.x}, lon={wp_msg.y}, alt={wp_msg.z}")
                return wp_msg.seq, wp_msg.x, wp_msg.y, wp_msg.z

def send_mav_cmd(master, mavfunc, params):
    '''
    Execute a command precisely
    (deleted)mavmsg:  xx_xx_send # For MAV_CMD_XXX, they are sent using [mission_item_send]? That's not true !!!
    mavfunc: MAV_CMD_XXX , etc.
    '''

    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavfunc, #cmd_id or cmd_name // use index(int)
        0,
        params[0], params[1], params[2], params[3], params[4], params[5], params[6])
    #print(f'executed {mavfunc} finished.')
    # process RC1-4 , flight mode?


'''Below not tested'''
def set_servo(master, servo, pwm):
    ''' Set a single servo output pwm value.
    'servo' can only be outputs that aren't assigned as motors, so is
    generally used for lights/camera etc.
    When in a per_thruster_control context in Servo mode, also allows
    controlling individual thrusters.
    '''
    # master = mavutil.mavlink_connection(*args, **kwargs)
    print(f'set_servo({servo=}, {pwm=})')
    master.set_servo(servo, pwm)

def send_rc(master, rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
            rcin5=65535, rcin6=65535, rcin7=65535, rcin8=65535,
            rcin9=65535, rcin10=65535, rcin11=65535, rcin12=65535,
            rcin13=65535, rcin14=65535, rcin15=65535, rcin16=65535,
            rcin17=65535, rcin18=65535, *, # keyword-only from here
            pitch=None, roll=None, throttle=None, yaw=None, forward=None,
            lateral=None, camera_pan=None, camera_tilt=None, lights1=None,
            lights2=None, video_switch=None):
    ''' Sets all 18 rc channels as specified.
    Values should be between 1100-1900, or left as 65535 to ignore.
    Can specify values:
        positionally,
        or with rcinX (X=1-18),
        or with default RC Input channel mapping names
            -> see https://ardusub.com/developers/rc-input-and-output.html
    It's possible to mix and match specifier types (although generally
        not recommended). Default channel mapping names override positional
        or rcinX specifiers.
    '''
    rc_channel_values = (
        pitch        or rcin1,
        roll         or rcin2,
        throttle     or rcin3,
        yaw          or rcin4,
        forward      or rcin5,
        lateral      or rcin6,
        camera_pan   or rcin7,
        camera_tilt  or rcin8,
        lights1      or rcin9,
        lights2      or rcin10,
        video_switch or rcin11,
        rcin12, rcin13, rcin14, rcin15, rcin16, rcin17, rcin18
    )
    print(rc_channel_values)
    #self.master = mavutil.mavlink_connection(*args, **kwargs)
    #self.mav = self.master.mav
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values
    )