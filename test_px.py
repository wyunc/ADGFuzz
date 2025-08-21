import time
import json
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink_dialect
import re
import os
import signal
import subprocess
import math
import argparse
import fuzzer.rvmethod as rvm
from fuzzer import rvmethod,runtimedict
from geopy.distance import geodesic

MAVLINK_CONNECTION_STRING = 'udp:localhost:14540'
PARAMS_FILE = 'px4_initial_params.json'
pid_file = "/tmp/px4_shell_pid.txt"
CONNECTION_TIMEOUT = 10
PARAM_FETCH_TIMEOUT = 30
PARAM_SET_RETRY_DELAY = 0.2


cmdid = runtimedict.MavcmdDictionary()
# Default MAVLink connection to jMAVSim (PX4 SITL)
connection_string = 'udp:127.0.0.1:14550' # For jMAVSim
#connection_string = 'udp:0.0.0.0:14540'

# connection_string = '/dev/ttyACM0' # Example for a real Pixhawk via USB
# baudrate = 115200 # Only needed for serial connections
# --- Mission Parameters ---
takeoff_alt = 10.0  # meters
waypoint_lat = 47.397742  # Example Latitude (replace with your desired waypoint)
waypoint_lon = 8.545594   # Example Longitude (replace with your desired waypoint)
waypoint_alt = 20.0  # meters above home
rtl_alt = 0 # Return to launch altitude (0 means land at current position after mission)

def connect_to_px4(conn_str='udp:127.0.0.1:14550'):
    """Establishes a MAVLink connection."""
    print(f"INFO: Connecting to MAVLink device on: {conn_str}")
    try:
        master = mavutil.mavlink_connection(conn_str)
        master.wait_heartbeat()
        print(f"INFO: Heartbeat from system (system {master.target_system} component {master.target_component})")
        return master
    except Exception as e:
        print(f"ERROR: Could not connect to MAVLink: {e}")
        return None


def upload_mission(master, mission_file='missiondata/px4_mission1.txt'):

    if not os.path.exists(mission_file):
        raise FileNotFoundError(f"Mission file not found: {mission_file}")

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
        x = int(float(parts[8]) * 1e7)  # lat
        y = int(float(parts[9]) * 1e7)  # lon
        z = float(parts[10])           # alt
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
        print(f"Sent mission item {item.seq}")




def get_vehicle_position_info(master):
    """Requests and prints Home Position and Global Position."""
    if not master:
        print("ERROR: No MAVLink connection for position info.")
        return None, None

    # Request HOME_POSITION
    # Note: HOME_POSITION might not be sent regularly without a request or if not set.
    # It's often set once on arming or first GPS lock.
    # For SITL, it should be set quickly.
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)
    print("INFO: Requested home position...")

    home_pos = None
    global_pos = None

    # Try to receive messages for a few seconds
    # It's better to listen for specific messages after a request,
    # but for diagnostics, a short timed listen can work.
    start_time = time.time()
    while time.time() - start_time < 5: # Listen for 5 seconds
        msg = master.recv_match(type=['HOME_POSITION', 'GLOBAL_POSITION_INT'], blocking=False)
        if msg:
            if msg.get_type() == 'HOME_POSITION' and not home_pos: # Get first one
                home_pos = msg
                print(f"INFO: Received HOME_POSITION: Lat={home_pos.latitude/1e7}, Lon={home_pos.longitude/1e7}, Alt={home_pos.altitude/1000.0}m AMSL")
            elif msg.get_type() == 'GLOBAL_POSITION_INT' and not global_pos: # Get first one
                global_pos = msg
                print(f"INFO: Received GLOBAL_POSITION_INT: Lat={global_pos.lat/1e7}, Lon={global_pos.lon/1e7}, RelAlt={global_pos.relative_alt/1000.0}m")
        if home_pos and global_pos:
            break
        time.sleep(0.1)

    if not home_pos:
        print("WARN: Did not receive HOME_POSITION message.")
    if not global_pos:
        print("WARN: Did not receive GLOBAL_POSITION_INT message.")
    return home_pos, global_pos

def arm_and_set_auto_mode(master):

    if not master:
        print("ERROR: No MAVLink connection.")
        return False

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Param1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0  # Unused params
    )

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("INFO: Vehicle armed!")
    else:
        print(f"ERROR: Arming failed. ACK: {ack}")
        # Try to disarm if arming failed partway
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        return False

    print("INFO: Setting mode to AUTO...")

    # PX4 custom mode for AUTO_MISSION is 4
    # Standard MAVLink mode for AUTO is mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED (but PX4 uses custom modes)
    px4_auto_mode = 4 # For PX4 AUTO_MISSION
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        px4_auto_mode
    )

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("=============== Mode set to AUTO.MISSION ===============")
    else:
        print(f"ERROR: Failed to set mode to AUTO. ACK: {ack}")
        return False

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0,
        0, # last_item (0 for last, i.e. execute all)
        0,0,0,0,0 # Unused params
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_MISSION_START and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("INFO: Mission start command accepted.")
        return True
    else:
        print(f"ERROR: Mission start command failed. ACK: {ack}")
        return False



def set_mode_px4(master, mode_name):

    px4_modes = {
        "MANUAL": 1,
        "ALTCTL": 2,
        "POSCTL": 3,
        "AUTO.MISSION": 4,
        "AUTO.LOITER": 5,
        "AUTO.RTL": 6,
        "OFFBOARD": 6,  # Same custom_mode as AUTO, sub_mode 1 handled internally
        "AUTO.TAKEOFF": 10
    }

    if mode_name not in px4_modes:
        raise ValueError(f"Unsupported PX4 mode: {mode_name}")

    custom_mode = px4_modes[mode_name]

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,  # main mode
        0, 0, 0, 0, 0
    )
    print(f"[+] Switched PX4 to mode: {mode_name}")

def set_px4_mode(master, mode_name):

    px4_modes = {
        "MANUAL": 1,
        "ALTCTL": 2,
        "POSCTL": 3,
        "AUTO.MISSION": 4,
        "AUTO.LOITER": 5,
        "AUTO.RTL": 6,
        "OFFBOARD": 6,
        "AUTO.TAKEOFF": 10
    }

    if mode_name not in px4_modes:
        raise ValueError(f"[!] Unsupported PX4 mode: {mode_name}")

    custom_mode = px4_modes[mode_name]

    master.set_mode(
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode
    )
    print(f"[+] PX4 flight mode set to: {mode_name}")


def px4init():
    PX4_HOME = os.path.expanduser('~/code/PX4/')
    #c = 'make clean && make distclean && make px4_sitl_default jmavsim'

    #before 1.13:rm -f build/px4_sitl_default/rootfs/fs/microsd/params
    c = 'rm -rf build/px4_sitl_default/rootfs/log/ && rm -f build/px4_sitl_default/rootfs/parameters*.bson && make px4_sitl_default jmavsim'
    #c = 'make px4_sitl_default jmavsim' #for test

    #pid_file = "/tmp/px4_shell_pid.txt" #act as global param
    if os.path.exists(pid_file):
        os.remove(pid_file)

    sim = subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        f"echo $$ > {pid_file}; cd {PX4_HOME} && {c}; exec bash"
    ])
    for _ in range(20):
        if os.path.exists(pid_file):
            break
        time.sleep(0.1)

    print("[+] PX4 jmavsim start...")
    time.sleep(20)

def mission_takeoff(master):
    upload_mission(master)
    set_px4_mode(master, "AUTO.MISSION")
    # set_mode_px4(master, "MANUAL")
    time.sleep(1)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[+] Vehicle armed")


def get_px4mode(master):
    print("PX4 modes are:")
    # for mode in master.mode_mapping():
    #     print(mode)
    print(master.mode_mapping())
    #MANUAL, STABILIZED, ACRO, RATTITUDE, ALTCTL, POSCTL, LOITER, MISSION, RTL, LAND, RTGS, FOLLOWME, OFFBOARD, TAKEOFF

def send_mav_cmd(master, cmdid, params):
    #used for post-processing[reuse_node]
    cmd = cmdid
    param = params.split(', ')
    p_0 = param[0].split('[')[1]
    p_6 = param[6].split(']')[0]
    p0 = float(p_0)

    p1 = float(param[1])
    p2 = float(param[2])
    p3 = float(param[3])
    p4 = float(param[4])
    p5 = float(param[5])
    p6 = float(p_6)
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        cmd,#mavfunc, #.encode('utf-8'),
        0, p0, p1, p2, p3, p4, p5, p6)

def check_oracle(master, label=True):
    # self.bug_oracle = RVoracle()
    try:
        retries = 6
        while label:
            msg = None
            for _ in range(retries):
                msg = master.recv_match(blocking=True, timeout=0.3)
                if msg is not None:
                    break
            if msg is None:
                print("##### Bug found: no-link/Arithm_Exception!! #####")  # fp
                # timeout_bug = True
                continue
    except Exception as e:
        print(f"Oracle_thread crashed with error: {e}")


def check_wp_deviation_px4(master):
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    waypoints = []
    count = None

    msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if not msg:
        #raise RuntimeError("no MISSION_COUNT")
        print("no MISSION_COUNT")
    count = msg.count
    print(f"Mission has {count} items")

    for seq in range(count):
        master.mav.mission_request_int_send(
            master.target_system,
            master.target_component,
            seq
        )
        item = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
        if not item:
            print(f"request wp {seq} timeout")
        print(item.to_dict().keys())
        lat = item.x / 1e7
        lon = item.y / 1e7
        alt = item.z / 1e3
        waypoints.append((lat, lon, alt))
        print(f"Got WP {seq}: {waypoints[-1]}")

    #get current mission.wp[count]
    while True:
        #msg = master.recv_match(type=['MISSION_COUNT', 'MISSION_ITEM'], blocking=True, timeout=5) # not right??
        current_pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1) #current_pos
        mission_pos = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1) #mission_pos
        if not current_pos or not mission_pos:
            continue

        idx = mission_pos.seq
        lat0 = current_pos.lat / 1e7
        lon0 = current_pos.lon / 1e7
        lat1, lon1, _ = waypoints[idx]

        phi1, phi2 = math.radians(lat0), math.radians(lat1)
        delta_phi = math.radians(lat1 - lat0)
        delta_lambda = math.radians(lon1 - lon0)
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        d = 2 * 6371000 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        print(f"[WP {idx}] distance is: {d:.1f} m")

def load_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            if not line or line.startswith('QGC'):
                continue
            #parts = line.split()
            parts = line.strip().split()
            if len(parts) < 12:
                continue
            lat = float(parts[8])
            lon = float(parts[9])
            alt1 = float(parts[10])
            waypoints.append({'lat': lat, 'lon': lon, 'alt': alt1})
    return waypoints

def location_distance(lat1, lon1, lat2, lon2):
    return geodesic((lat1, lon1), (lat2, lon2)).meters

def check_wpdev(master):

    waypoints = load_waypoints('missiondata/px4_mission1.txt')

    print(f"Loaded {len(waypoints)} waypoints.")

    last_distance = None
    deviate_count = 0

    while True:
        mission_current = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        if not mission_current:
            continue
        current_wp = mission_current.seq
        if current_wp >= len(waypoints):
            print("No more waypoints.")
            break

        pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if not pos_msg:
            continue
        curr_lat = pos_msg.lat / 1e7
        curr_lon = pos_msg.lon / 1e7
        curr_alt = pos_msg.alt / 1e3

        target_wp = waypoints[current_wp]
        target_lat = target_wp['lat']
        target_lon = target_wp['lon']
        target_alt = target_wp['alt']

        distance = location_distance(curr_lat, curr_lon, target_lat, target_lon)
        print(f"[WP{current_wp}] Distance to waypoint: {distance:.2f} m")

        if last_distance is not None:
            #if distance > last_distance:
            if distance - last_distance > 0.05: #set threshold=0.05 to prevent FP caused by small perturbations
                deviate_count += 1
            else:
                deviate_count = 0
        last_distance = distance

        if deviate_count >= 3:
            print("!!! ALERT: UAV is deviating from waypoint! !!!")

        time.sleep(1)


def get_home_point_and_save(master):

    print("Requesting current global position...")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if not msg:
        raise RuntimeError("Timeout: no GLOBAL_POSITION_INT received")
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000.0  # cm to m
    #lat=47.3977419, lon=8.5455938, alt=488.00
    print(f"Current position: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m")
    print(f"origin data: lat={msg.lat}, lon={msg.lon}, alt={msg.alt}")

def send_mav_cmd1(master, mavfunc, params):
    cmd = cmdid.get_index(mavfunc)
    param = params.split(', ')
    p_0 = param[0].split('[')[1]
    p_6 = param[6].split(']')[0]
    p0 = float(p_0)

    p1 = float(param[1])
    p2 = float(param[2])
    p3 = float(param[3])
    p4 = float(param[4])
    p5 = float(param[5])
    p6 = float(p_6)
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        cmd,#mavfunc, #.encode('utf-8'),
        0, p0, p1, p2, p3, p4, p5, p6)

def reuse_node(master, node, needsleep=True, sleeptime=0.1): #

    if node[0] == 'paramset':
        # done:1. node[1] should be a keyword{alt, speed, acc, etc.}, and a [param_name] should be randomly selected using this keyword
        pvalue = node[2]
        print(f'param set {node[1]} {pvalue}')# Using for post-processing
        rvmethod.paramset_px4(master, node[1].encode('utf-8'), pvalue)

    elif node[0] == 'mavcmd':
        cmdname = node[1]
        params = node[2]
        cmd = cmdname
        print(f'{cmdname} {params}')
        send_mav_cmd1(master, cmd, params)

    elif node[0] == 'rc':
        pwm = node[2]
        rc_channel = node[1]
        print(f'execute rc {rc_channel} {pwm}')
        rvmethod.set_rc_channel_pwm(rc_channel,pwm)

    elif node[0] == 'modeset':
        print(f'process mode set: mode {node[1]}')
        rvmethod.change_mode(master, node[1])

    if needsleep:
        time.sleep(sleeptime) # Wait a certain amount of time to make sure we catch the bug caused by the current input

def process_result_file(master, file_path, needsleep1=False,slptime=0.1):
    #o = fast, 1 = slow
    valid_nodes = []

    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            for line in file:
                node = line.strip().split(' ', 2)
                if node[0] in {'paramset', 'mavcmd'}:
                    valid_nodes.append(node)

    except FileNotFoundError:
        print(f"Error, file {file_path} not found。")
        return
    except Exception as e:
        print(f"{e}")
        return

    for node in valid_nodes:
        #print(node)
        #reuse_node(master, node)
        reuse_node(master, node, needsleep=needsleep1,sleeptime=slptime)


def test_func():
    px4init()
    master = connect_to_px4(connection_string)
    if not master:
        print("CRITICAL: Exiting due to connection failure.")

    # get_home_point_and_save(master)

    mission_takeoff(master)
    print("[<>]in sleep")
    time.sleep(10)
    check_wpdev(master)
    # check_wp_deviation_px4(master)

    #=================== just for test ===================
    #get_px4mode(master)
    #set_mode_px4(master, "OFFBOARD")
    #check_oracle(master)
    la = True
    print("[<>] loop of the msg.xx")

    msglst = []
    last_alt = None
    airborne = False
    while la:
        try:
            #msg = master.recv_match(blocking=True, timeout=0.3)
            msg = master.recv_match(type=['EXTENDED_SYS_STATE', 'VFR_HUD', 'GLOBAL_POSITION_INT', 'STATUSTEXT'],blocking=True, timeout=1)
            #msg = master.recv_match(type=['STATUSTEXT'], blocking=True, timeout=2) # still not right
            #msg = master.recv_match(type=['EXTENDED_SYS_STATE'], blocking=True, timeout=2)
            #msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=1)
            #FIXME: why no statustext? #why no msg????
            if msg is None:
                continue
            if msg.get_type() == 'EXTENDED_SYS_STATE':
                landed_state = msg.landed_state
                if landed_state == 2:  # IN_AIR
                    airborne = True
                elif landed_state == 1:  # ON_GROUND
                    if airborne:
                        print("ALERT: UAV has landed or possibly crashed!")
                        airborne = False

            if msg.get_type() == 'GLOBAL_POSITION_INT':
                alt = msg.relative_alt / 1000.0
                print(f"GLOBAL_POSITION_INT.relative_alt is : {alt}")
                if airborne and alt < 0.3:
                    print("ALERT: Altitude dropped below threshold, possible crash/landing.")

            if msg.get_type() == 'STATUSTEXT': #not used
                if 'Land' in msg.text or 'land' in msg.text:
                    print("ALERT: CRASH detected: ", msg.text)

            time.sleep(0.1)
        except Exception as e:
            print(e)

    # todo: I should test if each function in oracle is useful;  how to read status.
    # +bug_oracle:  (wp_dev)all_oracle->wp_oracle_px4, (nolink)check_oracle+(hit_ground)process_check,
    # right: timeout_bug(check_status: msg not None)
    # need_to_change: , wp_deviation, (done but not test)hit_ground:process_check->process_message_px4()

    #rv_method
    # right: send_mav_cmd, set_rc_channel_pwm
    # changed: (done)paramset->paramset_px4, (done)change_mode->change_mode_px4, (done)loadmission->loadmission_px4,
    #                 (done)arm_takeoff={load_mission+change_mode_px4(AUTO.MISSION)+arm_throttle},
    # Note: loadmission_px4() should be executed together with arm_takeoff() for UAV takeoff and task loading

    # param_name = "MC_PR_INT_LIM"
    # value = 0.3
    # rvm.paramset_px4(master, param_name.encode('utf-8'), value)

    # index = 21
    # params = '[1, 180, 84, 180, 399, 300, 699]'
    # send_mav_cmd(master, index, params)



    # =================== end of test ===================

    # with open(pid_file, "r") as f:
    #     shell_pid = int(f.read().strip())
    # print(f"[+] Started PX4 shell with PID {shell_pid}")

    # try:
    #     os.killpg(os.getpgid(shell_pid), signal.SIGTERM)
    #     print(f"[-] Terminated shell and children with PGID {os.getpgid(shell_pid)}")
    # except ProcessLookupError:
    #     print("[-] Shell already terminated.")




if __name__ == '__main__':

    arg_parser = argparse.ArgumentParser(description='Post-Processing')
    arg_parser.add_argument("--init", help='start sitl', type=int, default=0)
    arg_parser.add_argument("--run", help='reuse node', type=int, default=1)
    args = arg_parser.parse_args()
    init = args.init
    torun = args.run  # 0 = skip, 1=quick run , 2= run with sleep
    if init == 1:
        px4init()
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    print(master.target_system)
    print("Connected to MAVLink device. Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=3)
    if heartbeat:
        print("Heartbeat received!")
    if init ==1:
        mission_takeoff(master)
        print("[<>]mission start")

        #The following is to test route deviation and HitGround
        time.sleep(5)

        print("[<>]in sleep")
        time.sleep(10)
        check_wpdev(master)


        while True:
            latest_msg = None
            start_time = time.time()
            while True:
                msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if not msg:
                    break
                latest_msg = msg
                if time.time() - start_time > 0.05:
                    break

            if latest_msg:
                alt = latest_msg.relative_alt / 1000.0
                print(f"[Alt Report] alt is: {alt}")
            time.sleep(0.1)

        # while True:
        #     msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=0.3)
        #     if msg:
        #         alt = msg.relative_alt / 1000.0
        #         print(f"[Alt Report] alt is: {alt}")
        #         time.sleep(0.3)


    # filepath = "outfile/ArithmException_bug0.txt"
    filepath = "outfile/testin.txt"

    if torun == 1:
        process_result_file(master, filepath)
    elif torun == 2:
        process_result_file(master, filepath, needsleep1=True) #0.1
    elif torun == 3:
        process_result_file(master, filepath, needsleep1=True, slptime=1)
    elif torun == 4:
        process_result_file(master, filepath, needsleep1=True, slptime=0.02)
    elif torun == 5:
        process_result_file(master, filepath, needsleep1=True, slptime=0.5)
