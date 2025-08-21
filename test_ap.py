import time
import sys
import random
from symbol import return_stmt
import subprocess

from pymavlink import mavutil, mavwp
import re
import os
from subprocess import *
from fuzzer import rvmethod,runtimedict
import argparse

cmdid = runtimedict.MavcmdDictionary()
# Read GPS data function
def gps_data(the_connection):
    while True:
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
        exit(1) # need to exit?
    # Get mode ID and set new mode
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    return modename



def set_rc_channel_pwm(master, id, pwm=1500):
    if id < 1:
        print("Channel does not exist.")
        return

    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.


def arm_takeoff(master):
    #arm_command: 0 disarm, 1 arm
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    arm_command = 1

    master.mav.command_long_send(master.target_system, master.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(1)
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 50)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(15)



def loadmission1(master): # testing...
    loader = mavwp.MAVWPLoader()
    loader.target_system = master.target_system
    loader.target_component = master.target_component
    loader.load('missiondata/case1.txt')  # load mission
    timeout = 300 #fix
    master.waypoint_clear_all_send()
    master.waypoint_count_send(loader.count())
    try:
        # looping to send each waypoint information
        for i in range(loader.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=timeout)
            master.mav.send(loader.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))
        mission_ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=timeout)
        print(mission_ack_msg)
    except TimeoutError:
        print('upload mission timeout')




def send_mav_cmd(master, mavfunc, params):
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
    #params[0], params[1], params[2], params[3], params[4], params[5], params[6])

def send_mav_cmd1(master, mavfunc, param):
    #params = [float(p) for p in param]
    param = param.split(', ')
    params = []
    p_0 = param[0].split('[')[1]
    p_6 = param[6].split(']')[0]
    p0 = float(p_0)
    p1 = float(param[1])
    p2 = float(param[2])
    p3 = float(param[3])
    p4 = float(param[4])
    p5 = float(param[5])
    p6 = float(p_6)
    # p0 = int(param[0])
    # p1 = int(param[1])
    # p2 = int(param[2])
    # p3 = int(param[3])
    # p4 = int(param[4])
    # p5 = int(param[5])
    # p6 = int(param[6])
    print(p0)
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        43003,#mavfunc, #.encode('utf-8'),
        0, p0, p1, p2, p3, p4, p5, p6)
        #params[0], params[1], params[2], params[3], params[4], params[5], params[6])


def set_throttle(conn, pwm_value):
    chan_mask = 0b00000001000
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        chan_mask,
        0,  # channel1 (roll)
        0,  # channel2 (pitch)
        pwm_value,  # channel3 (throttle)
        0,  # channel4 (yaw)
        0, 0, 0, 0, 0, 0, 0, 0
    )
def reuse_node(master, node, needsleep=True, sleeptime=0.1): #

    if node[0] == 'paramset':
        # done:1. node[1] should be a keyword{alt, speed, acc, etc.}, and a [param_name] should be randomly selected using this keyword
        pvalue = node[2]
        print(f'param set {node[1]} {pvalue}')# Using for post-processing
        rvmethod.paramset(master, node[1].encode('utf-8'), pvalue)

    elif node[0] == 'mavcmd':
        cmdname = node[1]
        params = node[2]
        cmd = cmdname
        print(f'{cmdname} {params}')
        send_mav_cmd(master, cmd, params)

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
        #FIXME: maybe it needs to be 3

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


def init_ap():
    ARDUPILOT_HOME = '~/code/t2-ArduPilot/'
    #ARDUPILOT_HOME = '~/code/ArduPilot/'
    c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --coverage -w' #for debug: -G
    sim = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    time.sleep(50)
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    print(master.target_system)
    print("Connected to MAVLink device. Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=3)
    print("Heartbeat received!")

    rvmethod.change_mode(master, 'GUIDED')
    time.sleep(1)
    rvmethod.arm_takeoff(master, 30)
    time.sleep(5)
    rvmethod.loadmission(master, 1)
    time.sleep(3)

def get_sorted_arithm_exception_files(outpath):
    file_infos = []
    for filename in os.listdir(outpath or ""):
        if not filename.endswith('.txt'):
            continue
        name = filename[:-4]
        parts = name.split('_')
        if len(parts) < 3 or parts[1] != 'ArithmException':
            continue
        m = re.match(r'bug(\d+)$', parts[0])
        if not m:
            continue
        idx = int(m.group(1))
        file_infos.append((idx, os.path.join(outpath, filename)))
    file_infos.sort(key=lambda x: x[0])
    return file_infos

def process_arithm_exception_tests(outpath):

    target_params = [
        'SIM_RATE_HZ',
        'SIM_GPS1_HZ',
        'SIM_GPS2_HZ',
        'GPS2_TYPE',
        'SIM_PLD_ENABLE',
        'SIM_SONAR_SCALE',
        'SIM_GYR_FILE_RW',
        'INS_GYROFFS_Z',
        'SERIAL_PASS2',
        'MAV_CMD_EXTERNAL_POSITION_ESTIMATE',
        'MAV_CMD_CAN_FORWARD',
        'MAV_CMD_DO_SET_ROI_LOCATION',
        'MAV_CMD_DO_SET_HOME',
    ]
    target_params1 = [
        'SIM_GPS1_GLTCH_X',
        'SIM_GPS1_GLTCH_Y', 'SIM_GYR1_BIAS_X', 'SIM_GYR1_BIAS_Y', 'SIM_GYR1_BIAS_Z', 'SIM_GYR2_BIAS_X',
        'SIM_GYR2_BIAS_Y', 'SIM_GYR2_BIAS_Z',
        'SIM_ACC1_BIAS_X', 'SIM_ACC1_BIAS_Y', 'SIM_ACC1_BIAS_Z', 'SIM_ACC2_BIAS_X', 'SIM_ACC2_BIAS_Y',
        'SIM_ACC2_BIAS_Z', 'TERRAIN_SPACING',
        'SIM_DRIFT_SPEED', 'SIM_BATT_VOLTAGE', 'SIM_IMU_POS_X', 'SIM_IMU_POS_Y', 'SIM_IMU_POS_Z',
        'SIM_WIND_TURB',
        'INS_GYROFFS_X', 'INS_GYROFFS_Y', 'INS_GYR2OFFS_X', 'INS_GYR2OFFS_Y', 'INS_GYR2OFFS_Z'
    ]
    matches = {name: set() for name in target_params}

    file_infos = get_sorted_arithm_exception_files(outpath)
    minm_dir = os.path.join(outpath, "minm")
    os.makedirs(minm_dir, exist_ok=True)

    for idx, file_path in file_infos:

        found_in_this_file = set()

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    node = line.strip().split(' ', 2)
                    if not node or node[0] not in {'paramset', 'mavcmd'}:
                        continue

                    for name in target_params:
                        if name in node[1]:
                            matches[name].add(idx)
                            found_in_this_file.add(name)

        except FileNotFoundError:
            print(f"Error, file {file_path} not found。")
            continue
        except Exception as e:
            print(f"Error reading {file_path}: {e}")
            continue

        minm_file = os.path.join(minm_dir, f"bug{idx}_minm.txt")


        if found_in_this_file:
            with open(minm_file, 'w', encoding='utf-8') as mf:
                for name in sorted(found_in_this_file):
                    mf.write(name + "\n")
            continue

    summary_path = os.path.join(minm_dir, "match_summary.txt")
    with open(summary_path, 'w', encoding='utf-8') as sf:

        for name in target_params:
            idx_list = sorted(matches[name])
            idx_str = ",".join(map(str, idx_list))
            sf.write(f"'{name}': {idx_str}\n")

        # unmatched = [n for n in target_params if not matches[n]]
        # sf.write("\nUnmatched names: " + ",".join(unmatched))
        all_idxs = {idx for idx, _ in file_infos}
        matched_idxs = set().union(*matches.values())
        unmatched_idxs = sorted(all_idxs - matched_idxs)
        sf.write("\nUnmatched idx: " + ",".join(map(str, unmatched_idxs)))

def quick_match():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out_path", type=str, default=None,
                        help="The path to the folder containing result files")
    args = parser.parse_args()
    process_arithm_exception_tests(args.out_path)

def collect_coverage(index=1):
    time.sleep(15)
    os.system("pkill -SIGINT -f 'sim_vehicle.py'")
    print("Terminated the sim_vehicle.py processes")
    time.sleep(5)

    #todo: --directory : ARDUPILOT_HOME
    info_file = f"coverage/run_{index}.info"
    lcov_cmd = [
        "lcov",
        "--capture",
        "--directory", "/home/vvvic/code/t2-ArduPilot",
        "--base-directory", "/home/vvvic/code/t2-ArduPilot",
        "--output-file", info_file
    ]
    subprocess.run(lcov_cmd, check=True)
    # print("finished get .info, wait 10s")
    # time.sleep(10)
    # subprocess.run(["lcov", "--zerocounters", "--directory", "/home/vvvic/code/t2-ArduPilot/build/sitl/"], check=True)
    # print("zerocounters finished.")



if __name__ == "__main__":
    #quick_match()

    arg_parser = argparse.ArgumentParser(description='Post-Processing')
    arg_parser.add_argument("--init", help='start sitl', type=int, default=0)
    arg_parser.add_argument("--run", help='reuse node', type=int, default=1)
    args = arg_parser.parse_args()
    init = args.init
    torun = args.run # 0 = skip, 1=quick run , 2= run with sleep
    if init == 1:
        init_ap()
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    print(master.target_system)
    print("Connected to MAVLink device. Waiting for heartbeat...")
    heartbeat = master.wait_heartbeat(timeout=3)
    if heartbeat:
        print("Heartbeat received!")

    #time.sleep(15)

    #filepath = "outfile/ArithmException_bug0.txt"
    filepath = "outfile/testin.txt"

    if torun == 1:
        process_result_file(master, filepath)
    elif torun == 2:
        process_result_file(master, filepath, needsleep1=True)
        collect_coverage(index=1)
    elif torun ==3:
        process_result_file(master, filepath, needsleep1=True,slptime=1)
    elif torun ==4:
        process_result_file(master, filepath, needsleep1=True,slptime=0.02)
    elif torun == 5:
        process_result_file(master, filepath, needsleep1=True, slptime=0.5)

