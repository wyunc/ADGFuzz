import argparse
import logging
import time
import os
import sys
from subprocess import *
import json
import re
import atexit

from fuzzer.postprocess import PostProcess
from model.Mapping import Mapp

from fuzzer.fuzz import ADGfuzzer
from model.parsefile import Parsefile

#for PX4:
import subprocess
import signal
from fuzzer.fuzzpx4 import PX4fuzzer
#from fuzzer.fuzzabl import ABLfuzzer


init_paths = {}

def ardupilot_init(arg):

    type = 'ArduCopter' # default
    if arg == 'copter':
        type = 'ArduCopter'
    elif arg == 'plane':
        type = 'ArduPlane'
    elif arg == 'rover':
        type = 'Rover'

    ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")
    #ARDUPILOT_HOME = '~/work/ArduPilot/'

    if ARDUPILOT_HOME is None:
        raise Exception("ARDUPILOT_HOME environment variable is not set!")
    ARDUPILOT_HOME = os.path.expanduser(ARDUPILOT_HOME)

    c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
    #c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w'

    sim = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
    print(f"Simulator started with gnome-terminal (PID: {sim.pid})")
    #sim = Popen(c, shell=True)
    #stdout, stderr = sim.communicate()  # Capture stdout and stderr
    # if bug_occured:
    #     os.killpg(os.getpgid(sim.pid), sim.SIGTERM) #replaced by terminate_ardupilot()

def terminate_ardupilot():
    try:
        #os.system("pkill -f 'sim_vehicle.py'")
        os.system("pkill -SIGINT -f 'sim_vehicle.py'")
        print("Terminated the sim_vehicle.py processes")
    except Exception as e:
        print(f"Failed to terminate sim_vehicle.py processes: {e}")
    logging.info('============ Fuzzing End ============')

def px4_init(arg):
    #we default to: arg = px4_copter
    pid_file = "/tmp/px4_shell_pid.txt"

    PX4_HOME = os.getenv("PX4_HOME")
    PX4_HOME = os.path.expanduser(PX4_HOME)
    #PX4_HOME = os.path.expanduser('~/work/PX4/') #: for test

    if PX4_HOME is None:
        raise Exception("PX4_HOME environment variable is not set!")
    if os.path.exists(pid_file):
        os.remove(pid_file)
    c = 'make clean && make distclean && make px4_sitl_default jmavsim'

    # before PX4-v1.13: rm -f build/px4_sitl_default/rootfs/fs/microsd/params
    #c = 'rm -rf build/px4_sitl_default/rootfs/log && rm -f build/px4_sitl_default/rootfs/parameters*.bson && make px4_sitl_default jmavsim'

    sim = subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        f"echo $$ > {pid_file}; cd {PX4_HOME} && {c}; exec bash"
    ])

    print(f"[+] PX4 Simulator started ")

def terminate_px4():
    pid_file = "/tmp/px4_shell_pid.txt" #use this?
    try:
        with open(pid_file, "r") as f:
            shell_pid = int(f.read().strip())
        os.killpg(os.getpgid(shell_pid), signal.SIGTERM)
        print(f"[-] Terminated shell and children with PGID {os.getpgid(shell_pid)}")
    except ProcessLookupError:
        print("[-] Shell already terminated.")
    except Exception as e:
        print(f"Failed to terminate px4_sitl: jmavsim processes: {e}")
    logging.info('============ Fuzzing End ============')

def read_json_file(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
        return data
    except json.JSONDecodeError:
        print(f"Failed to parse JSON:{file_path}")
    except Exception as e:
        print(f"Failed to read file-{file_path}, error msg:{e}")
    return None

def read_initfile(folder_path):
    for filepath in folder_path:
        if not os.path.exists(filepath):
            print(f"The folder does not exist:{filepath}")
            sys.exit(1)
        for root, _, files in os.walk(filepath):
            for file in files:
                if file.endswith('.json'):
                    file_path = os.path.join(root, file)
                    data = read_json_file(file_path)
                    if data:
                        init_paths.update(data)

def get_sorted_arithm_exception_files(outpath):
    # if not os.path.isdir(outpath):
    #     raise ValueError(f"error path:{outpath}")
    file_infos = []
    for filename in os.listdir(outpath or ""):
        if not filename.endswith('.txt'):
            continue

        name = filename[:-4]
        parts = name.split('_')
        if len(parts) < 3:
            continue

        bug_tag, bug_type = parts[0], parts[1]
        if bug_type != 'ArithmException':
            continue

        m = re.match(r'bug(\d+)$', bug_tag)
        if not m:
            continue

        idx = int(m.group(1))
        full_path = os.path.join(outpath, filename)
        file_infos.append((idx, full_path))

    file_infos.sort(key=lambda x: x[0])
    #return [path for _, path in file_infos]
    return file_infos

def main():
    arg_parser = argparse.ArgumentParser(description='Fuzzer')
    arg_parser.add_argument("--time", help='The time budget for the fuzzer', type=int, default=3600)
    arg_parser.add_argument("--initfile", nargs='+',
                            help="The initpath directory cut out after performing static analysis", type=str, default='paths')
    arg_parser.add_argument("--out_path", help="The path to save the result", type=str, default=None)
    arg_parser.add_argument("--log_path", help="log path", type=str, default=None)
    arg_parser.add_argument("--log_level", help="log level", type=str, default="info")
    arg_parser.add_argument("--rvtype", help="the type of robot vehicle, AP{copter, plane, rover} and PX4{px4}", type=str, default="copter")
    arg_parser.add_argument("--bugpost", help="Skip fuzzing and go straight to Bug Post-Processing", type=bool, default=False)
    #arg_parser.add_argument("--n", help="xxx", type=int, default=1)
    args = arg_parser.parse_args()

    if args.log_level.lower() == "error":
        log_level = logging.ERROR
    elif args.log_level.lower() == "info":
        log_level = logging.INFO
    else:
        log_level = logging.DEBUG

    logging.basicConfig(filename=args.log_path,
                        level=log_level,
                        filemode="w",
                        format="%(asctime)s [%(levelname)s] [%(module)s]: %(message)s",
                        datefmt='%Y-%m-%d %H:%M:%S')

    filepath = args.initfile
    rvtype = args.rvtype
    runtime = args.time
    outpath = args.out_path
    is_bug_post = args.bugpost

    # ADGFUZZ_HOME = os.getenv("ADGFUZZ_HOME")
    # if ADGFUZZ_HOME is None:
    #     raise Exception("ADGFUZZ_HOME environment variable is not set!")

    ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")
    if ARDUPILOT_HOME is None:
        raise Exception("ARDUPILOT_HOME environment variable is not set!")
    PX4_HOME = os.getenv("PX4_HOME")
    if rvtype == "px4" and PX4_HOME is None:
        raise Exception("PX4_HOME environment variable is not set!")

    if is_bug_post:
        logging.info("Start Bug Post-Processing.................")
        file_infos = get_sorted_arithm_exception_files(outpath)
        minm_dir = os.path.join(outpath, "minm")
        os.makedirs(minm_dir, exist_ok=True)
        logging.info(f'ArithmException file number: {len(file_infos)}')
        for idx, file_path in file_infos:

            has_delay = False
            valid_nodes = []
            try:
                with open(file_path, 'r', encoding='utf-8') as file:
                    for line in file:
                        # split to [type, name, value]
                        node = line.strip().split(' ', 2)
                        if node and node[0] in {'paramset', 'mavcmd'}:
                            if "SIM_RATE_HZ" in node[1]:
                                has_delay = True
                            valid_nodes.append(node)

            except FileNotFoundError:
                print(f"Error, file {file_path} not found。")
                continue
            except Exception as e:
                print(f"Error reading {file_path}: {e}")
                continue

            test_set = valid_nodes
            minm_file = os.path.join(minm_dir, f"bug{idx}_minm.txt")

            if has_delay:
                with open(minm_file, 'w', encoding='utf-8') as mf:
                    mf.write('SIM_RATE_HZ')
                continue

            bugpost = PostProcess(idx=idx, testcases=test_set, minm_file=minm_file,rvtype=rvtype)

            bugpost.minm_inputs()
        atexit.register(terminate_ardupilot)
        return

    logging.info("Parsing source code")

    # {func_name : { y : [x, ...],... },... }
    #  'file_name+func_name+y_name' : [x, ...]
    path_init = []
    max_nodecount = 0
    read_initfile(filepath)
    for funcname, funcbody in init_paths.items():
        for yvar, path in funcbody.items():
            nodes, node_count = path
            if node_count > max_nodecount:
                max_nodecount = node_count
            path_init.append((nodes, node_count))

    print(f'max node count: {max_nodecount}')

    m = Mapp(rvtype=rvtype, max_Hcount=max_nodecount)


    logging.info(f'++++++++++++++++++ Total (init)Paths: {len(path_init)} ++++++++++++++++++')


    m.parse_paths(path_init)
    logging.info(f'--- After matching and removing duplicates, total paths: {len(m.paths)} ---')

    print("========================================")

    logging.info("Initialize the Ardupilot and start SITL")
    logging.info("Wait for 50 seconds to ensure that the Drone(Ardupilot) initialization is complete")

    if rvtype == 'px4':
        px4_init(rvtype)
        atexit.register(terminate_px4)

    else:
        ardupilot_init(rvtype)
        atexit.register(terminate_ardupilot)

    logging.info(f"============ {rvtype}-SITL started ============")
    # atexit.register(terminate_ardupilot)
    # os.killpg(os.getpgid(sim.pid), signal.SIGKILL) #SIGTERM
    time.sleep(50)
    logging.info("Begin Fuzzing")

    if rvtype == 'px4':
        adgfuzz = PX4fuzzer(m.paths, rvtype, bug_out_path=outpath, time_budget=runtime)
    else:
        adgfuzz = ADGfuzzer(m.paths, rvtype, bug_out_path=outpath, time_budget=runtime)
    adgfuzz.run()
    adgfuzz.print_final_status()


if __name__ == '__main__':
    main()


# pre:
# export ARDUPILOT_HOME=~/work/ArduPilot/
# export PX4_HOME=~/work/PX4/


