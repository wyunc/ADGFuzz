import time
import sys
import random
import json
import logging
import re
import os
import csv
import bisect
import threading
import numpy as np
from subprocess import Popen, PIPE
import signal
import subprocess
from datetime import datetime
from pymavlink import mavutil, mavwp
from fuzzer.oracle import RVoracle
from fuzzer import rvmethod
from model.parsefile import Parsefile

from fuzzer.runtimedict import RuntimeDictionary, MavcmdDictionary


#init: define RV start
class PX4fuzzer:
    def __init__(self, paths, rvtype, bug_out_path="outfile/test/", pars: Parsefile = Parsefile(), time_budget = 300):

        self.cmd_list = []
        self.cmd_id = []

        self.param_list = []
        self.param_set = RuntimeDictionary()  # done
        self.mavcmd_param = MavcmdDictionary()
        self.env_set = []
        self.paths = paths
        self.mav_list = pars.mavfunc_list # useless
        #Oracle
        self.bug_oracle = RVoracle()
        self.running = threading.Event()
        self.oracle_thread = None
        self.rvstatus_thread = None
        self.msgprocess_thread = None
        #self.oracle_lock = threading.Lock()
        #record related:
        self.begin_time = time.time()
        self.time_budget = time_budget  # minutes
        self.param_exec_time = 10
        self.cmd_exec_time = 5
        self.path_count = 0
        self.wp_count = 0
        self.bug_count = 0  # +1 when find a bug_curve
        self.bug_curve = [{
            "time": 0,
            "count": 0,
            "type" : ""
        }]
        self.temp_value = [] #self.temp_value = self.temp_value[-4:] + [new_value] # record the input of one path-fuzzing
        self.bug_log_file = "outfile/bug_log.txt" # Record the RV input that triggers the bug and the corresponding value
        self.minm_result_file = "outfile/result_min.txt"
        self.bug_out_path = bug_out_path

        self.bug_paths = []
        #self.pos_error = [] #set()
        self.hit_ground = []
        self.instab_error = []
        self.nolink_error = []
        self.wpdevi_error = []
       # self.inter_error = []


        self.bug_inputs = set()  # single input_name: it can cause bugs on its own
        #RV
        self.master = self.connect_init()
        self.oracle_master = self.Oconn_init()
        self.rvtype = rvtype
        #self.total_entropy = 0
        self.total_round = 0
        self.pid_file = "/tmp/px4_shell_pid.txt"

    def connect_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        master.wait_heartbeat()
        #logging.info("received heartbeat, get Plane gps location")
        return master

    def Oconn_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550') # We can use the same udp interface
        master.wait_heartbeat()
        #logging.info("received heartbeat, get Plane gps location")
        return master

    def run_oracle(self):
        #self.bug_oracle = RVoracle()
        try:
            self.bug_oracle.wp_oracle_px4(self.oracle_master,self)
        except Exception as e:
            logging.error(f"Oracle_thread crashed with error: {e}")

    def check_oracle(self):
        try:
            self.bug_oracle.check_status(self.oracle_master,self)
        except Exception as e:
            logging.error(f"rvstatus_thread crashed with error: {e}")
    def process_check(self):
        try:
            self.bug_oracle.process_messages_px4(self)
        except Exception as e:
            logging.error(f"process_messages crashed with error: {e}")

    def load_found_buginputs(self):
        try:
            with open('outfile/bug_input_px4.txt', 'r') as file:
                for line in file:
                    stripped_line = line.strip()
                    if stripped_line:
                        self.bug_inputs.add(stripped_line)
        except FileNotFoundError:
            return

    def select_from_paths(self):
        #C<O(n). With 5000 elements, this func runs in approximately: 0.0004 seconds
        if not self.paths:
            raise ValueError("self.paths is empty")

        entropies = np.array([x[1] for x in self.paths])
        # Convert to a probability distribution using softmax
        exp_entropies = np.exp(entropies - np.max(entropies))
        probabilities = exp_entropies / np.sum(exp_entropies)

        selected_index = np.random.choice(len(self.paths), p=probabilities)
        return selected_index

    def adj_entropy(self, selected_index, delta=1):

        selected_item = self.paths[selected_index]
        adjed_entropy = min((selected_item[1] - delta), (selected_item[1]/2))

        if adjed_entropy > 0:
            selected_item = (selected_item[0], adjed_entropy, selected_item[2])
            self.paths[selected_index] = selected_item
        else:
            print(f"////// delete the self.path[{selected_index}]: {selected_item[2]} //////")
            self.rmv_path(selected_index)

    def adj_entropy_notfound(self, selected_index, delta=2):

        selected_item = self.paths[selected_index]
        adjed_entropy = min((selected_item[1] - delta), (selected_item[1]*0.6))

        if adjed_entropy > 0:
            selected_item = (selected_item[0], adjed_entropy, selected_item[2])
            self.paths[selected_index] = selected_item
        else:
            print(f"////// delete the self.path[{selected_index}]: {selected_item[2]} //////")
            self.rmv_path(selected_index)

    def rmv_path(self, selected_index):
        self.paths.pop(selected_index)


    def weighted_random_choice(self, cumulative_weights, total_entropy):
        rand_val = random.uniform(0, total_entropy)
        index = bisect.bisect_left(cumulative_weights, rand_val)
        #return index
        return self.paths[index]

    #The main fuzzing function
    def run(self):
        self.begin_time = time.time()
        begin = self.begin_time
        cur_time = time.time() - begin
        round = 1

        logging.info("===================== Start Fuzzing =====================")

        self.load_found_buginputs()

        rvmethod.arm_takeoff_px4(self.master)

        logging.info("Wait for 8 seconds to confirm that the UAV is in the ascending state")
        time.sleep(8)
        # while time<budget time: / while True:
        while cur_time < self.time_budget:
            logging.info(f'--------------- Test the {round}-th path --------------- ')

            select_index = self.select_from_paths()
            path, entropy, i_path = self.paths[select_index]

            # receiving the result of static_analysis -> <path1, path2, ...>

            param_exec_n = 10
            cmd_exec_n = 5
            if not path:
                continue
            run_time = 50
            # if entropy < 1:
            #     continue
            if entropy > 500: #  reset this?
                run_time = 500
            elif entropy > 50:
                run_time = int(entropy)

            # self.master = self.connect_init()
            #self.bug_oracle.reset_ora()
            print(i_path.__str__(), path.__str__(), entropy)

            param_map = []
            cmd_map = []
            other_map = []
            all_map = []
            for node in path:
                if node[0] == 'mavcmd':
                    cmd_map.append(node[1])
                    all_map.append(node)
                elif node[0] == 'paramset':
                    param_map.append(node)
                else:
                    other_map.append(node)
                    all_map.append(node)
            # Read command=path[0], and execute it in the RV-simulator. Date: 2024/8/19
            len_cmd = len(cmd_map)
            len_param = len(param_map)
            if len_cmd < cmd_exec_n:
                cmd_exec_n = len(cmd_map)
            elif len_cmd//10 > cmd_exec_n:
                cmd_exec_n = len_cmd//5
            if len_param < param_exec_n:
                param_exec_n = len(param_map)
            elif len_param//10 > param_exec_n:
                param_exec_n = len_param//5

            temp_symbol = 0
            self.path_count += 1

            self.running.set()
            #[px4-new]
            self.rvstatus_thread = threading.Thread(target=self.check_oracle)
            self.rvstatus_thread.start()
            self.msgprocess_thread = threading.Thread(target=self.process_check)
            self.msgprocess_thread.start()

            self.oracle_thread = threading.Thread(target=self.run_oracle)
            self.oracle_thread.start()

            for i in range(run_time):
                print(f'----------------- path run round-{i} -----------------')

                type = random.randint(1, 4)


                if type == 1 and cmd_map:
                    if cmd_exec_n < self.cmd_exec_time:
                        selected_cmd = cmd_map
                    else:
                        selected_cmd = random.sample(cmd_map, cmd_exec_n)

                    for cmdname in selected_cmd:
                        # if cmdname in self.bug_inputs:
                        #     continue
                        self.process_cmd_node(cmdname)
                        time.sleep(0.5)  # : need sleep?
                elif type == 3 and other_map:
                    for other_node in other_map:
                        # if other_node[1] in self.bug_inputs:
                        #     continue
                        # rc
                        self.process_node(other_node)
                        time.sleep(1)
                elif param_map:
                    para_nodes = param_map

                    if param_exec_n < self.param_exec_time:
                        selected_nodes = para_nodes
                    else:
                        selected_nodes = random.sample(para_nodes, param_exec_n)

                    for para_node in selected_nodes:
                        # if para_node[1] in self.bug_inputs:
                        #     continue
                        self.process_node(para_node)
                    time.sleep(1)

                else:
                    # Handle the case when param_map(empty) is selected
                    for node in all_map:
                        # if node[1] in self.bug_inputs:
                        #     continue
                        self.process_node(node)

                # bug oracle
                if i == run_time -1:
                    print("Execution complete, wait 5 seconds to confirm that no bugs have been found")
                    time.sleep(5)

                time_stamp = time.time() - self.begin_time
                if self.bug_oracle.hit_ground:  # hit ground
                #elif self.bug_oracle.status_bug: # system_status >= 5
                    self.total_round += i+1
                    temp_symbol = 2 #FIX: 1
                    print(f'Drone is in the hit ground')
                    self.bug_count += 1
                    self.bug_curve.append({
                        "time": time.time() - begin,
                        "count": len(self.bug_paths),
                        "type": "hitground"
                    })
                    self.bug_paths.append(path)
                    # self.instab_error.append(path)
                    self.instab_error.append(self.bug_count - 1)
                    self.save_bug(i_path, path, 'HitGround', time_stamp)
                    self.bug_oracle.hit_ground = False
                    # self.paths.remove((path, entropy))
                    self.print_status()  # for debug
                    break

                elif self.bug_oracle.timeout_bug:  # hit ground & system_status >= 5
                    self.total_round += i+1
                    temp_symbol = 1
                    print(f'Drone no link, maybe an arithmetic overflow occurs')
                    self.bug_count += 1
                    self.bug_curve.append({
                        "time": time.time() - begin,
                        "count": len(self.bug_paths),
                        "type": "nolink"
                    })
                    self.bug_paths.append(path)
                    self.nolink_error.append(self.bug_count - 1)
                    self.save_bug(i_path, path, 'ArithmException', time_stamp)
                    self.bug_oracle.timeout_bug = False
                    # self.paths.remove((path, entropy))
                    self.print_status()  # for debug
                    break
                elif self.bug_oracle.wp_deviation_bug:
                    self.total_round += i+1
                    temp_symbol = 2
                    print(f'Deviation from waypoint detected')
                    self.bug_count += 1
                    self.bug_curve.append({
                        "time": time.time() - begin,
                        "count": len(self.bug_paths),
                        "type": "wpdevi"
                    })
                    self.bug_paths.append(path)
                    self.wpdevi_error.append(self.bug_count - 1)
                    self.save_bug(i_path, path, 'WpDeviation', time_stamp)
                    self.bug_oracle.wp_deviation_bug = False
                    # self.paths.remove((path, entropy))
                    self.print_status()  # for debug
                    break

            if temp_symbol == 0:
                self.total_round += run_time
                self.adj_entropy_notfound(select_index)
            else:
                self.adj_entropy(select_index)
                #self.rmv_path(select_index) #Ablation : no_entropy
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")


            if temp_symbol == 1:
                logging.info("========= We found an {Arithmetic Exception} bug ==========")
                #self.minm_inputs() # put here?or post-processing? post-processing
            elif temp_symbol == 2:
                logging.info("========= We found a {WP deviation/Hit ground} bug! skip... ==========")
                #self.minm_devi_inputs() #used for WP deviation bug.
                #Wait until the fuzzing time is exhausted, and then concentrate on processing

            self.temp_value.clear()
            self.close_and_relunch()

            cur_time = time.time() - begin
            round += 1
            print(f"#### {round} rounds have been run #### ")
            if len(self.paths) == 0:
                logging.info('======== All paths are capable of triggering at least one bug ========')
                break


    def find_bug(self):
        #while self.oracle_lock:
        if self.bug_oracle.hit_ground or self.bug_oracle.status_bug or self.bug_oracle.timeout_bug or self.bug_oracle.internal_error:
            #or self.bug_oracle.wp_deviation_bug:
            self.bug_oracle.reset_ora()
            return True
        return False

    def find_devibug(self):
        if self.bug_oracle.wp_deviation_bug:
            self.bug_oracle.wp_deviation_bug = False
            return True
        return False

    def close_and_relunch(self):
        logging.info("================ Restarting SITL ================")
        #self.bug_oracle.reset_all()
        self.running.clear()
        if self.oracle_thread and self.oracle_thread.is_alive():
            self.oracle_thread.join()
        if self.rvstatus_thread and self.rvstatus_thread.is_alive():
            self.rvstatus_thread.join()
        if self.msgprocess_thread and self.msgprocess_thread.is_alive():
            self.msgprocess_thread.join()

        try:
            #terminate_px4
            with open(self.pid_file, "r") as f:
                shell_pid = int(f.read().strip())
            os.killpg(os.getpgid(shell_pid), signal.SIGTERM)
            logging.info(f"[-] Terminated shell and children with PGID {os.getpgid(shell_pid)}")
        except Exception as e:
            print(f"Failed to terminate PX4 sitl processes: {e}")

        self.bug_oracle.reset_all()
        self.bug_oracle.reset_vars()
        time.sleep(1) #maybe 0
        #type = self.rvtype  # default

        PX4_HOME = os.getenv("PX4_HOME")
        #PX4_HOME = os.path.expanduser('~/code/PX4/')  # : for test
        if PX4_HOME is None:
            raise Exception("PX4_HOME environment variable is not set!")
        if os.path.exists(self.pid_file):
            os.remove(self.pid_file)

        c = 'rm -rf build/px4_sitl_default/rootfs/log/ && rm -f build/px4_sitl_default/rootfs/parameters*.bson && make px4_sitl_default jmavsim'
        sim = subprocess.Popen([
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            f"echo $$ > {self.pid_file}; cd {PX4_HOME} && {c}; exec bash"
        ])
        logging.info("[+] PX4 Simulator-{jmavsim} started ")

        logging.info("Wait for 20 seconds to ensure that the Drone(PX4) initialization is complete")
        time.sleep(20)

        self.master = self.connect_init()
        self.oracle_master = self.Oconn_init()

        rvmethod.arm_takeoff_px4(self.master) # Unlike AP, PX4 needs to load mission and take off right here

        logging.info("Wait for 8 seconds to confirm that the UAV is in the ascending state")
        time.sleep(8)
        #rvmethod.loadmission(self.master)



    def print_status(self):
        run_time = time.time() - self.begin_time
        logging.info("==================Status Report==================")
        logging.info(
            f'{datetime.now().strftime("%Y/%m/%d %H:%M:%S")}, fuzzing time: {run_time}s, find {self.bug_count} bugs, '
            f'[average_round: {self.total_round/self.bug_count} can found one bug], '
            f'of which {len(self.nolink_error)} are no_link bugs, '
            f'{len(self.wpdevi_error)} are wp_deviation bugs, '
            f'and {len(self.instab_error)} are hit_ground bugs. \n'
            #f'bug_curve: {self.bug_curve}'
        )
        logging.info("================Status Report End================")

    def print_final_status(self):
        logging.info("==================Fuzz Final Report==================")
        logging.info(
            f'find {self.bug_count} bugs, '
            f'average_round: {self.total_round/self.bug_count} can found one bug, '
            f'of which {len(self.nolink_error)} are no_link bugs, '
            f'{len(self.wpdevi_error)} are wp_deviation bugs, '
            f'and {len(self.instab_error)} are hit_ground bugs. \n'

        )

    def parse_path_new(self, path):
        cmdpath = []
        #hascmd = False
        for node in path:
            if node[0] == 'mavcmd':
                cmdpath.append(node[1])
                #hascmd = True
            else:
                self.process_node(node)
        #if hascmd:
        if cmdpath:
            for cmdname in cmdpath:
                self.process_cmd_node(cmdname)
                time.sleep(3)


    def save_bug(self, i_path, path, bugtype, time_stamp):
        if not self.temp_value:
            return
        if not bugtype:
            bugtype=''
        try:
            log_dir = self.bug_out_path
            os.makedirs(log_dir, exist_ok=True)
            logfile = os.path.join(log_dir, f"bug{self.bug_count}_{bugtype}_{time_stamp}.txt")
            with open(logfile, "w") as file:
                #file.write(f"===========[New bug path, type: {bugtype}]===========\n")
                file.write(f"init_path: {str(i_path)}\n")
                file.write(f"RVpath: {str(path)}\n")
                #file.write("=================================\n")
                for itype, iname, ivalues in self.temp_value:
                    file.write(f"{itype} {iname} {ivalues}\n")
            print(f"Saved {len(self.temp_value)} entries to {logfile}.")
        except Exception as e:
            print(f"Error writing to log file: {e}")
        # finally:
        #     self.temp_value.clear()

    def process_node(self, node):
        master = self.master
        if node[0] == 'paramset':
            # done:1. node[1] should be a keyword{alt, speed, acc, etc.}, and a [param_name] should be randomly selected using this keyword
            pvalue = self.param_set.get_parameter_value(node[1])
            print(f'param set {node[1]} {pvalue}')# Using for post-processing

            rvmethod.paramset_px4(master, node[1].encode('utf-8'), pvalue)
            self.temp_value.append(('paramset',node[1],pvalue))

        elif node[0] == 'mavcmd':
            cmdname = node[1]
            params = self.mavcmd_param.get_mav_param_set(cmdname)
            index = self.mavcmd_param.get_index(cmdname)
            print(f'{cmdname} {params}')
            rvmethod.send_mav_cmd(master, index, params)
            self.temp_value.append(('mavcmd',cmdname, params))

        elif node[0] == 'rc':
            pwm = random.randint(1000,2000)
            rc_channel = node[1]
            print(f'execute rc {rc_channel} {pwm}')
            rvmethod.set_rc_channel_pwm(rc_channel,pwm)
            self.temp_value.append(('rc', rc_channel, pwm))

        elif node[0] == 'modeset':
            print(f'process mode set: mode {node[1]}')
            rvmethod.change_mode_px4(master, node[1])
            #self.temp_value.append(('modeset','mode',node[1])) #keep same format
            self.temp_value.append(('modeset',node[1],'mode'))

        elif node[0] == 'takeoff':
            print('RV init: [arm throttle] and [takeoff]')
            rvmethod.arm_takeoff(master, 30)

        elif node[0] == 'loadmission':
            print('load a mission')
            rvmethod.loadmission(master, 1)


    def process_cmd_node(self, cmdname):
        master = self.master
        params = self.mavcmd_param.get_mav_param_set(cmdname)
        index = self.mavcmd_param.get_index(cmdname)  # Use 'int'value instead of 'str' for mav_cmdsend / or utf-8
        print(f'{cmdname} {params}')
        rvmethod.send_mav_cmd(master, index, params)
        self.temp_value.append(('mavcmd',cmdname,params))


    def send_cmd(self, mavmsg, mavfunc):
        '''
        Replaced by rvmethod.send_cmd()
        mavmsg:  xx_xx_send
        mavfunc: MAV_CMD_XXX , etc.
        '''
        master = self.master

        lenth = len(self.cmd_id)
        index = random.randint(1, lenth)

        params = []
        for i in range(7):
            params.append(random.randint(0, 100))

        method = getattr(master.mav, mavmsg)
        # then, map mavfunc to get the params[n]
        #params = match_mavfunc(mavfunc)

        # process RC1-4 , flight mode?
        master.mav.command_long_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            int(self.cmd_id[index]), #cmd_id or cmd_name
            0,
            params[0], params[1], params[2], params[3], params[4], params[5], params[6])

    def random_send_cmd(self):
        ''' Used for random fuzz... '''
        master = self.master
        cmdlenth = len(self.cmd_id)
        mavlength = len(self.mav_list)
        cmdindex = random.randint(1, cmdlenth)
        mavindex = random.randint(1, mavlength)
        params = []
        for i in range(7):
            params.append(random.randint(0, 100))

        mav_func_name = self.mav_list[mavindex]
        mav_method = getattr(master.mav, mav_func_name)

        logging.info(f'Randomly executing a mavlink command: {mav_func_name}({cmdindex})')

        mav_method(master.target_system,  # target_system
            master.target_component,  # target_component
            int(self.cmd_id[cmdindex]), #cmd_id or cmd_list
            0,
            params[0], params[1], params[2], params[3], params[4], params[5], params[6])

    def random_send_param(self):
        # random set a parameter
        master = self.master
        # len(params)
        index = random.randint(1,100) # reset range
        paramname = self.param_list[index]
        paramvalue = random.randint(1,1000)

        master.mav.param_set_send(master.target_system, master.target_component,
                                  paramname,
                                  paramvalue,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


    def write_result(self):
        with open('bugs.json', 'w') as data:
            json.dump({"result": self.bug_curve}, data)
