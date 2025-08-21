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
from datetime import datetime
from pymavlink import mavutil, mavwp
from fuzzer.oracle import RVoracle
from fuzzer import rvmethod
from model.parsefile import Parsefile

from fuzzer.runtimedict import RuntimeDictionary, MavcmdDictionary


#init: define RV start
class ADGfuzzer:
    def __init__(self, paths, rvtype, bug_out_path="outfile/test/", pars: Parsefile = Parsefile(), time_budget = 300):
        #paths: Paths = Paths() replaced by Mapp()
        # input space related
        self.cmd_list = []
        self.cmd_id = []

        self.param_list = []
        self.param_set = RuntimeDictionary()
        self.mavcmd_param = MavcmdDictionary()
        self.env_set = []
        self.paths = paths  # format like that: { ('paramset', node1), ('mavcmd', num[7])}

        self.mav_list = pars.mavfunc_list

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
        self.pos_error = [] #set()
        self.hit_ground = []
        self.instab_error = []
        self.nolink_error = []
        self.wpdevi_error = []
        self.inter_error = []
        #self.direc_error = set()
        #self.mission_error = set()

        self.bug_inputs = set()
        #RV
        self.master = self.connect_init()
        self.oracle_master = self.Oconn_init()
        self.rvtype = rvtype
        #self.total_entropy = 0
        self.total_round = 0

    def connect_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        master.wait_heartbeat()
        #logging.info("received heartbeat, get Plane gps location")
        return master

    def Oconn_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14551') # must use different port
        master.wait_heartbeat()
        #logging.info("received heartbeat, get Plane gps location")
        return master

    def run_oracle(self):
        #self.bug_oracle = RVoracle()
        try:
            self.bug_oracle.all_oracles(self.oracle_master,self)
        except Exception as e:
            logging.error(f"Oracle_thread crashed with error: {e}")

    def check_oracle(self):
        try:
            self.bug_oracle.check_status(self.oracle_master,self)
        except Exception as e:
            logging.error(f"rvstatus_thread crashed with error: {e}")
    def process_check(self):
        try:
            self.bug_oracle.process_messages(self)
        except Exception as e:
            logging.error(f"process_messages crashed with error: {e}")

    def load_found_buginputs(self):
        try:
            with open('outfile/bug_input.txt', 'r') as file:
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

        # cumulative_weights = []
        # total_entropy = 0
        # for _, entropy, i_path in self.paths:
        #     total_entropy += entropy
        #     cumulative_weights.append(total_entropy)

        logging.info("===================== Start Fuzzing =====================")

        self.load_found_buginputs()

        rvmethod.change_mode(self.master, 'GUIDED')
        time.sleep(1)
        rvmethod.arm_takeoff(self.master, 30)
        logging.info("Wait for 8 seconds to confirm that the UAV is in the ascending state")
        time.sleep(5)
        # while time<budget time: / while True:
        while cur_time < self.time_budget:
            logging.info(f'--------------- Test the {round}-th path --------------- ')
            #path, entropy, i_path = self.weighted_random_choice(cumulative_weights, total_entropy)
            # index = self.weighted_random_choice(cumulative_weights, total_entropy)

            select_index = self.select_from_paths() # after Ablation, recover
            #select_index = random.randint(0, len(self.paths) - 1) # [Ablation]: no_entropy
            path, entropy, i_path = self.paths[select_index]

            # receiving the result of static_analysis -> <path1, path2, ...>
            param_exec_n = 10
            cmd_exec_n = 5
            if not path:
                continue
            run_time = 50
            # if entropy < 1:
            #     continue
            if entropy > 500:
                run_time = 500
            elif entropy > 50:
                run_time = int(entropy)

            print(i_path.__str__(), path.__str__(), entropy)
            # (done) 11.10: when test each MIS, rv should reboot

            rvmethod.loadmission(self.master)
            time.sleep(3)

            param_map = []
            cmd_map = []
            other_map = []
            all_map = []
            for node in path:
                if node[0] == 'mavcmd':
                    cmd_map.append(node[1])
                    all_map.append(node)
                elif node[0] == 'paramset':
                    # index = node[2] - 1 #does this make sense?
                    # while len(param_map) <= index:
                    #     param_map.append([])
                    # param_map[index].append(node)
                    param_map.append(node)
                else:
                    other_map.append(node)
                    all_map.append(node)
            # Read command=path[0], and execute it in the RV-simulator.
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

            self.rvstatus_thread = threading.Thread(target=self.check_oracle)
            self.rvstatus_thread.start()
            self.msgprocess_thread = threading.Thread(target=self.process_check)
            self.msgprocess_thread.start()

            self.oracle_thread = threading.Thread(target=self.run_oracle)
            self.oracle_thread.start()
            # add a loop. add a time threshold t, each path execution time t : while not timeout
            for i in range(run_time):  # (done) It should be replaced by an energy scheduling algorithm
                print(f'----------------- path run round-{i} -----------------')
                # print(f'Now execute path is: {self.temp_value}') #for debug

                # self.random_parse_path(path)
                # self.execute_path1(path)  # main process , determine: send cmd/param/env/other.
                type = random.randint(1, 4)
                # maxindex = len(param_map)

                if type == 1 and cmd_map:
                    if cmd_exec_n < self.cmd_exec_time:
                        selected_cmd = cmd_map
                    else:
                        selected_cmd = random.sample(cmd_map, cmd_exec_n)

                    for cmdname in selected_cmd:
                        if cmdname in self.bug_inputs:
                            continue
                        self.process_cmd_node(cmdname)
                        time.sleep(1)
                elif type == 3 and other_map:
                    for other_node in other_map:
                        if other_node[1] in self.bug_inputs:
                            continue
                        # rc
                        self.process_node(other_node)
                        time.sleep(1)
                elif param_map:  # maxindex != 0:
                    # random_index = random.randint(0, maxindex - 1)
                    # para_nodes = param_map[random_index]
                    para_nodes = param_map

                    # if len(para_nodes) < self.param_exec_time:
                    if param_exec_n < self.param_exec_time:
                        selected_nodes = para_nodes
                    else:
                        selected_nodes = random.sample(para_nodes, param_exec_n)

                    for para_node in selected_nodes:
                        if para_node[1] in self.bug_inputs:
                            continue
                        self.process_node(para_node)
                    time.sleep(1)
                    # for para_node in param_map[random_index]:
                    #     self.process_node(para_node)
                else:
                    # Handle the case when param_map(empty) is selected
                    for node in all_map:
                        if node[1] in self.bug_inputs:
                            continue
                        self.process_node(node)

                # bug oracle
                # can add other oracle
                if i == run_time -1:
                    print("Execution complete, wait 5 seconds to confirm that no bugs have been found")
                    time.sleep(5)

                time_stamp = time.time() - self.begin_time

                if self.bug_oracle.status_bug or self.bug_oracle.hit_ground:  # hit ground & system_status >= 5
                #elif self.bug_oracle.hit_ground:
                    self.total_round += i+1
                    temp_symbol = 3
                    print(f'Drone is in the non-normal/failsafe status, or fall to the ground ')
                    self.bug_count += 1  # put to a new function? def record_bug(self, path)?
                    self.bug_curve.append({
                        "time": time.time() - begin,
                        "count": len(self.bug_paths),
                        "type": "status"
                    })
                    self.bug_paths.append(path)
                    # self.instab_error.append(path)
                    self.instab_error.append(self.bug_count - 1)
                    self.save_bug(i_path, path, 'StatusError', time_stamp)
                    self.bug_oracle.status_bug = False
                    self.bug_oracle.hit_ground = False
                    # self.paths.remove((path, entropy))
                    self.print_status()
                    break

                elif self.bug_oracle.timeout_bug:  # hit ground & system_status >= 5
                    self.total_round += i+1
                    temp_symbol = 1
                    print(f'Drone no link, maybe an arithmetic overflow occurs')
                    self.bug_count += 1  # put to a new function? def record_bug(self, path)?
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
                    self.print_status()
                    break
                elif self.bug_oracle.wp_deviation_bug:
                    self.total_round += i+1
                    temp_symbol = 2
                    print(f'Deviation from waypoint detected')
                    self.bug_count += 1  # put to a new function? def record_bug(self, path)?
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
                    self.print_status()
                    break
                # elif self.bug_oracle.internal_error: #: delete
                #     self.total_round += i+1
                #     temp_symbol = 1
                #     print(f'Internal Errors occurred')
                #     self.bug_count += 1  # put to a new function? def record_bug(self, path)?
                #     self.bug_curve.append({
                #         "time": time.time() - begin,
                #         "count": len(self.bug_paths),
                #         "type": "InterError"
                #     })
                #     self.bug_paths.append(path)
                #
                #     self.inter_error.append(self.bug_count - 1)
                #     self.save_bug(i_path, path, 'InterError', time_stamp)
                #     self.bug_oracle.internal_error = False
                #     # self.paths.remove((path, entropy))
                #     self.print_status()
                #     break

            if temp_symbol == 0:
                self.total_round += run_time
                self.adj_entropy_notfound(select_index) # after Ablation, recover
            else:
                self.adj_entropy(select_index) # after Ablation, recover
                #self.rmv_path(select_index) # [Ablation] : no_entropy
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")
                print("%%%%%%%%%%%% bug occurred %%%%%%%%%%%%%%")


            # 4.4 path-post-processing
            # (done) Record/Save input value(log) #use self.temp_value
            if temp_symbol == 1:
                logging.info("========= We found an Arithmetic overflow bug ==========")

            elif temp_symbol == 2:
                logging.info("========= We found a Route deviation bug!  ==========")

            elif temp_symbol ==3:
                logging.info("========= We found a Hit_ground/Status_failsafe bug!  ==========")

            self.temp_value.clear()
            self.close_and_relunch()

            #self.print_status()
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

            os.system("pkill -SIGINT -f 'sim_vehicle.py'")
            # os.system("fuser -k 5760/tcp")
            # os.system("lsof -t -i :5760 | xargs -r kill -9")
            logging.info("Terminated the sim_vehicle.py processes")
        except Exception as e:
            print(f"Failed to terminate sim_vehicle.py processes: {e}")
        self.bug_oracle.reset_all()
        time.sleep(1) #maybe 0
        type = self.rvtype  # default
        if type == 'copter':
            type = 'ArduCopter'
        elif type == 'plane':
            type = 'ArduPlane'
        elif type == 'rover':
            type = 'Rover'

        ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")
        if ARDUPILOT_HOME is None:
            ARDUPILOT_HOME = '~/code/t2-ArduPilot/'

        c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        #c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w'
        sim = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
        logging.info("Wait for 60 seconds to ensure that the Drone(Ardupilot) initialization is complete")
        time.sleep(60)

        self.master = self.connect_init()
        self.oracle_master = self.Oconn_init()
        rvmethod.change_mode(self.master, 'GUIDED')
        time.sleep(1)

        rvmethod.arm_takeoff(self.master, 30)
        logging.info("Wait for 5 seconds to confirm that the UAV is in the ascending state")
        time.sleep(5)
        #rvmethod.loadmission(self.master)



    def reboot_minm(self):
        self.close_and_relunch()
        #master = self.connect_init()
        rvmethod.loadmission(self.master)
        self.running.set()
        self.rvstatus_thread = threading.Thread(target=self.check_oracle)
        self.rvstatus_thread.start()
        self.oracle_thread = threading.Thread(target=self.run_oracle)
        self.oracle_thread.start()
        self.msgprocess_thread = threading.Thread(target=self.process_check)
        self.msgprocess_thread.start()

    def print_status(self):
        run_time = time.time() - self.begin_time
        logging.info("==================Status Report==================")
        logging.info(
            f'{datetime.now().strftime("%Y/%m/%d %H:%M:%S")}, fuzzing time: {run_time}s, find {self.bug_count} bugs, '
            f'[average_round: {self.total_round/self.bug_count} can found one bug], '
            #f'of which {len(self.pos_error)} are postion bugs, '
            f'of which {len(self.nolink_error)} are no_link (software crash) bugs, '
            f'{len(self.wpdevi_error)} are Wp deviation (route deviation) bugs, '
            f'{len(self.instab_error)} are status (hit ground) bugs.\n'
            #f'and {len(self.inter_error)} are Internal-Errors bugs.\n'
            #f'bug_curve: {self.bug_curve}'
        )
        logging.info("================Status Report End================")

    def print_final_status(self):
        logging.info("==================Fuzz Final Report==================")
        logging.info(
            f'find {self.bug_count} bugs, '
            f'average_round: {self.total_round/self.bug_count} can found one bug, '
            #f'of which {len(self.pos_error)} are postion bugs, '
            f'of which {len(self.nolink_error)} are no_link (software crash) bugs, '
            f'{len(self.wpdevi_error)} are wp deviation bugs, '
            f'{len(self.instab_error)} are status/instability bugs.\n'
            #f'and {len(self.inter_error)} are Internal-Errors bugs.\n'
        )


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

            rvmethod.paramset(master, node[1].encode('utf-8'), pvalue)
            # master.(mav.)param_set_send() ????  set_mode_send, param_request_read_send, ...
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
            rvmethod.change_mode(master, node[1])
            #self.temp_value.append(('modeset','mode',node[1])) #keep same format
            self.temp_value.append(('modeset',node[1],'mode'))

        elif node[0] == 'takeoff':
            print('RV init: [arm throttle] and [takeoff]')
            rvmethod.arm_takeoff(master, 30)

        elif node[0] == 'loadmission':
            print('load a mission')
            # future work: chose one mission from the mission_dict
            rvmethod.loadmission(master, 1)

    def reuse_node(self, node, needsleep=True, sleeptime=2): #
        master = self.master
        if node[0] == 'paramset':
            # done:1. node[1] should be a keyword{alt, speed, acc, etc.}, and a [param_name] should be randomly selected using this keyword
            pvalue = node[2]
            print(f'param set {node[1]} {pvalue}')# Using for post-processing
            rvmethod.paramset(master, node[1].encode('utf-8'), pvalue)

        elif node[0] == 'mavcmd':
            cmdname = node[1]
            params = node[2]
            index = self.mavcmd_param.get_index(cmdname)
            print(f'{cmdname} {params}')
            rvmethod.send_mav_cmd(master, index, params)

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
            #maybe it needs to be 3

    def minm_inputs(self, quick_lable=True, nnindex=1):
        def save_result_min(file_path1, result_min1):
            if len(result_min1) == 1:
                self.bug_inputs.add(result_min1[0][1])
            logging.info(f'{result_min1} is the smallest set of inputs that can trigger a bug')
            with open(file_path1, 'a') as file:
                file.write('================= \n')
                for node in result_min1:
                    file.write(' '.join(map(str, node)) + '\n')
        test_set = self.temp_value
        self.reboot_minm()
        result_file = self.minm_result_file
        result_min = []

        if not test_set:
            print("Test set is empty.")
            return

        index = 0
        index_max = -1
        index_min = 0

        first_index = 0
        if quick_lable and len(test_set) > 30:
            first_index = len(test_set) - 30
        if not quick_lable:
            first_index = len(test_set) //nnindex
            if first_index < 50:
                first_index = 0

        while index < first_index:
            node = test_set[index]
            self.reuse_node(node, False)
            index += 1

        if quick_lable or first_index>0:
            time.sleep(2)
            #self.bug_oracle.rv_alive(self.master) # means bug within the range[0,first_index]
            if self.find_bug():
                nval = nnindex*2
                self.minm_inputs(quick_lable=False, nnindex=nval)
                return

        while index < len(test_set):
            node = test_set[index]
            self.reuse_node(node)
            #self.bug_oracle.rv_alive(self.master)
            if self.find_bug():
                result_min.append(node)
                index_max = index
                self.reboot_minm()
                break
            index += 1

        if index_max == -1:
            print("No valid node found.")
            return

        current_index = index_max
        fright = True
        fleft = False

        while index_max - index_min > 1:
            for node in result_min:
                self.reuse_node(node)
            #self.bug_oracle.rv_alive(self.master)
            if self.find_bug():
                save_result_min(result_file, result_min)
                return

            while current_index > index_min and fright:
                current_index -= 1
                node = test_set[current_index]
                self.reuse_node(node)
                #self.bug_oracle.rv_alive(self.master)
                if self.find_bug():
                    result_min.append(node)
                    index_min = current_index
                    self.reboot_minm()
                    fright = False
                    fleft = True
                    break

            for node in result_min:
                self.reuse_node(node)
            #self.bug_oracle.rv_alive(self.master)
            if self.find_bug():
                save_result_min(result_file, result_min)
                return

            while current_index < index_max and fleft:
                current_index += 1
                node = test_set[current_index]
                self.reuse_node(node)
                #self.bug_oracle.rv_alive(self.master)
                if self.find_bug():
                    result_min.append(node)
                    index_max = current_index
                    self.reboot_minm()
                    fright = True
                    fleft = False
                    break

        #if not found exact minm_inputs:
        for i in range(index_min,index_max):
            result_min.append(test_set[i])
        save_result_min(result_file, result_min)

    def minm_devi_inputs(self):

        def save_result_min(file_path1, result_min1):
            if len(result_min1) == 1:
                self.bug_inputs.add(result_min1[0][1])
            logging.info(f'{result_min1} is the smallest set of inputs that can trigger a wp deviation bug')
            with open(file_path1, 'a') as file:
                file.write('=========WP deviation bug======== \n')
                for node in result_min1:
                    file.write(' '.join(map(str, node)) + '\n')

        def reboot_nothread():
            logging.info("================ Restarting SITL ================")
            try:
                # os.system("pkill -f 'sim_vehicle.py'")
                os.system("pkill -SIGINT -f 'sim_vehicle.py'")
                logging.info("Terminated the sim_vehicle.py processes")
            except Exception as e:
                print(f"Failed to terminate sim_vehicle.py processes: {e}")

            time.sleep(1)  # maybe 0
            type = self.rvtype  # default
            if type == 'copter':
                type = 'ArduCopter'
            elif type == 'plane':
                type = 'ArduPlane'

            ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")
            if ARDUPILOT_HOME is None:
                ARDUPILOT_HOME = '~/code/t2-ArduPilot/'

            c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w'
            sim = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
            logging.info("Wait for 60 seconds to ensure that the Drone(Ardupilot) initialization is complete")
            time.sleep(60)
            self.master = self.connect_init()
            rvmethod.change_mode(self.master, 'GUIDED')
            time.sleep(1)
            rvmethod.arm_takeoff(self.master, 30)
            logging.info("Wait for 10 seconds to confirm that the UAV is in the ascending state")
            time.sleep(5)
            rvmethod.loadmission(self.master)

        test_set = self.temp_value[-50:]

        self.running.clear()
        if self.oracle_thread and self.oracle_thread.is_alive():
            self.oracle_thread.join()
        if self.rvstatus_thread and self.rvstatus_thread.is_alive():
            self.rvstatus_thread.join()
        self.bug_oracle.wp_deviation_bug = False
        reboot_nothread()
        result_file = self.minm_result_file
        result_min = []

        if not test_set:
            print("Test set is empty.")
            return

        index = 0
        #index_max = -1
        index_max = len(test_set) - 1
        index_min = 0

        print("Searching for first bug trigger...")
        while index <= index_max:
            node = test_set[index]
            self.reuse_node(node,sleeptime=1)
            triggered_index = self.bug_oracle.minm_wp_deviation(self.master, index)

            if triggered_index is not None:
                print(f"Bug triggered by input at index {triggered_index}")
                result_min.append(test_set[triggered_index])
                index_max = triggered_index
                self.bug_oracle.wp_deviation_bug = False
                reboot_nothread()
                break
            index += 1

        if not result_min:
            print("No bug-triggering input(s) found.")
            return

        current_index = index_max
        fright = True
        fleft = False

        while index_max - index_min > 1:
            for node in result_min:
                self.reuse_node(node,sleeptime=1)
            #self.bug_oracle.rv_alive(self.master)
            if self.bug_oracle.wp_deviation_bug:
                save_result_min(result_file, result_min)
                return

            while current_index > index_min and fright:
                current_index -= 1
                node = test_set[current_index]
                self.reuse_node(node,sleeptime=1)
                triggered_index = self.bug_oracle.minm_wp_deviation(self.master, current_index)
                if triggered_index is not None:
                    result_min.append(test_set[triggered_index])
                    index_min = current_index
                    self.bug_oracle.wp_deviation_bug = False
                    reboot_nothread()
                    fright = False
                    fleft = True
                    break

            for node in result_min:
                self.reuse_node(node,sleeptime=1)
            if self.bug_oracle.wp_deviation_bug:
                save_result_min(result_file, result_min)
                return

            while current_index < index_max and fleft:
                current_index += 1
                node = test_set[current_index]
                self.reuse_node(node,sleeptime=1)
                triggered_index = self.bug_oracle.minm_wp_deviation(self.master, current_index)
                if triggered_index is not None:
                    result_min.append(test_set[triggered_index])
                    index_max = current_index
                    self.bug_oracle.wp_deviation_bug = False
                    reboot_nothread()
                    fright = True
                    fleft = False
                    break

        #if not found exact minm_inputs:
        for i in range(index_min,index_max):
            result_min.append(test_set[i])
        save_result_min(result_file, result_min)


    def process_cmd_node(self, cmdname):
        master = self.master
        params = self.mavcmd_param.get_mav_param_set(cmdname)
        index = self.mavcmd_param.get_index(cmdname)  # Use 'int'value instead of 'str' for mav_cmdsend / or utf-8
        print(f'{cmdname} {params}')
        rvmethod.send_mav_cmd(master, index, params)
        self.temp_value.append(('mavcmd',cmdname,params))


    def write_result(self): # not used
        with open('bugs.json', 'w') as data:
            json.dump({"result": self.bug_curve}, data)


if __name__ == "__main__":
    #logging.info("start")
    print("start") #  for test