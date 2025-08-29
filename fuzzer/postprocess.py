import time
import sys
import random
import json
import logging
import os
import csv
import bisect
import threading
from subprocess import Popen, PIPE
from datetime import datetime
from pymavlink import mavutil, mavwp
from fuzzer.oracle import RVoracle
from fuzzer import rvmethod
from fuzzer.runtimedict import MavcmdDictionary

class PostProcess:
    def __init__(self, idx, testcases, minm_file, rvtype):
        self.bug_idx = idx
        self.case_set = testcases
        self.rvtype = rvtype
        self.bug_inputs = set()
        self.bug_oracle = RVoracle()
        self.running = threading.Event()
        self.Arithm_thread = None
        #self.master = self.connect_init()
        #self.arithm_master = self.Oconn_init()
        self.master = None
        self.arithm_master = None
        self.temp_value = []
        self.minm_result_file = minm_file
        self.mavcmd_param = MavcmdDictionary()
        self._logged_min_sets = set()
        self._io_lock = threading.Lock()

    def reboot_minm(self):
        self.close_and_relunch()
        #master = self.connect_init()
        rvmethod.loadmission(self.master)
        self.running.set()
        self.Arithm_thread = threading.Thread(target=self.check_arithm)
        self.Arithm_thread.start()


    def check_arithm(self):
        try:
            self.bug_oracle.post_Arithm(self.arithm_master,self)
        except Exception as e:
            logging.error(f"rvstatus_thread crashed with error: {e}")

    def connect_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        master.wait_heartbeat()
        return master

    def Oconn_init(self):
        master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        master.wait_heartbeat()
        return master

    def find_bug(self):
        #if self.bug_oracle.hit_ground or self.bug_oracle.status_bug or self.bug_oracle.timeout_bug or self.bug_oracle.internal_error:
        if self.bug_oracle.timeout_bug:
            self.bug_oracle.reset_ora()
            return True
        return False

    def close_and_relunch(self):
        logging.info("================ Restarting SITL ================")
        #self.bug_oracle.reset_all()
        self.running.clear()
        if self.Arithm_thread and self.Arithm_thread.is_alive():
            self.Arithm_thread.join()
        self.bug_oracle.timeout_bug = False
        try:
            # os.system("pkill -f 'sim_vehicle.py'")
            os.system("pkill -SIGINT -f 'sim_vehicle.py'")
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
        #ARDUPILOT_HOME = '~/code/t2-ArduPilot/'

        c = 'gnome-terminal -- ' + ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + type + ' --console --map -w --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551'
        sim = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
        logging.info("Wait for 60 seconds to ensure that the Drone(Ardupilot) initialization is complete")
        time.sleep(60)
        #Does the master need to be reinitialized? //not easily reboot, the param file should reload -- yes
        self.master = self.connect_init()
        self.arithm_master = self.Oconn_init()
        rvmethod.change_mode(self.master, 'GUIDED')
        time.sleep(1)
        #if type != 'Rover':
        rvmethod.arm_takeoff(self.master, 30)
        logging.info("Wait for 10 seconds to confirm that the UAV is in the ascending state")
        time.sleep(10)

        #rvmethod.loadmission(self.master)


    def reuse_node(self, node, needsleep=True, sleeptime=2): #
        def send_mav_cmd(master, mavfunc, params):
            cmd = self.mavcmd_param.get_index(mavfunc)
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
                cmd,  # mavfunc, #.encode('utf-8'),
                0, p0, p1, p2, p3, p4, p5, p6)

        master = self.master
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

    def stop_threads(self):
        self.running.clear()
        if self.Arithm_thread and self.Arithm_thread.is_alive():
            self.Arithm_thread.join()

    def _freeze(self, obj):
        if isinstance(obj, dict):
            return tuple(sorted((k, self._freeze(v)) for k, v in obj.items()))
        if isinstance(obj, (list, tuple)):
            return tuple(self._freeze(x) for x in obj)
        if isinstance(obj, set):
            return tuple(sorted(self._freeze(x) for x in obj))
        return obj

    def minm_inputs(self, quick_lable=True, nnindex=1):

        def save_result_min(file_path1, result_min_indices, test_set):
            indices_sorted = tuple(sorted(result_min_indices))
            with self._io_lock:
                if indices_sorted in self._logged_min_sets:
                    logging.info(f"(dedup) Skip writing duplicated minimal set {indices_sorted}")
                    return
                self._logged_min_sets.add(indices_sorted)

                # nodes_tuple = tuple(test_set[i] for i in indices_sorted)
                nodes_tuple = tuple(self._freeze(test_set[i]) for i in indices_sorted)
                self.bug_inputs.add(nodes_tuple)


                logging.info(f'{[test_set[i] for i in indices_sorted]} is the smallest set of inputs that can trigger a bug')
                with open(file_path1, 'a') as file:
                    file.write('================= \n')
                    for i in indices_sorted:
                        node = test_set[i]
                        file.write(' '.join(map(str, node)) + '\n')

        try:
            test_set = self.case_set
            self.reboot_minm()
            result_file = self.minm_result_file
            result_min = []

            if not test_set:
                print("Test set is empty.")
                return

            result_min_indices = set()

            index = 0
            index_max = -1
            index_min = 0

            first_index = 0
            if quick_lable and len(test_set) > 100:
                first_index = len(test_set) - 100
            if not quick_lable:
                first_index = len(test_set) //nnindex
                if first_index < 50:
                    first_index = 0

            while index < first_index:
                node = test_set[index]
                #self.reuse_node(node, False)
                self.reuse_node(node,needsleep=True,sleeptime=0.05)
                index += 1

            if quick_lable or first_index>0:
                time.sleep(2)
                #self.bug_oracle.rv_alive(self.master) # means bug within the range[0,first_index]
                if self.find_bug():
                    nval = nnindex*2
                    self.bug_oracle.timeout_bug = False
                    self.minm_inputs(quick_lable=False, nnindex=nval)
                    return

            while index < len(test_set):
                node = test_set[index]
                self.reuse_node(node)
                #self.bug_oracle.rv_alive(self.master)
                if self.find_bug():
                    #result_min.append(node)
                    result_min_indices.add(index)
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
                for i in sorted(result_min_indices):
                    self.reuse_node(test_set[i])
                if self.find_bug():
                    save_result_min(result_file, result_min_indices, test_set)
                    return

                while current_index > index_min and fright:
                    current_index -= 1
                    self.reuse_node(test_set[current_index])
                    if self.find_bug():
                        result_min_indices.add(current_index)
                        index_min = current_index
                        self.reboot_minm()
                        fright = False
                        fleft = True
                        break

                for i in sorted(result_min_indices):
                    self.reuse_node(test_set[i])
                if self.find_bug():
                    save_result_min(result_file, result_min_indices, test_set)
                    return

                while current_index < index_max and fleft:
                    current_index += 1
                    self.reuse_node(test_set[current_index])
                    if self.find_bug():
                        result_min_indices.add(current_index)
                        index_max = current_index
                        self.reboot_minm()
                        fright = True
                        fleft = False
                        break

            #if not found exact minm_inputs:
            for i in range(index_min, index_max):
                if i not in result_min_indices:
                    result_min_indices.add(i)

            save_result_min(result_file, result_min, test_set)
        finally:
            logging.info(f"Cleaning up threads for bug{self.bug_idx}")
            self.stop_threads()
