import logging
import time
import queue
import random
from pymavlink import mavutil, mavwp
from fuzzer import rvmethod
from geopy.distance import geodesic

class PositionData:
    def __init__(self, lat=None, lon=None, alt=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def calculate_difference(self, other):
        # reconsider: remove this function to RVoracle?
        if not isinstance(other, PositionData):
            raise ValueError("The argument must be an instance of PositionData")
        lat_diff = abs(self.lat - other.lat) if self.lat is not None and other.lat is not None else 0
        lon_diff = abs(self.lon - other.lon) if self.lon is not None and other.lon is not None else 0
        alt_diff = abs(self.alt - other.alt) if self.alt is not None and other.alt is not None else 0
        diff = lat_diff + lon_diff + alt_diff
        return diff

class RVoracle:
    def __init__(self):
        self.position_threshold = 10 # useless
        self.position_data = PositionData() # target position
        self.pos_datas = []

        self.hit_ground = False
        self.status_bug = False
        self.timeout_bug = False
        # self.pos_error = set()
        # self.hit_ground = set()
        # self.instab_error = set()
        # self.direc_error = set()
        # self.mission_error = set()
        self.pos_bug = False
        self.pos_bias = None
        self.wp_deviation_bug = False
        self.wp_distance = None

        self.deviation_times = []
        self.first_deviation_index = None # used for record the test_set[index]
        self.internal_error = False
        self.message_queue = queue.Queue()
        self.msg_handlers = {
            'HEARTBEAT': self.handle_heartbeat,
            'STATUSTEXT': self.handle_statustext
            #'SYS_STATUS': self.handle_sys_status
        }
        self.msg_handlers_px4 = {
            #'HEARTBEAT': self.handle_heartbeat,
            'EXTENDED_SYS_STATE': self.handle_extendedsys,
            'GLOBAL_POSITION_INT': self.handle_globalpos
            # 'SYS_STATUS': self.handle_sys_status
        }
        self.airborne = False
        self.last_alt = None
        self.last_distance = None
        self.deviate_count = 0
        #self.current_wp = None
        self.waypoints = self.load_waypoints('missiondata/px4_mission1.txt') #load once?

    def reset_ora(self):
        #just for test
        self.hit_ground = False
        self.status_bug = False
        self.pos_bug = False
        self.internal_error = False
        self.timeout_bug = False
        #self.wp_deviation_bug = False

    def reset_all(self):
        self.hit_ground = False
        self.status_bug = False
        self.timeout_bug = False
        self.pos_bug = False
        self.wp_deviation_bug = False
        self.internal_error = False


    def reset_vars(self):
        self.airborne = False
        self.last_alt = None
        self.last_distance = None
        self.deviate_count = 0
        #self.current_wp = None

    def check_position_error(self, master):
        try:
            msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
            #msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=False, timeout=1)
            if msg:
                pos_error = msg.pos_horiz_variance
                #print(f'Position Error: {pos_error}')
                if pos_error > 100:
                    self.pos_bias = pos_error
                if pos_error > 10: # Check if this threshold is appropriate
                    print(f'############## Position error detected: {pos_error} ##############')
                    self.pos_bug = True
            #else:
                #print('No EKF_STATUS_REPORT received.')
        except Exception as e:
            print(f'Error in check_position_error: {e}')

    def check_wp_deviation(self, master, target_wp_distance=99999999999):
        try:
            msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=1)
            #msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=False, timeout=1)
            if msg:
                current_distance = msg.wp_dist
                #print(f'Waypoint Distance: {current_distance}')
                if current_distance > target_wp_distance:
                    print('Potential waypoint deviation detected.')

                    current_time = time.time()
                    self.deviation_times.append(current_time)
                    self.deviation_times = [t for t in self.deviation_times if current_time - t <= 12]
                    #FIXME: copter & rover: t<=5, len >=3 ; plane: t<=12, len>=9
                    if len(self.deviation_times) >= 9:
                        print('############## Waypoint deviation bug triggered! ##############')
                        self.wp_deviation_bug = True

                        # if self.first_deviation_index is not None:
                        #     self.result_min.append(self.test_set[self.first_deviation_index])
                        #     print(f'Added input at index {self.first_deviation_index} to result_min.')

                        self.deviation_times.clear()
                        #self.first_deviation_index = None

                self.wp_distance = current_distance

        except Exception as e:
            print(f'Error in check_wp_deviation: {e}')

    def minm_wp_deviation(self, master, current_input_index):
        target_wp_distance = 99999999999
        try:
            msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=1)
            if msg:
                current_distance = msg.wp_dist
                #print(f'Waypoint Distance: {current_distance}')
                if current_distance > target_wp_distance:
                    print('Potential waypoint deviation detected.')
                    if self.first_deviation_index is None:
                        self.first_deviation_index = current_input_index
                        print(f'First deviation triggered by input at index {self.first_deviation_index}')

                    current_time = time.time()
                    self.deviation_times.append(current_time)
                    self.deviation_times = [t for t in self.deviation_times if current_time - t <= 5]

                    if len(self.deviation_times) >= 2:
                        print('############## Waypoint deviation bug triggered! ##############')
                        self.wp_deviation_bug = True
                        if self.first_deviation_index is not None:
                            return self.first_deviation_index

                        self.deviation_times.clear()
                        self.first_deviation_index = None

                self.wp_distance = current_distance

            #else:
                #print('No NAV_CONTROLLER_OUTPUT received.')

        except Exception as e:
            print(f'Error in check_wp_deviation: {e}')
        return None

    def check_attitude(self, master):
        #seems incorrect /not used
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        print(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')
        if abs(roll) > 1.0 or abs(pitch) > 1.0 or abs(yaw) > 1.0:
            print('============ Unstable attitude detected ============')
            self.attitude_bug = True

    def rv_alive(self, master):
        # Get HEARTBEAT message
        ''' when system_status in [5?,6,7,8] means error occurred
            3:System is grounded and on standby. It can be launched any time.
            /4:System is active and might be already airborne. Motors are engaged.
            ?5:System is in a non-normal flight mode (failsafe). It can however still navigate.
            6.System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down.
            7.System just initialized its power-down sequence, will shut down now.
            8.System is terminating itself (failsafe or commanded).
        '''
        try:
            msg = master.wait_heartbeat(timeout=1) #  blocking=True
            if msg:
                rv_status = msg.system_status
                print(f'############### current status: {rv_status} ###############')
                if rv_status == 3:
                    print('############## drone hits ground ##############')
                    self.hit_ground = True #We should not use this as an oracle for hit_ground bug
                elif rv_status > 5:
                    print(f'############## drone error!! system_status = {rv_status} ##############')
                    self.status_bug = True
        except Exception as e: # no exception.
            if "timeout" in str(e).lower():
                print('############## Connection timeout! No heartbeat in 1s ##############')
                self.timeout_bug = True
            else:
                print(f'############## Unexpected error: {e} ##############')
                self.timeout_bug = True

    def handle_heartbeat(self, msg):
        try:
            state = msg.system_status
            #print(f'system status: {mavutil.mavlink.enums["MAV_STATE"][state].name}')
            # if state == mavutil.mavlink.MAV_STATE_STANDBY:
            #     print('############## state=3, drone hits ground ##############')
                #self.hit_ground = True
            # elif state == mavutil.mavlink.MAV_STATE_CRITICAL:  # CRITICAL (5)
            #     print('############## CRITICAL failure (controlled fall) ##############')
            #     self.critical_bug = True
            if state == mavutil.mavlink.MAV_STATE_EMERGENCY:  # EMERGENCY (6)
                print('############## EMERGENCY (totally out of control) ##############')
                self.status_bug = True
            elif state == mavutil.mavlink.MAV_STATE_POWEROFF:  # POWEROFF (7)
                print('############## POWEROFF ##############')
                self.status_bug = True
        except Exception as e:
            print(f'############## Unknown error: {e} ##############')
            #self.timeout_bug = True

    def handle_statustext(self, msg):
        text = msg.text #.decode('ascii', errors='ignore').strip()
        #print(f'STATUSTEXT: {text}')
        #if any(kw in text for kw in ['Hit ground', 'Crash detected']):
        if 'Hit ground' in text:
            print('##############[STATUSTEXT] drone Hit ground! ##############')
            self.hit_ground = True
        if 'Internal Errors' in text:
            #FIXME: This is a 'ghost' bug (sometimes it gets triggered, sometimes it doesn't), and the developers doesn't know for sure why
            print('##############[STATUSTEXT] Internal Errors!! ##############')
            self.internal_error = True
        if 'no link' in text or 'link 1 down' in text:
            print('##############[STATUSTEXT] No link found !!  ##############')
            self.timeout_bug = True

    def handle_extendedsys(self, msg):
        # 0: MAV_LANDED_STATE_UNDEFINED, 1: MAV_LANDED_STATE_ON_GROUND, 2: MAV_LANDED_STATE_IN_AIR,
        # 3: MAV_LANDED_STATE_TAKEOFF,4: MAV_LANDED_STATE_LANDING
        landed_state = msg.landed_state
        if landed_state == 2:
            self.airborne = True
        elif landed_state == 1:
            if self.airborne:
                print("#####[Hit Ground] landed_state = 1: UAV has landed or possibly crashed!")
                #self.airborne = False # logic error./ This should be a lift/air lock
                self.hit_ground = True

    def handle_globalpos(self, msg):
        alt = msg.relative_alt / 1000.0
        #print(f"[Alt Report] alt is: {alt}")

        # Alt can spike up to a few hundred/thousand depending on the parameter Settings and then slowly decrease, which takes some time to discover;
        # In addition, there may be a situation where the UAV is flying in the air, but the Alt becomes negative
        # These should be a perturbation
        if self.airborne and alt < 1:
            if not self.hit_ground:
                print("#####[Hit Ground] Bug found: alt error, possible crash/landing. #####")
            self.hit_ground = True


    # This function can be used by both ardupilot and px4
    def check_status(self, master, fuzz):
        try:
            retries = 6 #Increase this if there are some overflow-FP's as this is a mavlink implementation issue
            while fuzz.running.is_set():
                try:
                    msg = None
                    for _ in range(retries):
                        msg = master.recv_match(blocking=True, timeout=0.3)
                        if msg is not None:
                            break
                    #msg = master.recv_match(blocking=True, timeout=2) #when post-processing, this should be changed to 1s or 2s
                    if msg is None:
                        # e-timeout would be better, but mavlink doesn't implement it, so we'll settle for this instead
                        print("##### Bug found: no-link/Arithm_Exception!! #####")
                        self.timeout_bug = True
                        continue
                    self.message_queue.put(msg)

                except Exception as e:
                    if "timeout" in str(e).lower():
                        # mavlink is implemented without throwing exceptions. took me an afternoon to debug  :(
                        print("############## timeout!! Arithm_Exception ##############")
                        self.timeout_bug = True
                    #useless
                    print("[rvstatus_thread]: not recvied")
                    self.timeout_bug = True

        except Exception as e:
            logging.error(f"[Oracle(Status) Thread] Oracle monitoring crashed: {e}")

    def process_messages(self, fuzz):
        try:
            while fuzz.running.is_set():
                msg = self.message_queue.get(timeout=5)
                handler = self.msg_handlers.get(msg.get_type())
                if handler:
                    handler(msg)
        except queue.Empty:
            print("[MessageProcessor]: No messages for a while.")
        except Exception as e:
            logging.error(f"[MessageProcessor]: crashed with error: {e}")

    def process_messages_px4(self, fuzz):
        try:
            while fuzz.running.is_set():
                msg = self.message_queue.get(timeout=5)
                handler = self.msg_handlers_px4.get(msg.get_type()) # px4
                if handler:
                    handler(msg)
        except queue.Empty:
            print("[MessageProcessor]: No messages for a while.")
        except Exception as e:
            logging.error(f"[MessageProcessor]: crashed with error: {e}")

    # just for ardupilot
    def all_oracles(self, master, fuzz):
        try:
            while fuzz.running.is_set():
                #self.rv_alive(master)
                #self.check_statustext(master)
                #self.check_position_error(master) #for test
                #self.check_attitude(master)
                self.check_wp_deviation(master, self.wp_distance or 99999999999)
                time.sleep(0.5)
        except Exception as e:
            logging.error(f"[Oracle Thread] Oracle monitoring crashed: {e}")
        print('[Oracle Thread] Oracles disabled as fuzzing stopped.')

    @staticmethod
    def load_waypoints(filename):
        waypoints = []
        with open(filename, 'r') as f:
            lines = f.readlines()
            for line in lines:
                line = line.strip()
                if not line or line.startswith('QGC'):
                    continue
                # parts = line.split()
                parts = line.strip().split()
                if len(parts) < 12:
                    continue
                lat = float(parts[8])
                lon = float(parts[9])
                alt1 = float(parts[10])
                waypoints.append({'lat': lat, 'lon': lon, 'alt': alt1})
        return waypoints

    @staticmethod
    def location_distance(lat1, lon1, lat2, lon2):
        return geodesic((lat1, lon1), (lat2, lon2)).meters


    def check_wp_deviation_px4(self, master):
        #waypoints = self.load_waypoints('missiondata/px4_mission1.txt')
        #print(f"Loaded {len(self.waypoints)} waypoints.")

        current_wp = None
        mission_current = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        if mission_current:
            current_wp = mission_current.seq
            #current_wp = mission_current.seq
            # if self.current_wp >= len(self.waypoints):
            #     print("No more waypoints.")

            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            #or we should use [local]current_wp? -> if pos_msg and mission_current
            if pos_msg:

                curr_lat = pos_msg.lat / 1e7
                curr_lon = pos_msg.lon / 1e7

                target_wp = self.waypoints[current_wp]
                target_lat = target_wp['lat']
                target_lon = target_wp['lon']

                distance = self.location_distance(curr_lat, curr_lon, target_lat, target_lon)
                #print(f"[WP{current_wp}] Distance to waypoint: {distance:.2f} m") # for debug

                if self.last_distance is not None:
                    # if distance > last_distance:
                    if distance - self.last_distance > 0.05:  # set threshold=0.05 to prevent FP caused by small perturbations
                        self.deviate_count += 1
                    else:
                        self.deviate_count = 0
                self.last_distance = distance

                if self.deviate_count > 3:
                    print("#####[WP deviation] Bug found: UAV is deviating from waypoint! #####")
                    self.wp_deviation_bug = True

    # just for px4
    def wp_oracle_px4(self, master, fuzz):
        try:
            while fuzz.running.is_set():
                self.check_wp_deviation_px4(master)
                time.sleep(0.5)
        except Exception as e:
            logging.error(f"[Oracle Thread] Oracle monitoring crashed: {e}")
        print('[Oracle Thread] Oracles disabled as fuzzing stopped.')

    def post_Arithm(self, master, bugpost):
        try:
            retries = 2
            while bugpost.running.is_set():
                try:
                    msg = None
                    for _ in range(retries):
                        msg = master.recv_match(blocking=True, timeout=0.3)
                        if msg is not None:
                            break
                    if msg is None:
                        print("##### Bug found: no-link/Arithm_Exception!! #####") # fp
                        self.timeout_bug = True
                        continue

                except Exception as e:
                    if "timeout" in str(e).lower():
                        #mavlink is implemented without throwing exceptions. took me an afternoon to debug  :(
                        # So this code seems useless. But we still keep it
                        print("############## timeout!! Arithm_Exception ##############")
                        self.timeout_bug = True
                    #useless
                    print("[rvstatus_thread]: not recvied")
                    self.timeout_bug = True
        except Exception as e:
            logging.error(f"[Oracle(Status) Thread] Oracle monitoring crashed: {e}")


    #useless functions
    def assign_pos(self, master):
        lat, lon, alt = rvmethod.gps_data(master)
        # -353632622 1491652375 -35
        self.position_data.lat = lat
        self.position_data.lon = lon
        self.position_data.alt = alt

    def get_curr_pos(self, master):
        lat, lon, alt = rvmethod.gps_data(master)
        pos = PositionData(lat, lon, alt)
        return pos

    def cal_pos(self, master): #not yet used
        ''' cal postion distance'''
        #FIX: add a condition: RV must in fly_mission.
        # and why don't use speed???

        pos1 = self.get_curr_pos(master)
        time.sleep(2)

        pos2 = self.get_curr_pos(master)
        diif1 = pos2.calculate_difference(pos1)

        time.sleep(2)
        pos3 = self.get_curr_pos(master)
        diif2 = pos3.calculate_difference(pos2)

        return abs(diif2 - diif1)

    def get_seq_pos(self, master):
        ''' Calculate avg speed.(mission distance)
        The flight path should be a straight line from point to point
        This means that in the same time step {t}, the distance from wp changes the same
        '''
        seq, targx, targy, targz = rvmethod.get_mission_wp_data(master)
        # store the cup of data? seq: PositionData(targx, targy, targz)
        targdata = PositionData(targx, targy, targz)
        #self.pos_datas[seq] = targdata

        self.assign_pos(master) # get current position
        diss1 = self.position_data.calculate_difference(targdata)
        time.sleep(2) # FIX: I think should define a MAX_Speed, and the fly time should > 4 seconds
        self.assign_pos(master)
        diss2 = self.position_data.calculate_difference(targdata)
        time.sleep(2)
        self.assign_pos(master)
        diss3 = self.position_data.calculate_difference(targdata)
        diff1 = diss1 - diss2 # calculate the distance within one time step
        diff2 = diss2 - diss3
        print(f'=========== 3 points disstance is: {diss1}, {diss2} and {diss3} ===========')
        print(f"diff1 = {diff1}, diff2 = {diff2}") #for debug

        if abs(diff2-diff1) > self.position_threshold:
            print("position oracle triggerd")
            # post-process : record and pop this path
            self.pos_bug = True

    def position_oracle(self, master):
        # calculate once when mission/fly finish
        lat, lon, alt =  rvmethod.gps_data(master)
        lat_diff = abs(self.position_data.lat - lat) #if self.lat is not None and lat is not None else None
        lon_diff = abs(self.position_data.lon - lon) #if self.lon is not None and lon is not None else None
        alt_diff = abs(self.position_data.alt - alt) #if self.alt is not None and alt is not None else None
        position_error = lat_diff + lon_diff + alt_diff
        #position_error = abs(lat-targ_lat) + abs(lon-targ_lon) + abs(alt-targ_alt)
        if position_error > self.position_threshold:
            logging.info(f'Bug captured: 2 - position error')
            #  save the path
            # for example: Use a Data construct to save, BugPath.add(path)
            return True
        return False



