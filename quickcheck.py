import os
import re
from collections import defaultdict

copter_one_word = [
'MAV_CMD_NAV_LOITER_TO_ALT',
'SIM_BATT_VOLTAGE',
'SIM_RATE_HZ',
'GPS2_TYPE',
'SIM_WIND_TURB',
'TERRAIN_SPACING',
'SIM_IMU_POS_X',
'SIM_IMU_POS_Y',
'SIM_IMU_POS_Z',
'INS_GYROFFS_X',
'INS_GYROFFS_Y',
'INS_GYR2OFFS_X',
'INS_GYR2OFFS_Y',
'INS_GYR2OFFS_Z',
'SIM_DRIFT_SPEED',
'SIM_ACC1_BIAS_X',
'SIM_ACC1_BIAS_Y',
'SIM_ACC1_BIAS_Z',
'SIM_ACC2_BIAS_X',
'SIM_ACC2_BIAS_Y',
'SIM_ACC2_BIAS_Z',
'SIM_GYR1_BIAS_X',
'SIM_GYR1_BIAS_Y',
'SIM_GYR1_BIAS_Z',
'SIM_GYR2_BIAS_X',
'SIM_GYR2_BIAS_Y',
'SIM_GYR2_BIAS_Z',
'SIM_GPS1_GLTCH_X',
'SIM_GPS1_GLTCH_Y',
'SIM_GPS1_HZ',
'SIM_GPS2_HZ',
'SIM_PLD_ENABLE',
'SIM_SONAR_SCALE',
'SERIAL_PASS2',
'SIM_GYR_FILE_RW',
'MAV_CMD_CAN_FORWARD',
'MAV_CMD_DO_SET_HOME',
'MAV_CMD_DO_SET_ROI_LOCATION',
'MAV_CMD_EXTERNAL_POSITION_ESTIMATE',
'AHRS_EKF_TYPE',
'SERVO3_FUNCTION',
'SIM_GPS1_LAG_MS',
'SIM_GPS1_ENABLE',
'FS_THR_VALUE',
'SIM_ENGINE_MUL',
'FLTMODE_CH',
'MAV_CMD_NAV_LOITER_UNLIM',
'SIM_CAN_SRV_MSK ',
'BARO_ALT_OFFSET',
'MOT_PWM_MIN',
'MOT_SPIN_MIN'
]

copter_two_wrod=[
('SIM_RATE_HZ', 'SIM_TIME_JITTER'),
('MOT_BAT_CURR_MAX','MOT_PWM_MIN'),
('AHRS_EKF_TYPE', 'AHRS_TRIM_X'),
('SIM_TIME_JITTER', 'MAV_CMD_DO_SET_MODE'),
('MOT_BAT_VOLT_MAX', 'MOT_PWM_MIN'),
('ANGLE_MAX','SIM_WIND_SPD'),
('AHRS_EKF_TYPE', 'AHRS_TRIM_X')
]


def main():

    one_word = copter_one_word
    two_word = copter_two_wrod

    folder = "outfile/copter/"
    file_pattern = re.compile(r'bug(\d+)_.*\.txt$')
    result = {}


    one_word_dict = {w: [] for w in one_word}
    two_word_dict = {pair: [] for pair in two_word}

    files = [f for f in os.listdir(folder) if file_pattern.match(f)]
    for filename in files:
        match = file_pattern.match(filename)
        if not match:
            continue
        X = int(match.group(1))
        filepath = os.path.join(folder, filename)

        all_vars = set()
        with open(filepath, "r", encoding="utf-8") as f:
            for line in f:
                tokens = line.strip().split()
                if tokens[0] not in {'paramset', 'mavcmd'}:
                    continue
                if len(tokens) >= 2:
                    xxx = tokens[1]
                    all_vars.add(xxx)

        found = False

        for w in one_word:
            if w in all_vars:
                one_word_dict[w].append(X)
                found = True

        for pair in two_word:
            if pair[0] in all_vars and pair[1] in all_vars:
                two_word_dict[pair].append(X)
                found = True

    result = []
    t_result = []
    for w in one_word:
        if one_word_dict[w]:
            result.append((w, sorted(one_word_dict[w])))
        else:
            t_result.append((w, ''))
    for pair in two_word:
        if two_word_dict[pair]:
            result.append((pair, sorted(two_word_dict[pair])))
        else:
            t_result.append((pair, ''))


    print("result:")
    for item in result:
        print(f"{item[0]} : {','.join(map(str, item[1]))}")


    index = []
    used = set()
    for item in result:
        nums = item[1]
        for x in nums:
            if x not in used:
                index.append(x)
                used.add(x)
                break  # only get the first one (without repeat)
    print("\nindex:")
    print(index)




if __name__ == "__main__":
    main()

