import random
import re
import csv

from model.constant import rc_copter_map, rc_plane_map, rc_rover_map
class Mapp:
    '''
    This calss should and should only be used once.
    Convert the source code paths to RV paths.
    '''

    def __init__(self, rvtype='copter', max_Hcount=1):
        #self.paths = Paths() #  use this class or not? no
        self.paths = []
        self.rvtype = rvtype
        self.max_count = max_Hcount

        #self.mav_cmd_to_para = [] # xx['command_long_send'] = {MAV_XXX_XXX1, MAV_XXX_XXX2}
    def remove_path(self, path):
        self.paths.remove(path)

    def clear_path(self):
        self.paths = []


    def parse_ipaths(self, paths):
        cmd_file = 'map/cmd_term1.txt'
        # config_file_1 = 'map/config_term_1.txt'
        # config_file_2 = 'map/config_term_2.txt'
        config_file = f'map/param_{self.rvtype}_term.txt'  # use this replace config_file1 and 2
        env_file = 'map/env_term.txt'
        mav_cmds_csv = 'data/mav_cmds.csv'
        # params_csv = 'data/parameters-copter-v460-1.csv'
        params_csv = f'data/ap-{self.rvtype}-v470.csv'

        env_csv = 'data/env.csv'

        fix_file = 'map/fix.txt'

        def load_terms(file_path):
            with open(file_path, 'r') as file:
                return [line.strip() for line in file]

        def load_csv(file_path, key_name):
            data = []
            with open(file_path, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    data.append(row[key_name])
            return data

        def load_fix_terms(file_path):
            fix_dict = {}
            with open(file_path, 'r') as file:
                for line in file:
                    key, values = line.strip().split(':')
                    key = key.strip().strip("'")

                    values_set = [v.strip().strip("'") for v in values.split(',')]
                    fix_dict[key] = {v for v in values_set if v}
            return fix_dict

        cmd_terms = load_terms(cmd_file)
        # config_terms_1 = load_terms(config_file_1) #why split to config_1 and 2?
        # config_terms_2 = load_terms(config_file_2)
        config_terms = load_terms(config_file)
        env_terms = load_terms(env_file)

        map_dict = {
            'copter': rc_copter_map,
            'plane': rc_plane_map,
            'rover': rc_rover_map
        }
        rv_terms = map_dict.get(self.rvtype)  # to use: rv_terms.get(term, None)
        if rv_terms is None:
            raise ValueError(f"Invalid rvtype: {self.rvtype}. Choose from copter, plane, or rover.")

        # convert to set, query time complexity: O(n) -> O(1)
        cmd_terms_set = set(term.lower() for term in cmd_terms)
        # config_terms_set = set(term.lower() for term in config_terms_1 + config_terms_2)
        config_terms_set = set(term.lower() for term in config_terms)
        env_terms_set = set(term.lower() for term in env_terms)
        rv_terms_set = set(term.lower() for term in rv_terms)

        mav_cmds = load_csv(mav_cmds_csv, 'CMD_Name')
        params = load_csv(params_csv, 'Parameter_Name')
        envs = load_csv(env_csv, 'Env_Name')

        fix_terms = load_fix_terms(fix_file)

        # Step1: Convert variable names to more exact RV variables
        final_result = set()
        for path, node_num in paths:
            result = []
            H_count = 0
            H_sum = 0
            hasmatch = False

            for node in path:

                split_terms = re.split(r'_+', node)
                filtered_terms = []
                for term in split_terms:
                    if term and not term.isdigit():
                        filtered_terms.append(term)

                if len(filtered_terms) < 2:
                    continue  # skip node with a single term
                terms = filtered_terms
                # terms = [term for term in re.split(r'_+', node) if term and not term.isdigit()]

                matched = set()


                # H_count_term = 0
                for term in terms:
                    # has_matched = False
                    # new_terms = fix_terms.get(term, [term])
                    if term in fix_terms:
                        new_terms = fix_terms[term]
                    else:
                        new_terms = [term]

                    for aterm in map(str.lower, new_terms):
                        #  if aterm.len<2, continue. except for x y z //seems don't need do this
                        # but we can filter out some unwanted terms here
                        if aterm in cmd_terms_set or aterm in config_terms_set or aterm in env_terms_set or aterm in rv_terms_set:  # ==
                            matched.add(aterm)

                if matched:
                    result.append(tuple(matched))
                    hasmatch = True

                # H_count += len(matched_cmd) + len(matched_config) + len(matched_env) + len(matched_rc)
                # H_count += H_count_term

            H_count = node_num

            seen_phars1 = set()
            deduplicated_res = []
            for item in result:
                item_tuple = tuple(item)
                if item_tuple not in seen_phars1:
                    seen_phars1.add(item_tuple)
                    deduplicated_res.append(item)

            result = deduplicated_res
            if hasmatch:
                final_result.add(tuple(result))
                #final_result.add((tuple(path), tuple(result)))
        with open("paths/matched_ipaths.txt", "w", encoding="utf-8") as f:
            for item in final_result:
                f.write(f"{item}\n")

    def parse_paths(self, paths):
        cmd_file = 'map/cmd_term1.txt'
        # config_file_1 = 'map/config_term_1.txt'
        # config_file_2 = 'map/config_term_2.txt'
        config_file = f'map/param_{self.rvtype}_term.txt' # use this replace config_file1 and 2
        env_file = 'map/env_term.txt'
        mav_cmds_csv = 'data/mav_cmds.csv'
        #params_csv = 'data/parameters-copter-v460-1.csv'
        params_csv = f'data/ap-{self.rvtype}-v470.csv'
        if self.rvtype == 'px4':
            params_csv = f'data/px4-parameters.csv' # Due to the need to augment the experiments, we hardcode here
        env_csv = 'data/env.csv'

        fix_file = 'map/fix.txt'

        def load_terms(file_path):
            with open(file_path, 'r') as file:
                return [line.strip() for line in file]

        def load_csv(file_path, key_name):
            data = []
            with open(file_path, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    data.append(row[key_name])
            return data

        def load_fix_terms(file_path):
            fix_dict = {}
            with open(file_path, 'r') as file:
                for line in file:
                    key, values = line.strip().split(':')
                    key = key.strip().strip("'")

                    values_set = [v.strip().strip("'") for v in values.split(',')]
                    fix_dict[key] = {v for v in values_set if v}
            return fix_dict

        cmd_terms = load_terms(cmd_file)
        # config_terms_1 = load_terms(config_file_1) #why split to config_1 and 2?
        # config_terms_2 = load_terms(config_file_2)
        config_terms = load_terms(config_file)
        env_terms = load_terms(env_file)

        map_dict = {
            'copter': rc_copter_map,
            'plane': rc_plane_map,
            'rover': rc_rover_map,
            'px4': rc_copter_map  # : We assume that, only px4-copter is tested
        }
        rv_terms = map_dict.get(self.rvtype) # to use: rv_terms.get(term, None)
        if rv_terms is None:
            raise ValueError(f"Invalid rvtype: {self.rvtype}. Choose from copter, plane, or rover.")

        # convert to set, query time complexity: O(n) -> O(1)
        cmd_terms_set = set(term.lower() for term in cmd_terms)
        #config_terms_set = set(term.lower() for term in config_terms_1 + config_terms_2)
        config_terms_set = set(term.lower() for term in config_terms)
        env_terms_set = set(term.lower() for term in env_terms)
        rv_terms_set = set(term.lower() for term in rv_terms)

        mav_cmds = load_csv(mav_cmds_csv, 'CMD_Name')
        params = load_csv(params_csv, 'Parameter_Name')
        envs = load_csv(env_csv, 'Env_Name')

        fix_terms = load_fix_terms(fix_file)
        seen_set = set()
        # Step1: Convert variable names to more exact RV variables
        for path, node_num in paths:
            result = []
            H_count = 0
            H_sum = 0
            #has_matched = False

            for node in path:

                split_terms = re.split(r'_+', node)
                filtered_terms = []
                for term in split_terms:
                    if term and not term.isdigit():
                        filtered_terms.append(term)

                if len(filtered_terms) < 2:
                    continue # skip node with a single term
                terms = filtered_terms
                #terms = [term for term in re.split(r'_+', node) if term and not term.isdigit()]

                matched_cmd = set()
                matched_config = set()
                matched_env = set()
                matched_rc = set()

                #H_count_term = 0
                for term in terms:
                    #has_matched = False
                    # new_terms = fix_terms.get(term, [term])
                    if term in fix_terms:
                        new_terms = fix_terms[term]
                    else:
                        new_terms = [term]

                    for aterm in map(str.lower, new_terms):
                        #  if aterm.len<2, continue. except for x y z //seems don't need do this
                        # :but we can filter out some unwanted terms here
                        if aterm in cmd_terms_set: # ==
                            matched_cmd.add(aterm)
                            #has_matched = True
                        if aterm in config_terms_set:
                            matched_config.add(aterm)
                            #has_matched = True
                        if aterm in env_terms_set:
                            matched_env.add(aterm)
                            #has_matched = True
                        if aterm in rv_terms_set:
                            matched_rc.add(aterm)
                            #has_matched = True
                    # if has_matched:
                    #     H_count_term += 1

                if matched_cmd:
                    # for cmd in matched_cmd:
                    #     result.append(['mavcmd', cmd]) #or extend?
                    result.append(['mavcmd', ','.join(matched_cmd)])

                if matched_config:
                    result.append(['paramset', ','.join(matched_config)])
                if matched_env:
                    result.append(['envset', ','.join(matched_env)])

                if matched_rc:
                    for rc_channel in matched_rc:
                        result.append(['rc', rv_terms.get(rc_channel)])

                #H_count += len(matched_cmd) + len(matched_config) + len(matched_env) + len(matched_rc)
                #H_count += H_count_term

            if not result:
                continue

            H_count = node_num

            seen_phars1 = set()
            deduplicated_res = []
            for item in result:
                item_tuple = tuple(item)
                if item_tuple not in seen_phars1:
                    seen_phars1.add(item_tuple)
                    deduplicated_res.append(item)

            result = deduplicated_res

            tuple_result = tuple(map(tuple, result))
            if tuple_result in seen_set:
                continue
            seen_set.add(tuple_result)

            final_result = []

            # for category, terms in result:
            #     terms = terms.split(', ')
            #     if category == 'mavcmd':
            #         for term in terms:
            #             matches = [cmd for cmd in mav_cmds if term.upper() == cmd]
            #             final_result.extend([category, match] for match in matches or [None])
            #     elif category == 'paramset':
            #         matched_params = [param for param in params if all(term.upper() in param for term in terms)]
            #         final_result.extend(['paramset', match] for match in matched_params or [None])
            #     elif category == 'envset':
            #         matches = [env for env in envs if any(term.upper() in env for term in terms)]
            #         final_result.extend(['envset', match] for match in matches or [None])

            for res in result:
                category = res[0]
                terms_str = res[1]
                #terms = res[1].split(', ')
                terms = [term.strip() for term in str(terms_str).split(',')]

                if category == 'mavcmd':
                    for term in terms:
                        term_upper = term.upper()
                        matches = []
                        for cmd in mav_cmds:
                            # if term_upper == cmd: # error match logic?
                            #     matches.append(cmd)
                            if cmd.startswith(term_upper) or f'_{term_upper}' in cmd:
                                matches.append(cmd)

                        if matches:
                            for match in matches:
                                final_result.append(['mavcmd', match])
                            H_sum += 1/len(matches)
                        # else:
                        #     final_result.append(['mavcmd', None])

                elif category == 'rc':
                    for term in terms:
                        final_result.append(['rc', term])
                    # H_sum += 1/len(terms)

                elif category == 'paramset':
                    matched_params = {}

                    for param in params:
                        match_count = 0
                        for term in terms:
                            term_upper = term.upper()
                            pattern = re.compile(rf'^{term_upper}_|(?<=_)({term_upper})(?=_|$)')
                            if pattern.search(param):
                            #if param.startswith(term_upper) or f'_{term_upper}' in param:
                            #if term.upper() in param: #param is not a set, so 'in' is not a strict match
                                match_count += 1

                        if match_count > 0:
                            if match_count not in matched_params:
                                matched_params[match_count] = [] # init
                            matched_params[match_count].append(param)

                            H_sum += match_count / (len(matched_params[match_count]) or 1)

                    for count, params1 in matched_params.items():
                        if not params1:
                            continue
                        for match in params1:
                            final_result.append(['paramset', match, count])
                            #if want to safely use this: if len(node) > 2 and node[2] == 1/2/3/...:

                elif category == 'envset':
                    for term in terms:
                        term_upper = term.upper()
                        matches = []
                        for env in envs:
                            #if term_upper in env:
                            if env.startswith(term_upper) or f'_{term_upper}' in env:
                                matches.append(env)

                        if matches:
                            for match in matches:
                                final_result.append(['envset', match])
                            H_sum += 1 / len(matches)
                        # else:
                        #     final_result.append(['envset', None])

            entropy = H_count + H_sum
            #entropy = H_sum    #for ablation exp - E2-1 E2-2  //delete this
            if entropy == 0:
                continue

            #print(H_count/self.max_count)
            seen_re = set()
            final_unique = []
            for item in final_result:
                if item[1] not in seen_re:
                    seen_re.add(item[1])
                    final_unique.append(item)
            #random.shuffle(final_result)

            self.paths.append((final_unique,entropy,path))
            #done: Remove empty paths, (entropy=0 or entropy<2) => I put this in fuzz.py
        self.paths.sort(key=lambda x: x[1], reverse=True)

    def get_paths_by_entropy(self):
        for path, entropy in self.paths:
            yield path
    # def parse_paths(self, paths):
    #     for path in paths:
    #         result = self.parse_path(path)
    #         self.paths.append(result)