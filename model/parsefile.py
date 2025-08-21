import json
import random
class Parsefile:
    def __init__(self):
        self.envname = []

        self.mavfunc_list = []

    def parse_env(self, filepath):
        print('##### (Start) Read a meta file for environmental factors #####')
        cnt = 0

        for line in open(filepath, 'r').readlines():
            row = line.replace("\n", "")
            self.envname.append(row)
            cnt += 1

    @staticmethod
    def parse_dict_file(filepath):  # test
        with open(filepath, 'r') as file_a:
            dict_raw = json.load(file_a)
        for name, val in dict_raw.items():
                name = val # error
        return dict_raw

    def parse_mavfunc(self):
        filepath = '../data/Amavmsg.txt'

        for func in open(filepath, 'r').readlines():
            self.mavfunc_list.append(func)

    def rco_mavfunc(self):
        num = len(self.mavfunc_list)
        index = random.randint(1, num)
        return self.mavfunc_list[index]

# p = Parsefile()
# p.parse_mavfunc()
# print(len(p.mavfunc_list))
# for funcname in p.mavfunc_list:
#     print(funcname)
