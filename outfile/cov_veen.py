import os
from itertools import product

def parse_lcov_file(filepath):

    coverage = set()
    current_file = None

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('SF:'):
                current_file = os.path.normpath(line[3:])
            elif line.startswith('DA:') and current_file:
                lineno, count = line[3:].split(',')
                if int(count) > 0:
                    coverage.add((current_file, int(lineno)))
    return coverage

def main():
    file_dir = "cov_plane/finalcov/result"
    files = {
        'A': f'{file_dir}/final_1.info',
        'B': f'{file_dir}/final_2.info',
        'C': f'{file_dir}/final_3.info',
        'D': f'{file_dir}/final_4.info'
    }

    cover_sets = {k: parse_lcov_file(v) for k, v in files.items()}
    all_lines = set().union(*cover_sets.values())


    venn_count = {bin(i)[2:].zfill(4): 0 for i in range(16)}
    venn_lines = {bin(i)[2:].zfill(4): set() for i in range(16)}


    for line in all_lines:
        flag = ''.join(['1' if line in cover_sets[k] else '0' for k in ['A', 'B', 'C', 'D']])
        venn_count[flag] += 1
        venn_lines[flag].add(line)

    print("Assembly\tNumber\tDescription")
    for key, count in sorted(venn_count.items(), key=lambda x: -x[1]):
        desc = []
        for i, k in enumerate(['A', 'B', 'C', 'D']):
            if key[i] == '1':
                desc.append(k)
        if len(desc) == 0:
            desc = ['None']
        print(f"{key}\t{count}\t{' ∩ '.join(desc)}")

if __name__ == "__main__":
    main()
