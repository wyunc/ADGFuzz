import os
import csv

def load_all_terms(map_dir, filenames):
    all_terms = set()
    for fname in filenames:
        file_path = os.path.join(map_dir, fname)
        if not os.path.isfile(file_path):
            print(f"Warning: {file_path} not found, skipping.")
            continue
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                term = line.strip()
                if term:
                    all_terms.add(term)
    return all_terms

def main():

    map_dir = '../map/'
    map_files = [
        'cmd_term1.txt',
        'param_px4_term.txt',
        # 'config_term.txt',
        # 'env_term.txt',
        # 'param_copter_term.txt',
        # 'param_plane_term.txt',
        # 'param_rover_term.txt',
        'excluded_terms.txt'
    ]
    all_terms = load_all_terms(map_dir, map_files)

    with open('px4_terms/all_terms_px4.txt', 'w', encoding='utf-8') as f:
        for term in sorted(all_terms):
            f.write(term + '\n')

    matched_terms = []
    unmatched_terms = []
    yes_count = 0
    no_count = 0

    with open('px4_terms/term_countpx4.txt', 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or ':' not in line:
                continue

            if line.startswith("'"):
                parts = line.split("'")
                if len(parts) >= 3:
                    term = parts[1]
                    count = int(parts[2][1:])
                else:
                    continue
            else:
                parts = line.split(':', 1)
                if len(parts) == 2:
                    term = parts[0].strip()
                    count = int(parts[1].strip())
                else:
                    continue

            if term in all_terms:
                matched_terms.append(term)
                yes_count += count
            else:
                unmatched_terms.append(term)
                no_count += count

    with open('px4_terms/matched_terms_px4.txt', 'w', encoding='utf-8') as f:
        for t in matched_terms:
            f.write(t + '\n')

    with open('px4_terms/unmatched_terms_px4.txt', 'w', encoding='utf-8') as f:
        for t in unmatched_terms:
            f.write(t + '\n')

    print(f"Matched terms count: {len(matched_terms)}")
    print(f"Unmatched terms count: {len(unmatched_terms)}")
    print(f"yes_count (sum of matched): {yes_count}")
    print(f"no_count (sum of unmatched): {no_count}")

def read_terms(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        terms = set(line.strip() for line in f if line.strip())
    return terms

def read_first_column_csv(filename):
    param_names = set()
    with open(filename, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row:
                param_names.add(row[0].strip().lower())
    return param_names

def main1():
    # Used to determine whether matched_terms can fully cover all RV inputs: YES!
    matched_terms = read_terms('px4_terms/matched_terms_px4.txt')
    param_names = set()
    #for csv_path in ['../data/ap-copter-v470.csv','../data/ap-plane-v470.csv', '../data/ap-rover-v470.csv', '../data/mav_cmds1.csv']:
    for csv_path in ['../data/px4-parameters.csv', '../data/mav_cmds1.csv']:
        param_names |= read_first_column_csv(csv_path)

    print(f'Number of unique param_names before filtering: {len(param_names)}')
    to_remove = set()
    for term in matched_terms:
        if len(term) == 1:
            continue
        for name in param_names:
            if term in name:
                to_remove.add(name)
    param_names.difference_update(to_remove)

    with open('px4_terms/unmatched_param_px4.txt', 'w', encoding='utf-8') as f:
        for name in sorted(param_names):
            f.write(name + '\n')

if __name__ == '__main__':
    #main()
    main1()
