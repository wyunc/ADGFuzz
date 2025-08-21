import os

def parse_lcov_file(filepath):
    coverage = {}
    current_file = None

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('SF:'):
                current_file = os.path.normpath(line[3:])
                if current_file not in coverage:
                    coverage[current_file] = set()
            elif line.startswith('DA:'):
                line_num, count = line[3:].split(',')
                if int(count) > 0:
                    coverage[current_file].add(int(line_num))
    return coverage

def compare_coverage(cov_a, cov_b):
    common = {}
    only_in_a = {}
    only_in_b = {}

    all_files = set(cov_a.keys()).union(set(cov_b.keys()))

    for file in all_files:
        lines_a = cov_a.get(file, set())
        lines_b = cov_b.get(file, set())
        common[file] = lines_a & lines_b
        only_in_a[file] = lines_a - lines_b
        only_in_b[file] = lines_b - lines_a

    return common, only_in_a, only_in_b

def count_total_lines(line_map):
    return sum(len(lines) for lines in line_map.values())

def main():
    a_info = 'cov_copter/finalcov/runall_mis1.info'
    b_info = 'cov_copter/finalcov/final_4.info'

    cov_a = parse_lcov_file(a_info)
    cov_b = parse_lcov_file(b_info)

    common, only_in_a, only_in_b = compare_coverage(cov_a, cov_b)

    print(f"Number of lines of code that are covered in common: {count_total_lines(common)}")
    print(f"Number of more rows covered by B than by A: {count_total_lines(only_in_b)}")
    print(f"Number of more rows covered by A than by B: {count_total_lines(only_in_a)}")


    # for file, lines in only_in_a.items():
    #     if lines:
    #         print(f"{file}: {sorted(list(lines))[:10]}")
    #

    # for file, lines in only_in_b.items():
    #     if lines:
    #         print(f"{file}: {sorted(list(lines))[:10]}")

if __name__ == "__main__":
    main()
