import os
import subprocess
import re


def get_divi_cov():
    info_dir = "cov_copter/mis4"
    results = {}

    # coverage summary
    line_re = re.compile(r"lines\.+: ([\d\.]+)% \((\d+) of (\d+) lines\)")
    func_re = re.compile(r"functions\.+: ([\d\.]+)% \((\d+) of (\d+) functions\)")

    for fname in os.listdir(info_dir):
        if fname.endswith(".info"):
            fpath = os.path.join(info_dir, fname)
            try:
                output = subprocess.check_output(
                    ["lcov", "--summary", fpath], stderr=subprocess.STDOUT, text=True
                )
            except Exception as e:
                print(f"Error processing {fname}: {e}")
                continue

            lines_cov, lines_hit, lines_total = None, None, None
            for line in output.splitlines():
                m = line_re.search(line)
                if m:
                    lines_cov = float(m.group(1))
                    lines_hit = int(m.group(2))
                    lines_total = int(m.group(3))
                    break
            if lines_cov is not None:
                results[fname] = {
                    "coverage": lines_cov,
                    "hit": lines_hit,
                    "total": lines_total,
                }


    for fname, res in results.items():
        print(f"{fname}: {res['coverage']}% ({res['hit']} of {res['total']} lines)")


    if results:
        minfile = min(results.items(), key=lambda x: x[1]["coverage"])
        maxfile = max(results.items(), key=lambda x: x[1]["coverage"])

        print("\nMinimum coverage:")
        print(f"{minfile[0]}: {minfile[1]['coverage']}% ({minfile[1]['hit']} of {minfile[1]['total']} lines)")

        print("Maximum coverage:")
        print(f"{maxfile[0]}: {maxfile[1]['coverage']}% ({maxfile[1]['hit']} of {maxfile[1]['total']} lines)")
    else:
        print("No valid .info files found.")

def merge_cov():
    infoname = "mis2"
    info_dir = f"cov_copter/{infoname}"
    output_file = os.path.join(info_dir, f"amerged_{infoname}.info")

    info_files = []
    for fname in os.listdir(info_dir):
        if fname.endswith('.info'):
            fpath = os.path.join(info_dir, fname)
            if os.path.getsize(fpath) > 0:
                info_files.append(fpath)

    if not info_files:
        print("No non-empty .info files found!")
        exit(1)


    cmd = ["lcov", "--add-tracefile", info_files[0]]
    for f in info_files[1:]:
        cmd.extend(["--add-tracefile", f])
    cmd.extend(["--output-file", output_file])

    print("Merging the following files:")
    for f in info_files:
        print("  ", f)
    print(f"Output file: {output_file}")

    subprocess.run(cmd, check=True)
    print("Merge completed.")

merge_cov()