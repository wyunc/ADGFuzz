import csv
import re

def extract_unique_terms(input_csv, output_txt):
    # parameter
    unique_terms = set()
    with open(input_csv, mode='r', newline='', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            parameter_name = row['Parameter_Name']

            #terms = parameter_name.split('_')
            terms = [term.lower() for term in parameter_name.split('_') if not re.search(r'\d', term)]
            unique_terms.update(terms)  # duplicate removal

    excluded_terms = {'mav', 'do', 'cmd', 'set', 'get', 'checks', 'to', 'return', 'rad', 'km', 'ms',
                      'max', 'min', 'only', 'to', 'id', 'p','t','v', 'w', 'udp', 'i', 'h','g','go','io',
                      'd','q','m','log','txt','a', 'b','c','f','k','r','s', 'e','hz','l','n','o','x','y','z'}
    unique_terms -= excluded_terms

    with open(output_txt, mode='w', encoding='utf-8') as outfile:
        for term in sorted(unique_terms):  # Sort alphabetically
            outfile.write(term + '\n')

def extract_cmd_terms(input_csv, output_txt):
    unique_terms = set()
    with open(input_csv, mode='r', newline='', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            parameter_name = row['CMD_Name']

            #terms = parameter_name.split('_')
            terms = [term.lower() for term in parameter_name.split('_') if not re.search(r'\d', term)]
            unique_terms.update(terms)  # duplicate removal

    with open(output_txt, mode='w', encoding='utf-8') as outfile:
        for term in sorted(unique_terms):  # Sort alphabetically
            outfile.write(term + '\n')

# input_csv = 'mav_cmds1.csv'
# output_txt = 'cmd_term1.txt'
# extract_cmd_terms(input_csv, output_txt)


input_csv = 'ap-plane-v470.csv' # ArduPilot
output_txt = 'param_plane_term.txt' # ArduPilot
#input_csv = 'px4-parameters.csv' # PX4
#output_txt = 'param_px4_term.txt' # PX4
extract_unique_terms(input_csv, output_txt)

print(f"Unique terms have been saved to {output_txt}.")
