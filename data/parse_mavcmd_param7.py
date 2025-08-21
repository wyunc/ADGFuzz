import csv
import re
from bs4 import BeautifulSoup


def parse_mav_cmd_html(file_path, output_csv):
    with open(file_path, 'r', encoding='utf-8') as file:
        soup = BeautifulSoup(file, 'html.parser')


    with open(output_csv, 'w', newline='', encoding='utf-8') as csvfile:
        csvwriter = csv.writer(csvfile)

        csvwriter.writerow(['CMD_Name', 'Index', 'param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7'])

        # search MAV_CMD_XXX
        for h3 in soup.find_all('h3'):
            cmd_name_match = re.match(r'MAV_CMD_\w+ \((\d+)\)', h3.text)
            if cmd_name_match:
                cmd_name = h3.text.split(' ')[0]
                cmd_index = cmd_name_match.group(1)

                table = h3.find_next('table')
                if table:
                    params = ['N'] * 7  # default set
                    for row in table.find_all('tr')[1:]:
                        cols = row.find_all('td')
                        if len(cols) >= 2:
                            param_index = int(re.search(r'\d+', cols[0].text).group()) - 1
                            description = cols[1].text.lower()

                            if 'empty' in description:
                                params[param_index] = 'E'
                            else:
                                # parse 'Values'
                                if len(cols) > 2 and cols[2].text.strip():
                                    values = cols[2].text.lower()
                                    if 'min' in values or 'max' in values or 'inc' in values:
                                        min_val = re.search(r'min: (\d+)', values)
                                        max_val = re.search(r'max: (\d+)', values)
                                        inc_val = re.search(r'inc: (\d+)', values)
                                        min_val = min_val.group(1) if min_val else 'N'
                                        max_val = max_val.group(1) if max_val else 'N'
                                        inc_val = inc_val.group(1) if inc_val else 'N'
                                        params[param_index] = f'[{min_val},{max_val},{inc_val}]'
                                    else:
                                        # If there is no min/max/inc in Values, just take the value
                                        #params[param_index] = values.strip()
                                        params[param_index] = cols[2].text.strip()
                                elif len(cols) > 3 and cols[3].text.strip():
                                    params[param_index] = cols[3].text.strip()

                    csvwriter.writerow([cmd_name, cmd_index] + params)


def extract_unique_params(csv_file, output_file):
    unique_values = set()

    with open(csv_file, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)

        for row in reader:
            for i in range(1, 8):
                param = row[f'param{i}'].strip()
                if param and not param.startswith('['):
                    unique_values.add(param)

    sorted_values = sorted(unique_values)

    with open(output_file, 'w', encoding='utf-8') as f:
        for value in sorted_values:
            f.write(f"{value}\n")


#parse_mav_cmd_html('MAV_CMD.html', 'mav_cmds1.csv')

extract_unique_params('mav_cmds1.csv', 'mavcmd_param_value.txt')

