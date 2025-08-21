from bs4 import BeautifulSoup
import csv

# Load the HTML content
#copter470/CopterParameterV470.html
#plane470/PlaneParameterV470.html
#rover470/RoverParameterV470.html
with open("rover470/RoverParameterV470.html", "r", encoding="utf-8") as file:
    content = file.read()

soup = BeautifulSoup(content, "lxml")

constant_map = {
    'meter':(0,100),
    'meters per second':(0,15),
    'seconds':(0,100),
    'microseconds': (0, 1000000),  # # microseconds, typically up to millions
    'milliseconds': (0, 1000),  # milliseconds, typically up to 1000ms
    'radians per square second': (0, 10),
    'volt': (0, 60),  # voltage, taking into account the maximum battery voltage
    'degrees per second': (-180, 360),
    'radians per second': (-3.14, 6.28), #2pi
    'degrees Celsius': (-40, 85),
    'meters per square second': (0, 20),  # acceleration, generally 0-20
    'Newtons': (0, 500),  # Thrust, depending on the device, take 500 Newtons as the reference
    'meters per volt': (0, 10),  # may be the relationship between voltage and location, reasonable value
    'ampere per volt': (0, 100),
    'kilometers': (0, 10),
    'hertz': (0, 1000),
    'ampere hour': (0, 50),
    'degrees': (-360, 360),
    'pascal': (30000, 110000),
    'meters': (0, 1000),
    'kilograms': (0, 50),
    'percent': (0, 100),
    'kilobytes': (0, 1024),  # data，0-1024KB
    'hectopascal': (300, 1100)  # 300-1100hPa
}

# Initialize a list to store the parsed data
parameters = []
seen_params = set()
# Find all sections
sections = soup.find_all('section')

#unique_cons=set()

for section in sections:
    try:
        # Extract the parameter name
        header = section.find('h3')
        if header:
            param_name = header.text.split(':')[0].strip()

            if '(' in param_name:
                param_name = param_name.split('(')[0].strip()

            #if param_name.startswith(('LOG_', 'SIM_')):
            if param_name.startswith('LOG_'):
                continue

            if param_name.isupper() and param_name not in seen_params:
                seen_params.add(param_name)
                increment = "N"
                range_min = "N"
                range_max = "N"
                value = "N"

                table = section.find('table', class_='docutils')
                if table:
                    rows = table.find_all('tr')
                    headers = [th.text.strip() for th in rows[0].find_all('th')]
                    if "Increment" in headers and "Range" in headers:
                        cols = rows[1].find_all('td')
                        increment = cols[0].text.strip()
                        range_values = cols[1].text.strip().split('to')
                        range_min = range_values[0].strip()
                        range_max = range_values[1].strip()
                    elif "Range" in headers:
                        cols = rows[1].find_all('td')
                        range_values = cols[0].text.strip().split('to')
                        range_min = range_values[0].strip()
                        range_max = range_values[1].strip()
                        #increment = "N"
                    elif "Increment" in headers:
                        cols = rows[1].find_all('td')
                        increment = cols[0].text.strip()

                    elif "Values" in headers or "Bitmask" in headers:
                        values = []
                        inner_table = table.find('table', class_='docutils')
                        if inner_table:
                            for row in inner_table.find_all('tr')[1:]:
                                cols = row.find_all('td')
                                if len(cols) >= 1:
                                    val = cols[0].text.strip()
                                    values.append(val)
                        value = "[" + ",".join(values) + "]"
                        if "Values" in headers:
                            increment = "V"
                        elif "Bitmask" in headers:
                            increment = "B"

                if increment == "N" and range_min == "N" and range_max == "N" and value == "N":

                    unit_row = table.find('tr', class_='row-even')
                    if unit_row:
                        unit_col = unit_row.find('td')
                        if unit_col:
                            unit = unit_col.text.strip()
                            #unique_cons.add(unit)
                            if unit in constant_map:
                                range_min, range_max = map(str, constant_map[unit])

                parameters.append((param_name, increment, range_min, range_max, value))


    except Exception as e:
        #print(f"Error parsing section: {e}")
        continue
# for a in unique_cons:
#     print(f'\'{a}\':(),')

#print(f'The number of ardupilot parameter is {parameters.count()}')
#for param in parameters:
#    print(f"Parameter_Name: {param[0]}, Increment: {param[1]}, Range_Min: {param[2]}, Range_Max: {param[3]}, Value: {param[4]}")


with open("ap-rover-v470.csv", "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Parameter_Name", "Increment", "Range_Min", "Range_Max", "Value"])
    writer.writerows(parameters)
