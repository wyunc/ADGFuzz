import json
import csv

def parse_px4_json(json_path, csv_path):
    with open(json_path, 'r') as f:
        raw_data = json.load(f)

    if isinstance(raw_data, dict):
        # {"parameters": [...]}
        for key, value in raw_data.items():
            if isinstance(value, list) and all(isinstance(p, dict) for p in value):
                data = value
                break
        else:
            raise ValueError("error JSON format")
    elif isinstance(raw_data, list):
        data = raw_data
    else:
        raise ValueError("unknown JSON structure")

    rows = []

    for param in data:
        name = param.get('name', 'N')
        inc = param.get('increment', 'N')
        rmin = param.get('min', 'N')
        rmax = param.get('max', 'N')
        dtype = param.get('type', 'N')
        default = param.get('default', 'N')
        units = param.get('units', 'N')

        value_field = 'N'
        if 'bitmask' in param:
            inc = 'B'
            value_field = [str(v.get('index', v.get('value', ''))) for v in param['bitmask']]
        elif 'values' in param:
            inc = 'V'
            value_field = [str(v.get('index', v.get('value', ''))) for v in param['values']]

        rows.append({
            'Parameter_Name': name,
            'Increment': inc,
            'Range_Min': rmin,
            'Range_Max': rmax,
            'Value': str(value_field) if isinstance(value_field, list) else value_field,
            'Type': dtype,
            'Default': default,
            'Units': units
        })

    fieldnames = ['Parameter_Name', 'Increment', 'Range_Min', 'Range_Max', 'Value', 'Type', 'Default', 'Units']

    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

parse_px4_json('px4/parameters.json', 'px4-parameters.csv')