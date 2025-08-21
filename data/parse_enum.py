import csv
import re
from bs4 import BeautifulSoup


def parse_enum_html(file_path, output_csv):
    with open(file_path, 'r', encoding='utf-8') as file:
        soup = BeautifulSoup(file, 'html.parser')

    with open(output_csv, 'w', newline='', encoding='utf-8') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Enum_Name', 'Value'])

        for h3 in soup.find_all('h3'):
            enum_name = h3.text.strip()  # get param nane (Enum_Name)

            table = h3.find_next('table')
            if table:
                values = []
                for row in table.find_all('tr')[1:]:  # traversal
                    cols = row.find_all('td')
                    if len(cols) >= 2:
                        value = re.search(r'\d+', cols[0].text.strip()).group(0)
                        values.append(value)

                if values:
                    csvwriter.writerow([enum_name, f"[{', '.join(values)}]"])


parse_enum_html('enum.html', 'enum_values.csv')
