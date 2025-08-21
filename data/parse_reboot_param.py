from bs4 import BeautifulSoup


def parse_html_for_reboot_required(html_content, output_file):

    soup = BeautifulSoup(html_content, 'html.parser')
    seen_params = set()
    reboot_required_params = []

    for section in soup.find_all('section'):

        section_id = section.get('id', '')
        h3_tag = section.find('h3')
        if h3_tag:
            param_name = h3_tag.text.split(":")[0].strip()

            if "Reboot required" in section.get_text():
                if '(' in param_name:
                    param_name = param_name.split('(')[0].strip()
                if param_name not in seen_params:
                    reboot_required_params.append(param_name)
                    seen_params.add(param_name)

    with open(output_file, 'w') as file:
        for param in reboot_required_params:
            file.write(param + '\n')

with open('rover470/RoverParameterV470.html', 'r', encoding='utf-8') as file:
    html_content = file.read()

output_file = 'ap-rover-rebootrequired.txt'
parse_html_for_reboot_required(html_content, output_file)

