import argparse

def dict_index(d, target_value):
    return [key for key, value in d.items() if value == target_value][0]

def extract_enums_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    collecting = False
    current_enum = None
    enums = {}
    for line in lines:
        if 'typedef enum {' in line:
            # Start collecting enum entries
            collecting = True
            enum_name = line.split()[-1].strip('{')
            current_enum = "working"
            enums[current_enum] = {}
            value_counter = 0
        elif collecting:
            if '}' in line:
                # Stop collecting enum entries
                enum_name = line.split()[-1].strip('{').strip(';')
                enums[enum_name] = enums["working"]
                collecting = False
                continue
            if '=' in line:
                # Explicit value assignment
                key, value = line.split('=')
                key = key.strip()
                value = value.split(',')[0].split('//')[0].strip()
                value = value
                # Handle hexadecimal values
                base = 16 if 'x' in value else 10
                try:
                    value = int(value, base)
                except:
                    pass
                enums[current_enum][key] = value
                value_counter += 1
            else:
                # No explicit value, assume increment
                key = line.split(',')[0].split('//')[0].strip()
                enums[current_enum][key] = value_counter
                value_counter += 1

    if "working" in enums:
        del enums["working"]
    return enums

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Get modes configuration from FC')
    parser.add_argument('--file', action='store', required=True, help='serial port')
    arguments = parser.parse_args()
    extracted_enums = extract_enums_from_file(arguments.file)
    string = "enums = {\n"
    for enum_name, enum_dict in extracted_enums.items():
        string+= '    "'+enum_name+'" : {'+str(enum_dict)+'},\n'
        #string+=f'    {"enum_name"}: {enum_dict},\n'
    string=string.rstrip(',\n')
    string+='\n}'
    print(string)