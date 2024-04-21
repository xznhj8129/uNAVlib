import argparse
import json
import glob, os
import unavlib
from unavlib.modules.utils import dict_index
from unavlib.enums import base_enums

#nb: turns hex values into integers

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
            line = line.split('//')[0]
            if '}' in line:
                # Stop collecting enum entries
                enum_name = line.split()[-1].strip('{').strip(';')
                enums[enum_name] = enums["working"]
                collecting = False
                continue
            if '/*' not in line and '*' not in line and '#' not in line and '/*' not in line and len(line)>1:
                if '=' in line:
                    # Explicit value assignment
                    key, value = line.split('=')
                    key = key.strip()
                    if key=="":
                        continue
                    value = value.split(',')[0].split('//')[0].strip()
                    # Handle hexadecimal values
                    try:
                        if 'x' in value:
                            value = eval(value)
                        else:
                            value = int(value)
                    except:
                        pass
                    
                    if value in enums[current_enum].keys():
                        value = enums[current_enum][value]

                    enums[current_enum][key] = value
                    value_counter += 1
                else:
                    # No explicit value, assume increment
                    key = line.split(',')[0].split('//')[0].strip()
                    if key=="":
                        continue
                    enums[current_enum][key] = value_counter
                    value_counter += 1

    if "working" in enums:
        del enums["working"]
    if len(enums)>0:
        return enums

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate INAV enums')
    parser.add_argument('--src', action='store', required=True, help='INAV source code directory')
    arguments = parser.parse_args()
    string="# all enums available with reverse-lookup by integer by referencing R_enumName dict\n\n# imported from enums/base_enums.py\n"

    for enum_name in dir(base_enums):
        if not enum_name.startswith("__"):
            attr = getattr(base_enums, enum_name)
            if isinstance(attr, dict): 
                s = f"{enum_name} = {{\n    "
                print(s, end="")
                pretty_string = json.dumps(attr, indent=4)[1:-2].strip().replace("\n    ", "\n    ")
                s += pretty_string + "\n}\n"
                print(pretty_string, end="")
                print("\n}")
                string+=s

    string+='\n\n# extracted from source'
    for d in ['navigation','sensors']: #next(os.walk(arguments.src+'src/main/'))[1]: # we're not ready for that yet 
        #print(d)
        for f in glob.glob(f'{arguments.src}src/main/{d}/*.h'):
            extracted_enums = extract_enums_from_file(f)
            if extracted_enums:
                print('#',f)
                string += f"\n#{f}\n"
                for enum_name, enum_dict in extracted_enums.items():
                    en = enum_name.replace("_e","").replace("_t","")
                    s = f"{en} = {{\n    "
                    print(s, end="")
                    pretty_string = json.dumps(enum_dict, indent=4)[1:-2].strip().replace("\n    ", "\n    ")
                    s += pretty_string + "\n}\n"
                    print(pretty_string, end="")
                    print("\n}")
                    string+=s
    with open(os.path.dirname(unavlib.__file__)+"/enums/inav_enums.py","w+") as file:
        file.write(string)