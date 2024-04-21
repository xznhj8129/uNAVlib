
from unavlib.enums.msp_codes import MSPCodes
from unavlib.modules.process import processMSP

funcs = []
for key, value in vars(processMSP).items():
    if key.startswith('process_'):
        funcs.append(key)

all_codes = MSPCodes.keys()
for code in all_codes:
    inthere = False
    for func in funcs:
        if code in func:
            inthere = True
            break
    if not inthere:
        print(code)