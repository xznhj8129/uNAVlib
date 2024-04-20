import os
import re

def find_number_after_whitespace(text):
    pattern = r'\s+(\d+)'
    match = re.search(pattern, text)
    if match:
        number = int(match.group(1)) 
        position = match.start(1)
        return position
    else:
        return -1

src = ["../inav/src/main/msp/msp_protocol.h",
        "../inav/src/main/msp/msp_protocol_v2_common.h",
        "../inav/src/main/msp/msp_protocol_v2_inav.h",
        "../inav/src/main/msp/msp_protocol_v2_sensor.h"]

cmds=[]
for f in src:
    with open(f,"r") as file:
        a = file.read()
    a = a.split('\n')
    for i in range(len(a)):
        if a[i].startswith("#define"):
            q = a[i][7:]
            j = find_number_after_whitespace(q)
            k = q.find('//')
            if k>=0:
                b = q[:j].strip()
                c = q[:k][j:].strip()
                d = q[k:][2:].strip()
            else:
                b = q[:j].strip()
                c = q[j:].strip()
                d = ""
            if '(' in b or '"' in b or not b.startswith("MSP"):
                continue
            #print(b,c,d)
            cmds.append((b,eval(c),d))

txt = "MSPCodes = {\n"
for i in cmds:
    if len(i[2])>0:
        txt+='\t"{:<36s}:\t{},\t# {}\n'.format(i[0]+'"',i[1],i[2])
    else:
        txt+='\t"{:<36s}:\t{},\n'.format(i[0]+'"',i[1])
txt+= "}"
print(txt)
with open("../enums/msp_codes.py","w+") as file:
    file.write(txt)