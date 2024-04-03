import time
from yamspy import MSPy
from yamspy.inav_modes import modes_id
from argparse import ArgumentParser
import json

def deserialise_modes(buf):
    PERM_ARM = 0
    MAX_MODE_ACTIVATION_CONDITION_COUNT = 40
    i = 0
    mranges = []
    while i < len(buf) and len(mranges) < MAX_MODE_ACTIVATION_CONDITION_COUNT:
        if buf[i+3] != 0:
            invalid = (buf[0] == PERM_ARM and (buf[i+3]-buf[i+2]) > 40)
            if not invalid:
                print(buf[i], buf[i+1], buf[i+2], buf[i+3])
                mranges.append((buf[i], buf[i+1], buf[i+2], buf[i+3]))
        i += 4

    mranges.sort(key=lambda x: (x[0], x[1]))
    return mranges

def get_modes(serial_port):
    mranges = []
    with MSPy(device=serial_port, baudrate=115200) as board:
        if board==1:
            print('Could not connect to board, check connection or device')
            exit(1)
        prev = time.monotonic()
        code_value = MSPy.MSPCodes['MSP_MODE_RANGES']
        boardinfo = board.basic_info(returninfo=True)
        msg_processed = False

        while not msg_processed:
            if board.send_RAW_msg(code_value, data=[]):
                dataHandler = board.receive_msg()
                
                if dataHandler['packet_error']==1:
                    raise Exception("Packet Error")

                if dataHandler['code'] == code_value:
                    msg_processed = True
                    buf = dataHandler["dataView"]
                    #print(buf)

                    PERM_ARM = 0
                    MAX_MODE_ACTIVATION_CONDITION_COUNT = 40
                    i = 0
                    while i < len(buf) and len(mranges) < MAX_MODE_ACTIVATION_CONDITION_COUNT:
                        if buf[i+3] != 0:
                            invalid = (buf[0] == PERM_ARM and (buf[i+3]-buf[i+2]) > 40)
                            if not invalid:
                                mranges.append((buf[i], buf[i+1], buf[i+2], buf[i+3]))
                        i += 4
                    return (boardinfo,mranges)

if __name__ == '__main__':
    parser = ArgumentParser(description='Get modes configuration from FC')
    parser.add_argument('--serialport', action='store', default="/dev/serial0", help='serial port')
    arguments = parser.parse_args()
    serial_port = arguments.serialport
    boardinfo, moderanges = get_modes(serial_port)
    if len(moderanges)==0:
        print("Error: No mode settings returned")
        exit(1)
    channels = [[]] * 16
    jsonmodes = {"board_info": boardinfo}

    for i in moderanges:
        modename = list(modes_id.keys())[list(modes_id.values()).index(i[0])]
        ch = i[1]
        valmin = 900+(i[2]*25)
        valmax = 900+(i[3]*25)
        print("ID: {}\t{:<16s}:\t{} (channel {})\t= {} to {}".format(i[0], modename,ch,ch+5,valmin,valmax))
        #channels[ch+4].append([i[0], valmin, valmax])
        jsonmodes[modename] = [ch+5,[valmin, valmax]]

    with open("modes.json", "w+") as f:
        json.dump(jsonmodes, f, sort_keys=True, indent=4)