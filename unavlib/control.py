import threading
from collections import deque
from itertools import cycle
import argparse

# import asyncio

from . import mspy
from . import msp_ctrl
from . import msp_codes
from . import msp_vars

# This order is the important bit: it will depend on how your flight controller is configured.
# Below it is considering the flight controller is set to use AETR.
# The names here don't really matter, they just need to match what is used for the channel_index dictionary.
# In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes ch5, ch6...
channel_order = ['roll', 'pitch', 'throttle', 'yaw', 'ch5', 'ch6', 'ch7', 'ch8', 'ch9', 'ch10', 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16', 'ch17', 'ch18']

class unav_controller():
    def __init__(self, msp_override_channels, device, baudrate=115200, trials=1, use_tcp=False, use_proxy=False):
        self.channels = [0] * 18
        self.serial_port = device
        self.pwm_range = [1000,2000]
        self.modes = {}
        self.mode_range = [900,2100]
        self.mode_increments = 25
        self.pwm_midpoint = mean(self.pwm_range)
        self.msp_override = True
        self.msp_override_chs = msp_override_channels
        self.mspy_min_time_between_writes = 1/100
        self.mspy_loglevel='INFO'
        self.mspy_timeout=1/100
        self.mspy_trials=1
        self.mspy_logfilename='MSPy.log' 
        self.mspy_logfilemode='a'
        self.board = None

    def load_modes_config_file(self,mode_config_file):
        with open(modes_config_file,"r") as file:
            cfg = json.loads(file.read())
        del cfg["board_info"]
        self.modes = cfg

    def get_modes(self):
        for i in self.modes:
            print(i)

    def toggle_mode(self,mode):
        modemean = mean(self.modes[mode][1])
        if self.channels[ self.modes[mode][0]-1 ] == modemean:
            self.channels[ self.modes[mode][0]-1 ] = 0
        else:
            self.channels[ self.modes[mode][0]-1 ] = modemean
        print(channels) #debug just making sure

    def pwm_clamp(self, n):
        return max(min(self.pwm_range[0], n), self.pwm_range[1])

    def set_rc_channel(ch, value):
        if self.msp_override and ch in self.msp_override_chs:
            self.channel[channel_order.index(ch)] = value
    
    def control_loop(self): 
        while True:

            # Max periods for:
            CTRL_LOOP_TIME = 1/100
            SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

            NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

            print("\nConnecting to the FC...")

            with MSPy(device=self.serial_port, loglevel='WARNING', baudrate=115200) as board:
                self.board = board
                print("\nConnected")

                average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

                # It's necessary to send some messages or the RX failsafe will be activated
                # and it will not be possible to arm.
                command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                                'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                                'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

                if board.INAV:
                    command_list.append('MSPV2_INAV_ANALOG')
                    command_list.append('MSP_VOLTAGE_METER_CONFIG')

                for msg in command_list: 
                    if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                if board.INAV:
                    cellCount = board.BATTERY_STATE['cellCount']
                else:
                    cellCount = 0 # MSPV2_INAV_ANALOG is necessary

                min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
                warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
                max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

                print("\napiVersion: {}".format(board.CONFIG['apiVersion']))
                print("\nflightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
                print("\nflightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
                print("\nboardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
                print("\nboardName: {}".format(board.CONFIG['boardName']))
                print("\nname: {}".format(board.CONFIG['name']))

                slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

                last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
                while True:
                    start_time = time.time()

                    # 
                    # GET USER INPUTS HERE
                    #
                    
                    #
                    # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                    #
                    if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        # Send the RC channel values to the FC
                        if board.send_RAW_RC(channels): #[channel_index[ki] for ki in channel_order]
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)

                    #
                    # SLOW MSG processing (user GUI)
                    #


                    if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                        last_slow_msg_time = time.time()

                        next_msg = next(slow_msgs) # circular list

                        # Read info from the FC
                        if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                            
                        if next_msg == 'MSP_ANALOG':
                            voltage = board.ANALOG['voltage']
                            voltage_msg = ""
                            if min_voltage < voltage <= warn_voltage:
                                voltage_msg = "LOW BATT WARNING"
                            elif voltage <= min_voltage:
                                voltage_msg = "ULTRA LOW BATT!!!"
                            elif voltage >= max_voltage:
                                voltage_msg = "VOLTAGE TOO HIGH"

                            print("Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                            print(voltage_msg)

                        elif next_msg == 'MSP_STATUS_EX':
                            ARMED = board.bit_check(board.CONFIG['mode'],0)
                            print("ARMED: {}".format(ARMED))

                            print("armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))

                            print("cpuload: {}".format(board.CONFIG['cpuload']))
                            print("cycleTime: {}".format(board.CONFIG['cycleTime']))

                            print("mode: {}".format(board.CONFIG['mode']))

                            print("Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))


                        elif next_msg == 'MSP_MOTOR':
                            print("Motor Values: {}".format(board.MOTOR_DATA))

                        elif next_msg == 'MSP_RC':
                            print("RC Channels Values: {}".format(board.RC['channels']))
                            if board.RC['channels'][5] >1300 and board.RC['channels'][5] <1800  and flightmode == 1:
                                flightmode = 2
                                print('\n########## MISSION MODE ############')

                        #print("GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*900,  1/(sum(average_cycle)/len(average_cycle))))
                        

                    end_time = time.time()
                    last_cycleTime = end_time-start_time
                    if (end_time-start_time)<CTRL_LOOP_TIME:
                        time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                        
                    average_cycle.append(end_time-start_time)
                    average_cycle.popleft()

                    time.sleep(0.1)


if __name__ == '__main__':
    parser = ArgumentParser(description='MSP Flight Control')
    parser.add_argument('--serialport', action='store', required=True, help='serial port')
    arguments = parser.parse_args()
    print(arguments)


                
