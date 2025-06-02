
bin_type_map = {
    "enum": "B",
    "uint8_t": "B",
    "uint16_t": "H",
    "uint32_t": "I",
    "uint64_t": "Q",
    "int8_t": "b",
    "int16_t": "h",
    "int32_t": "i",
    "int64_t": "q",
    "float": "f",
    "double": "d",
    "char": "c",
    "bool": "?"
}

msp_structs_inav = {
    # MSPv1 Core & Versioning Commands (0-5)
    1: "<BBB",  # MSP_API_VERSION (Out)
    2: "<4s",  # MSP_FC_VARIANT (Out)
    3: "<BBB",  # MSP_FC_VERSION (Out)
    4: "<4sHBBD",  # MSP_BOARD_INFO (Out) # Followed by targetName char[targetNameLength]
    5: "<11s8s7s",  # MSP_BUILD_INFO (Out)

    # MSPv1 INAV Configuration Commands (6-24)
    6: "<BHHBBHBBBBB",  # MSP_INAV_PID (Out)
    7: "<BHHBBHBBBBB",  # MSP_SET_INAV_PID (In)
    10: "",  # MSP_NAME (Out) # VAR_LENGTH_STRING (craftName)
    11: "",  # MSP_SET_NAME (In) # VAR_LENGTH_STRING (craftName, 1 to MAX_NAME_LENGTH)
    12: "<BHHHHBBH",  # MSP_NAV_POSHOLD (Out)
    13: "<BHHHHBBH",  # MSP_SET_NAV_POSHOLD (In)
    14: "<BHHHHHHHHHHHHH",  # MSP_CALIBRATION_DATA (Out)
    15: "<HHHHHHHHHHHHH",  # MSP_SET_CALIBRATION_DATA (In) (13 * H)
    16: "<HHHHHBB",  # MSP_POSITION_ESTIMATION_CONFIG (Out)
    17: "<HHHHHBB",  # MSP_SET_POSITION_ESTIMATION_CONFIG (In)
    18: "<B",  # MSP_WP_MISSION_LOAD (In)
    19: "<B",  # MSP_WP_MISSION_SAVE (In)
    20: "<BBBB",  # MSP_WP_GETINFO (Out)
    21: "<HBBBBBH HHHHHH",  # MSP_RTH_AND_LAND_CONFIG (Out)
    22: "<HBBBBBH HHHHHH",  # MSP_SET_RTH_AND_LAND_CONFIG (In)
    23: "<HHHBBBBH",  # MSP_FW_CONFIG (Out)
    24: "<HHHBBBBH",  # MSP_SET_FW_CONFIG (In)

    # MSPv1 Cleanflight/Betaflight/INAV Feature Commands (34-58)
    34: "<BBBB",  # MSP_MODE_RANGES (Out) # One range, Repeated MAX_MODE_ACTIVATION_CONDITION_COUNT times
    35: "<BBBBB",  # MSP_SET_MODE_RANGE (In)
    36: "<I",  # MSP_FEATURE (Out)
    37: "<I",  # MSP_SET_FEATURE (In)
    38: "<HHH",  # MSP_BOARD_ALIGNMENT (Out)
    39: "<HHH",  # MSP_SET_BOARD_ALIGNMENT (In)
    40: "<HHBH",  # MSP_CURRENT_METER_CONFIG (Out)
    41: "<HHBH",  # MSP_SET_CURRENT_METER_CONFIG (In)
    42: "<B",  # MSP_MIXER (Out)
    43: "<B",  # MSP_SET_MIXER (In)
    44: "<BHHHBH HBBH BIBBB",  # MSP_RX_CONFIG (Out)
    45: "<BHHHBH HBBH BIBBB",  # MSP_SET_RX_CONFIG (In)
    46: "<HBB",  # MSP_LED_COLORS (Out) # One color, Repeated LED_CONFIGURABLE_COLOR_COUNT times
    47: "<HBB",  # MSP_SET_LED_COLORS (In) # One color, Repeated LED_CONFIGURABLE_COLOR_COUNT times payload
    48: "<I",  # MSP_LED_STRIP_CONFIG (Out) # One LED config, Repeated LED_MAX_STRIP_LENGTH times
    49: "<BI",  # MSP_SET_LED_STRIP_CONFIG (In)
    50: "<B",  # MSP_RSSI_CONFIG (Out)
    51: "<B",  # MSP_SET_RSSI_CONFIG (In)
    52: "<BBBBBB",  # MSP_ADJUSTMENT_RANGES (Out) # One range, Repeated MAX_ADJUSTMENT_RANGE_COUNT times
    53: "<BBBBBBB",  # MSP_SET_ADJUSTMENT_RANGE (In)
    54: "",  # MSP_CF_SERIAL_CONFIG (Out) # NOT_IMPLEMENTED
    55: "",  # MSP_SET_CF_SERIAL_CONFIG (In) # NOT_IMPLEMENTED
    56: "<BBBB",  # MSP_VOLTAGE_METER_CONFIG (Out)
    57: "<BBBB",  # MSP_SET_VOLTAGE_METER_CONFIG (In)
    58: "<I",  # MSP_SONAR_ALTITUDE (Out)

    # MSPv1 Baseflight/INAV Commands (64-99, plus others)
    64: "<18B",  # MSP_RX_MAP (Out) # Assuming MAX_MAPPABLE_RX_INPUTS = 18
    65: "<18B",  # MSP_SET_RX_MAP (In) # Assuming MAX_MAPPABLE_RX_INPUTS = 18
    68: "",  # MSP_REBOOT (Out) # No payload
    70: "<BIII",  # MSP_DATAFLASH_SUMMARY (Out)
    71: "<I",  # MSP_DATAFLASH_READ (In/Out) # Reply fixed part, Followed by data uint8_t[]
    72: "",  # MSP_DATAFLASH_ERASE (In) # No payload
    73: "<H",  # MSP_LOOP_TIME (Out)
    74: "<H",  # MSP_SET_LOOP_TIME (In)
    75: "<BBHB HBB HHHHH B",  # MSP_FAILSAFE_CONFIG (Out)
    76: "<BBHB HBB HHHHH B",  # MSP_SET_FAILSAFE_CONFIG (In)
    79: "<BBBII",  # MSP_SDCARD_SUMMARY (Out)
    80: "<BBBB",  # MSP_BLACKBOX_CONFIG (Out) # Legacy
    81: "",  # MSP_SET_BLACKBOX_CONFIG (In) # NOT_IMPLEMENTED
    82: "",  # MSP_TRANSPONDER_CONFIG (Out) # NOT_IMPLEMENTED
    83: "",  # MSP_SET_TRANSPONDER_CONFIG (In) # NOT_IMPLEMENTED
    84: "<BBBBHHHHH",  # MSP_OSD_CONFIG (Out) # Fixed part, Followed by itemPositions uint16_t[OSD_ITEM_COUNT]
    85: "<BBBBHHHHH",  # MSP_SET_OSD_CONFIG (In) # Format 1 (General Config, full optional fields)
    86: "",  # MSP_OSD_CHAR_READ (Out) # NOT_IMPLEMENTED
    87: "<B",  # MSP_OSD_CHAR_WRITE (In) # Smallest variant (address uint8_t), Followed by variable charData[]. Use H for 16bit address.
    88: "<BBBBBBBBBBB",  # MSP_VTX_CONFIG (Out)
    89: "<HBBBHBBHBBB",  # MSP_SET_VTX_CONFIG (In) # Largest defined payload
    90: "<BBBBHHB",  # MSP_ADVANCED_CONFIG (Out)
    91: "<BBBBHHB",  # MSP_SET_ADVANCED_CONFIG (In)
    92: "<BHHHHHHHHHHH",  # MSP_FILTER_CONFIG (Out)
    93: "<BHHHHHHHHHHH",  # MSP_SET_FILTER_CONFIG (In) # Largest payload (22 bytes)
    94: "<HHHBBBBHBHH",  # MSP_PID_ADVANCED (Out)
    95: "<HHHBBBBHBHH",  # MSP_SET_PID_ADVANCED (In)
    96: "<BBBBBB",  # MSP_SENSOR_CONFIG (Out)
    97: "<BBBBBB",  # MSP_SET_SENSOR_CONFIG (In)
    98: "",  # MSP_SPECIAL_PARAMETERS (Out) # NOT_IMPLEMENTED
    99: "",  # MSP_SET_SPECIAL_PARAMETERS (In) # NOT_IMPLEMENTED

    # MSPv1 MultiWii Original Commands (100-127, 130)
    # 100: "" # MSP_STATUS (Old, Obsolete)
    101: "<HHHIB",  # MSP_STATUS (Out)
    102: "<hhhhhhhhh",  # MSP_RAW_IMU (Out)
    103: "<8h",  # MSP_SERVO (Out) # Assuming MAX_SUPPORTED_SERVOS=8 for this context
    104: "<8H",  # MSP_MOTOR (Out)
    105: "<H",  # MSP_RC (Out) # One channel, Repeated rxRuntimeConfig.channelCount times
    106: "<BBIIHHHH",  # MSP_RAW_GPS (Out)
    107: "<HHB",  # MSP_COMP_GPS (Out)
    108: "<hhh",  # MSP_ATTITUDE (Out)
    109: "<IhI",  # MSP_ALTITUDE (Out)
    110: "<BHHh",  # MSP_ANALOG (Out)
    111: "<BBBBBBBBHB",  # MSP_RC_TUNING (Out)
    113: "<Q",  # MSP_ACTIVEBOXES (Out) # boxBitmask_t is uint64_t
    114: "<HHHHHBBBBBHBBBB",  # MSP_MISC (Out)
    116: "",  # MSP_BOXNAMES (Out) # VAR_LENGTH_STRING (boxNamesString)
    117: "",  # MSP_PIDNAMES (Out) # VAR_LENGTH_STRING (pidNamesString)
    118: "<BBIIIHHHB",  # MSP_WP (In/Out) # Reply structure
    119: "<B",  # MSP_BOXIDS (Out) # One ID, Repeated per box ID
    120: "<HHHBBBBI",  # MSP_SERVO_CONFIGURATIONS (Out) # One servo, Repeated MAX_SUPPORTED_SERVOS times
    121: "<BBBBBH",  # MSP_NAV_STATUS (Out)
    122: "",  # MSP_NAV_CONFIG # NOT_IMPLEMENTED
    124: "<HHH",  # MSP_3D (Out)
    125: "<BBBH",  # MSP_RC_DEADBAND (Out)
    126: "<BBBB",  # MSP_SENSOR_ALIGNMENT (Out)
    127: "<BBB",  # MSP_LED_STRIP_MODECOLOR (Out) # One entry, Repeated many times
    130: "<BHBHhBH",  # MSP_BATTERY_STATE (Out)
    137: "",  # MSP_VTXTABLE_BAND (In/Out) # NOT_IMPLEMENTED (handler missing)
    138: "<BHB",  # MSP_VTXTABLE_POWERLEVEL (In/Out) # Reply fixed part, Followed by label char[labelLength]

    # MSPv1 MultiWii Original Input Commands (200-221)
    200: "<H",  # MSP_SET_RAW_RC (In) # One channel, Repeated channelCount times
    201: "<BBIIHHH",  # MSP_SET_RAW_GPS (In)
    203: "",  # MSP_SET_BOX (In) # NOT_IMPLEMENTED
    204: "<BBBBBBBBHB",  # MSP_SET_RC_TUNING (In) # Full 11 byte payload
    205: "",  # MSP_ACC_CALIBRATION (In) # No payload
    206: "",  # MSP_MAG_CALIBRATION (In) # No payload
    207: "<HHHHHBBBBBHBBBB",  # MSP_SET_MISC (In)
    208: "",  # MSP_RESET_CONF (In) # No payload
    209: "<BBIIIHHHB",  # MSP_SET_WP (In)
    210: "<B",  # MSP_SELECT_SETTING (In)
    211: "<H",  # MSP_SET_HEAD (In)
    212: "<BHHHBBBBI",  # MSP_SET_SERVO_CONFIGURATION (In)
    214: "<8H",  # MSP_SET_MOTOR (In)
    215: "",  # MSP_SET_NAV_CONFIG (In) # NOT_IMPLEMENTED
    217: "<HHH",  # MSP_SET_3D (In)
    218: "<BBBH",  # MSP_SET_RC_DEADBAND (In)
    219: "",  # MSP_SET_RESET_CURR_PID (In) # No payload
    220: "<BBBB",  # MSP_SET_SENSOR_ALIGNMENT (In)
    221: "<BBB",  # MSP_SET_LED_STRIP_MODECOLOR (In)

    # MSPv1 System & Debug Commands (239-254)
    239: "",  # MSP_SET_ACC_TRIM (In) # NOT_IMPLEMENTED
    240: "",  # MSP_ACC_TRIM (Out) # NOT_IMPLEMENTED
    241: "<BBHBBBB",  # MSP_SERVO_MIX_RULES (Out) # One rule, Repeated MAX_SERVO_RULES times
    242: "<BBBHBHB",  # MSP_SET_SERVO_MIX_RULE (In)
    245: "<B",  # MSP_SET_PASSTHROUGH (In/Out) # Reply structure
    246: "<iH",  # MSP_RTC (Out)
    247: "<iH",  # MSP_SET_RTC (In)
    250: "",  # MSP_EEPROM_WRITE (In) # No payload
    253: "",  # MSP_DEBUGMSG (Out) # VAR_LENGTH_STRING (Message Text)
    254: "<4H",  # MSP_DEBUG (Out)
    # 255: "" # MSP_V2_FRAME (Indicator, not a message with payload struct)

    # MSPv1 Extended/INAV Commands (150-166)
    150: "<HHHIBHHB",  # MSP_STATUS_EX (Out)
    151: "<9B",  # MSP_SENSOR_STATUS (Out)
    160: "<III",  # MSP_UID (Out)
    164: "<BBBB",  # MSP_GPSSVINFO (Out)
    166: "<HIIIHHH",  # MSP_GPSSTATISTICS (Out)
    187: "<BB",  # MSP_TX_INFO (Out)
    186: "<B",  # MSP_SET_TX_INFO (In)

    # MSPv2 Common Commands (0x1000 Range / 4096+)
    4097: "<hB",  # MSP2_COMMON_TZ (Out)
    4098: "<hB",  # MSP2_COMMON_SET_TZ (In) # Format 2 (Offset + DST)
    4099: "",  # MSP2_COMMON_SETTING (In/Out) # Reply: VAR_LENGTH_BYTE_ARRAY (settingValue)
    4100: "",  # MSP2_COMMON_SET_SETTING (In) # TOO_VARIABLE_FOR_FIXED_STRUCT
    4101: "<HHHH",  # MSP2_COMMON_MOTOR_MIXER (Out) # One motor, repeated MAX_SUPPORTED_MOTORS times for profile 1
    4102: "<BHHHH",  # MSP2_COMMON_SET_MOTOR_MIXER (In)
    4103: "",  # MSP2_COMMON_SETTING_INFO (In/Out) # TOO_VARIABLE_FOR_FIXED_STRUCT (Reply is highly variable)
    4104: "<HHH",  # MSP2_COMMON_PG_LIST (In/Out) # Reply: One PGN entry, Repeated per PGN found
    4105: "<BIBBBB",  # MSP2_COMMON_SERIAL_CONFIG (Out) # One port config, Repeated per serial port
    4106: "<BIBBBB",  # MSP2_COMMON_SET_SERIAL_CONFIG (In) # One port config, Repeated per serial port
    4107: "<BBIIIHHB",  # MSP2_COMMON_SET_RADAR_POS (In)
    4108: "",  # MSP2_COMMON_SET_RADAR_ITD (In) # NOT_IMPLEMENTED
    4109: "<BBBBBBb",  # MSP2_COMMON_SET_MSP_RC_LINK_STATS (In) # No reply
    4110: "<BHH4s6s",  # MSP2_COMMON_SET_MSP_RC_INFO (In) # No reply

    # MSPv2 INAV Specific Commands (0x2000 Range / 8192+)
    8192: "<HHHHBIQB",  # MSP2_INAV_STATUS (Out)
    8193: "<Bhhhh",  # MSP2_INAV_OPTICAL_FLOW (Out)
    8194: "<BHHIIIIBH",  # MSP2_INAV_ANALOG (Out)
    8195: "<HHHHHBBBBHHBBHHHHIIIB",  # MSP2_INAV_MISC (Out)
    8196: "<HHHHHBBBBHHBBHHHHIIIB",  # MSP2_INAV_SET_MISC (In)
    8197: "<HBBHHHHHHIIIB",  # MSP2_INAV_BATTERY_CONFIG (Out)
    8198: "<HBBHHHHHHIIIB",  # MSP2_INAV_SET_BATTERY_CONFIG (In)
    8199: "<BBBHBBBBBBBBBB",  # MSP2_INAV_RATE_PROFILE (Out)
    8200: "<BBBHBBBBBBBBBB",  # MSP2_INAV_SET_RATE_PROFILE (In)
    8201: "<I",  # MSP2_INAV_AIR_SPEED (Out)
    8202: "<B",  # MSP2_INAV_OUTPUT_MAPPING (Out) # One timer usage, Repeated per Motor/Servo timer
    8203: "<HHHBH HHB",  # MSP2_INAV_MC_BRAKING (Out)
    8204: "<HHHBH HHB",  # MSP2_INAV_SET_MC_BRAKING (In)
    8205: "<BB",  # MSP2_INAV_OUTPUT_MAPPING_EXT (Out) # One timer, Repeated per Motor/Servo timer
    8206: "<BB",  # MSP2_INAV_TIMER_OUTPUT_MODE (In/Out) # Reply for one timer, or List All: Repeated HARDWARE_TIMER_DEFINITION_COUNT times
    8207: "<BB",  # MSP2_INAV_SET_TIMER_OUTPUT_MODE (In)
    8461: "<BIB",  # MSP2_INAV_OUTPUT_MAPPING_EXT2 (Out) # One timer, Repeated per Motor/Servo timer
    8208: "<BBBBBHBB",  # MSP2_INAV_MIXER (Out)
    8209: "<BBBBBHBB",  # MSP2_INAV_SET_MIXER (In)
    8210: "<BB",  # MSP2_INAV_OSD_LAYOUTS (In/Out) # Reply for Get Counts
    8211: "<BBH",  # MSP2_INAV_OSD_SET_LAYOUT_ITEM (In)
    8212: "<BHHHHHhhBHHHHHH",  # MSP2_INAV_OSD_ALARMS (Out)
    8213: "<BHHHHHhhBHHHH",  # MSP2_INAV_OSD_SET_ALARMS (In)
    8214: "<9B",  # MSP2_INAV_OSD_PREFERENCES (Out)
    8215: "<9B",  # MSP2_INAV_OSD_SET_PREFERENCES (In)
    8216: "<B",  # MSP2_INAV_SELECT_BATTERY_PROFILE (In)
    8217: "<8I",  # MSP2_INAV_DEBUG (Out) # Assuming DEBUG32_VALUE_COUNT=8
    8218: "<BBHHI",  # MSP2_BLACKBOX_CONFIG (Out)
    8219: "<BHHI",  # MSP2_SET_BLACKBOX_CONFIG (In)
    8220: "<BQHHB16s",  # MSP2_INAV_TEMP_SENSOR_CONFIG (Out) # One sensor (TEMPERATURE_LABEL_LEN=16), Repeated MAX_TEMP_SENSORS times
    8221: "<BQHHB16s",  # MSP2_INAV_SET_TEMP_SENSOR_CONFIG (In) # One sensor (TEMPERATURE_LABEL_LEN=16), for setting all.
    8222: "<h",  # MSP2_INAV_TEMPERATURES (Out) # One sensor, Repeated MAX_TEMP_SENSORS times
    8223: "<HHHHBIhhhBBBBB",  # MSP_SIMULATOR (In/Out) # Reply fixed part, Followed by osdRleData uint8_t[]
    8224: "<BBHBB",  # MSP2_INAV_SERVO_MIXER (Out) # One rule, Repeated MAX_SERVO_RULES times
    8225: "<BBBHBB",  # MSP2_INAV_SET_SERVO_MIXER (In)
    8226: "<BBBBI BIB",  # MSP2_INAV_LOGIC_CONDITIONS (Out) # One condition, Repeated MAX_LOGIC_CONDITIONS times
    8227: "<BBBBBI BIB",  # MSP2_INAV_SET_LOGIC_CONDITIONS (In)
    8228: "",  # MSP2_INAV_GLOBAL_FUNCTIONS (Out) # NOT_IMPLEMENTED
    8229: "",  # MSP2_INAV_SET_GLOBAL_FUNCTIONS (In) # NOT_IMPLEMENTED
    8230: "<I",  # MSP2_INAV_LOGIC_CONDITIONS_STATUS (Out) # One status, Repeated MAX_LOGIC_CONDITIONS times
    8231: "<I",  # MSP2_INAV_GVAR_STATUS (Out) # One GVAR, Repeated MAX_GLOBAL_VARIABLES times
    8232: "<BBI BIHHHH",  # MSP2_INAV_PROGRAMMING_PID (Out) # One PID, Repeated MAX_PROGRAMMING_PID_COUNT times
    8233: "<BBBI BIHHHH",  # MSP2_INAV_SET_PROGRAMMING_PID (In)
    8234: "<I",  # MSP2_INAV_PROGRAMMING_PID_STATUS (Out) # One status, Repeated MAX_PROGRAMMING_PID_COUNT times
    8240: "<BBBB",  # MSP2_PID (Out) # One PID set, Repeated PID_ITEM_COUNT times
    8241: "<BBBB",  # MSP2_SET_PID (In) # One PID set, Repeated PID_ITEM_COUNT times
    8242: "",  # MSP2_INAV_OPFLOW_CALIBRATION (In) # No payload
    8243: "<I",  # MSP2_INAV_FWUPDT_PREPARE (In)
    8244: "",  # MSP2_INAV_FWUPDT_STORE (In) # VAR_LENGTH_BYTE_ARRAY (firmwareChunk)
    8245: "<B",  # MSP2_INAV_FWUPDT_EXEC (In)
    8246: "",  # MSP2_INAV_FWUPDT_ROLLBACK_PREPARE (In) # No payload
    8247: "",  # MSP2_INAV_FWUPDT_ROLLBACK_EXEC (In) # No payload
    8248: "<BBII",  # MSP2_INAV_SAFEHOME (In/Out) # Reply structure
    8249: "<BBII",  # MSP2_INAV_SET_SAFEHOME (In)
    8250: "<IIBB",  # MSP2_INAV_MISC2 (Out)
    8251: "<BBBBI BIB",  # MSP2_INAV_LOGIC_CONDITIONS_SINGLE (In/Out) # Reply structure
    8256: "<I",  # MSP2_INAV_ESC_RPM (Out) # One ESC RPM, Repeated getMotorCount() times
    8257: "<B",  # MSP2_INAV_ESC_TELEM (Out) # Fixed part (motorCount), followed by motorCount * (HHHIRHBB) for escSensorData_t
    8264: "<HBBBBBB",  # MSP2_INAV_LED_STRIP_CONFIG_EX (Out) # One ledConfig_t, Repeated LED_MAX_STRIP_LENGTH times
    8265: "<BHBBBBBB",  # MSP2_INAV_SET_LED_STRIP_CONFIG_EX (In)
    8266: "<BIIBhhB",  # MSP2_INAV_FW_APPROACH (In/Out) # Reply structure
    8267: "<BIIBhhB",  # MSP2_INAV_SET_FW_APPROACH (In)
    8272: "",  # MSP2_INAV_GPS_UBLOX_COMMAND (In) # VAR_LENGTH_BYTE_ARRAY (ubxCommand, >=8 bytes)
    8288: "<6B",  # MSP2_INAV_RATE_DYNAMICS (Out)
    8289: "<6B",  # MSP2_INAV_SET_RATE_DYNAMICS (In)
    8304: "<BHBBBBBBBB",  # MSP2_INAV_EZ_TUNE (Out)
    8305: "<BHBBBBBBBB",  # MSP2_INAV_EZ_TUNE_SET (In) # Max 11 byte payload (includes snappiness)
    8320: "<B",  # MSP2_INAV_SELECT_MIXER_PROFILE (In)
    8336: "<BBII",  # MSP2_ADSB_VEHICLE_LIST (Out) # Fixed part, Followed by (9sIIIHHBBB) repeated maxVehicles times (ADSB_CALL_SIGN_MAX_LENGTH=9)
    8448: "<BBB",  # MSP2_INAV_CUSTOM_OSD_ELEMENTS (Out)
    8449: "<BHBHBHBH16s", # MSP2_INAV_CUSTOM_OSD_ELEMENT (In/Out) # Reply for one element (assuming CUSTOM_ELEMENTS_PARTS=3, OSD_CUSTOM_ELEMENT_TEXT_SIZE-1=16)
    8450: "<BBHBHBHBH16s", # MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS (In) # Payload for one element including index
    8704: "<HHHB",  # MSP2_INAV_SERVO_CONFIG (Out) # One servo, Repeated MAX_SUPPORTED_SERVOS times
    8705: "<BHHHB",  # MSP2_INAV_SET_SERVO_CONFIG (In)
    8720: "<BBBIIBBB",  # MSP2_INAV_GEOZONE (In/Out) # Reply structure
    8721: "<BBBIIBBB",  # MSP2_INAV_SET_GEOZONE (In)
    8722: "<BBII",  # MSP2_INAV_GEOZONE_VERTEX (In/Out) # Reply for Polygon vertex
    8723: "<BBII",  # MSP2_INAV_SET_GEOZONE_VERTEX (In) # For Polygon vertex

    # MSPv2 Betaflight Commands (0x3000 Range / 12288+)
    12288: "",  # MSP2_BETAFLIGHT_BIND (In) # No payload

    # MSPv2 Sensor Input Commands (0x1F00 Range / 7936+)
    7937: "<Bi",  # MSP2_SENSOR_RANGEFINDER (In)
    7938: "<Bii",  # MSP2_SENSOR_OPTIC_FLOW (In)
    7939: "<BHIBBHHHHiiiiiHHHBBBBB",  # MSP2_SENSOR_GPS (In)
    7940: "<BIhhh",  # MSP2_SENSOR_COMPASS (In)
    7941: "<BIfh",  # MSP2_SENSOR_BAROMETER (In)
    7942: "<BIfh",  # MSP2_SENSOR_AIRSPEED (In)
    7943: "<hhh",  # MSP2_SENSOR_HEADTRACKER (In) # Assuming Roll, Pitch, Yaw as int16_t
}

# Example of how to use it:
# import struct
#
# # For MSP_API_VERSION (Out)
# code = 1
# data_tuple = (0, 7, 0) # mspProtocolVersion, apiVersionMajor, apiVersionMinor
# packed_data = struct.pack(msp_structs[code], *data_tuple)
#
# # For MSP_WP (Reply)
# code = 118
# # Example: wp_idx=1, action=16(WAYPOINT), lat=45deg, lon=10deg, alt=100m, p1=0, p2=0, p3=0, flag=0
# data_tuple_wp = (1, 16, 450000000, 100000000, 10000, 0, 0, 0, 0)
# packed_data_wp = struct.pack(msp_structs[code], *data_tuple_wp)
# unpacked_data_wp = struct.unpack(msp_structs[code], packed_data_wp)
# assert data_tuple_wp == unpacked_data_wp