import enum
from typing import Union, Dict, Any

# Base enums

# --- Base Enums ---
armingDisableFlagNames_BF: Any = {'NOGYRO': 0, 'FAILSAFE': 1, 'RXLOSS': 2, 'BADRX': 3, 'BOXFAILSAFE': 4, 'RUNAWAY': 5, 'CRASH': 6, 'THROTTLE': 7, 'ANGLE': 8, 'BOOTGRACE': 9, 'NOPREARM': 10, 'LOAD': 11, 'CALIB': 12, 'CLI': 13, 'CMS': 14, 'BST': 15, 'MSP': 16, 'PARALYZE': 17, 'GPS': 18, 'RESCUE SW': 19, 'RPMFILTER': 20, 'ARMSWITCH': 21}
armingDisableFlagNames_INAV: Any = {'OK_TO_ARM': 0, 'PREVENT_ARMING': 1, 'ARMED': 2, 'WAS_EVER_ARMED': 3, 'SIMULATOR_MODE': 4, 'BLOCKED_FAILSAFE_SYSTEM': 7, 'BLOCKED_UAV_NOT_LEVEL': 8, 'BLOCKED_SENSORS_CALIBRATING': 9, 'BLOCKED_SYSTEM_OVERLOADED': 10, 'BLOCKED_NAVIGATION_SAFETY': 11, 'BLOCKED_COMPASS_NOT_CALIBRATED': 12, 'BLOCKED_ACCELEROMETER_NOT_CALIBRATED': 13, 'BLOCKED_ARMING_DISABLED_ARM_SWITCH': 14, 'BLOCKED_HARDWARE_FAILURE': 15, 'BLOCKED_ARMING_DISABLED_BOXFAILSAFE': 16, 'BLOCKED_ARMING_DISABLED_BOXKILLSWITCH': 17, 'BLOCKED_ARMING_DISABLED_RC_LINK': 18, 'BLOCKED_ARMING_DISABLED_THROTTLE': 19, 'BLOCKED_ARMING_DISABLED_CLI': 20, 'BLOCKED_ARMING_DISABLED_CMS_MENU': 21, 'BLOCKED_ARMING_DISABLED_OSD_MENU': 22, 'BLOCKED_ARMING_DISABLED_ROLLPITCH_NOT_CENTERED': 23, 'BLOCKED_ARMING_DISABLED_SERVO_AUTOTRIM': 24, 'BLOCKED_ARMING_DISABLED_OOM': 25, 'BLOCKED_INVALID_SETTING': 26, 'BLOCKED_ARMING_DISABLED_PWM_OUTPUT_ERROR': 27, 'BLOCKED_ARMING_DISABLED_NO_PREARM': 28, 'BLOCKED_ARMING_DISABLED_DSHOT_BEEPER': 29, 'BLOCKED_ARMING_DISABLED_LANDING_DETECTED': 30}
modesID_INAV: Any = {'ARM': 0, 'ANGLE': 1, 'HORIZON': 2, 'NAV ALTHOLD': 3, 'HEADING HOLD': 5, 'HEADFREE': 6, 'HEADADJ': 7, 'CAMSTAB': 8, 'NAV RTH': 10, 'NAV POSHOLD': 11, 'MANUAL': 12, 'BEEPER': 13, 'LEDS OFF': 15, 'LIGHTS': 16, 'OSD OFF': 19, 'TELEMETRY': 20, 'AUTO TUNE': 21, 'BLACKBOX': 26, 'FAILSAFE': 27, 'NAV WP': 28, 'AIR MODE': 29, 'HOME RESET': 30, 'GCS NAV': 31, 'FPV ANGLE MIX': 32, 'SURFACE': 33, 'FLAPERON': 34, 'TURN ASSIST': 35, 'NAV LAUNCH': 36, 'SERVO AUTOTRIM': 37, 'KILLSWITCH': 38, 'CAMERA CONTROL 1': 39, 'CAMERA CONTROL 2': 40, 'CAMERA CONTROL 3': 41, 'OSD ALT 1': 42, 'OSD ALT 2': 43, 'OSD ALT 3': 44, 'NAV COURSE HOLD': 45, 'MC BRAKING': 46, 'USER1': 47, 'USER2': 48, 'LOITER CHANGE': 49, 'MSP RC OVERRIDE': 50, 'PREARM': 51, 'TURTLE': 52, 'NAV CRUISE': 53, 'AUTO LEVEL': 54, 'WP PLANNER': 55, 'MISSION CHANGE': 59, 'BEEPER MUTE': 60, 'MULTI FUNCTION': 61, 'MIXER PROFILE 2': 62, 'MIXER TRANSITION': 63, 'ANG HOLD': 64}
platformType: Any = {'PLATFORM_MULTIROTOR': 0, 'PLATFORM_AIRPLANE': 1, 'PLATFORM_HELICOPTER': 2, 'PLATFORM_TRICOPTER': 3, 'PLATFORM_ROVER': 4, 'PLATFORM_BOAT': 5, 'PLATFORM_OTHER': 6}

# --- Defines Extracted from Source ---

# --- Defines ---
# Source: sensors/acceleration.h
ACC_CLIPPING_THRESHOLD_G: Any = 15.9f
# Source: sensors/acceleration.h
ACC_VIBE_FILT_HZ: Any = 2.0f
# Source: sensors/acceleration.h
ACC_VIBE_FLOOR_FILT_HZ: Any = 5.0f
# Source: flight/adaptive_filter.h
ADAPTIVE_FILTER_BUFFER_SIZE: Any = 64
# Source: flight/adaptive_filter.h
ADAPTIVE_FILTER_RATE_HZ: Any = 100
# Source: fc/rc_adjustments.h
ADJUSTMENT_INDEX_OFFSET: Any = 1
# Source: io/adsb.h
ADSB_CALL_SIGN_MAX_LENGTH: Any = 9
# Source: io/adsb.h
ADSB_MAX_SECONDS_KEEP_INACTIVE_PLANE_IN_LIST: Any = 10
# Source: common/axis.h
ANGLE_INDEX_COUNT: Any = 2
# Source: flight/pid.h
ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF: Any = 15
# Source: msp/msp_protocol.h
API_VERSION_LENGTH: Any = 2
# Source: msp/msp_protocol.h
API_VERSION_MAJOR: Any = 2
# Source: msp/msp_protocol.h
API_VERSION_MINOR: Any = 5
# Source: fc/runtime_config.h
ARMING_DISABLED_EMERGENCY_OVERRIDE: Any = (ARMING_DISABLED_GEOZONE                                              | ARMING_DISABLED_NOT_LEVEL                                              | ARMING_DISABLED_NAVIGATION_UNSAFE                                              | ARMING_DISABLED_COMPASS_NOT_CALIBRATED                                              | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED                                              | ARMING_DISABLED_ARM_SWITCH                                              | ARMING_DISABLED_HARDWARE_FAILURE)
# Source: fc/firmware_update_common.h
# Skipped Define: AVAILABLE_FIRMWARE_SPACE = (FLASH_END - FIRMWARE_START_ADDRESS) # (Unparseable/Complex/Filtered)
# Source: flight/pid.h
AXIS_ACCEL_MIN_LIMIT: Any = 50
# Source: msp/msp_protocol.h
# Skipped Define: BASEFLIGHT_IDENTIFIER = "BAFL"; # (Unparseable/Complex/Filtered)
# Source: msp/msp_protocol.h
BETAFLIGHT_IDENTIFIER: Any = "BTFL"
# Source: common/filter.h
BIQUAD_BANDWIDTH: Any = 1.9f
# Source: common/filter.h
# Skipped Define: BIQUAD_Q = 1.0f / sqrtf(2.0f) # (Unparseable/Complex/Filtered)
# Source: blackbox/blackbox_io.h
BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET: Any = 256
# Source: blackbox/blackbox_io.h
BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION: Any = 64
# Source: msp/msp_protocol.h
BOARD_HARDWARE_REVISION_LENGTH: Any = 2
# Source: msp/msp_protocol.h
BOARD_IDENTIFIER_LENGTH: Any = 4
# Source: fc/rc_modes.h
BOXID_NONE: Any = 255
# Source: io/piniobox.h
BOX_PERMANENT_ID_NONE: Any = 255
# Source: io/piniobox.h
BOX_PERMANENT_ID_USER1: Any = 47
# Source: io/piniobox.h
BOX_PERMANENT_ID_USER2: Any = 48
# Source: io/piniobox.h
BOX_PERMANENT_ID_USER3: Any = 57
# Source: io/piniobox.h
BOX_PERMANENT_ID_USER4: Any = 58
# Source: sensors/sensors.h
CALIBRATING_ACC_TIME_MS: Any = 500
# Source: sensors/sensors.h
CALIBRATING_BARO_TIME_MS: Any = 2000
# Source: navigation/navigation_pos_estimator_private.h
CALIBRATING_GRAVITY_TIME_MS: Any = 2000
# Source: sensors/sensors.h
CALIBRATING_GYRO_MORON_THRESHOLD: Any = 32
# Source: sensors/sensors.h
CALIBRATING_GYRO_TIME_MS: Any = 2000
# Source: sensors/sensors.h
CALIBRATING_PITOT_TIME_MS: Any = 4000
# Source: msp/msp_protocol.h
CAP_BASEFLIGHT_CONFIG: Any = (1 << 30)
# Source: msp/msp_protocol.h
CAP_DYNBALANCE: Any = (1 << 2)
# Source: msp/msp_protocol.h
CAP_EXTAUX: Any = (1 << 5)
# Source: msp/msp_protocol.h
CAP_FLAPS: Any = (1 << 3)
# Source: msp/msp_protocol.h
CAP_NAVCAP: Any = (1 << 4)
# Source: msp/msp_protocol.h
CAP_NAVI_VERSION_BIT_1_LSB: Any = (1 << 28)
# Source: msp/msp_protocol.h
CAP_NAVI_VERSION_BIT_2: Any = (1 << 29)
# Source: msp/msp_protocol.h
CAP_NAVI_VERSION_BIT_3: Any = (1 << 30)
# Source: msp/msp_protocol.h
CAP_NAVI_VERSION_BIT_4_MSB: Any = (1 << 31)
# Source: msp/msp_protocol.h
CAP_PLATFORM_32BIT: Any = (1 << 31)
# Source: fc/rc_modes.h
CHANNEL_RANGE_MAX: Any = 2100
# Source: fc/rc_modes.h
CHANNEL_RANGE_MIN: Any = 900
# Source: fc/rc_modes.h
CHANNEL_RANGE_STEP_WIDTH: Any = 25
# Source: msp/msp_protocol.h
CLEANFLIGHT_IDENTIFIER: Any = "CLFL"
# Source: fc/firmware_update_common.h
# Skipped Define: CONFIG_END_ADDRESS = ((uint32_t)&__config_end) # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
# Skipped Define: CONFIG_START_ADDRESS = ((uint32_t)&__config_start) # (Unparseable/Complex/Filtered)
# Source: config/config_streamer.h
CONFIG_STREAMER_BUFFER_SIZE: Any = 4
# Source: fc/rc_controls.h
CONTROL_DEADBAND: Any = 10
# Source: rx/crsf.h
CRSF_BAUDRATE: Any = 420000
# Source: rx/crsf.h
CRSF_MAX_CHANNEL: Any = 17
# Source: telemetry/crsf.h
CRSF_MSP_RX_BUF_SIZE: Any = 128
# Source: telemetry/crsf.h
CRSF_MSP_TX_BUF_SIZE: Any = 128
# Source: rx/crsf.h
CRSF_PORT_MODE: Any = MODE_RXTX
# Source: rx/crsf.h
CRSF_PORT_OPTIONS: Any = (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
# Source: rx/crsf.h
CRSF_TELEMETRY_SYNC_BYTE: Any = 0XC8
# Source: sensors/battery.h
CURRENT_METER_OFFSET: Any = 0
# Source: sensors/battery.h
CURRENT_METER_SCALE: Any = 400
# Source: rx/rx.h
DEFAULT_SERVO_MAX: Any = 2000
# Source: rx/rx.h
DEFAULT_SERVO_MIDDLE: Any = 1500
# Source: rx/rx.h
DEFAULT_SERVO_MIN: Any = 1000
# Source: common/maths.h
DEGREES_PER_DEKADEGREE: Any = 10
# Source: rx/rx.h
DELAY_10_HZ: Any = (1000000 / 10)
# Source: rx/rx.h
DELAY_50_HZ: Any = (1000000 / 50)
# Source: rx/rx.h
DELAY_5_HZ: Any = (1000000 / 5)
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_BLINK: Any = 6
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_BLINK_MASK: Any = (1 << DISPLAYPORT_MSP_ATTR_BLINK)
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_FONTPAGE: Any = 0
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_FONTPAGE_MASK: Any = 0x3
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_VERSION: Any = 7
# Source: io/displayport_msp_osd.h
DISPLAYPORT_MSP_ATTR_VERSION_MASK: Any = (1 << DISPLAYPORT_MSP_ATTR_VERSION)
# Source: navigation/navigation_private.h
DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR: Any = 1.113195f
# Source: io/osd_dji_hd.h
# Skipped Define: DJI_ALTERNATING_DURATION_LONG = (djiOsdConfig()->craftNameAlternatingDuration * 100) # (Unparseable/Complex/Filtered)
# Source: io/osd_dji_hd.h
DJI_ALTERNATING_DURATION_SHORT: Any = 1000
# Source: io/osd_dji_hd.h
DJI_API_VERSION_MAJOR: Any = 1
# Source: io/osd_dji_hd.h
DJI_API_VERSION_MINOR: Any = 42
# Source: io/osd_dji_hd.h
DJI_ARMING_DISABLE_FLAGS_COUNT: Any = 25
# Source: io/osd_dji_hd.h
DJI_CRAFT_NAME_LENGTH: Any = 24
# Source: io/osd_dji_hd.h
DJI_MSP_ALTITUDE: Any = 109
# Source: io/osd_dji_hd.h
DJI_MSP_ANALOG: Any = 110
# Source: io/osd_dji_hd.h
DJI_MSP_API_VERSION: Any = 1
# Source: io/osd_dji_hd.h
DJI_MSP_ATTITUDE: Any = 108
# Source: io/osd_dji_hd.h
DJI_MSP_BATTERY_STATE: Any = 130
# Source: io/osd_dji_hd.h
DJI_MSP_BAUDRATE: Any = 115200
# Source: io/osd_dji_hd.h
DJI_MSP_COMP_GPS: Any = 107
# Source: io/osd_dji_hd.h
DJI_MSP_ESC_SENSOR_DATA: Any = 134
# Source: io/osd_dji_hd.h
DJI_MSP_FC_VARIANT: Any = 2
# Source: io/osd_dji_hd.h
DJI_MSP_FC_VERSION: Any = 3
# Source: io/osd_dji_hd.h
DJI_MSP_FILTER_CONFIG: Any = 92
# Source: io/osd_dji_hd.h
DJI_MSP_NAME: Any = 10
# Source: io/osd_dji_hd.h
DJI_MSP_OSD_CONFIG: Any = 84
# Source: io/osd_dji_hd.h
DJI_MSP_PID: Any = 112
# Source: io/osd_dji_hd.h
DJI_MSP_PID_ADVANCED: Any = 94
# Source: io/osd_dji_hd.h
DJI_MSP_RAW_GPS: Any = 106
# Source: io/osd_dji_hd.h
DJI_MSP_RC: Any = 105
# Source: io/osd_dji_hd.h
DJI_MSP_RC_TUNING: Any = 111
# Source: io/osd_dji_hd.h
DJI_MSP_RTC: Any = 247
# Source: io/osd_dji_hd.h
DJI_MSP_SET_FILTER_CONFIG: Any = 93
# Source: io/osd_dji_hd.h
DJI_MSP_SET_PID: Any = 202
# Source: io/osd_dji_hd.h
DJI_MSP_SET_PID_ADVANCED: Any = 95
# Source: io/osd_dji_hd.h
DJI_MSP_SET_RC_TUNING: Any = 204
# Source: io/osd_dji_hd.h
DJI_MSP_STATUS: Any = 101
# Source: io/osd_dji_hd.h
DJI_MSP_STATUS_EX: Any = 150
# Source: io/osd_dji_hd.h
DJI_OSD_FLAGS_OSD_FEATURE: Any = (1 << 0)
# Source: io/osd_dji_hd.h
DJI_OSD_TIMER_COUNT: Any = 2
# Source: io/osd_dji_hd.h
DJI_OSD_WARNING_COUNT: Any = 16
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_0: Any = 0x80
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_1: Any = 0x81
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_2: Any = 0x82
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_3: Any = 0x83
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_4: Any = 0x84
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_5: Any = 0x85
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_6: Any = 0x86
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_7: Any = 0x87
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_BAR9_8: Any = 0x88
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_CENTER: Any = 0x73
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_CENTER_LINE: Any = 0x72
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_CENTER_LINE_RIGHT: Any = 0x74
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_DECORATION: Any = 0x13
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_LEFT: Any = 0x03
# Source: io/dji_osd_symbols.h
DJI_SYM_AH_RIGHT: Any = 0x02
# Source: io/dji_osd_symbols.h
DJI_SYM_ALTITUDE: Any = 0x7F
# Source: io/dji_osd_symbols.h
DJI_SYM_AMP: Any = 0x9A
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_10: Any = 0x69
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_11: Any = 0x6A
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_12: Any = 0x6B
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_14: Any = 0x6D
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_15: Any = 0x6E
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_16: Any = 0x6F
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_2: Any = 0x61
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_3: Any = 0x62
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_4: Any = 0x63
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_6: Any = 0x65
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_7: Any = 0x66
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_8: Any = 0x67
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_EAST: Any = 0x64
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_NORTH: Any = 0x68
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_SMALL_DOWN: Any = 0x76
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_SMALL_LEFT: Any = 0x78
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_SMALL_RIGHT: Any = 0x77
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_SMALL_UP: Any = 0x75
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_SOUTH: Any = 0x60
# Source: io/dji_osd_symbols.h
DJI_SYM_ARROW_WEST: Any = 0x6C
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_1: Any = 0x95
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_2: Any = 0x94
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_3: Any = 0x93
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_4: Any = 0x92
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_5: Any = 0x91
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_EMPTY: Any = 0x96
# Source: io/dji_osd_symbols.h
DJI_SYM_BATT_FULL: Any = 0x90
# Source: io/dji_osd_symbols.h
DJI_SYM_BBLOG: Any = 0x10
# Source: io/dji_osd_symbols.h
DJI_SYM_BLANK: Any = 0x20
# Source: io/dji_osd_symbols.h
DJI_SYM_C: Any = 0x0E
# Source: io/dji_osd_symbols.h
DJI_SYM_CURSOR: Any = DJI_SYM_AH_LEFT
# Source: io/dji_osd_symbols.h
DJI_SYM_END_OF_FONT: Any = 0xFF
# Source: io/dji_osd_symbols.h
DJI_SYM_F: Any = 0x0D
# Source: io/dji_osd_symbols.h
DJI_SYM_FLY_M: Any = 0x9C
# Source: io/dji_osd_symbols.h
DJI_SYM_FT: Any = 0x0F
# Source: io/dji_osd_symbols.h
DJI_SYM_FTPS: Any = 0x99
# Source: io/dji_osd_symbols.h
DJI_SYM_GPS_DEGREE: Any = DJI_SYM_STICK_OVERLAY_SPRITE_HIGH
# Source: io/dji_osd_symbols.h
DJI_SYM_GPS_MINUTE: Any = 0x27
# Source: io/dji_osd_symbols.h
DJI_SYM_GPS_SECOND: Any = 0x22
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_DIVIDED_LINE: Any = 0x1C
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_E: Any = 0x1A
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_LINE: Any = 0x1D
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_N: Any = 0x18
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_S: Any = 0x19
# Source: io/dji_osd_symbols.h
DJI_SYM_HEADING_W: Any = 0x1B
# Source: io/dji_osd_symbols.h
DJI_SYM_HOMEFLAG: Any = 0x11
# Source: io/dji_osd_symbols.h
DJI_SYM_HYPHEN: Any = 0x2D
# Source: io/dji_osd_symbols.h
DJI_SYM_KM: Any = 0x7D
# Source: io/dji_osd_symbols.h
DJI_SYM_KPH: Any = 0x9E
# Source: io/dji_osd_symbols.h
DJI_SYM_LAT: Any = 0x89
# Source: io/dji_osd_symbols.h
DJI_SYM_LINK_QUALITY: Any = 0x7B
# Source: io/dji_osd_symbols.h
DJI_SYM_LON: Any = 0x98
# Source: io/dji_osd_symbols.h
DJI_SYM_M: Any = 0x0C
# Source: io/dji_osd_symbols.h
DJI_SYM_MAH: Any = 0x07
# Source: io/dji_osd_symbols.h
DJI_SYM_MAIN_BATT: Any = 0x97
# Source: io/dji_osd_symbols.h
DJI_SYM_MAX: Any = 0x24
# Source: io/dji_osd_symbols.h
DJI_SYM_MILES: Any = 0x7E
# Source: io/dji_osd_symbols.h
DJI_SYM_MPH: Any = 0x9D
# Source: io/dji_osd_symbols.h
DJI_SYM_MPS: Any = 0x9F
# Source: io/dji_osd_symbols.h
DJI_SYM_NONE: Any = 0x00
# Source: io/dji_osd_symbols.h
DJI_SYM_ON_M: Any = 0x9B
# Source: io/dji_osd_symbols.h
DJI_SYM_OVER_HOME: Any = 0x05
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_CLOSE: Any = 0x8F
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_EMPTY: Any = 0x8D
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_END: Any = 0x8E
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_FULL: Any = 0x8B
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_HALF: Any = 0x8C
# Source: io/dji_osd_symbols.h
DJI_SYM_PB_START: Any = 0x8A
# Source: io/dji_osd_symbols.h
DJI_SYM_PITCH: Any = 0x15
# Source: io/dji_osd_symbols.h
DJI_SYM_ROLL: Any = 0x14
# Source: io/dji_osd_symbols.h
DJI_SYM_RPM: Any = 0x12
# Source: io/dji_osd_symbols.h
DJI_SYM_RSSI: Any = 0x01
# Source: io/dji_osd_symbols.h
DJI_SYM_SAT_L: Any = 0x1E
# Source: io/dji_osd_symbols.h
DJI_SYM_SAT_R: Any = 0x1F
# Source: io/dji_osd_symbols.h
DJI_SYM_SPEED: Any = 0x70
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_CENTER: Any = 0x0B
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_HORIZONTAL: Any = 0x17
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_SPRITE_HIGH: Any = 0x08
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_SPRITE_LOW: Any = 0x0A
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_SPRITE_MID: Any = 0x09
# Source: io/dji_osd_symbols.h
DJI_SYM_STICK_OVERLAY_VERTICAL: Any = 0x16
# Source: io/dji_osd_symbols.h
DJI_SYM_TEMPERATURE: Any = 0x7A
# Source: io/dji_osd_symbols.h
DJI_SYM_THR: Any = 0x04
# Source: io/dji_osd_symbols.h
DJI_SYM_TOTAL_DISTANCE: Any = 0x71
# Source: io/dji_osd_symbols.h
DJI_SYM_VOLT: Any = 0x06
# Source: io/dji_osd_symbols.h
DJI_SYM_WATT: Any = 0x57
# Source: flight/mixer.h
DSHOT_3D_DEADBAND_HIGH: Any = 1048
# Source: flight/mixer.h
DSHOT_3D_DEADBAND_LOW: Any = 1047
# Source: flight/mixer.h
DSHOT_DISARM_COMMAND: Any = 0
# Source: flight/mixer.h
DSHOT_MAX_THROTTLE: Any = 2047
# Source: flight/mixer.h
DSHOT_MIN_THROTTLE: Any = 48
# Source: flight/dynamic_gyro_notch.h
DYNAMIC_NOTCH_DEFAULT_CENTER_HZ: Any = 350
# Source: flight/dynamic_gyro_notch.h
DYN_NOTCH_PEAK_COUNT: Any = 3
# Source: config/config_eeprom.h
EEPROM_CONF_VERSION: Any = 126
# Source: io/osd_dji_hd.h
# Skipped Define: EFFICIENCY_UPDATE_INTERVAL = (5 * 1000) # (Unparseable/Complex/Filtered)
# Source: sensors/esc_sensor.h
ERPM_PER_LSB: Any = 100.0f
# Source: sensors/esc_sensor.h
ESC_DATA_INVALID: Any = 255
# Source: sensors/esc_sensor.h
ESC_DATA_MAX_AGE: Any = 10
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_INPUT = setEscInput(selected_esc) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_IS_HI = isEscHi(selected_esc) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_IS_LO = isEscLo(selected_esc) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_OUTPUT = setEscOutput(selected_esc) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_SET_HI = setEscHi(selected_esc) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way_impl.h
# Skipped Define: ESC_SET_LO = setEscLo(selected_esc) # (Unparseable/Complex/Filtered)
# Source: rx/jetiexbus.h
EXBUS_CRC_LEN: Any = 2
# Source: rx/jetiexbus.h
EXBUS_EX_REQUEST: Any = (0x3A)
# Source: rx/jetiexbus.h
EXBUS_HEADER_LEN: Any = 6
# Source: rx/jetiexbus.h
# Skipped Define: EXBUS_MAX_CHANNEL_FRAME_SIZE = (EXBUS_HEADER_LEN + JETIEXBUS_CHANNEL_COUNT*2 + EXBUS_CRC_LEN) # (Unparseable/Complex/Filtered)
# Source: rx/jetiexbus.h
EXBUS_MAX_REQUEST_FRAME_SIZE: Any = 32
# Source: rx/jetiexbus.h
EXBUS_OVERHEAD: Any = (EXBUS_HEADER_LEN + EXBUS_CRC_LEN)
# Source: flight/failsafe.h
# Skipped Define: FAILSAFE_POWER_ON_DELAY_US = (1000 * 1000 * 5) # (Unparseable/Complex/Filtered)
# Source: common/utils.h
# Skipped Define: FALLTHROUGH = do {} while(0) # (Unparseable/Complex/Filtered)
# Source: common/maths.h
# Skipped Define: FAST_MATH # (No Value)
# Source: common/constants.h
FEET_PER_KILOFEET: Any = 1000
# Source: common/constants.h
FEET_PER_MILE: Any = 5280
# Source: common/constants.h
FEET_PER_NAUTICALMILE: Any = 6076.118f
# Source: flight/gyroanalyse.h
FFT_WINDOW_SIZE: Any = 64
# Source: fc/firmware_update_common.h
# Skipped Define: FIRMWARE_START_ADDRESS = ((uint32_t)&__firmware_start) # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
# Skipped Define: FIRMWARE_UPDATE_BACKUP_FILENAME = "firmware.bak" # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
# Skipped Define: FIRMWARE_UPDATE_FIRMWARE_FILENAME = "firmware.upt" # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
FIRMWARE_UPDATE_METADATA_MAGIC: Any = 0xAABBCCDD
# Source: fc/firmware_update_common.h
# Skipped Define: FIRMWARE_UPDATE_META_FILENAME = "update.mta" # (Unparseable/Complex/Filtered)
# Source: io/rcdevice_cam.h
FIVE_KEY_CABLE_JOYSTICK_MAX: Any = 1920
# Source: io/rcdevice_cam.h
FIVE_KEY_CABLE_JOYSTICK_MID_END: Any = 1650
# Source: io/rcdevice_cam.h
FIVE_KEY_CABLE_JOYSTICK_MID_START: Any = 1350
# Source: io/rcdevice_cam.h
FIVE_KEY_CABLE_JOYSTICK_MIN: Any = 1080
# Source: flight/pid.h
FIXED_WING_LEVEL_TRIM_DEADBAND_DEFAULT: Any = 5
# Source: flight/servos.h
FLAPERON_THROW_DEFAULT: Any = 200
# Source: flight/servos.h
FLAPERON_THROW_MAX: Any = 450
# Source: flight/servos.h
FLAPERON_THROW_MIN: Any = 50
# Source: io/flashfs.h
FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN: Any = 64
# Source: io/flashfs.h
FLASHFS_WRITE_BUFFER_SIZE: Any = 128
# Source: io/flashfs.h
# Skipped Define: FLASHFS_WRITE_BUFFER_USABLE = (FLASHFS_WRITE_BUFFER_SIZE - 1) # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
# Skipped Define: FLASH_END = (FLASH_START_ADDRESS + MCU_FLASH_SIZE * 1024) # (Unparseable/Complex/Filtered)
# Source: fc/firmware_update_common.h
FLASH_START_ADDRESS: Any = 0x08000000UL
# Source: msp/msp_protocol.h
FLIGHT_CONTROLLER_IDENTIFIER_LENGTH: Any = 4
# Source: msp/msp_protocol.h
FLIGHT_CONTROLLER_VERSION_LENGTH: Any = 3
# Source: msp/msp_protocol.h
FLIGHT_CONTROLLER_VERSION_MASK: Any = 0xFFF
# Source: common/axis.h
FLIGHT_DYNAMICS_INDEX_COUNT: Any = 3
# Source: blackbox/blackbox_fielddefs.h
FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG: Any = 128
# Source: common/time.h
FORMATTED_DATE_TIME_BUFSIZE: Any = 30
# Source: flight/pid.h
# Skipped Define: FP_PID_LEVEL_P_MULTIPLIER = 1.0f / 6.56f # (Unparseable/Complex/Filtered)
# Source: flight/pid.h
FP_PID_RATE_D_FF_MULTIPLIER: Any = 7270.0f
# Source: flight/pid.h
FP_PID_RATE_D_MULTIPLIER: Any = 1905.0f
# Source: flight/pid.h
FP_PID_RATE_FF_MULTIPLIER: Any = 31.0f
# Source: flight/pid.h
FP_PID_RATE_I_MULTIPLIER: Any = 4.0f
# Source: flight/pid.h
FP_PID_RATE_P_MULTIPLIER: Any = 31.0f
# Source: flight/pid.h
FP_PID_YAWHOLD_P_MULTIPLIER: Any = 80.0f
# Source: common/typeconversion.h
FTOA_BUFFER_SIZE: Any = 13
# Source: io/serial.h
FUNCTION_VTX_MSP: Any = FUNCTION_MSP_OSD
# Source: navigation/navigation.h
GEOZONE_SHAPE_CIRCULAR: Any = 0
# Source: navigation/navigation.h
GEOZONE_SHAPE_POLYGON: Any = 1
# Source: navigation/navigation.h
GEOZONE_TYPE_EXCLUSIVE: Any = 0
# Source: navigation/navigation.h
GEOZONE_TYPE_INCLUSIVE: Any = 1
# Source: rx/ghst_protocol.h
# Skipped Define: GHST_BYTE_TIME_FAST_US = ((1000000/GHST_TX_BAUDRATE_FAST)*10) # (Unparseable/Complex/Filtered)
# Source: rx/ghst_protocol.h
# Skipped Define: GHST_BYTE_TIME_SLOW_US = ((1000000/GHST_TX_BAUDRATE_SLOW)*10) # (Unparseable/Complex/Filtered)
# Source: rx/ghst_protocol.h
GHST_FRAME_SIZE: Any = 14
# Source: rx/ghst_protocol.h
GHST_FRAME_SIZE_MAX: Any = 24
# Source: rx/ghst.h
GHST_MAX_NUM_CHANNELS: Any = 16
# Source: rx/ghst_protocol.h
GHST_PAYLOAD_SIZE_MAX: Any = 14
# Source: rx/ghst_protocol.h
GHST_RC_CTR_VAL_12BIT: Any = 0x7C0
# Source: rx/ghst_protocol.h
GHST_RC_CTR_VAL_8BIT: Any = 0x7C
# Source: rx/ghst_protocol.h
GHST_RX_BAUDRATE: Any = 420000
# Source: rx/ghst_protocol.h
GHST_TX_BAUDRATE_FAST: Any = 400000
# Source: rx/ghst_protocol.h
GHST_TX_BAUDRATE_SLOW: Any = 115200
# Source: rx/ghst_protocol.h
GHST_UART_WORDLENGTH: Any = UART_WORDLENGTH_8B
# Source: rx/ghst_protocol.h
# Skipped Define: GHST_UL_RC_CHANS_FRAME_COUNT = (GHST_UL_RC_CHANS_HS4_13TO16 - GHST_UL_RC_CHANS_HS4_5TO8 + 1) # (Unparseable/Complex/Filtered)
# Source: rx/ghst_protocol.h
GHST_UL_RC_CHANS_SIZE: Any = 12
# Source: rx/ghst_protocol.h
# Skipped Define: GHST_UL_RC_TOTAL_FRAME_COUNT = (GHST_UL_RC_CHANS_HS4_LAST - GHST_UL_RC_CHANS_HS4_FIRST + 1) # (Unparseable/Complex/Filtered)
# Source: io/gps.h
GPS_BAUDRATE_MAX: Any = GPS_BAUDRATE_9600
# Source: io/gps_private.h
GPS_BAUD_CHANGE_DELAY: Any = (100)
# Source: io/gps_private.h
GPS_BOOT_DELAY: Any = (3000)
# Source: io/gps_ublox.h
GPS_CAPA_INTERVAL: Any = 5000
# Source: io/gps_ublox.h
GPS_CFG_CMD_TIMEOUT_MS: Any = 500
# Source: io/gps.h
GPS_DBHZ_MAX: Any = 55
# Source: io/gps.h
GPS_DBHZ_MIN: Any = 0
# Source: io/gps.h
GPS_DEGREES_DIVIDER: Any = 10000000
# Source: rx/ghst_protocol.h
GPS_FLAGS_FIX: Any = 0x01
# Source: rx/ghst_protocol.h
GPS_FLAGS_FIX_HOME: Any = 0x02
# Source: io/gps_private.h
GPS_HDOP_TO_EPH_MULTIPLIER: Any = 2
# Source: io/gps_private.h
GPS_INIT_DELAY: Any = (500)
# Source: io/gps_private.h
GPS_SHORT_TIMEOUT: Any = (500)
# Source: io/gps_private.h
GPS_TIMEOUT: Any = (1000)
# Source: io/gps_ublox.h
GPS_VERSION_RETRY_TIMES: Any = 3
# Source: sensors/acceleration.h
GRAVITY_CMSS: Any = 980.665f
# Source: sensors/acceleration.h
GRAVITY_MSS: Any = 9.80665f
# Source: flight/pid.h
GYRO_SATURATION_LIMIT: Any = 1800
# Source: io/gps.h
HDOP_SCALE: Any = (100)
# Source: flight/pid.h
HEADING_HOLD_ERROR_LPF_FREQ: Any = 2
# Source: flight/pid.h
HEADING_HOLD_RATE_LIMIT_DEFAULT: Any = 90
# Source: flight/pid.h
HEADING_HOLD_RATE_LIMIT_MAX: Any = 250
# Source: flight/pid.h
HEADING_HOLD_RATE_LIMIT_MIN: Any = 10
# Source: io/gimbal_serial.h
# Skipped Define: HEADTRACKER_PAYLOAD_SIZE = (sizeof(gimbalHtkAttitudePkt_t) - 4) # (Unparseable/Complex/Filtered)
# Source: telemetry/hott.h
HOTTV4_BINARY_MODE_REQUEST_ID: Any = 0x80
# Source: telemetry/hott.h
HOTTV4_BUTTON_DEC: Any = 0xB
# Source: telemetry/hott.h
HOTTV4_BUTTON_INC: Any = 0xD
# Source: telemetry/hott.h
HOTTV4_BUTTON_NEXT: Any = 0xE
# Source: telemetry/hott.h
HOTTV4_BUTTON_NIL: Any = 0xF
# Source: telemetry/hott.h
HOTTV4_BUTTON_PREV: Any = 0x7
# Source: telemetry/hott.h
HOTTV4_BUTTON_SET: Any = 0x9
# Source: telemetry/hott.h
HOTTV4_RXTX: Any = 4
# Source: telemetry/hott.h
HOTTV4_TEXT_MODE_REQUEST_ID: Any = 0x7f
# Source: telemetry/hott.h
HOTT_BINARY_MODE_REQUEST_ID: Any = 0x80
# Source: telemetry/hott.h
HOTT_EAM_OFFSET_HEIGHT: Any = 500
# Source: telemetry/hott.h
HOTT_EAM_OFFSET_M2S: Any = 72
# Source: telemetry/hott.h
HOTT_EAM_OFFSET_M3S: Any = 120
# Source: telemetry/hott.h
HOTT_EAM_OFFSET_TEMPERATURE: Any = 20
# Source: telemetry/hott.h
HOTT_EAM_SENSOR_TEXT_ID: Any = 0xE0
# Source: telemetry/hott.h
HOTT_GPS_ALTITUDE_OFFSET: Any = 500
# Source: telemetry/hott.h
HOTT_GPS_SENSOR_TEXT_ID: Any = 0xA0
# Source: telemetry/hott.h
HOTT_TELEMETRY_AIRESC_SENSOR_ID: Any = 0x8c
# Source: telemetry/hott.h
HOTT_TELEMETRY_EAM_SENSOR_ID: Any = 0x8e
# Source: telemetry/hott.h
HOTT_TELEMETRY_GAM_SENSOR_ID: Any = 0x8d
# Source: telemetry/hott.h
HOTT_TELEMETRY_GPS_SENSOR_ID: Any = 0x8a
# Source: telemetry/hott.h
HOTT_TELEMETRY_NO_SENSOR_ID: Any = 0x80
# Source: telemetry/hott.h
HOTT_TELEMETRY_VARIO_SENSOR_ID: Any = 0x89
# Source: telemetry/hott.h
HOTT_TEXTMODE_DISPLAY_COLUMNS: Any = 21
# Source: telemetry/hott.h
HOTT_TEXTMODE_DISPLAY_ROWS: Any = 8
# Source: telemetry/hott.h
HOTT_TEXTMODE_ESC: Any = 0x01
# Source: telemetry/hott.h
HOTT_TEXTMODE_START: Any = 0x7B
# Source: telemetry/hott.h
HOTT_TEXTMODE_STOP: Any = 0x7D
# Source: telemetry/hott.h
HOTT_TEXT_MODE_REQUEST_ID: Any = 0x7f
# Source: telemetry/hott.h
HOTT_VARIO_MSG_TEXT_LEN: Any = 21
# Source: common/color.h
HSV_COLOR_COMPONENT_COUNT: Any = (HSV_VALUE + 1)
# Source: common/color.h
HSV_HUE_MAX: Any = 359
# Source: common/color.h
HSV_SATURATION_MAX: Any = 255
# Source: common/color.h
HSV_VALUE_MAX: Any = 255
# Source: io/gimbal_serial.h
HTKATTITUDE_SYNC0: Any = 0xA5
# Source: io/gimbal_serial.h
HTKATTITUDE_SYNC1: Any = 0x5A
# Source: telemetry/ibus_shared.h
IBUS_BAUDRATE: Any = (115200)
# Source: telemetry/ibus_shared.h
IBUS_CHECKSUM_SIZE: Any = (2)
# Source: telemetry/ibus_shared.h
IBUS_CYCLE_TIME_MS: Any = (8)
# Source: telemetry/ibus_shared.h
IBUS_MAX_RX_LEN: Any = (4)
# Source: telemetry/ibus_shared.h
IBUS_MAX_TX_LEN: Any = (6)
# Source: telemetry/ibus_shared.h
IBUS_MIN_LEN: Any = (2 + IBUS_CHECKSUM_SIZE)
# Source: telemetry/ibus_shared.h
# Skipped Define: IBUS_RX_BUF_LEN = (IBUS_MAX_RX_LEN) # (Unparseable/Complex/Filtered)
# Source: telemetry/ibus_shared.h
IBUS_TASK_PERIOD_US: Any = (500)
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: INAV_ACC_BIAS_ACCEPTANCE_VALUE = (GRAVITY_CMSS * 0.25f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: INAV_ACC_CLIPPING_RC_CONSTANT = (0.010f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation_pos_estimator_private.h
INAV_BARO_AVERAGE_HZ: Any = 1.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_BARO_TIMEOUT_MS: Any = 200
# Source: common/circular_queue.h
# Skipped Define: INAV_CIRCULAR_QUEUE_H # (No Value)
# Source: navigation/navigation_pos_estimator_private.h
INAV_FLOW_TIMEOUT_MS: Any = 200
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_ACCEPTANCE_EPE: Any = 500.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_DEFAULT_EPH: Any = 200.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_DEFAULT_EPV: Any = 500.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_GLITCH_ACCEL: Any = 1000.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_GLITCH_RADIUS: Any = 250.0f
# Source: navigation/navigation_pos_estimator_private.h
INAV_GPS_TIMEOUT_MS: Any = 1500
# Source: msp/msp_protocol.h
INAV_IDENTIFIER: Any = "INAV"
# Source: navigation/navigation_pos_estimator_private.h
INAV_PITOT_UPDATE_RATE: Any = 10
# Source: navigation/navigation_pos_estimator_private.h
INAV_POSITION_PUBLISH_RATE_HZ: Any = 50
# Source: navigation/navigation_pos_estimator_private.h
INAV_SURFACE_AVERAGE_HZ: Any = 1.0f
# Source: navigation/navigation_private.h
INAV_SURFACE_MAX_DISTANCE: Any = 40
# Source: navigation/navigation_pos_estimator_private.h
INAV_SURFACE_TIMEOUT_MS: Any = 400
# Source: telemetry/jetiexbus.h
JETI_EXBUS_TELEMETRY_FRAME_LEN: Any = 128
# Source: io/gps.h
LAT: Any = 0
# Source: io/ledstrip.h
LED_BASEFUNCTION_COUNT: Any = 8
# Source: io/ledstrip.h
LED_COLOR_BITCNT: Any = 4
# Source: io/ledstrip.h
LED_CONFIGURABLE_COLOR_COUNT: Any = 16
# Source: io/ledstrip.h
LED_DIRECTION_BITCNT: Any = 6
# Source: io/ledstrip.h
LED_DIRECTION_COUNT: Any = 6
# Source: io/ledstrip.h
# Skipped Define: LED_FLAG_OVERLAY_MASK = ((1 << LED_OVERLAY_BITCNT) - 1) # (Unparseable/Complex/Filtered)
# Source: io/ledstrip.h
LED_FUNCTION_BITCNT: Any = 8
# Source: io/ledstrip.h
# Skipped Define: LED_FUNCTION_MASK = LED_MOV_FUNCTION(((1 << LED_FUNCTION_BITCNT) - 1)) # (Unparseable/Complex/Filtered)
# Source: io/ledstrip.h
LED_FUNCTION_OFFSET: Any = 8
# Source: io/ledstrip.h
LED_MAX_STRIP_LENGTH: Any = 128
# Source: io/ledstrip.h
LED_MODE_COUNT: Any = 6
# Source: io/ledstrip.h
LED_OVERLAY_BITCNT: Any = 8
# Source: io/ledstrip.h
LED_OVERLAY_COUNT: Any = 7
# Source: io/ledstrip.h
# Skipped Define: LED_OVERLAY_MASK = LED_MOV_OVERLAY(LED_FLAG_OVERLAY_MASK) # (Unparseable/Complex/Filtered)
# Source: io/ledstrip.h
LED_OVERLAY_OFFSET: Any = 16
# Source: io/ledstrip.h
LED_PARAMS_BITCNT: Any = 6
# Source: io/ledstrip.h
LED_POS_BITCNT: Any = 8
# Source: io/ledstrip.h
LED_SPECIAL_COLOR_COUNT: Any = 9
# Source: io/ledstrip.h
LED_XY_MASK: Any = 0x0F
# Source: io/ledstrip.h
LED_X_BIT_OFFSET: Any = 4
# Source: io/ledstrip.h
LED_Y_BIT_OFFSET: Any = 0
# Source: common/log.h
LOG_LEVEL_DEBUG: Any = 4
# Source: common/log.h
LOG_LEVEL_ERROR: Any = 0
# Source: common/log.h
LOG_LEVEL_INFO: Any = 2
# Source: common/log.h
LOG_LEVEL_VERBOSE: Any = 3
# Source: common/log.h
LOG_LEVEL_WARNING: Any = 1
# Source: io/gps.h
LON: Any = 1
# Source: telemetry/ltm.h
LTM_AFRAME_PAYLOAD_SIZE: Any = 6
# Source: telemetry/ltm.h
LTM_GFRAME_PAYLOAD_SIZE: Any = 14
# Source: telemetry/ltm.h
LTM_MAX_MESSAGE_SIZE: Any = (LTM_MAX_PAYLOAD_SIZE+4)
# Source: telemetry/ltm.h
LTM_MAX_PAYLOAD_SIZE: Any = 14
# Source: telemetry/ltm.h
LTM_NFRAME_PAYLOAD_SIZE: Any = 6
# Source: telemetry/ltm.h
LTM_OFRAME_PAYLOAD_SIZE: Any = 14
# Source: telemetry/ltm.h
LTM_SFRAME_PAYLOAD_SIZE: Any = 7
# Source: telemetry/ltm.h
LTM_XFRAME_PAYLOAD_SIZE: Any = 6
# Source: rx/mavlink.h
MAVLINK_COMM_NUM_BUFFERS: Any = 1
# Source: fc/rc_adjustments.h
MAX_ADJUSTMENT_RANGE_COUNT: Any = 20
# Source: rx/rx.h
# Skipped Define: MAX_AUX_CHANNEL_COUNT = (MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT) # (Unparseable/Complex/Filtered)
# Source: sensors/battery.h
MAX_BATTERY_PROFILE_COUNT: Any = SETTING_CONSTANT_MAX_BATTERY_PROFILE_COUNT
# Source: io/vtx_control.h
MAX_CHANNEL_ACTIVATION_CONDITION_COUNT: Any = 10
# Source: io/gps_ublox.h
MAX_CONFIG_SET_VAL_VALUES: Any = 32
# Source: fc/controlrate_profile.h
MAX_CONTROL_RATE_PROFILE_COUNT: Any = SETTING_CONSTANT_MAX_CONTROL_RATE_PROFILE_COUNT
# Source: navigation/navigation.h
MAX_FW_LAND_APPOACH_SETTINGS: Any = (MAX_SAFE_HOMES + 9)
# Source: programming/global_variables.h
MAX_GLOBAL_VARIABLES: Any = 8
# Source: io/gps_ublox.h
MAX_GNSS: Any = 7
# Source: io/gps_ublox.h
# Skipped Define: MAX_GNSS_SIZE_BYTES = (sizeof(ubx_gnss_msg_t) + sizeof(ubx_gnss_element_t)*MAX_GNSS) # (Unparseable/Complex/Filtered)
# Source: rx/rx.h
MAX_INVALID_RX_PULSE_TIME: Any = 300
# Source: flight/kalman.h
MAX_KALMAN_WINDOW_SIZE: Any = 64
# Source: programming/logic_condition.h
MAX_LOGIC_CONDITIONS: Any = 64
# Source: rx/rx.h
MAX_MAPPABLE_RX_INPUTS: Any = 4
# Source: flight/mixer_profile.h
MAX_MIXER_PROFILE_COUNT: Any = 2
# Source: fc/rc_modes.h
MAX_MODE_ACTIVATION_CONDITION_COUNT: Any = 40
# Source: fc/rc_modes.h
# Skipped Define: MAX_MODE_RANGE_STEP = ((CHANNEL_RANGE_MAX - CHANNEL_RANGE_MIN) / CHANNEL_RANGE_STEP_WIDTH) # (Unparseable/Complex/Filtered)
# Source: msp/msp_serial.h
MAX_MSP_PORT_COUNT: Any = 3
# Source: fc/config.h
MAX_NAME_LENGTH: Any = 16
# Source: navigation/navigation_private.h
# Skipped Define: MAX_POSITION_UPDATE_INTERVAL_US = HZ2US(MIN_POSITION_UPDATE_RATE_HZ) # (Unparseable/Complex/Filtered)
# Source: fc/config.h
MAX_PROFILE_COUNT: Any = 3
# Source: programming/pid.h
MAX_PROGRAMMING_PID_COUNT: Any = 4
# Source: rx/rx.h
# Skipped Define: MAX_RXFAIL_RANGE_STEP = ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 25) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation.h
MAX_SAFE_HOMES: Any = 0
# Source: flight/servos.h
# Skipped Define: MAX_SERVO_RULES = (2 * MAX_SUPPORTED_SERVOS) # (Unparseable/Complex/Filtered)
# Source: flight/servos.h
MAX_SERVO_SPEED: Any = UINT8_MAX
# Source: fc/rc_adjustments.h
MAX_SIMULTANEOUS_ADJUSTMENT_COUNT: Any = 4
# Source: flight/smith_predictor.h
MAX_SMITH_SAMPLES: Any = 64
# Source: flight/mixer.h
MAX_SUPPORTED_MOTORS: Any = 12
# Source: rx/rx.h
MAX_SUPPORTED_RC_CHANNEL_COUNT: Any = 34
# Source: flight/servos.h
MAX_SUPPORTED_SERVOS: Any = 18
# Source: sensors/temperature.h
MAX_TEMP_SENSORS: Any = 8
# Source: io/gps_ublox.h
# Skipped Define: MAX_UBLOX_PAYLOAD_SIZE = ((UBLOX_MAX_SIGNALS * 16) + 8) # (Unparseable/Complex/Filtered)
# Source: io/rcdevice.h
MAX_WAITING_RESPONSES: Any = 2
# Source: flight/pid.h
MC_ITERM_RELAX_CUTOFF_DEFAULT: Any = 15
# Source: flight/pid.h
MC_ITERM_RELAX_SETPOINT_THRESHOLD: Any = 40.0f
# Source: navigation/navigation_private.h
MC_LAND_CHECK_VEL_XY_MOVING: Any = 100.0f
# Source: navigation/navigation_private.h
MC_LAND_CHECK_VEL_Z_MOVING: Any = 100.0f
# Source: navigation/navigation_private.h
MC_LAND_DESCEND_THROTTLE: Any = 40
# Source: navigation/navigation_private.h
MC_LAND_SAFE_SURFACE: Any = 5.0f
# Source: navigation/navigation_private.h
MC_LAND_THR_STABILISE_DELAY: Any = 1
# Source: navigation/navigation_private.h
MC_POS_CONTROL_JERK_LIMIT_CMSSS: Any = 1700.0f
# Source: common/constants.h
METERS_PER_FOOT: Any = 3.28084f
# Source: common/constants.h
METERS_PER_KILOMETER: Any = 1000
# Source: common/constants.h
METERS_PER_MILE: Any = 1609.344f
# Source: common/constants.h
METERS_PER_NAUTICALMILE: Any = 1852.001f
# Source: rx/rx.h
MIDRC_MAX: Any = 1700
# Source: rx/rx.h
MIDRC_MIN: Any = 1200
# Source: common/time.h
MILLISECS_PER_SEC: Any = 1000
# Source: flight/failsafe.h
MILLIS_PER_SECOND: Any = 1000
# Source: flight/failsafe.h
MILLIS_PER_TENTH_SECOND: Any = 100
# Source: fc/rc_modes.h
MIN_MODE_RANGE_STEP: Any = 0
# Source: navigation/navigation_private.h
MIN_POSITION_UPDATE_RATE_HZ: Any = 5
# Source: msp/msp_protocol_v2_inav.h
MSP2_ADSB_VEHICLE_LIST: Any = 0x2090
# Source: msp/msp_protocol_v2_common.h
MSP2_BETAFLIGHT_BIND: Any = 0x3000
# Source: msp/msp_protocol_v2_inav.h
MSP2_BLACKBOX_CONFIG: Any = 0x201A
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_MOTOR_MIXER: Any = 0x1005
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_PG_LIST: Any = 0x1008
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SERIAL_CONFIG: Any = 0x1009
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SETTING: Any = 0x1003
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SETTING_INFO: Any = 0x1007
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_MOTOR_MIXER: Any = 0x1006
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_MSP_RC_INFO: Any = 0x100E
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_MSP_RC_LINK_STATS: Any = 0x100D
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_RADAR_ITD: Any = 0x100C
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_RADAR_POS: Any = 0x100B
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_SERIAL_CONFIG: Any = 0x100A
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_SETTING: Any = 0x1004
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_SET_TZ: Any = 0x1002
# Source: msp/msp_protocol_v2_common.h
MSP2_COMMON_TZ: Any = 0x1001
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_AIR_SPEED: Any = 0x2009
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_ANALOG: Any = 0x2002
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_BATTERY_CONFIG: Any = 0x2005
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_CUSTOM_OSD_ELEMENT: Any = 0x2101
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_CUSTOM_OSD_ELEMENTS: Any = 0x2100
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_DEBUG: Any = 0x2019
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_ESC_RPM: Any = 0x2040
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_ESC_TELEM: Any = 0x2041
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_EZ_TUNE: Any = 0x2070
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_EZ_TUNE_SET: Any = 0x2071
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FWUPDT_EXEC: Any = 0x2035
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FWUPDT_PREPARE: Any = 0x2033
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FWUPDT_ROLLBACK_EXEC: Any = 0x2037
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FWUPDT_ROLLBACK_PREPARE: Any = 0x2036
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FWUPDT_STORE: Any = 0x2034
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_FW_APPROACH: Any = 0x204A
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_GEOZONE: Any = 0x2210
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_GEOZONE_VERTEX: Any = 0x2212
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_GLOBAL_FUNCTIONS: Any = 0x2024
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_GPS_UBLOX_COMMAND: Any = 0x2050
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_GVAR_STATUS: Any = 0x2027
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_LED_STRIP_CONFIG_EX: Any = 0x2048
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_LOGIC_CONDITIONS: Any = 0x2022
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_LOGIC_CONDITIONS_SINGLE: Any = 0x203B
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_LOGIC_CONDITIONS_STATUS: Any = 0x2026
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_MC_BRAKING: Any = 0x200B
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_MISC: Any = 0x2003
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_MISC2: Any = 0x203A
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_MIXER: Any = 0x2010
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OPFLOW_CALIBRATION: Any = 0x2032
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OPTICAL_FLOW: Any = 0x2001
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_ALARMS: Any = 0x2014
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_LAYOUTS: Any = 0x2012
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_PREFERENCES: Any = 0x2016
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_SET_ALARMS: Any = 0x2015
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_SET_LAYOUT_ITEM: Any = 0x2013
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OSD_SET_PREFERENCES: Any = 0x2017
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OUTPUT_MAPPING: Any = 0x200A
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OUTPUT_MAPPING_EXT: Any = 0x200D
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_OUTPUT_MAPPING_EXT2: Any = 0x210D
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_PROGRAMMING_PID: Any = 0x2028
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_PROGRAMMING_PID_STATUS: Any = 0x202A
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_RATE_DYNAMICS: Any = 0x2060
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_RATE_PROFILE: Any = 0x2007
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SAFEHOME: Any = 0x2038
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SELECT_BATTERY_PROFILE: Any = 0x2018
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SELECT_MIXER_PROFILE: Any = 0x2080
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SERVO_CONFIG: Any = 0x2200
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SERVO_MIXER: Any = 0x2020
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_BATTERY_CONFIG: Any = 0x2006
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS: Any = 0x2102
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_FW_APPROACH: Any = 0x204B
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_GEOZONE: Any = 0x2211
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_GEOZONE_VERTEX: Any = 0x2213
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_GLOBAL_FUNCTIONS: Any = 0x2025
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_LED_STRIP_CONFIG_EX: Any = 0x2049
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_LOGIC_CONDITIONS: Any = 0x2023
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_MC_BRAKING: Any = 0x200C
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_MISC: Any = 0x2004
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_MIXER: Any = 0x2011
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_PROGRAMMING_PID: Any = 0x2029
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_RATE_DYNAMICS: Any = 0x2061
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_RATE_PROFILE: Any = 0x2008
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_SAFEHOME: Any = 0x2039
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_SERVO_CONFIG: Any = 0x2201
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_SERVO_MIXER: Any = 0x2021
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_TEMP_SENSOR_CONFIG: Any = 0x201D
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_SET_TIMER_OUTPUT_MODE: Any = 0x200F
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_STATUS: Any = 0x2000
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_TEMPERATURES: Any = 0x201E
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_TEMP_SENSOR_CONFIG: Any = 0x201C
# Source: msp/msp_protocol_v2_inav.h
MSP2_INAV_TIMER_OUTPUT_MODE: Any = 0x200E
# Source: msp/msp_protocol_v2_inav.h
MSP2_PID: Any = 0x2030
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_AIRSPEED: Any = 0x1F06
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_BAROMETER: Any = 0x1F05
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_COMPASS: Any = 0x1F04
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_GPS: Any = 0x1F03
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_HEADTRACKER: Any = 0x1F07
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_OPTIC_FLOW: Any = 0x1F02
# Source: msp/msp_protocol_v2_sensor.h
MSP2_SENSOR_RANGEFINDER: Any = 0x1F01
# Source: msp/msp_protocol_v2_inav.h
MSP2_SET_BLACKBOX_CONFIG: Any = 0x201B
# Source: msp/msp_protocol_v2_inav.h
MSP2_SET_PID: Any = 0x2031
# Source: msp/msp_protocol.h
MSP_3D: Any = 124
# Source: msp/msp_protocol.h
MSP_ACC_CALIBRATION: Any = 205
# Source: msp/msp_protocol.h
MSP_ACC_TRIM: Any = 240
# Source: msp/msp_protocol.h
MSP_ACTIVEBOXES: Any = 113
# Source: msp/msp_protocol.h
MSP_ADJUSTMENT_RANGES: Any = 52
# Source: msp/msp_protocol.h
MSP_ADVANCED_CONFIG: Any = 90
# Source: msp/msp_protocol.h
MSP_ALTITUDE: Any = 109
# Source: msp/msp_protocol.h
MSP_ANALOG: Any = 110
# Source: msp/msp_protocol.h
MSP_API_VERSION: Any = 1
# Source: msp/msp_protocol.h
MSP_ATTITUDE: Any = 108
# Source: msp/msp_protocol.h
MSP_BATTERY_STATE: Any = 130
# Source: msp/msp_protocol.h
MSP_BLACKBOX_CONFIG: Any = 80
# Source: msp/msp_protocol.h
MSP_BOARD_ALIGNMENT: Any = 38
# Source: msp/msp_protocol.h
MSP_BOARD_INFO: Any = 4
# Source: msp/msp_protocol.h
MSP_BOXIDS: Any = 119
# Source: msp/msp_protocol.h
MSP_BOXNAMES: Any = 116
# Source: msp/msp_protocol.h
MSP_BUILD_INFO: Any = 5
# Source: msp/msp_protocol.h
MSP_CALIBRATION_DATA: Any = 14
# Source: msp/msp_protocol.h
MSP_CF_SERIAL_CONFIG: Any = 54
# Source: msp/msp_protocol.h
MSP_COMP_GPS: Any = 107
# Source: msp/msp_protocol.h
MSP_CURRENT_METER_CONFIG: Any = 40
# Source: msp/msp_protocol.h
MSP_DATAFLASH_ERASE: Any = 72
# Source: msp/msp_protocol.h
MSP_DATAFLASH_READ: Any = 71
# Source: msp/msp_protocol.h
MSP_DATAFLASH_SUMMARY: Any = 70
# Source: msp/msp_protocol.h
MSP_DEBUG: Any = 254
# Source: msp/msp_protocol.h
MSP_DEBUGMSG: Any = 253
# Source: msp/msp_protocol.h
MSP_DISPLAYPORT: Any = 182
# Source: msp/msp_protocol.h
MSP_EEPROM_WRITE: Any = 250
# Source: msp/msp_protocol.h
MSP_FAILSAFE_CONFIG: Any = 75
# Source: msp/msp_protocol.h
MSP_FC_VARIANT: Any = 2
# Source: msp/msp_protocol.h
MSP_FC_VERSION: Any = 3
# Source: msp/msp_protocol.h
MSP_FEATURE: Any = 36
# Source: msp/msp_protocol.h
MSP_FILTER_CONFIG: Any = 92
# Source: msp/msp_protocol.h
MSP_FW_CONFIG: Any = 23
# Source: msp/msp_protocol.h
MSP_GPSSTATISTICS: Any = 166
# Source: msp/msp_protocol.h
MSP_GPSSVINFO: Any = 164
# Source: msp/msp_protocol.h
MSP_INAV_PID: Any = 6
# Source: msp/msp_protocol.h
MSP_LED_COLORS: Any = 46
# Source: msp/msp_protocol.h
MSP_LED_STRIP_CONFIG: Any = 48
# Source: msp/msp_protocol.h
MSP_LED_STRIP_MODECOLOR: Any = 127
# Source: msp/msp_protocol.h
MSP_LOOP_TIME: Any = 73
# Source: msp/msp_protocol.h
MSP_MAG_CALIBRATION: Any = 206
# Source: msp/msp_serial.h
MSP_MAX_HEADER_SIZE: Any = 9
# Source: msp/msp_protocol.h
MSP_MISC: Any = 114
# Source: msp/msp_protocol.h
MSP_MIXER: Any = 42
# Source: msp/msp_protocol.h
MSP_MODE_RANGES: Any = 34
# Source: msp/msp_protocol.h
MSP_MOTOR: Any = 104
# Source: msp/msp_protocol.h
MSP_NAME: Any = 10
# Source: msp/msp_protocol.h
MSP_NAV_CONFIG: Any = 122
# Source: msp/msp_protocol.h
MSP_NAV_POSHOLD: Any = 12
# Source: msp/msp_protocol.h
MSP_NAV_STATUS: Any = 121
# Source: msp/msp_protocol.h
MSP_OSD_CHAR_READ: Any = 86
# Source: msp/msp_protocol.h
MSP_OSD_CHAR_WRITE: Any = 87
# Source: msp/msp_protocol.h
MSP_OSD_CONFIG: Any = 84
# Source: msp/msp_protocol.h
MSP_OSD_VIDEO_CONFIG: Any = 180
# Source: msp/msp_protocol.h
MSP_PIDNAMES: Any = 117
# Source: msp/msp_protocol.h
MSP_PID_ADVANCED: Any = 94
# Source: msp/msp_serial.h
MSP_PORT_DATAFLASH_BUFFER_SIZE: Any = 4096
# Source: msp/msp_serial.h
MSP_PORT_DATAFLASH_INFO_SIZE: Any = 16
# Source: msp/msp_serial.h
MSP_PORT_INBUF_SIZE: Any = 192
# Source: msp/msp_serial.h
MSP_PORT_OUTBUF_SIZE: Any = 512
# Source: msp/msp_protocol.h
MSP_POSITION_ESTIMATION_CONFIG: Any = 16
# Source: msp/msp_protocol.h
MSP_PROTOCOL_VERSION: Any = 0
# Source: msp/msp_protocol.h
MSP_RAW_GPS: Any = 106
# Source: msp/msp_protocol.h
MSP_RAW_IMU: Any = 102
# Source: msp/msp_protocol.h
MSP_RC: Any = 105
# Source: msp/msp_protocol.h
MSP_RC_DEADBAND: Any = 125
# Source: msp/msp_protocol.h
MSP_RC_TUNING: Any = 111
# Source: msp/msp_protocol.h
MSP_REBOOT: Any = 68
# Source: msp/msp_protocol.h
MSP_RESERVE_1: Any = 251
# Source: msp/msp_protocol.h
MSP_RESERVE_2: Any = 252
# Source: msp/msp_protocol.h
MSP_RESET_CONF: Any = 208
# Source: msp/msp_protocol.h
MSP_RSSI_CONFIG: Any = 50
# Source: msp/msp_protocol.h
MSP_RTC: Any = 246
# Source: msp/msp_protocol.h
MSP_RTH_AND_LAND_CONFIG: Any = 21
# Source: msp/msp_protocol.h
MSP_RX_CONFIG: Any = 44
# Source: msp/msp_protocol.h
MSP_RX_MAP: Any = 64
# Source: msp/msp_protocol.h
MSP_SDCARD_SUMMARY: Any = 79
# Source: msp/msp_protocol.h
MSP_SELECT_SETTING: Any = 210
# Source: msp/msp_protocol.h
MSP_SENSOR_ALIGNMENT: Any = 126
# Source: msp/msp_protocol.h
MSP_SENSOR_CONFIG: Any = 96
# Source: msp/msp_protocol.h
MSP_SENSOR_STATUS: Any = 151
# Source: msp/msp_protocol.h
MSP_SERVO: Any = 103
# Source: msp/msp_protocol.h
MSP_SERVO_CONFIGURATIONS: Any = 120
# Source: msp/msp_protocol.h
MSP_SERVO_MIX_RULES: Any = 241
# Source: msp/msp_protocol.h
MSP_SET_3D: Any = 217
# Source: msp/msp_protocol.h
MSP_SET_ACC_TRIM: Any = 239
# Source: msp/msp_protocol.h
MSP_SET_ADJUSTMENT_RANGE: Any = 53
# Source: msp/msp_protocol.h
MSP_SET_ADVANCED_CONFIG: Any = 91
# Source: msp/msp_protocol.h
MSP_SET_BLACKBOX_CONFIG: Any = 81
# Source: msp/msp_protocol.h
MSP_SET_BOARD_ALIGNMENT: Any = 39
# Source: msp/msp_protocol.h
MSP_SET_BOX: Any = 203
# Source: msp/msp_protocol.h
MSP_SET_CALIBRATION_DATA: Any = 15
# Source: msp/msp_protocol.h
MSP_SET_CF_SERIAL_CONFIG: Any = 55
# Source: msp/msp_protocol.h
MSP_SET_CURRENT_METER_CONFIG: Any = 41
# Source: msp/msp_protocol.h
MSP_SET_FAILSAFE_CONFIG: Any = 76
# Source: msp/msp_protocol.h
MSP_SET_FEATURE: Any = 37
# Source: msp/msp_protocol.h
MSP_SET_FILTER_CONFIG: Any = 93
# Source: msp/msp_protocol.h
MSP_SET_FW_CONFIG: Any = 24
# Source: msp/msp_protocol.h
MSP_SET_HEAD: Any = 211
# Source: msp/msp_protocol.h
MSP_SET_INAV_PID: Any = 7
# Source: msp/msp_protocol.h
MSP_SET_LED_COLORS: Any = 47
# Source: msp/msp_protocol.h
MSP_SET_LED_STRIP_CONFIG: Any = 49
# Source: msp/msp_protocol.h
MSP_SET_LED_STRIP_MODECOLOR: Any = 221
# Source: msp/msp_protocol.h
MSP_SET_LOOP_TIME: Any = 74
# Source: msp/msp_protocol.h
MSP_SET_MISC: Any = 207
# Source: msp/msp_protocol.h
MSP_SET_MIXER: Any = 43
# Source: msp/msp_protocol.h
MSP_SET_MODE_RANGE: Any = 35
# Source: msp/msp_protocol.h
MSP_SET_MOTOR: Any = 214
# Source: msp/msp_protocol.h
MSP_SET_NAME: Any = 11
# Source: msp/msp_protocol.h
MSP_SET_NAV_CONFIG: Any = 215
# Source: msp/msp_protocol.h
MSP_SET_NAV_POSHOLD: Any = 13
# Source: msp/msp_protocol.h
MSP_SET_OSD_CONFIG: Any = 85
# Source: msp/msp_protocol.h
MSP_SET_OSD_VIDEO_CONFIG: Any = 181
# Source: msp/msp_protocol.h
MSP_SET_PASSTHROUGH: Any = 245
# Source: msp/msp_protocol.h
MSP_SET_PID_ADVANCED: Any = 95
# Source: msp/msp_protocol.h
MSP_SET_POSITION_ESTIMATION_CONFIG: Any = 17
# Source: msp/msp_protocol.h
MSP_SET_RAW_GPS: Any = 201
# Source: msp/msp_protocol.h
MSP_SET_RAW_RC: Any = 200
# Source: msp/msp_protocol.h
MSP_SET_RC_DEADBAND: Any = 218
# Source: msp/msp_protocol.h
MSP_SET_RC_TUNING: Any = 204
# Source: msp/msp_protocol.h
MSP_SET_RESET_CURR_PID: Any = 219
# Source: msp/msp_protocol.h
MSP_SET_RSSI_CONFIG: Any = 51
# Source: msp/msp_protocol.h
MSP_SET_RTC: Any = 247
# Source: msp/msp_protocol.h
MSP_SET_RTH_AND_LAND_CONFIG: Any = 22
# Source: msp/msp_protocol.h
MSP_SET_RX_CONFIG: Any = 45
# Source: msp/msp_protocol.h
MSP_SET_RX_MAP: Any = 65
# Source: msp/msp_protocol.h
MSP_SET_SENSOR_ALIGNMENT: Any = 220
# Source: msp/msp_protocol.h
MSP_SET_SENSOR_CONFIG: Any = 97
# Source: msp/msp_protocol.h
MSP_SET_SERVO_CONFIGURATION: Any = 212
# Source: msp/msp_protocol.h
MSP_SET_SERVO_MIX_RULE: Any = 242
# Source: msp/msp_protocol.h
MSP_SET_SPECIAL_PARAMETERS: Any = 99
# Source: msp/msp_protocol.h
MSP_SET_TRANSPONDER_CONFIG: Any = 83
# Source: msp/msp_protocol.h
MSP_SET_TX_INFO: Any = 186
# Source: msp/msp_protocol.h
MSP_SET_VOLTAGE_METER_CONFIG: Any = 57
# Source: msp/msp_protocol.h
MSP_SET_VTX_CONFIG: Any = 89
# Source: msp/msp_protocol.h
MSP_SET_WP: Any = 209
# Source: msp/msp_protocol_v2_inav.h
MSP_SIMULATOR: Any = 0x201F
# Source: msp/msp_protocol.h
MSP_SONAR_ALTITUDE: Any = 58
# Source: msp/msp_protocol.h
MSP_SPECIAL_PARAMETERS: Any = 98
# Source: msp/msp_protocol.h
MSP_STATUS: Any = 101
# Source: msp/msp_protocol.h
MSP_STATUS_EX: Any = 150
# Source: msp/msp_protocol.h
MSP_TRANSPONDER_CONFIG: Any = 82
# Source: msp/msp_protocol.h
MSP_TX_INFO: Any = 187
# Source: msp/msp_protocol.h
MSP_UID: Any = 160
# Source: msp/msp_protocol.h
MSP_V2_FRAME: Any = 255
# Source: msp/msp.h
MSP_V2_FRAME_ID: Any = 255
# Source: msp/msp.h
# Skipped Define: MSP_VERSION_MAGIC_INITIALIZER = { 'M', 'M', 'X' } # (Unparseable/Complex/Filtered)
# Source: msp/msp_protocol.h
MSP_VOLTAGE_METER_CONFIG: Any = 56
# Source: msp/msp_protocol.h
MSP_VTXTABLE_BAND: Any = 137
# Source: msp/msp_protocol.h
MSP_VTXTABLE_POWERLEVEL: Any = 138
# Source: msp/msp_protocol.h
MSP_VTX_CONFIG: Any = 88
# Source: msp/msp_protocol.h
MSP_WP: Any = 118
# Source: msp/msp_protocol.h
MSP_WP_GETINFO: Any = 20
# Source: msp/msp_protocol.h
MSP_WP_MISSION_LOAD: Any = 18
# Source: msp/msp_protocol.h
MSP_WP_MISSION_SAVE: Any = 19
# Source: msp/msp_protocol.h
# Skipped Define: MULTIWII_IDENTIFIER = "MWII"; # (Unparseable/Complex/Filtered)
# Source: common/maths.h
M_Ef: Any = 2.71828182845904523536f
# Source: common/maths.h
M_LN2f: Any = 0.69314718055994530942f
# Source: common/maths.h
M_PIf: Any = 3.14159265358979323846f
# Source: navigation/navigation_private.h
NAV_ACCELERATION_XY_MAX: Any = 980.0f
# Source: navigation/navigation.h
NAV_ACCEL_CUTOFF_FREQUENCY_HZ: Any = 2
# Source: navigation/navigation_private.h
NAV_DTERM_CUT_HZ: Any = 10.0f
# Source: navigation/navigation_private.h
NAV_FW_CONTROL_MONITORING_RATE: Any = 2
# Source: navigation/navigation.h
NAV_MAX_WAYPOINTS: Any = 15
# Source: navigation/rth_trackback.h
NAV_RTH_TRACKBACK_MIN_DIST_TO_START: Any = 50
# Source: navigation/rth_trackback.h
NAV_RTH_TRACKBACK_MIN_TRIP_DIST_TO_SAVE: Any = 10
# Source: navigation/rth_trackback.h
NAV_RTH_TRACKBACK_MIN_XY_DIST_TO_SAVE: Any = 20
# Source: navigation/rth_trackback.h
NAV_RTH_TRACKBACK_MIN_Z_DIST_TO_SAVE: Any = 10
# Source: navigation/rth_trackback.h
NAV_RTH_TRACKBACK_POINTS: Any = 50
# Source: navigation/navigation_private.h
NAV_THROTTLE_CUTOFF_FREQENCY_HZ: Any = 4
# Source: navigation/navigation_private.h
NAV_VEL_Z_DERIVATIVE_CUT_HZ: Any = 5.0f
# Source: navigation/navigation_private.h
NAV_VEL_Z_ERROR_CUT_HZ: Any = 5.0f
# Source: rx/rx.h
NON_AUX_CHANNEL_COUNT: Any = 4
# Source: common/olc.h
OLC_DEG_MULTIPLIER: Any = (10000000)
# Source: fc/config.h
ONESHOT_FEATURE_CHANGED_DELAY_ON_BOOT_MS: Any = 1500
# Source: io/osd_common.h
OSD_AHI_HEIGHT: Any = 9
# Source: io/osd_common.h
OSD_AHI_H_SYM_COUNT: Any = 9
# Source: io/osd_common.h
# Skipped Define: OSD_AHI_PREV_SIZE = (OSD_AHI_WIDTH > OSD_AHI_HEIGHT ? OSD_AHI_WIDTH : OSD_AHI_HEIGHT) # (Unparseable/Complex/Filtered)
# Source: io/osd_common.h
OSD_AHI_V_SYM_COUNT: Any = 6
# Source: io/osd_common.h
OSD_AHI_WIDTH: Any = 11
# Source: io/osd_common.h
OSD_AH_SIDEBAR_HEIGHT_POS: Any = 3
# Source: io/osd_common.h
OSD_AH_SIDEBAR_WIDTH_POS: Any = 7
# Source: io/osd.h
OSD_ALTERNATE_LAYOUT_COUNT: Any = 3
# Source: io/osd_common.h
OSD_HEADING_GRAPH_DECIDEGREES_PER_CHAR: Any = 225
# Source: io/osd_common.h
OSD_HEADING_GRAPH_WIDTH: Any = 9
# Source: io/osd.h
OSD_HOMING_LIM_H1: Any = 6
# Source: io/osd.h
OSD_HOMING_LIM_H2: Any = 16
# Source: io/osd.h
OSD_HOMING_LIM_H3: Any = 38
# Source: io/osd.h
OSD_HOMING_LIM_V1: Any = 5
# Source: io/osd.h
OSD_HOMING_LIM_V2: Any = 10
# Source: io/osd.h
OSD_HOMING_LIM_V3: Any = 15
# Source: io/osd.h
OSD_LAYOUT_COUNT: Any = (OSD_ALTERNATE_LAYOUT_COUNT + 1)
# Source: io/osd.h
OSD_MSG_ACC_FAIL: Any = "ACCELEROMETER FAILURE"
# Source: io/osd.h
OSD_MSG_ACC_NOT_CAL: Any = "ACCELEROMETER NOT CALIBRATED"
# Source: io/osd.h
OSD_MSG_ADJUSTING_WP_ALT: Any = "ADJUSTING WP ALTITUDE"
# Source: io/osd.h
OSD_MSG_AIRCRAFT_UNLEVEL: Any = "AIRCRAFT IS NOT LEVEL"
# Source: io/osd.h
OSD_MSG_ALTITUDE_HOLD: Any = "(ALTITUDE HOLD)"
# Source: io/osd.h
OSD_MSG_ANGLEHOLD_LEVEL: Any = "(ANGLEHOLD LEVEL)"
# Source: io/osd.h
OSD_MSG_ANGLEHOLD_PITCH: Any = "(ANGLEHOLD PITCH)"
# Source: io/osd.h
OSD_MSG_ANGLEHOLD_ROLL: Any = "(ANGLEHOLD ROLL)"
# Source: io/osd.h
OSD_MSG_AUTOLAUNCH: Any = "AUTOLAUNCH"
# Source: io/osd.h
# Skipped Define: OSD_MSG_AUTOLAUNCH_MANUAL = "AUTOLAUNCH (MANUAL)" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
OSD_MSG_AUTOLEVEL: Any = "(AUTO LEVEL TRIM)"
# Source: io/osd.h
OSD_MSG_AUTOTRIM: Any = ""
# Source: io/osd.h
OSD_MSG_AUTOTRIM_ACTIVE: Any = "AUTOTRIM IS ACTIVE"
# Source: io/osd.h
OSD_MSG_AUTOTUNE: Any = ""
# Source: io/osd.h
OSD_MSG_AUTOTUNE_ACRO: Any = "SWITCH TO ACRO"
# Source: io/osd.h
OSD_MSG_AVOIDING_ALT_BREACH: Any = "REACHED ZONE ALTITUDE LIMIT"
# Source: io/osd.h
OSD_MSG_AVOIDING_FB: Any = "AVOIDING FENCE BREACH"
# Source: io/osd.h
OSD_MSG_AVOID_ZONES_RTH: Any = "AVOIDING NO FLY ZONES"
# Source: io/osd.h
OSD_MSG_BARO_FAIL: Any = "BAROMETER FAILURE"
# Source: io/osd.h
OSD_MSG_CLI_ACTIVE: Any = "CLI IS ACTIVE"
# Source: io/osd.h
OSD_MSG_DISABLED_BY_FS: Any = "DISABLED BY FAILSAFE"
# Source: io/osd.h
OSD_MSG_DISABLE_NAV_FIRST: Any = "DISABLE NAVIGATION FIRST"
# Source: io/osd.h
OSD_MSG_DISARM_1ST: Any = "DISABLE ARM SWITCH FIRST"
# Source: io/osd.h
OSD_MSG_DIVERT_SAFEHOME: Any = "DIVERTING TO SAFEHOME"
# Source: io/osd.h
OSD_MSG_DSHOT_BEEPER: Any = "MOTOR BEEPER ACTIVE"
# Source: io/osd.h
OSD_MSG_EMERG_LANDING: Any = "EMERGENCY LANDING"
# Source: io/osd.h
OSD_MSG_EMERG_LANDING_FS: Any = "(EMERGENCY LANDING)"
# Source: io/osd.h
OSD_MSG_ENTERING_NFZ: Any = "ENTERING NFZ IN %s %s"
# Source: io/osd.h
OSD_MSG_FLYOUT_NFZ: Any = "FLY OUT NFZ"
# Source: io/osd.h
OSD_MSG_FS_EN: Any = "FAILSAFE MODE ENABLED"
# Source: io/osd.h
OSD_MSG_GEOZONE_ACTION: Any = "PERFORM ACTION IN %s %s"
# Source: io/osd.h
OSD_MSG_GPS_FAIL: Any = "GPS FAILURE"
# Source: io/osd.h
# Skipped Define: OSD_MSG_GRD_TEST_MODE = "GRD TEST > MOTORS DISABLED" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
OSD_MSG_GYRO_FAILURE: Any = "GYRO FAILURE"
# Source: io/osd.h
OSD_MSG_HEADFREE: Any = ""
# Source: io/osd.h
OSD_MSG_HEADING_HOME: Any = "EN ROUTE TO HOME"
# Source: io/osd.h
OSD_MSG_HOVERING: Any = "HOVERING"
# Source: io/osd.h
OSD_MSG_HW_FAIL: Any = "HARDWARE FAILURE"
# Source: io/osd.h
OSD_MSG_INVALID_SETTING: Any = "INVALID SETTING"
# Source: io/osd.h
OSD_MSG_JUMP_WP_MISCONFIG: Any = "JUMP WAYPOINT MISCONFIGURED"
# Source: io/osd.h
OSD_MSG_LANDED: Any = "LANDED"
# Source: io/osd.h
OSD_MSG_LANDING: Any = "LANDING"
# Source: io/osd.h
OSD_MSG_LEAVING_FZ: Any = "LEAVING FZ IN %s"
# Source: io/osd.h
OSD_MSG_LOITERING_HOME: Any = "LOITERING AROUND HOME"
# Source: io/osd.h
OSD_MSG_LOITERING_SAFEHOME: Any = "LOITERING AROUND SAFEHOME"
# Source: io/osd.h
OSD_MSG_MAG_FAIL: Any = "COMPASS FAILURE"
# Source: io/osd.h
OSD_MSG_MAG_NOT_CAL: Any = "COMPASS NOT CALIBRATED"
# Source: io/osd.h
OSD_MSG_MISSION_PLANNER: Any = "(WP MISSION PLANNER)"
# Source: io/osd.h
OSD_MSG_MOVE_EXIT_FS: Any = "!MOVE STICKS TO EXIT FS!"
# Source: io/osd.h
OSD_MSG_MOVE_STICKS: Any = "MOVE STICKS TO ABORT"
# Source: io/osd.h
OSD_MSG_NAV_SOARING: Any = ""
# Source: io/osd.h
OSD_MSG_NFZ: Any = "NO FLY ZONE"
# Source: io/osd.h
OSD_MSG_NOT_ENOUGH_MEMORY: Any = "NOT ENOUGH MEMORY"
# Source: io/osd.h
OSD_MSG_NO_PREARM: Any = "NO PREARM"
# Source: io/osd.h
OSD_MSG_NO_RC_LINK: Any = "NO RC LINK"
# Source: io/osd.h
OSD_MSG_OUTSIDE_FZ: Any = "OUTSIDE FZ"
# Source: io/osd.h
OSD_MSG_PITOT_FAIL: Any = "PITOT METER FAILURE"
# Source: io/osd.h
OSD_MSG_PREPARE_NEXT_WP: Any = "PREPARING FOR NEXT WAYPOINT"
# Source: io/osd.h
OSD_MSG_PREPARING_LAND: Any = "PREPARING TO LAND"
# Source: io/osd.h
OSD_MSG_PWM_INIT_ERROR: Any = "PWM INIT ERROR"
# Source: io/osd.h
OSD_MSG_RANGEFINDER_FAIL: Any = "RANGE FINDER FAILURE"
# Source: io/osd.h
OSD_MSG_RC_RX_LINK_LOST: Any = "!RC RX LINK LOST!"
# Source: io/osd.h
OSD_MSG_RETURN_TO_ZONE: Any = "RETURN TO FZ"
# Source: io/osd.h
OSD_MSG_ROLLPITCH_OFFCENTER: Any = "ROLLPITCH NOT CENTERED"
# Source: io/osd.h
OSD_MSG_RTH_CLIMB: Any = "ADJUSTING RTH ALTITUDE"
# Source: io/osd.h
OSD_MSG_RTH_FS: Any = ""
# Source: io/osd.h
OSD_MSG_RTH_LINEAR_DESCENT: Any = "BEGIN LINEAR DESCENT"
# Source: io/osd.h
OSD_MSG_RTH_TRACKBACK: Any = "RTH BACK TRACKING"
# Source: io/osd.h
# Skipped Define: OSD_MSG_SAVING_SETTNGS = "** SAVING SETTINGS **" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
OSD_MSG_SENSORS_CAL: Any = "SENSORS CALIBRATING"
# Source: io/osd.h
# Skipped Define: OSD_MSG_SETTINGS_SAVED = "** SETTINGS SAVED **" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
OSD_MSG_STARTING_RTH: Any = "STARTING RTH"
# Source: io/osd.h
OSD_MSG_SYS_OVERLOADED: Any = "SYSTEM OVERLOADED"
# Source: io/osd.h
OSD_MSG_THROTTLE_NOT_LOW: Any = "THROTTLE IS NOT LOW"
# Source: io/osd.h
OSD_MSG_TURN_ARM_SW_OFF: Any = "TURN ARM SWITCH OFF"
# Source: io/osd.h
OSD_MSG_UNABLE_ARM: Any = "UNABLE TO ARM"
# Source: io/osd.h
OSD_MSG_WAITING_GPS_FIX: Any = "WAITING FOR GPS FIX"
# Source: io/osd.h
# Skipped Define: OSD_MSG_WP_FINISHED = "WP END>HOLDING POSITION" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
# Skipped Define: OSD_MSG_WP_LANDED = "WP END>LANDED" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
# Skipped Define: OSD_MSG_WP_MISSION_LOADED = "* MISSION LOADED *" # (Unparseable/Complex/Filtered)
# Source: io/osd.h
OSD_MSG_WP_RTH_CANCEL: Any = "CANCEL WP TO EXIT RTH"
# Source: io/osd.h
OSD_POS_MAX: Any = 0xFFF
# Source: io/osd.h
OSD_POS_MAX_CLI: Any = (OSD_POS_MAX | OSD_VISIBLE_FLAG)
# Source: io/osd.h
OSD_SWITCH_INDICATOR_NAME_LENGTH: Any = 4
# Source: io/osd_common.h
OSD_VARIO_CM_S_PER_ARROW: Any = 50
# Source: io/osd_common.h
OSD_VARIO_HEIGHT_ROWS: Any = 5
# Source: io/osd.h
OSD_VISIBLE_FLAG: Any = 0x2000
# Source: io/osd.h
OSD_VISIBLE_FLAG_SD: Any = 0x0800
# Source: rx/srxl2_types.h
# Skipped Define: PACKED = __attribute__((packed)) # (Unparseable/Complex/Filtered)
# Source: flight/failsafe.h
# Skipped Define: PERIOD_OF_1_SECONDS = 1 * MILLIS_PER_SECOND # (Unparseable/Complex/Filtered)
# Source: flight/failsafe.h
# Skipped Define: PERIOD_OF_30_SECONDS = 30 * MILLIS_PER_SECOND # (Unparseable/Complex/Filtered)
# Source: flight/failsafe.h
# Skipped Define: PERIOD_OF_3_SECONDS = 3 * MILLIS_PER_SECOND # (Unparseable/Complex/Filtered)
# Source: flight/failsafe.h
PERIOD_RXDATA_FAILURE: Any = 200
# Source: flight/failsafe.h
PERIOD_RXDATA_RECOVERY: Any = 200
# Source: config/parameter_group_ids.h
PG_ACCELEROMETER_CONFIG: Any = 35
# Source: config/parameter_group_ids.h
PG_ADC_CHANNEL_CONFIG: Any = 1010
# Source: config/parameter_group_ids.h
PG_ADJUSTMENT_RANGE_CONFIG: Any = 37
# Source: config/parameter_group_ids.h
PG_ARMING_CONFIG: Any = 16
# Source: config/parameter_group_ids.h
PG_BAROMETER_CONFIG: Any = 38
# Source: config/parameter_group_ids.h
PG_BATTERY_METERS_CONFIG: Any = 45
# Source: config/parameter_group_ids.h
PG_BATTERY_PROFILES: Any = 11
# Source: config/parameter_group_ids.h
PG_BEEPER_CONFIG: Any = 1005
# Source: config/parameter_group_ids.h
PG_BLACKBOX_CONFIG: Any = 5
# Source: config/parameter_group_ids.h
PG_BOARD_ALIGNMENT: Any = 2
# Source: config/parameter_group_ids.h
PG_CF_END: Any = 56
# Source: config/parameter_group_ids.h
PG_CF_START: Any = 1
# Source: config/parameter_group_ids.h
PG_COMPASS_CONFIG: Any = 40
# Source: config/parameter_group_ids.h
PG_CONTROL_RATE_PROFILES: Any = 12
# Source: config/parameter_group_ids.h
PG_DISPLAY_CONFIG: Any = 1013
# Source: config/parameter_group_ids.h
PG_DJI_OSD_CONFIG: Any = 1027
# Source: config/parameter_group_ids.h
PG_ESC_SENSOR_CONFIG: Any = 1021
# Source: config/parameter_group_ids.h
PG_EZ_TUNE: Any = 1033
# Source: config/parameter_group_ids.h
PG_FAILSAFE_CONFIG: Any = 1
# Source: config/parameter_group_ids.h
PG_FEATURE_CONFIG: Any = 19
# Source: config/parameter_group_ids.h
PG_FW_AUTOLAND_APPROACH_CONFIG: Any = 1037
# Source: config/parameter_group_ids.h
PG_FW_AUTOLAND_CONFIG: Any = 1036
# Source: config/parameter_group_ids.h
PG_GENERAL_SETTINGS: Any = 1019
# Source: config/parameter_group_ids.h
PG_GEOZONES: Any = 1043
# Source: config/parameter_group_ids.h
PG_GEOZONE_CONFIG: Any = 1042
# Source: config/parameter_group_ids.h
PG_GEOZONE_VERTICES: Any = 1044
# Source: config/parameter_group_ids.h
PG_GIMBAL_CONFIG: Any = 1039
# Source: config/parameter_group_ids.h
PG_GIMBAL_SERIAL_CONFIG: Any = 1040
# Source: config/parameter_group_ids.h
PG_GLOBAL_FUNCTIONS: Any = 1020
# Source: config/parameter_group_ids.h
PG_GLOBAL_VARIABLE_CONFIG: Any = 1023
# Source: config/parameter_group_ids.h
PG_GPS_CONFIG: Any = 30
# Source: config/parameter_group_ids.h
PG_GYRO_CONFIG: Any = 10
# Source: config/parameter_group_ids.h
PG_HEADTRACKER_CONFIG: Any = 1041
# Source: config/parameter_group_ids.h
PG_ID_FIRST: Any = PG_CF_START
# Source: config/parameter_group_ids.h
PG_ID_INVALID: Any = 0
# Source: config/parameter_group_ids.h
PG_ID_LAST: Any = PG_INAV_END
# Source: config/parameter_group_ids.h
PG_IMU_CONFIG: Any = 22
# Source: config/parameter_group_ids.h
PG_INAV_END: Any = PG_GEOZONE_VERTICES
# Source: config/parameter_group_ids.h
PG_INAV_START: Any = 1000
# Source: config/parameter_group_ids.h
PG_LEDPIN_CONFIG: Any = 1034
# Source: config/parameter_group_ids.h
PG_LED_STRIP_CONFIG: Any = 27
# Source: config/parameter_group_ids.h
PG_LIGHTS_CONFIG: Any = 1014
# Source: config/parameter_group_ids.h
PG_LOGIC_CONDITIONS: Any = 1016
# Source: config/parameter_group_ids.h
PG_LOG_CONFIG: Any = 1017
# Source: config/parameter_group_ids.h
PG_MIXER_PROFILE: Any = 20
# Source: config/parameter_group_ids.h
PG_MODE_ACTIVATION_OPERATOR_CONFIG: Any = 1003
# Source: config/parameter_group_ids.h
PG_MODE_ACTIVATION_PROFILE: Any = 41
# Source: config/parameter_group_ids.h
PG_MOTOR_CONFIG: Any = 6
# Source: config/parameter_group_ids.h
PG_MOTOR_MIXER: Any = 4
# Source: config/parameter_group_ids.h
PG_NAV_CONFIG: Any = 1002
# Source: config/parameter_group_ids.h
PG_OPFLOW_CONFIG: Any = 1012
# Source: config/parameter_group_ids.h
PG_OSD_COMMON_CONFIG: Any = 1031
# Source: config/parameter_group_ids.h
PG_OSD_CONFIG: Any = 1004
# Source: config/parameter_group_ids.h
PG_OSD_CUSTOM_ELEMENTS_CONFIG: Any = 1038
# Source: config/parameter_group_ids.h
PG_OSD_JOYSTICK_CONFIG: Any = 1035
# Source: config/parameter_group_ids.h
PG_OSD_LAYOUTS_CONFIG: Any = 1025
# Source: config/parameter_group.h
# Skipped Define: PG_PACKED = __attribute__((packed)) # (Unparseable/Complex/Filtered)
# Source: config/parameter_group_ids.h
PG_PID_AUTOTUNE_CONFIG: Any = 1008
# Source: config/parameter_group_ids.h
PG_PID_PROFILE: Any = 14
# Source: config/parameter_group_ids.h
PG_PINIOBOX_CONFIG: Any = 1015
# Source: config/parameter_group_ids.h
PG_PITOTMETER_CONFIG: Any = 1000
# Source: config/parameter_group_ids.h
PG_POSITION_ESTIMATION_CONFIG: Any = 1001
# Source: config/parameter_group_ids.h
PG_POWER_LIMITS_CONFIG: Any = 1030
# Source: config/parameter_group_ids.h
PG_PROGRAMMING_PID: Any = 1028
# Source: config/parameter_group_ids.h
PG_RANGEFINDER_CONFIG: Any = 1006
# Source: config/parameter_group_ids.h
PG_RCDEVICE_CONFIG: Any = 1018
# Source: config/parameter_group_ids.h
PG_RC_CONTROLS_CONFIG: Any = 25
# Source: config/parameter_group.h
# Skipped Define: PG_REGISTER_ATTRIBUTES = __attribute__ ((section(".pg_registry"), used, aligned(4))) # (Unparseable/Complex/Filtered)
# Source: config/parameter_group.h
# Skipped Define: PG_REGISTRY_SIZE = (__pg_registry_end - __pg_registry_start) # (Unparseable/Complex/Filtered)
# Source: config/parameter_group_ids.h
PG_RESERVED_FOR_TESTING_1: Any = 4095
# Source: config/parameter_group_ids.h
PG_RESERVED_FOR_TESTING_2: Any = 4094
# Source: config/parameter_group_ids.h
PG_RESERVED_FOR_TESTING_3: Any = 4093
# Source: config/parameter_group.h
# Skipped Define: PG_RESETDATA_ATTRIBUTES = __attribute__ ((section(".pg_resetdata"), used, aligned(2))) # (Unparseable/Complex/Filtered)
# Source: config/parameter_group_ids.h
PG_REVERSIBLE_MOTORS_CONFIG: Any = 26
# Source: config/parameter_group_ids.h
PG_RPM_FILTER_CONFIG: Any = 1022
# Source: config/parameter_group_ids.h
PG_RX_CHANNEL_RANGE_CONFIG: Any = 44
# Source: config/parameter_group_ids.h
PG_RX_CONFIG: Any = 24
# Source: config/parameter_group_ids.h
PG_SAFE_HOME_CONFIG: Any = 1026
# Source: config/parameter_group_ids.h
PG_SERIAL_CONFIG: Any = 13
# Source: config/parameter_group_ids.h
PG_SERVO_CONFIG: Any = 52
# Source: config/parameter_group_ids.h
PG_SERVO_PARAMS: Any = 42
# Source: config/parameter_group_ids.h
PG_SMARTPORT_MASTER_CONFIG: Any = 1024
# Source: config/parameter_group_ids.h
PG_STATS_CONFIG: Any = 1009
# Source: config/parameter_group_ids.h
PG_SYSTEM_CONFIG: Any = 18
# Source: config/parameter_group_ids.h
PG_TELEMETRY_CONFIG: Any = 31
# Source: config/parameter_group_ids.h
PG_TEMP_SENSOR_CONFIG: Any = 56
# Source: config/parameter_group_ids.h
PG_TIMER_OVERRIDE_CONFIG: Any = 1032
# Source: config/parameter_group_ids.h
PG_TIME_CONFIG: Any = 1011
# Source: config/parameter_group_ids.h
PG_UNUSED_1: Any = 1029
# Source: config/parameter_group_ids.h
PG_VTX_CONFIG: Any = 54
# Source: config/parameter_group_ids.h
PG_VTX_SETTINGS_CONFIG: Any = 259
# Source: config/parameter_group_ids.h
PG_WAYPOINT_MISSION_STORAGE: Any = 1007
# Source: flight/pid.h
PID_SUM_LIMIT_DEFAULT: Any = 500
# Source: flight/pid.h
PID_SUM_LIMIT_MAX: Any = 1000
# Source: flight/pid.h
PID_SUM_LIMIT_MIN: Any = 100
# Source: flight/pid.h
PID_SUM_LIMIT_YAW_DEFAULT: Any = 400
# Source: sensors/pitotmeter.h
PITOT_MAX: Any = PITOT_FAKE
# Source: sensors/pitotmeter.h
PITOT_SAMPLE_COUNT_MAX: Any = 48
# Source: rx/rx.h
PWM_PULSE_MAX: Any = 2250
# Source: rx/rx.h
PWM_PULSE_MIN: Any = 750
# Source: rx/rx.h
PWM_RANGE_MAX: Any = 2000
# Source: rx/rx.h
# Skipped Define: PWM_RANGE_MIDDLE = (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2)) # (Unparseable/Complex/Filtered)
# Source: rx/rx.h
PWM_RANGE_MIN: Any = 1000
# Source: msp/msp_protocol.h
RACEFLIGHT_IDENTIFIER: Any = "RCFL"
# Source: common/maths.h
# Skipped Define: RAD = (M_PIf / 180.0f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation.h
RADAR_MAX_POIS: Any = 5
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: RANGEFINDER_RELIABILITY_HIGH_THRESHOLD = (0.75f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: RANGEFINDER_RELIABILITY_LIGHT_THRESHOLD = (0.15f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: RANGEFINDER_RELIABILITY_LOW_THRESHOLD = (0.33f) # (Unparseable/Complex/Filtered)
# Source: navigation/navigation_pos_estimator_private.h
# Skipped Define: RANGEFINDER_RELIABILITY_RC_CONSTANT = (0.47802f) # (Unparseable/Complex/Filtered)
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION: Any = 0x04
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS: Any = 0x02
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE: Any = 0x03
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL: Any = 0x01
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO: Any = 0x00
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_HEADER: Any = 0xCC
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_MAX_DATA_SIZE: Any = 20
# Source: io/rcdevice.h
RCDEVICE_PROTOCOL_MAX_PACKET_SIZE: Any = 64
# Source: io/rcdevice.h
RCSPLIT_PACKET_CMD_CTRL: Any = 0x01
# Source: io/rcdevice.h
RCSPLIT_PACKET_HEADER: Any = 0x55
# Source: io/rcdevice.h
RCSPLIT_PACKET_TAIL: Any = 0xaa
# Source: rx/rx.h
# Skipped Define: REMAPPABLE_CHANNEL_COUNT = ARRAYLEN(((rxConfig_t *)0)->rcmap) # (Unparseable/Complex/Filtered)
# Source: common/color.h
RGB_COLOR_COMPONENT_COUNT: Any = (RGB_BLUE + 1)
# Source: flight/rpm_filter.h
RPM_FILTER_UPDATE_RATE_HZ: Any = 500
# Source: flight/rpm_filter.h
# Skipped Define: RPM_FILTER_UPDATE_RATE_US = (1000000.0f / RPM_FILTER_UPDATE_RATE_HZ) # (Unparseable/Complex/Filtered)
# Source: rx/rx.h
RSSI_MAX_VALUE: Any = 1023
# Source: rx/rx.h
RSSI_VISIBLE_FACTOR: Any = (RSSI_MAX_VALUE/RSSI_VISIBLE_VALUE_MAX)
# Source: rx/rx.h
RSSI_VISIBLE_VALUE_MAX: Any = 100
# Source: rx/rx.h
RSSI_VISIBLE_VALUE_MIN: Any = 0
# Source: io/vtx_smartaudio.h
SA_FREQ_GETPIT: Any = (1 << 14)
# Source: io/vtx_smartaudio.h
SA_FREQ_MASK: Any = (~(SA_FREQ_GETPIT|SA_FREQ_SETPIT))
# Source: io/vtx_smartaudio.h
SA_FREQ_SETPIT: Any = (1 << 15)
# Source: io/vtx_smartaudio.h
SA_MODE_CLR_PITMODE: Any = 4
# Source: io/vtx_smartaudio.h
SA_MODE_GET_DEFERRED_FREQ: Any = 32
# Source: io/vtx_smartaudio.h
SA_MODE_GET_FREQ_BY_FREQ: Any = 1
# Source: io/vtx_smartaudio.h
SA_MODE_GET_IN_RANGE_PITMODE: Any = 4
# Source: io/vtx_smartaudio.h
SA_MODE_GET_OUT_RANGE_PITMODE: Any = 8
# Source: io/vtx_smartaudio.h
SA_MODE_GET_PITMODE: Any = 2
# Source: io/vtx_smartaudio.h
SA_MODE_GET_UNLOCK: Any = 16
# Source: io/vtx_smartaudio.h
SA_MODE_SET_DEFERRED_FREQ: Any = 16
# Source: io/vtx_smartaudio.h
SA_MODE_SET_IN_RANGE_PITMODE: Any = 1
# Source: io/vtx_smartaudio.h
SA_MODE_SET_LOCK: Any = 0
# Source: io/vtx_smartaudio.h
SA_MODE_SET_OUT_RANGE_PITMODE: Any = 2
# Source: io/vtx_smartaudio.h
SA_MODE_SET_UNLOCK: Any = 8
# Source: io/gps.h
SBAS_MODE_MAX: Any = SBAS_GAGAN
# Source: telemetry/sbus2.h
# Skipped Define: SBUS2_DEADTIME = MS2US(2) # (Unparseable/Complex/Filtered)
# Source: rx/sbus_channels.h
SBUS2_HIGHFRAME_BEGIN_BYTE: Any = (0x2F)
# Source: telemetry/sbus2.h
# Skipped Define: SBUS2_SLOT_COUNT = (SBUS2_TELEMETRY_PAGES * SBUS2_TELEMETRY_SLOTS) # (Unparseable/Complex/Filtered)
# Source: telemetry/sbus2.h
SBUS2_SLOT_DELAY: Any = 200
# Source: telemetry/sbus2.h
SBUS2_SLOT_TIME: Any = 650
# Source: telemetry/sbus2.h
SBUS2_TELEMETRY_ITEM_SIZE: Any = 3
# Source: telemetry/sbus2.h
SBUS2_TELEMETRY_PAGES: Any = 4
# Source: telemetry/sbus2.h
SBUS2_TELEMETRY_PAYLOAD_SIZE: Any = 3
# Source: telemetry/sbus2.h
SBUS2_TELEMETRY_SLOTS: Any = 8
# Source: telemetry/sbus2.h
# Skipped Define: SBUS2_TRANSMIT_TIME = ((8 + 1 + 2 + 1 + 1) * 3 * 10) # (Unparseable/Complex/Filtered)
# Source: rx/sbus_channels.h
SBUS_BAUDRATE: Any = 100000
# Source: rx/sbus_channels.h
SBUS_BAUDRATE_FAST: Any = 200000
# Source: rx/sbus_channels.h
# Skipped Define: SBUS_CHANNEL_DATA_LENGTH = sizeof(sbusChannels_t) # (Unparseable/Complex/Filtered)
# Source: rx/sbus.h
SBUS_DEFAULT_INTERFRAME_DELAY_US: Any = 3000
# Source: rx/sbus_channels.h
SBUS_FLAG_CHANNEL_DG1: Any = (1 << 0)
# Source: rx/sbus_channels.h
SBUS_FLAG_CHANNEL_DG2: Any = (1 << 1)
# Source: rx/sbus_channels.h
SBUS_FLAG_FAILSAFE_ACTIVE: Any = (1 << 3)
# Source: rx/sbus_channels.h
SBUS_FLAG_SIGNAL_LOSS: Any = (1 << 2)
# Source: rx/sbus_channels.h
SBUS_FRAME_BEGIN_BYTE: Any = (0x0F)
# Source: rx/sbus_channels.h
SBUS_FRAME_SIZE: Any = (SBUS_CHANNEL_DATA_LENGTH + 2)
# Source: rx/sbus_channels.h
SBUS_MAX_CHANNEL: Any = 34
# Source: rx/sbus_channels.h
SBUS_PORT_OPTIONS: Any = (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
# Source: flight/servos.h
SERVO_DUALCOPTER_INDEX_MAX: Any = SERVO_DUALCOPTER_RIGHT
# Source: flight/servos.h
SERVO_DUALCOPTER_INDEX_MIN: Any = SERVO_DUALCOPTER_LEFT
# Source: flight/servos.h
SERVO_FLAPPERONS_MAX: Any = SERVO_FLAPPERON_2
# Source: flight/servos.h
SERVO_FLAPPERONS_MIN: Any = SERVO_FLAPPERON_1
# Source: flight/servos.h
SERVO_OUTPUT_MAX: Any = 2500
# Source: flight/servos.h
SERVO_OUTPUT_MIN: Any = 500
# Source: flight/servos.h
SERVO_PLANE_INDEX_MAX: Any = SERVO_RUDDER
# Source: flight/servos.h
SERVO_PLANE_INDEX_MIN: Any = SERVO_ELEVATOR
# Source: io/servo_sbus.h
SERVO_SBUS_MAX_SERVOS: Any = 18
# Source: flight/servos.h
SERVO_SINGLECOPTER_INDEX_MAX: Any = SERVO_SINGLECOPTER_4
# Source: flight/servos.h
SERVO_SINGLECOPTER_INDEX_MIN: Any = SERVO_SINGLECOPTER_1
# Source: fc/settings.h
SETTING_MODE_MASK: Any = (0xC0)
# Source: fc/settings.h
SETTING_MODE_OFFSET: Any = 6
# Source: fc/settings.h
SETTING_SECTION_MASK: Any = (0x38)
# Source: fc/settings.h
SETTING_SECTION_OFFSET: Any = 3
# Source: fc/settings.h
SETTING_TYPE_MASK: Any = (0x07)
# Source: fc/settings.h
SETTING_TYPE_OFFSET: Any = 0
# Source: fc/runtime_config.h
SIMULATOR_BARO_TEMP: Any = 25
# Source: fc/runtime_config.h
SIMULATOR_FULL_BATTERY: Any = 126
# Source: fc/runtime_config.h
SIMULATOR_MSP_VERSION: Any = 2
# Source: telemetry/sim.h
SIM_DEFAULT_TRANSMIT_INTERVAL: Any = 60
# Source: telemetry/sim.h
SIM_DEFAULT_TX_FLAGS: Any = "f"
# Source: telemetry/sim.h
SIM_MIN_TRANSMIT_INTERVAL: Any = 10
# Source: telemetry/sim.h
SIM_N_TX_FLAGS: Any = 5
# Source: telemetry/sim.h
SIM_PIN: Any = "0000"
# Source: telemetry/smartport.h
SMARTPORT_MSP_RX_BUF_SIZE: Any = 64
# Source: telemetry/smartport.h
SMARTPORT_MSP_TX_BUF_SIZE: Any = 256
# Source: rx/spektrum.h
SPEKTRUM_1024_CHANNEL_COUNT: Any = 7
# Source: rx/spektrum.h
SPEKTRUM_2048_CHANNEL_COUNT: Any = 12
# Source: rx/spektrum.h
SPEKTRUM_BAUDRATE: Any = 115200
# Source: rx/spektrum.h
SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT: Any = 12
# Source: rx/spektrum.h
SPEKTRUM_NEEDED_FRAME_INTERVAL: Any = 5000
# Source: rx/spektrum.h
SPEKTRUM_SAT_BIND_DISABLED: Any = 0
# Source: rx/spektrum.h
SPEKTRUM_SAT_BIND_MAX: Any = 10
# Source: telemetry/srxl.h
SPEKTRUM_SRXL_TEXTGEN_BUFFER_COLS: Any = 12
# Source: telemetry/srxl.h
SPEKTRUM_SRXL_TEXTGEN_BUFFER_ROWS: Any = 9
# Source: telemetry/srxl.h
SPEKTRUM_SRXL_TEXTGEN_CLEAR_SCREEN: Any = 255
# Source: rx/spektrum.h
SPEK_FRAME_SIZE: Any = 16
# Source: rx/srxl2_types.h
SRXL_BIND_OPT_BIND_TX_ENABLE: Any = (0x02)
# Source: rx/srxl2_types.h
SRXL_BIND_OPT_NONE: Any = (0x00)
# Source: rx/srxl2_types.h
SRXL_BIND_OPT_TELEM_TX_ENABLE: Any = (0x01)
# Source: rx/spektrum.h
SRXL_FRAME_OVERHEAD: Any = 5
# Source: rx/spektrum.h
SRXL_FRAME_SIZE_MAX: Any = (SPEK_FRAME_SIZE + SRXL_FRAME_OVERHEAD)
# Source: common/maths.h
SSL_AIR_DENSITY: Any = 1.225f
# Source: common/maths.h
SSL_AIR_PRESSURE: Any = 101325.01576f
# Source: common/maths.h
SSL_AIR_TEMPERATURE: Any = 288.15f
# Source: rx/rx.h
STICK_CHANNEL_COUNT: Any = 4
# Source: flight/pid.h
TASK_AUX_RATE_HZ: Any = 100
# Source: fc/config.h
TASK_GYRO_LOOPTIME: Any = 250
# Source: telemetry/telemetry.h
TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK: Any = (FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_IBUS)
# Source: sensors/temperature.h
# Skipped Define: TEMPERATURE_INVALID_VALUE = -1250 # (Unparseable/Complex/Filtered)
# Source: sensors/temperature.h
TEMPERATURE_LABEL_LEN: Any = 4
# Source: common/time.h
TIMEDELTALARGE_MAX: Any = INT64_MAX
# Source: common/time.h
TIMEDELTA_MAX: Any = INT32_MAX
# Source: common/time.h
TIMEMS_MAX: Any = UINT32_MAX
# Source: common/time.h
TIMEUS_MAX: Any = UINT32_MAX
# Source: io/gps_ublox.h
UBLOX_BUFFER_SIZE: Any = MAX_UBLOX_PAYLOAD_SIZE
# Source: io/gps_ublox.h
UBLOX_CFG_GLO_ENA: Any = 0x10310025
# Source: io/gps_ublox.h
UBLOX_CFG_GLO_L1_ENA: Any = 0x10310018
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_POSLLH_UART1: Any = 0x2091002a
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_PVT_UART1: Any = 0x20910007
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_SAT_UART1: Any = 0x20910016
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_SIG_UART1: Any = 0x20910346
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_STATUS_UART1: Any = 0x2091001b
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_TIMEUTC_UART1: Any = 0x2091005c
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NAV_VELNED_UART1: Any = 0x20910043
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1: Any = 0x209100bb
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1: Any = 0x209100ca
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1: Any = 0x209100c0
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1: Any = 0x209100ac
# Source: io/gps_ublox.h
UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1: Any = 0x209100b1
# Source: io/gps_ublox.h
UBLOX_CFG_NAVSPG_DYNMODEL: Any = 0x20110021
# Source: io/gps_ublox.h
UBLOX_CFG_NAVSPG_FIXMODE: Any = 0x20110011
# Source: io/gps_ublox.h
UBLOX_CFG_QZSS_ENA: Any = 0x10310024
# Source: io/gps_ublox.h
UBLOX_CFG_QZSS_L1CA_ENA: Any = 0x10310012
# Source: io/gps_ublox.h
UBLOX_CFG_QZSS_L1S_ENA: Any = 0x10310014
# Source: io/gps_ublox.h
UBLOX_CFG_RATE_MEAS: Any = 0x30210001
# Source: io/gps_ublox.h
UBLOX_CFG_RATE_NAV: Any = 0x30210002
# Source: io/gps_ublox.h
UBLOX_CFG_RATE_TIMEREF: Any = 0x30210002
# Source: io/gps_ublox.h
UBLOX_CFG_SBAS_PRNSCANMASK: Any = 0x50360006
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_BDS_B1C_ENA: Any = 0x1031000f
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_BDS_B1_ENA: Any = 0x1031000d
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_BDS_ENA: Any = 0x10310022
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_GAL_E1_ENA: Any = 0x10310007
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_GAL_ENA: Any = 0x10310021
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_SBAS_ENA: Any = 0x10310020
# Source: io/gps_ublox.h
UBLOX_CFG_SIGNAL_SBAS_L1CA_ENA: Any = 0x10310005
# Source: io/gps_ublox.h
UBLOX_MAX_SIGNALS: Any = 64
# Source: io/gps_ublox.h
UBLOX_SBAS_ALL: Any = 0x0000000000000000
# Source: io/gps_ublox.h
UBLOX_SBAS_MESSAGE_LENGTH: Any = 16
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN120: Any = 0x0000000000000001
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN121: Any = 0x0000000000000002
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN122: Any = 0x0000000000000004
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN123: Any = 0x0000000000000008
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN124: Any = 0x0000000000000010
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN125: Any = 0x0000000000000020
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN126: Any = 0x0000000000000040
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN127: Any = 0x0000000000000080
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN128: Any = 0x0000000000000100
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN129: Any = 0x0000000000000200
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN130: Any = 0x0000000000000400
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN131: Any = 0x0000000000000800
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN132: Any = 0x0000000000001000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN133: Any = 0x0000000000002000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN134: Any = 0x0000000000004000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN135: Any = 0x0000000000008000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN136: Any = 0x0000000000010000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN137: Any = 0x0000000000020000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN138: Any = 0x0000000000040000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN139: Any = 0x0000000000080000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN140: Any = 0x0000000000100000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN141: Any = 0x0000000000200000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN142: Any = 0x0000000000400000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN143: Any = 0x0000000000800000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN144: Any = 0x0000000001000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN145: Any = 0x0000000002000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN146: Any = 0x0000000004000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN147: Any = 0x0000000008000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN148: Any = 0x0000000010000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN149: Any = 0x0000000020000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN150: Any = 0x0000000040000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN151: Any = 0x0000000080000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN152: Any = 0x0000000100000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN153: Any = 0x0000000200000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN154: Any = 0x0000000400000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN155: Any = 0x0000000800000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN156: Any = 0x0000001000000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN157: Any = 0x0000002000000000
# Source: io/gps_ublox.h
UBLOX_SBAS_PRN158: Any = 0x0000004000000000
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_AUTHSTATUS = (BIT(9)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_CRCORRUSED = (BIT(7)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_CRUSED = (BIT(4)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_DOCORRUSED = (BIT(8)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_DOUSED = (BIT(5)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_HEALTH_MASK = (BIT(0) | BIT(1)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_PRCORRUSED = (BIT(6)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_PRSMOOTHED = (BIT(2)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
# Skipped Define: UBLOX_SIG_PRUSED = (BIT(3)) # (Unparseable/Complex/Filtered)
# Source: io/gps_ublox.h
UBX_DYNMODEL_AIR_1G: Any = 6
# Source: io/gps_ublox.h
UBX_DYNMODEL_AIR_2G: Any = 7
# Source: io/gps_ublox.h
UBX_DYNMODEL_AIR_4G: Any = 8
# Source: io/gps_ublox.h
UBX_DYNMODEL_AUTOMOVITE: Any = 4
# Source: io/gps_ublox.h
UBX_DYNMODEL_BIKE: Any = 10
# Source: io/gps_ublox.h
UBX_DYNMODEL_ESCOOTER: Any = 12
# Source: io/gps_ublox.h
UBX_DYNMODEL_MOWER: Any = 11
# Source: io/gps_ublox.h
UBX_DYNMODEL_PEDESTRIAN: Any = 3
# Source: io/gps_ublox.h
UBX_DYNMODEL_PORTABLE: Any = 0
# Source: io/gps_ublox.h
UBX_DYNMODEL_SEA: Any = 5
# Source: io/gps_ublox.h
UBX_DYNMODEL_STATIONARY: Any = 2
# Source: io/gps_ublox.h
UBX_DYNMODEL_WRIST: Any = 9
# Source: io/gps_ublox.h
UBX_FIXMODE_2D_ONLY: Any = 1
# Source: io/gps_ublox.h
UBX_FIXMODE_3D_ONLY: Any = 2
# Source: io/gps_ublox.h
UBX_FIXMODE_AUTO: Any = 3
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX10: Any = 1000
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX5: Any = 500
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX6: Any = 600
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX7: Any = 700
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX8: Any = 800
# Source: io/gps_ublox.h
UBX_HW_VERSION_UBLOX9: Any = 900
# Source: io/gps_ublox.h
UBX_HW_VERSION_UNKNOWN: Any = 0
# Source: io/gps_ublox.h
UBX_MON_GNSS_BEIDOU_MASK: Any = (1 << 2)
# Source: io/gps_ublox.h
UBX_MON_GNSS_GALILEO_MASK: Any = (1 << 3)
# Source: io/gps_ublox.h
UBX_MON_GNSS_GLONASS_MASK: Any = (1 << 1)
# Source: io/gps_ublox.h
UBX_MON_GNSS_GPS_MASK: Any = (1 << 0)
# Source: common/time.h
# Skipped Define: USECS_PER_SEC = (1000 * 1000) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way.h
# Skipped Define: USE_SERIAL_4WAY_BLHELI_BOOTLOADER # (No Value)
# Source: io/serial_4way.h
# Skipped Define: USE_SERIAL_4WAY_SK_BOOTLOADER # (No Value)
# Source: flight/kalman.h
VARIANCE_SCALE: Any = 0.67f
# Source: sensors/battery.h
VBAT_SCALE_DEFAULT: Any = 1100
# Source: io/vtx_ffpv24g.h
VTX_FFPV_BAND_COUNT: Any = 2
# Source: io/vtx_ffpv24g.h
VTX_FFPV_CHANNEL_COUNT: Any = 8
# Source: io/vtx_ffpv24g.h
VTX_FFPV_POWER_COUNT: Any = 4
# Source: io/vtx_msp.h
VTX_MSP_BAND_COUNT: Any = 5
# Source: io/vtx_msp.h
VTX_MSP_CHANNEL_COUNT: Any = 8
# Source: io/vtx_msp.h
# Skipped Define: VTX_MSP_H # (No Value)
# Source: io/vtx_msp.h
VTX_MSP_POWER_COUNT: Any = 4
# Source: io/vtx_msp.h
VTX_MSP_TIMEOUT: Any = 250
# Source: io/vtx_smartaudio.h
# Skipped Define: VTX_SMARTAUDIO_BAND_COUNT = (VTX_SMARTAUDIO_MAX_BAND - VTX_SMARTAUDIO_MIN_BAND + 1) # (Unparseable/Complex/Filtered)
# Source: io/vtx_smartaudio.h
# Skipped Define: VTX_SMARTAUDIO_CHANNEL_COUNT = (VTX_SMARTAUDIO_MAX_CHANNEL - VTX_SMARTAUDIO_MIN_CHANNEL + 1) # (Unparseable/Complex/Filtered)
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_DEFAULT_POWER: Any = 1
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_DEFAULT_POWER_COUNT: Any = 4
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MAX_BAND: Any = 5
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MAX_CHANNEL: Any = 8
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MAX_FREQUENCY_MHZ: Any = 5999
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MAX_POWER_COUNT: Any = 8
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MIN_BAND: Any = 1
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MIN_CHANNEL: Any = 1
# Source: io/vtx_smartaudio.h
VTX_SMARTAUDIO_MIN_FREQUENCY_MHZ: Any = 5000
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_BAND_COUNT: Any = 2
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_CHANNEL_COUNT: Any = 8
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_DEFAULT_POWER: Any = 1
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_MAX_FREQUENCY_MHZ: Any = 1399
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_MAX_POWER_COUNT: Any = 3
# Source: io/vtx_tramp.h
VTX_TRAMP_1G3_MIN_FREQUENCY_MHZ: Any = 1000
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_BAND_COUNT: Any = 5
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_CHANNEL_COUNT: Any = 8
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_DEFAULT_POWER: Any = 1
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_MAX_FREQUENCY_MHZ: Any = 5999
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_MAX_POWER_COUNT: Any = 5
# Source: io/vtx_tramp.h
VTX_TRAMP_5G8_MIN_FREQUENCY_MHZ: Any = 5000
# Source: common/axis.h
XYZ_AXIS_COUNT: Any = 3
# Source: io/serial_4way_avrootloader.h
brERRORCOMMAND: Any = 0xC1
# Source: io/serial_4way_avrootloader.h
brERRORCRC: Any = 0xC2
# Source: io/serial_4way_avrootloader.h
brERRORVERIFY: Any = 0xC0
# Source: io/serial_4way_avrootloader.h
brNONE: Any = 0xFF
# Source: io/serial_4way_avrootloader.h
brSUCCESS: Any = 0x30
# Source: sensors/battery.h
# Skipped Define: currentBatteryProfileMutable = ((batteryProfile_t*)currentBatteryProfile) # (Unparseable/Complex/Filtered)
# Source: io/serial_4way.h
imARM_BLB: Any = 4
# Source: io/serial_4way.h
imATM_BLB: Any = 2
# Source: io/serial_4way.h
imC2: Any = 0
# Source: io/serial_4way.h
imSIL_BLB: Any = 1
# Source: io/serial_4way.h
imSK: Any = 3


# --- Enums Extracted from Source ---

# --- Enums ---

# Source: io/rcdevice.h
class RCDEVICE_5key_connection_event_e(enum.Enum):
    RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN = 0x01
    RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE = 0x02


# Source: rx/srxl2_types.h
class Srxl2BindRequest(enum.Enum):
    EnterBindMode = 0xEB
    RequestBindStatus = 0xB5
    BoundDataReport = 0xDB
    SetBindInfo = 0x5B


# Source: rx/srxl2_types.h
class Srxl2BindType(enum.Enum):
    NotBound = 0x0
    DSM2_1024_22ms = 0x01
    DSM2_1024_MC24 = 0x02
    DMS2_2048_11ms = 0x12
    DMSX_22ms = 0xA2
    DMSX_11ms = 0xB2
    Surface_DSM2_16_5ms = 0x63
    DSMR_11ms_22ms = 0xE2
    DSMR_5_5ms = 0xE4


# Source: rx/srxl2_types.h
class Srxl2ControlDataCommand(enum.Enum):
    ChannelData = 0x00
    FailsafeChannelData = 0x01
    VTXData = 0x02


# Source: rx/srxl2_types.h
class Srxl2DeviceId(enum.Enum):
    FlightControllerDefault = 0x30
    FlightControllerMax = 0x3F
    Broadcast = 0xFF


# Source: rx/srxl2_types.h
class Srxl2DeviceType(enum.Enum):
    NoDevice = 0
    RemoteReceiver = 1
    Receiver = 2
    FlightController = 3
    ESC = 4
    Reserved = 5
    SRXLServo = 6
    SRXLServo_2 = 7
    VTX = 8


# Source: rx/srxl2_types.h
class Srxl2PacketType(enum.Enum):
    Handshake = 0x21
    BindInfo = 0x41
    ParameterConfiguration = 0x50
    SignalQuality = 0x55
    TelemetrySensorData = 0x80
    ControlData = 0xCD


# Source: rx/srxl2_types.h
class Srxl2State(enum.Enum):
    Disabled = 0
    ListenForActivity = 1
    SendHandshake = 2
    ListenForHandshake = 3
    Running = 4


# Source: sensors/acceleration.h
class accelerationSensor_e(enum.Enum):
    ACC_NONE = 0
    ACC_AUTODETECT = 1
    ACC_MPU6000 = 2
    ACC_MPU6500 = 3
    ACC_MPU9250 = 4
    ACC_BMI160 = 5
    ACC_ICM20689 = 6
    ACC_BMI088 = 7
    ACC_ICM42605 = 8
    ACC_BMI270 = 9
    ACC_LSM6DXX = 10
    ACC_FAKE = 11
    ACC_MAX = ACC_FAKE


# Source: fc/rc_adjustments.h
class adjustmentFunction_e(enum.Enum):
    ADJUSTMENT_NONE = 0
    ADJUSTMENT_RC_RATE = 1
    ADJUSTMENT_RC_EXPO = 2
    ADJUSTMENT_THROTTLE_EXPO = 3
    ADJUSTMENT_PITCH_ROLL_RATE = 4
    ADJUSTMENT_YAW_RATE = 5
    ADJUSTMENT_PITCH_ROLL_P = 6
    ADJUSTMENT_PITCH_ROLL_I = 7
    ADJUSTMENT_PITCH_ROLL_D = 8
    ADJUSTMENT_PITCH_ROLL_FF = 9
    ADJUSTMENT_PITCH_P = 10
    ADJUSTMENT_PITCH_I = 11
    ADJUSTMENT_PITCH_D = 12
    ADJUSTMENT_PITCH_FF = 13
    ADJUSTMENT_ROLL_P = 14
    ADJUSTMENT_ROLL_I = 15
    ADJUSTMENT_ROLL_D = 16
    ADJUSTMENT_ROLL_FF = 17
    ADJUSTMENT_YAW_P = 18
    ADJUSTMENT_YAW_I = 19
    ADJUSTMENT_YAW_D = 20
    ADJUSTMENT_YAW_FF = 21
    ADJUSTMENT_RATE_PROFILE = 22
    ADJUSTMENT_PITCH_RATE = 23
    ADJUSTMENT_ROLL_RATE = 24
    ADJUSTMENT_RC_YAW_EXPO = 25
    ADJUSTMENT_MANUAL_RC_EXPO = 26
    ADJUSTMENT_MANUAL_RC_YAW_EXPO = 27
    ADJUSTMENT_MANUAL_PITCH_ROLL_RATE = 28
    ADJUSTMENT_MANUAL_ROLL_RATE = 29
    ADJUSTMENT_MANUAL_PITCH_RATE = 30
    ADJUSTMENT_MANUAL_YAW_RATE = 31
    ADJUSTMENT_NAV_FW_CRUISE_THR = 32
    ADJUSTMENT_NAV_FW_PITCH2THR = 33
    ADJUSTMENT_ROLL_BOARD_ALIGNMENT = 34
    ADJUSTMENT_PITCH_BOARD_ALIGNMENT = 35
    ADJUSTMENT_LEVEL_P = 36
    ADJUSTMENT_LEVEL_I = 37
    ADJUSTMENT_LEVEL_D = 38
    ADJUSTMENT_POS_XY_P = 39
    ADJUSTMENT_POS_XY_I = 40
    ADJUSTMENT_POS_XY_D = 41
    ADJUSTMENT_POS_Z_P = 42
    ADJUSTMENT_POS_Z_I = 43
    ADJUSTMENT_POS_Z_D = 44
    ADJUSTMENT_HEADING_P = 45
    ADJUSTMENT_VEL_XY_P = 46
    ADJUSTMENT_VEL_XY_I = 47
    ADJUSTMENT_VEL_XY_D = 48
    ADJUSTMENT_VEL_Z_P = 49
    ADJUSTMENT_VEL_Z_I = 50
    ADJUSTMENT_VEL_Z_D = 51
    ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE = 52
    ADJUSTMENT_VTX_POWER_LEVEL = 53
    ADJUSTMENT_TPA = 54
    ADJUSTMENT_TPA_BREAKPOINT = 55
    ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS = 56
    ADJUSTMENT_FW_TPA_TIME_CONSTANT = 57
    ADJUSTMENT_FW_LEVEL_TRIM = 58
    ADJUSTMENT_NAV_WP_MULTI_MISSION_INDEX = 59
    ADJUSTMENT_NAV_FW_ALT_CONTROL_RESPONSE = 60
    ADJUSTMENT_FUNCTION_COUNT = 61


# Source: fc/rc_adjustments.h
class adjustmentMode_e(enum.Enum):
    ADJUSTMENT_MODE_STEP = 0
    ADJUSTMENT_MODE_SELECT = 1


# Source: fc/rc_controls.h
class airmodeHandlingType_e(enum.Enum):
    STICK_CENTER = 0
    THROTTLE_THRESHOLD = 1
    STICK_CENTER_ONCE = 2


# Source: common/axis.h
class angle_index_t(enum.Enum):
    AI_ROLL = 0
    AI_PITCH = 1


# Source: config/general_settings.h
class appliedDefaults_e(enum.Enum):
    APPLIED_DEFAULTS_NONE = 0
    APPLIED_DEFAULTS_CUSTOM = 1
    APPLIED_DEFAULTS_MULTIROTOR = 2
    APPLIED_DEFAULTS_AIRPLANE_WITH_TAIL = 3
    APPLIED_DEFAULTS_AIRPLANE_WITHOUT_TAIL = 4


# Source: fc/runtime_config.h
class armingFlag_e(enum.Enum):
    ARMED = (1 << 2)
    WAS_EVER_ARMED = (1 << 3)
    SIMULATOR_MODE_HITL = (1 << 4)
    SIMULATOR_MODE_SITL = (1 << 5)
    ARMING_DISABLED_GEOZONE = (1 << 6)
    ARMING_DISABLED_FAILSAFE_SYSTEM = (1 << 7)
    ARMING_DISABLED_NOT_LEVEL = (1 << 8)
    ARMING_DISABLED_SENSORS_CALIBRATING = (1 << 9)
    ARMING_DISABLED_SYSTEM_OVERLOADED = (1 << 10)
    ARMING_DISABLED_NAVIGATION_UNSAFE = (1 << 11)
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED = (1 << 12)
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED = (1 << 13)
    ARMING_DISABLED_ARM_SWITCH = (1 << 14)
    ARMING_DISABLED_HARDWARE_FAILURE = (1 << 15)
    ARMING_DISABLED_BOXFAILSAFE = (1 << 16)
    ARMING_DISABLED_RC_LINK = (1 << 18)
    ARMING_DISABLED_THROTTLE = (1 << 19)
    ARMING_DISABLED_CLI = (1 << 20)
    ARMING_DISABLED_CMS_MENU = (1 << 21)
    ARMING_DISABLED_OSD_MENU = (1 << 22)
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED = (1 << 23)
    ARMING_DISABLED_SERVO_AUTOTRIM = (1 << 24)
    ARMING_DISABLED_OOM = (1 << 25)
    ARMING_DISABLED_INVALID_SETTING = (1 << 26)
    ARMING_DISABLED_PWM_OUTPUT_ERROR = (1 << 27)
    ARMING_DISABLED_NO_PREARM = (1 << 28)
    ARMING_DISABLED_DSHOT_BEEPER = (1 << 29)
    ARMING_DISABLED_LANDING_DETECTED = (1 << 30)
    ARMING_DISABLED_ALL_FLAGS = (ARMING_DISABLED_GEOZONE | ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | 
                                                       ARMING_DISABLED_SENSORS_CALIBRATING | ARMING_DISABLED_SYSTEM_OVERLOADED | ARMING_DISABLED_NAVIGATION_UNSAFE |
                                                       ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE |
                                                       ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI |
                                                       ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU | ARMING_DISABLED_ROLLPITCH_NOT_CENTERED |
                                                       ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM | ARMING_DISABLED_INVALID_SETTING |
                                                       ARMING_DISABLED_PWM_OUTPUT_ERROR | ARMING_DISABLED_NO_PREARM | ARMING_DISABLED_DSHOT_BEEPER |
                                                       ARMING_DISABLED_LANDING_DETECTED)


# Source: sensors/barometer.h
class baroSensor_e(enum.Enum):
    BARO_NONE = 0
    BARO_AUTODETECT = 1
    BARO_BMP085 = 2
    BARO_MS5611 = 3
    BARO_BMP280 = 4
    BARO_MS5607 = 5
    BARO_LPS25H = 6
    BARO_SPL06 = 7
    BARO_BMP388 = 8
    BARO_DPS310 = 9
    BARO_B2SMPB = 10
    BARO_MSP = 11
    BARO_FAKE = 12
    BARO_MAX = BARO_FAKE


# Source: sensors/battery_config_structs.h
class batCapacityUnit_e(enum.Enum):
    BAT_CAPACITY_UNIT_MAH = 0
    BAT_CAPACITY_UNIT_MWH = 1


# Source: sensors/battery_config_structs.h
class batVoltageSource_e(enum.Enum):
    BAT_VOLTAGE_RAW = 0
    BAT_VOLTAGE_SAG_COMP = 1


# Source: sensors/battery.h
class batteryState_e(enum.Enum):
    BATTERY_OK = 0
    BATTERY_WARNING = 1
    BATTERY_CRITICAL = 2
    BATTERY_NOT_PRESENT = 3


# Source: io/serial.h
class baudRate_e(enum.Enum):
    BAUD_AUTO = 0
    BAUD_1200 = 1
    BAUD_2400 = 2
    BAUD_4800 = 3
    BAUD_9600 = 4
    BAUD_19200 = 5
    BAUD_38400 = 6
    BAUD_57600 = 7
    BAUD_115200 = 8
    BAUD_230400 = 9
    BAUD_250000 = 10
    BAUD_460800 = 11
    BAUD_921600 = 12
    BAUD_1000000 = 13
    BAUD_1500000 = 14
    BAUD_2000000 = 15
    BAUD_2470000 = 16
    BAUD_MIN = BAUD_AUTO
    BAUD_MAX = BAUD_2470000


# Source: io/beeper.h
class beeperMode_e(enum.Enum):
    BEEPER_SILENCE = 0
    BEEPER_RUNTIME_CALIBRATION_DONE = 1
    BEEPER_HARDWARE_FAILURE = 2
    BEEPER_RX_LOST = 3
    BEEPER_RX_LOST_LANDING = 4
    BEEPER_DISARMING = 5
    BEEPER_ARMING = 6
    BEEPER_ARMING_GPS_FIX = 7
    BEEPER_BAT_CRIT_LOW = 8
    BEEPER_BAT_LOW = 9
    BEEPER_GPS_STATUS = 10
    BEEPER_RX_SET = 11
    BEEPER_ACTION_SUCCESS = 12
    BEEPER_ACTION_FAIL = 13
    BEEPER_READY_BEEP = 14
    BEEPER_MULTI_BEEPS = 15
    BEEPER_DISARM_REPEAT = 16
    BEEPER_ARMED = 17
    BEEPER_SYSTEM_INIT = 18
    BEEPER_USB = 19
    BEEPER_LAUNCH_MODE_ENABLED = 20
    BEEPER_LAUNCH_MODE_LOW_THROTTLE = 21
    BEEPER_LAUNCH_MODE_IDLE_START = 22
    BEEPER_CAM_CONNECTION_OPEN = 23
    BEEPER_CAM_CONNECTION_CLOSE = 24
    BEEPER_ALL = 25
    BEEPER_PREFERENCE = 26


# Source: common/filter.h
class biquadFilterType_e(enum.Enum):
    FILTER_LPF = 0
    FILTER_NOTCH = 1


# Source: blackbox/blackbox_io.h
class blackboxBufferReserveStatus_e(enum.Enum):
    BLACKBOX_RESERVE_SUCCESS = 0
    BLACKBOX_RESERVE_TEMPORARY_FAILURE = 1
    BLACKBOX_RESERVE_PERMANENT_FAILURE = 2


# Source: blackbox/blackbox.h
class blackboxFeatureMask_e(enum.Enum):
    BLACKBOX_FEATURE_NAV_ACC = 1 << 0
    BLACKBOX_FEATURE_NAV_POS = 1 << 1
    BLACKBOX_FEATURE_NAV_PID = 1 << 2
    BLACKBOX_FEATURE_MAG = 1 << 3
    BLACKBOX_FEATURE_ACC = 1 << 4
    BLACKBOX_FEATURE_ATTITUDE = 1 << 5
    BLACKBOX_FEATURE_RC_DATA = 1 << 6
    BLACKBOX_FEATURE_RC_COMMAND = 1 << 7
    BLACKBOX_FEATURE_MOTORS = 1 << 8
    BLACKBOX_FEATURE_GYRO_RAW = 1 << 9
    BLACKBOX_FEATURE_GYRO_PEAKS_ROLL = 1 << 10
    BLACKBOX_FEATURE_GYRO_PEAKS_PITCH = 1 << 11
    BLACKBOX_FEATURE_GYRO_PEAKS_YAW = 1 << 12
    BLACKBOX_FEATURE_SERVOS = 1 << 13


# Source: fc/rc_modes.h
class boxId_e(enum.Enum):
    BOXARM = 0
    BOXANGLE = 1
    BOXHORIZON = 2
    BOXNAVALTHOLD = 3
    BOXHEADINGHOLD = 4
    BOXHEADFREE = 5
    BOXHEADADJ = 6
    BOXCAMSTAB = 7
    BOXNAVRTH = 8
    BOXNAVPOSHOLD = 9
    BOXMANUAL = 10
    BOXBEEPERON = 11
    BOXLEDLOW = 12
    BOXLIGHTS = 13
    BOXNAVLAUNCH = 14
    BOXOSD = 15
    BOXTELEMETRY = 16
    BOXBLACKBOX = 17
    BOXFAILSAFE = 18
    BOXNAVWP = 19
    BOXAIRMODE = 20
    BOXHOMERESET = 21
    BOXGCSNAV = 22
    BOXSURFACE = 24
    BOXFLAPERON = 25
    BOXTURNASSIST = 26
    BOXAUTOTRIM = 27
    BOXAUTOTUNE = 28
    BOXCAMERA1 = 29
    BOXCAMERA2 = 30
    BOXCAMERA3 = 31
    BOXOSDALT1 = 32
    BOXOSDALT2 = 33
    BOXOSDALT3 = 34
    BOXNAVCOURSEHOLD = 35
    BOXBRAKING = 36
    BOXUSER1 = 37
    BOXUSER2 = 38
    BOXFPVANGLEMIX = 39
    BOXLOITERDIRCHN = 40
    BOXMSPRCOVERRIDE = 41
    BOXPREARM = 42
    BOXTURTLE = 43
    BOXNAVCRUISE = 44
    BOXAUTOLEVEL = 45
    BOXPLANWPMISSION = 46
    BOXSOARING = 47
    BOXUSER3 = 48
    BOXUSER4 = 49
    BOXCHANGEMISSION = 50
    BOXBEEPERMUTE = 51
    BOXMULTIFUNCTION = 52
    BOXMIXERPROFILE = 53
    BOXMIXERTRANSITION = 54
    BOXANGLEHOLD = 55
    BOXGIMBALTLOCK = 56
    BOXGIMBALRLOCK = 57
    BOXGIMBALCENTER = 58
    BOXGIMBALHTRK = 59
    CHECKBOX_ITEM_COUNT = 60


# Source: navigation/navigation_private.h
class climbRateToAltitudeControllerMode_e(enum.Enum):
    ROC_TO_ALT_CURRENT = 0
    ROC_TO_ALT_CONSTANT = 1
    ROC_TO_ALT_TARGET = 2


# Source: common/color.h
class colorComponent_e(enum.Enum):
    RGB_RED = 0
    RGB_GREEN = 1
    RGB_BLUE = 2


# Source: io/ledstrip.h
class colorId_e(enum.Enum):
    COLOR_BLACK = 0
    COLOR_WHITE = 1
    COLOR_RED = 2
    COLOR_ORANGE = 3
    COLOR_YELLOW = 4
    COLOR_LIME_GREEN = 5
    COLOR_GREEN = 6
    COLOR_MINT_GREEN = 7
    COLOR_CYAN = 8
    COLOR_LIGHT_BLUE = 9
    COLOR_BLUE = 10
    COLOR_DARK_VIOLET = 11
    COLOR_MAGENTA = 12
    COLOR_DEEP_PINK = 13


# Source: rx/crsf.h
class crsfAddress_e(enum.Enum):
    CRSF_ADDRESS_BROADCAST = 0x00
    CRSF_ADDRESS_USB = 0x10
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80
    CRSF_ADDRESS_RESERVED1 = 0x8A
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0
    CRSF_ADDRESS_GPS = 0xC2
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
    CRSF_ADDRESS_RESERVED2 = 0xCA
    CRSF_ADDRESS_RACE_TAG = 0xCC
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE


# Source: rx/crsf.h
class crsfFrameType_e(enum.Enum):
    CRSF_FRAMETYPE_GPS = 0x02
    CRSF_FRAMETYPE_VARIO_SENSOR = 0x07
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
    CRSF_FRAMETYPE_ATTITUDE = 0x1E
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
    CRSF_FRAMETYPE_DEVICE_PING = 0x28
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D
    CRSF_FRAMETYPE_COMMAND = 0x32
    CRSF_FRAMETYPE_MSP_REQ = 0x7A
    CRSF_FRAMETYPE_MSP_RESP = 0x7B
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D


# Source: sensors/battery_config_structs.h
class currentSensor_e(enum.Enum):
    CURRENT_SENSOR_NONE = 0
    CURRENT_SENSOR_ADC = 1
    CURRENT_SENSOR_VIRTUAL = 2
    CURRENT_SENSOR_FAKE = 3
    CURRENT_SENSOR_ESC = 4
    CURRENT_SENSOR_MAX = CURRENT_SENSOR_FAKE


# Source: io/displayport_msp.h
class displayportMspCommand_e(enum.Enum):
    MSP_DP_HEARTBEAT = 0
    MSP_DP_RELEASE = 1
    MSP_DP_CLEAR_SCREEN = 2
    MSP_DP_WRITE_STRING = 3
    MSP_DP_DRAW_SCREEN = 4
    MSP_DP_OPTIONS = 5
    MSP_DP_SYS = 6
    MSP_DP_COUNT = 7


# Source: sensors/gyro.h
class dynamicGyroNotchMode_e(enum.Enum):
    DYNAMIC_NOTCH_MODE_2D = 0
    DYNAMIC_NOTCH_MODE_3D = 1


# Source: flight/failsafe.h
class emergLandState_e(enum.Enum):
    EMERG_LAND_IDLE = 0
    EMERG_LAND_IN_PROGRESS = 1
    EMERG_LAND_HAS_LANDED = 2


# Source: flight/failsafe.h
class failsafePhase_e(enum.Enum):
    FAILSAFE_IDLE = 0
    FAILSAFE_RX_LOSS_DETECTED = 1
    FAILSAFE_RX_LOSS_IDLE = 2
    FAILSAFE_RETURN_TO_HOME = 3
    FAILSAFE_LANDING = 4
    FAILSAFE_LANDED = 5
    FAILSAFE_RX_LOSS_MONITORING = 6
    FAILSAFE_RX_LOSS_RECOVERED = 7


# Source: flight/failsafe.h
class failsafeProcedure_e(enum.Enum):
    FAILSAFE_PROCEDURE_AUTO_LANDING = 0
    FAILSAFE_PROCEDURE_DROP_IT = 1
    FAILSAFE_PROCEDURE_RTH = 2
    FAILSAFE_PROCEDURE_NONE = 3


# Source: flight/failsafe.h
class failsafeRxLinkState_e(enum.Enum):
    FAILSAFE_RXLINK_DOWN = 0
    FAILSAFE_RXLINK_UP = 1


# Source: fc/config.h
class features_e(enum.Enum):
    FEATURE_THR_VBAT_COMP = 1 << 0
    FEATURE_VBAT = 1 << 1
    FEATURE_TX_PROF_SEL = 1 << 2
    FEATURE_BAT_PROFILE_AUTOSWITCH = 1 << 3
    FEATURE_GEOZONE = 1 << 4
    FEATURE_UNUSED_1 = 1 << 5
    FEATURE_SOFTSERIAL = 1 << 6
    FEATURE_GPS = 1 << 7
    FEATURE_UNUSED_3 = 1 << 8
    FEATURE_UNUSED_4 = 1 << 9
    FEATURE_TELEMETRY = 1 << 10
    FEATURE_CURRENT_METER = 1 << 11
    FEATURE_REVERSIBLE_MOTORS = 1 << 12
    FEATURE_UNUSED_5 = 1 << 13
    FEATURE_UNUSED_6 = 1 << 14
    FEATURE_RSSI_ADC = 1 << 15
    FEATURE_LED_STRIP = 1 << 16
    FEATURE_DASHBOARD = 1 << 17
    FEATURE_UNUSED_7 = 1 << 18
    FEATURE_BLACKBOX = 1 << 19
    FEATURE_UNUSED_10 = 1 << 20
    FEATURE_TRANSPONDER = 1 << 21
    FEATURE_AIRMODE = 1 << 22
    FEATURE_SUPEREXPO_RATES = 1 << 23
    FEATURE_VTX = 1 << 24
    FEATURE_UNUSED_8 = 1 << 25
    FEATURE_UNUSED_9 = 1 << 26
    FEATURE_UNUSED_11 = 1 << 27
    FEATURE_PWM_OUTPUT_ENABLE = 1 << 28
    FEATURE_OSD = 1 << 29
    FEATURE_FW_LAUNCH = 1 << 30
    FEATURE_FW_AUTOTRIM = 1 << 31


# Source: common/filter.h
class filterType_e(enum.Enum):
    FILTER_PT1 = 0
    FILTER_BIQUAD = 1
    FILTER_PT2 = 2
    FILTER_PT3 = 3
    FILTER_LULU = 4


# Source: fc/runtime_config.h
class flightModeFlags_e(enum.Enum):
    ANGLE_MODE = (1 << 0)
    HORIZON_MODE = (1 << 1)
    HEADING_MODE = (1 << 2)
    NAV_ALTHOLD_MODE = (1 << 3)
    NAV_RTH_MODE = (1 << 4)
    NAV_POSHOLD_MODE = (1 << 5)
    HEADFREE_MODE = (1 << 6)
    NAV_LAUNCH_MODE = (1 << 7)
    MANUAL_MODE = (1 << 8)
    FAILSAFE_MODE = (1 << 9)
    AUTO_TUNE = (1 << 10)
    NAV_WP_MODE = (1 << 11)
    NAV_COURSE_HOLD_MODE = (1 << 12)
    FLAPERON = (1 << 13)
    TURN_ASSISTANT = (1 << 14)
    TURTLE_MODE = (1 << 15)
    SOARING_MODE = (1 << 16)
    ANGLEHOLD_MODE = (1 << 17)
    NAV_FW_AUTOLAND = (1 << 18)
    NAV_SEND_TO = (1 << 19)


# Source: fc/runtime_config.h
class flightModeForTelemetry_e(enum.Enum):
    FLM_MANUAL = 0
    FLM_ACRO = 1
    FLM_ACRO_AIR = 2
    FLM_ANGLE = 3
    FLM_HORIZON = 4
    FLM_ALTITUDE_HOLD = 5
    FLM_POSITION_HOLD = 6
    FLM_RTH = 7
    FLM_MISSION = 8
    FLM_COURSE_HOLD = 9
    FLM_CRUISE = 10
    FLM_LAUNCH = 11
    FLM_FAILSAFE = 12
    FLM_ANGLEHOLD = 13
    FLM_COUNT = 14


# Source: common/axis.h
class flight_dynamics_index_t(enum.Enum):
    FD_ROLL = 0
    FD_PITCH = 1
    FD_YAW = 2


# Source: flight/mixer.h
class flyingPlatformType_e(enum.Enum):
    PLATFORM_MULTIROTOR = 0
    PLATFORM_AIRPLANE = 1
    PLATFORM_HELICOPTER = 2
    PLATFORM_TRICOPTER = 3
    PLATFORM_ROVER = 4
    PLATFORM_BOAT = 5


# Source: io/frsky_osd.h
class frskyOSDColor_e(enum.Enum):
    FRSKY_OSD_COLOR_BLACK = 0
    FRSKY_OSD_COLOR_TRANSPARENT = 1
    FRSKY_OSD_COLOR_WHITE = 2
    FRSKY_OSD_COLOR_GRAY = 3


# Source: io/frsky_osd.h
class frskyOSDLineOutlineType_e(enum.Enum):
    FRSKY_OSD_OUTLINE_TYPE_NONE = 0
    FRSKY_OSD_OUTLINE_TYPE_TOP = 1 << 0
    FRSKY_OSD_OUTLINE_TYPE_RIGHT = 1 << 1
    FRSKY_OSD_OUTLINE_TYPE_BOTTOM = 1 << 2
    FRSKY_OSD_OUTLINE_TYPE_LEFT = 1 << 3


# Source: io/frsky_osd.h
class frskyOSDTransactionOptions_e(enum.Enum):
    FRSKY_OSD_TRANSACTION_OPT_PROFILED = 1 << 0
    FRSKY_OSD_TRANSACTION_OPT_RESET_DRAWING = 1 << 1


# Source: io/frsky_osd.h
class frskyOSDWidgetID_e(enum.Enum):
    FRSKY_OSD_WIDGET_ID_AHI = 0
    FRSKY_OSD_WIDGET_ID_SIDEBAR_0 = 1
    FRSKY_OSD_WIDGET_ID_SIDEBAR_1 = 2
    FRSKY_OSD_WIDGET_ID_GRAPH_0 = 3
    FRSKY_OSD_WIDGET_ID_GRAPH_1 = 4
    FRSKY_OSD_WIDGET_ID_GRAPH_2 = 5
    FRSKY_OSD_WIDGET_ID_GRAPH_3 = 6
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_0 = 7
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_1 = 8
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_2 = 9
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_3 = 10
    FRSKY_OSD_WIDGET_ID_SIDEBAR_FIRST = FRSKY_OSD_WIDGET_ID_SIDEBAR_0
    FRSKY_OSD_WIDGET_ID_SIDEBAR_LAST = FRSKY_OSD_WIDGET_ID_SIDEBAR_1
    FRSKY_OSD_WIDGET_ID_GRAPH_FIRST = FRSKY_OSD_WIDGET_ID_GRAPH_0
    FRSKY_OSD_WIDGET_ID_GRAPH_LAST = FRSKY_OSD_WIDGET_ID_GRAPH_3
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_FIRST = FRSKY_OSD_WIDGET_ID_CHARGAUGE_0
    FRSKY_OSD_WIDGET_ID_CHARGAUGE_LAST = FRSKY_OSD_WIDGET_ID_CHARGAUGE_3


# Source: navigation/navigation.h
class fwAutolandApproachDirection_e(enum.Enum):
    FW_AUTOLAND_APPROACH_DIRECTION_LEFT = 0
    FW_AUTOLAND_APPROACH_DIRECTION_RIGHT = 1


# Source: navigation/navigation.h
class fwAutolandState_t(enum.Enum):
    FW_AUTOLAND_STATE_IDLE = 0
    FW_AUTOLAND_STATE_LOITER = 1
    FW_AUTOLAND_STATE_DOWNWIND = 2
    FW_AUTOLAND_STATE_BASE_LEG = 3
    FW_AUTOLAND_STATE_FINAL_APPROACH = 4
    FW_AUTOLAND_STATE_GLIDE = 5
    FW_AUTOLAND_STATE_FLARE = 6


# Source: navigation/navigation_private.h
class fwAutolandWaypoint_t(enum.Enum):
    FW_AUTOLAND_WP_TURN = 0
    FW_AUTOLAND_WP_FINAL_APPROACH = 1
    FW_AUTOLAND_WP_LAND = 2
    FW_AUTOLAND_WP_COUNT = 3


# Source: flight/pid.h
class fw_autotune_rate_adjustment_e(enum.Enum):
    FIXED = 0
    LIMIT = 1
    AUTO = 2


# Source: navigation/navigation.h
class geoAltitudeConversionMode_e(enum.Enum):
    GEO_ALT_ABSOLUTE = 0
    GEO_ALT_RELATIVE = 1


# Source: navigation/navigation.h
class geoAltitudeDatumFlag_e(enum.Enum):
    NAV_WP_TAKEOFF_DATUM = 0
    NAV_WP_MSL_DATUM = 1


# Source: navigation/navigation.h
class geoOriginResetMode_e(enum.Enum):
    GEO_ORIGIN_SET = 0
    GEO_ORIGIN_RESET_ALTITUDE = 1


# Source: navigation/navigation.h
class geozoneMessageState_e(enum.Enum):
    GEOZONE_MESSAGE_STATE_NONE = 0
    GEOZONE_MESSAGE_STATE_NFZ = 1
    GEOZONE_MESSAGE_STATE_LEAVING_FZ = 2
    GEOZONE_MESSAGE_STATE_OUTSIDE_FZ = 3
    GEOZONE_MESSAGE_STATE_ENTERING_NFZ = 4
    GEOZONE_MESSAGE_STATE_AVOIDING_FB = 5
    GEOZONE_MESSAGE_STATE_RETURN_TO_ZONE = 6
    GEOZONE_MESSAGE_STATE_FLYOUT_NFZ = 7
    GEOZONE_MESSAGE_STATE_AVOIDING_ALTITUDE_BREACH = 8
    GEOZONE_MESSAGE_STATE_POS_HOLD = 9


# Source: rx/ghst_protocol.h
class ghstAddr_e(enum.Enum):
    GHST_ADDR_RADIO = 0x80
    GHST_ADDR_TX_MODULE_SYM = 0x81
    GHST_ADDR_TX_MODULE_ASYM = 0x88
    GHST_ADDR_FC = 0x82
    GHST_ADDR_GOGGLES = 0x83
    GHST_ADDR_QUANTUM_TEE1 = 0x84
    GHST_ADDR_QUANTUM_TEE2 = 0x85
    GHST_ADDR_QUANTUM_GW1 = 0x86
    GHST_ADDR_5G_CLK = 0x87
    GHST_ADDR_RX = 0x89


# Source: rx/ghst_protocol.h
class ghstDl_e(enum.Enum):
    GHST_DL_OPENTX_SYNC = 0x20
    GHST_DL_LINK_STAT = 0x21
    GHST_DL_VTX_STAT = 0x22
    GHST_DL_PACK_STAT = 0x23
    GHST_DL_GPS_PRIMARY = 0x25
    GHST_DL_GPS_SECONDARY = 0x26


# Source: rx/ghst_protocol.h
class ghstUl_e(enum.Enum):
    GHST_UL_RC_CHANS_HS4_FIRST = 0x10
    GHST_UL_RC_CHANS_HS4_5TO8 = 0x10
    GHST_UL_RC_CHANS_HS4_9TO12 = 0x11
    GHST_UL_RC_CHANS_HS4_13TO16 = 0x12
    GHST_UL_RC_CHANS_HS4_RSSI = 0x13
    GHST_UL_RC_CHANS_HS4_LAST = 0x1f


# Source: io/gimbal_serial.h
class gimbalHeadtrackerState_e(enum.Enum):
    WAITING_HDR1 = 0
    WAITING_HDR2 = 1
    WAITING_PAYLOAD = 2
    WAITING_CRCH = 3
    WAITING_CRCL = 4


# Source: io/gps.h
class gpsAutoBaud_e(enum.Enum):
    GPS_AUTOBAUD_OFF = 0
    GPS_AUTOBAUD_ON = 1


# Source: io/gps.h
class gpsAutoConfig_e(enum.Enum):
    GPS_AUTOCONFIG_OFF = 0
    GPS_AUTOCONFIG_ON = 1


# Source: io/gps.h
class gpsBaudRate_e(enum.Enum):
    GPS_BAUDRATE_115200 = 0
    GPS_BAUDRATE_57600 = 1
    GPS_BAUDRATE_38400 = 2
    GPS_BAUDRATE_19200 = 3
    GPS_BAUDRATE_9600 = 4
    GPS_BAUDRATE_230400 = 5
    GPS_BAUDRATE_460800 = 6
    GPS_BAUDRATE_921600 = 7
    GPS_BAUDRATE_COUNT = 8


# Source: io/gps.h
class gpsDynModel_e(enum.Enum):
    GPS_DYNMODEL_PEDESTRIAN = 0
    GPS_DYNMODEL_AUTOMOTIVE = 1
    GPS_DYNMODEL_AIR_1G = 2
    GPS_DYNMODEL_AIR_2G = 3
    GPS_DYNMODEL_AIR_4G = 4
    GPS_DYNMODEL_SEA = 5
    GPS_DYNMODEL_MOWER = 6


# Source: io/gps.h
class gpsFixType_e(enum.Enum):
    GPS_NO_FIX = 0
    GPS_FIX_2D = 1
    GPS_FIX_3D = 2


# Source: io/gps.h
class gpsProvider_e(enum.Enum):
    GPS_UBLOX = 0
    GPS_MSP = 1
    GPS_FAKE = 2
    GPS_PROVIDER_COUNT = 3


# Source: io/gps_private.h
class gpsState_e(enum.Enum):
    GPS_UNKNOWN = 0
    GPS_INITIALIZING = 1
    GPS_RUNNING = 2
    GPS_LOST_COMMUNICATION = 3


# Source: sensors/gyro.h
class gyroFilterMode_e(enum.Enum):
    GYRO_FILTER_MODE_OFF = 0
    GYRO_FILTER_MODE_STATIC = 1
    GYRO_FILTER_MODE_DYNAMIC = 2
    GYRO_FILTER_MODE_ADAPTIVE = 3


# Source: sensors/gyro.h
class gyroSensor_e(enum.Enum):
    GYRO_NONE = 0
    GYRO_AUTODETECT = 1
    GYRO_MPU6000 = 2
    GYRO_MPU6500 = 3
    GYRO_MPU9250 = 4
    GYRO_BMI160 = 5
    GYRO_ICM20689 = 6
    GYRO_BMI088 = 7
    GYRO_ICM42605 = 8
    GYRO_BMI270 = 9
    GYRO_LSM6DXX = 10
    GYRO_FAKE = 11


# Source: sensors/diagnostics.h
class hardwareSensorStatus_e(enum.Enum):
    HW_SENSOR_NONE = 0
    HW_SENSOR_OK = 1
    HW_SENSOR_UNAVAILABLE = 2
    HW_SENSOR_UNHEALTHY = 3


# Source: telemetry/hott.h
class hottEamAlarm1Flag_e(enum.Enum):
    HOTT_EAM_ALARM1_FLAG_NONE = 0
    HOTT_EAM_ALARM1_FLAG_MAH = (1 << 0)
    HOTT_EAM_ALARM1_FLAG_BATTERY_1 = (1 << 1)
    HOTT_EAM_ALARM1_FLAG_BATTERY_2 = (1 << 2)
    HOTT_EAM_ALARM1_FLAG_TEMPERATURE_1 = (1 << 3)
    HOTT_EAM_ALARM1_FLAG_TEMPERATURE_2 = (1 << 4)
    HOTT_EAM_ALARM1_FLAG_ALTITUDE = (1 << 5)
    HOTT_EAM_ALARM1_FLAG_CURRENT = (1 << 6)
    HOTT_EAM_ALARM1_FLAG_MAIN_VOLTAGE = (1 << 7)


# Source: telemetry/hott.h
class hottEamAlarm2Flag_e(enum.Enum):
    HOTT_EAM_ALARM2_FLAG_NONE = 0
    HOTT_EAM_ALARM2_FLAG_MS = (1 << 0)
    HOTT_EAM_ALARM2_FLAG_M3S = (1 << 1)
    HOTT_EAM_ALARM2_FLAG_ALTITUDE_DUPLICATE = (1 << 2)
    HOTT_EAM_ALARM2_FLAG_MS_DUPLICATE = (1 << 3)
    HOTT_EAM_ALARM2_FLAG_M3S_DUPLICATE = (1 << 4)
    HOTT_EAM_ALARM2_FLAG_UNKNOWN_1 = (1 << 5)
    HOTT_EAM_ALARM2_FLAG_UNKNOWN_2 = (1 << 6)
    HOTT_EAM_ALARM2_FLAG_ON_SIGN_OR_TEXT_ACTIVE = (1 << 7)


# Source: common/color.h
class hsvColorComponent_e(enum.Enum):
    HSV_HUE = 0
    HSV_SATURATION = 1
    HSV_VALUE = 2


# Source: telemetry/ibus_shared.h
class ibusSensorType1_e(enum.Enum):
    IBUS_MEAS_TYPE1_INTV = 0x00
    IBUS_MEAS_TYPE1_TEM = 0x01
    IBUS_MEAS_TYPE1_MOT = 0x02
    IBUS_MEAS_TYPE1_EXTV = 0x03
    IBUS_MEAS_TYPE1_CELL = 0x04
    IBUS_MEAS_TYPE1_BAT_CURR = 0x05
    IBUS_MEAS_TYPE1_FUEL = 0x06
    IBUS_MEAS_TYPE1_RPM = 0x07
    IBUS_MEAS_TYPE1_CMP_HEAD = 0x08
    IBUS_MEAS_TYPE1_CLIMB_RATE = 0x09
    IBUS_MEAS_TYPE1_COG = 0x0a
    IBUS_MEAS_TYPE1_GPS_STATUS = 0x0b
    IBUS_MEAS_TYPE1_ACC_X = 0x0c
    IBUS_MEAS_TYPE1_ACC_Y = 0x0d
    IBUS_MEAS_TYPE1_ACC_Z = 0x0e
    IBUS_MEAS_TYPE1_ROLL = 0x0f
    IBUS_MEAS_TYPE1_PITCH = 0x10
    IBUS_MEAS_TYPE1_YAW = 0x11
    IBUS_MEAS_TYPE1_VERTICAL_SPEED = 0x12
    IBUS_MEAS_TYPE1_GROUND_SPEED = 0x13
    IBUS_MEAS_TYPE1_GPS_DIST = 0x14
    IBUS_MEAS_TYPE1_ARMED = 0x15
    IBUS_MEAS_TYPE1_FLIGHT_MODE = 0x16
    IBUS_MEAS_TYPE1_PRES = 0x41
    IBUS_MEAS_TYPE1_SPE = 0x7e
    IBUS_MEAS_TYPE1_GPS_LAT = 0x80
    IBUS_MEAS_TYPE1_GPS_LON = 0x81
    IBUS_MEAS_TYPE1_GPS_ALT = 0x82
    IBUS_MEAS_TYPE1_ALT = 0x83
    IBUS_MEAS_TYPE1_S84 = 0x84
    IBUS_MEAS_TYPE1_S85 = 0x85
    IBUS_MEAS_TYPE1_S86 = 0x86
    IBUS_MEAS_TYPE1_S87 = 0x87
    IBUS_MEAS_TYPE1_S88 = 0x88
    IBUS_MEAS_TYPE1_S89 = 0x89
    IBUS_MEAS_TYPE1_S8a = 0x8a


# Source: telemetry/ibus_shared.h
class ibusSensorType_e(enum.Enum):
    IBUS_MEAS_TYPE_INTERNAL_VOLTAGE = 0x00
    IBUS_MEAS_TYPE_TEMPERATURE = 0x01
    IBUS_MEAS_TYPE_RPM = 0x02
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE = 0x03
    IBUS_MEAS_TYPE_HEADING = 0x04
    IBUS_MEAS_TYPE_CURRENT = 0x05
    IBUS_MEAS_TYPE_CLIMB = 0x06
    IBUS_MEAS_TYPE_ACC_Z = 0x07
    IBUS_MEAS_TYPE_ACC_Y = 0x08
    IBUS_MEAS_TYPE_ACC_X = 0x09
    IBUS_MEAS_TYPE_VSPEED = 0x0a
    IBUS_MEAS_TYPE_SPEED = 0x0b
    IBUS_MEAS_TYPE_DIST = 0x0c
    IBUS_MEAS_TYPE_ARMED = 0x0d
    IBUS_MEAS_TYPE_MODE = 0x0e
    IBUS_MEAS_TYPE_PRES = 0x41
    IBUS_MEAS_TYPE_SPE = 0x7e
    IBUS_MEAS_TYPE_COG = 0x80
    IBUS_MEAS_TYPE_GPS_STATUS = 0x81
    IBUS_MEAS_TYPE_GPS_LON = 0x82
    IBUS_MEAS_TYPE_GPS_LAT = 0x83
    IBUS_MEAS_TYPE_ALT = 0x84
    IBUS_MEAS_TYPE_S85 = 0x85
    IBUS_MEAS_TYPE_S86 = 0x86
    IBUS_MEAS_TYPE_S87 = 0x87
    IBUS_MEAS_TYPE_S88 = 0x88
    IBUS_MEAS_TYPE_S89 = 0x89
    IBUS_MEAS_TYPE_S8A = 0x8A
    IBUS_MEAS_TYPE_GALT = 0xf9
    IBUS_MEAS_TYPE_GPS = 0xfd


# Source: telemetry/ibus_shared.h
class ibusSensorValue_e(enum.Enum):
    IBUS_MEAS_VALUE_NONE = 0x00
    IBUS_MEAS_VALUE_TEMPERATURE = 0x01
    IBUS_MEAS_VALUE_MOT = 0x02
    IBUS_MEAS_VALUE_EXTERNAL_VOLTAGE = 0x03
    IBUS_MEAS_VALUE_CELL = 0x04
    IBUS_MEAS_VALUE_CURRENT = 0x05
    IBUS_MEAS_VALUE_FUEL = 0x06
    IBUS_MEAS_VALUE_RPM = 0x07
    IBUS_MEAS_VALUE_HEADING = 0x08
    IBUS_MEAS_VALUE_CLIMB = 0x09
    IBUS_MEAS_VALUE_COG = 0x0a
    IBUS_MEAS_VALUE_GPS_STATUS = 0x0b
    IBUS_MEAS_VALUE_ACC_X = 0x0c
    IBUS_MEAS_VALUE_ACC_Y = 0x0d
    IBUS_MEAS_VALUE_ACC_Z = 0x0e
    IBUS_MEAS_VALUE_ROLL = 0x0f
    IBUS_MEAS_VALUE_PITCH = 0x10
    IBUS_MEAS_VALUE_YAW = 0x11
    IBUS_MEAS_VALUE_VSPEED = 0x12
    IBUS_MEAS_VALUE_SPEED = 0x13
    IBUS_MEAS_VALUE_DIST = 0x14
    IBUS_MEAS_VALUE_ARMED = 0x15
    IBUS_MEAS_VALUE_MODE = 0x16
    IBUS_MEAS_VALUE_PRES = 0x41
    IBUS_MEAS_VALUE_SPE = 0x7e
    IBUS_MEAS_VALUE_GPS_LAT = 0x80
    IBUS_MEAS_VALUE_GPS_LON = 0x81
    IBUS_MEAS_VALUE_GALT4 = 0x82
    IBUS_MEAS_VALUE_ALT4 = 0x83
    IBUS_MEAS_VALUE_GALT = 0x84
    IBUS_MEAS_VALUE_ALT = 0x85
    IBUS_MEAS_VALUE_STATUS = 0x87
    IBUS_MEAS_VALUE_GPS_LAT1 = 0x88
    IBUS_MEAS_VALUE_GPS_LON1 = 0x89
    IBUS_MEAS_VALUE_GPS_LAT2 = 0x90
    IBUS_MEAS_VALUE_GPS_LON2 = 0x91
    IBUS_MEAS_VALUE_GPS = 0xfd


# Source: flight/imu.h
class imu_inertia_comp_method_e(enum.Enum):
    COMPMETHOD_VELNED = 0
    COMPMETHOD_TURNRATE = 1
    COMPMETHOD_ADAPTIVE = 2


# Source: flight/servos.h
class inputSource_e(enum.Enum):
    INPUT_STABILIZED_ROLL = 0
    INPUT_STABILIZED_PITCH = 1
    INPUT_STABILIZED_YAW = 2
    INPUT_STABILIZED_THROTTLE = 3
    INPUT_RC_ROLL = 4
    INPUT_RC_PITCH = 5
    INPUT_RC_YAW = 6
    INPUT_RC_THROTTLE = 7
    INPUT_RC_CH5 = 8
    INPUT_RC_CH6 = 9
    INPUT_RC_CH7 = 10
    INPUT_RC_CH8 = 11
    INPUT_GIMBAL_PITCH = 12
    INPUT_GIMBAL_ROLL = 13
    INPUT_FEATURE_FLAPS = 14
    INPUT_RC_CH9 = 15
    INPUT_RC_CH10 = 16
    INPUT_RC_CH11 = 17
    INPUT_RC_CH12 = 18
    INPUT_RC_CH13 = 19
    INPUT_RC_CH14 = 20
    INPUT_RC_CH15 = 21
    INPUT_RC_CH16 = 22
    INPUT_STABILIZED_ROLL_PLUS = 23
    INPUT_STABILIZED_ROLL_MINUS = 24
    INPUT_STABILIZED_PITCH_PLUS = 25
    INPUT_STABILIZED_PITCH_MINUS = 26
    INPUT_STABILIZED_YAW_PLUS = 27
    INPUT_STABILIZED_YAW_MINUS = 28
    INPUT_MAX = 29
    INPUT_GVAR_0 = 30
    INPUT_GVAR_1 = 31
    INPUT_GVAR_2 = 32
    INPUT_GVAR_3 = 33
    INPUT_GVAR_4 = 34
    INPUT_GVAR_5 = 35
    INPUT_GVAR_6 = 36
    INPUT_GVAR_7 = 37
    INPUT_MIXER_TRANSITION = 38
    INPUT_HEADTRACKER_PAN = 39
    INPUT_HEADTRACKER_TILT = 40
    INPUT_HEADTRACKER_ROLL = 41
    INPUT_RC_CH17 = 42
    INPUT_RC_CH18 = 43
    INPUT_RC_CH19 = 44
    INPUT_RC_CH20 = 45
    INPUT_RC_CH21 = 46
    INPUT_RC_CH22 = 47
    INPUT_RC_CH23 = 48
    INPUT_RC_CH24 = 49
    INPUT_RC_CH25 = 50
    INPUT_RC_CH26 = 51
    INPUT_RC_CH27 = 52
    INPUT_RC_CH28 = 53
    INPUT_RC_CH29 = 54
    INPUT_RC_CH30 = 55
    INPUT_RC_CH31 = 56
    INPUT_RC_CH32 = 57
    INPUT_RC_CH33 = 58
    INPUT_RC_CH34 = 59
    INPUT_SOURCE_COUNT = 60


# Source: flight/pid.h
class itermRelax_e(enum.Enum):
    ITERM_RELAX_OFF = 0
    ITERM_RELAX_RP = 1
    ITERM_RELAX_RPY = 2


# Source: io/ledstrip.h
class ledBaseFunctionId_e(enum.Enum):
    LED_FUNCTION_COLOR = 0
    LED_FUNCTION_FLIGHT_MODE = 1
    LED_FUNCTION_ARM_STATE = 2
    LED_FUNCTION_BATTERY = 3
    LED_FUNCTION_RSSI = 4
    LED_FUNCTION_GPS = 5
    LED_FUNCTION_THRUST_RING = 6
    LED_FUNCTION_CHANNEL = 7


# Source: io/ledstrip.h
class ledDirectionId_e(enum.Enum):
    LED_DIRECTION_NORTH = 0
    LED_DIRECTION_EAST = 1
    LED_DIRECTION_SOUTH = 2
    LED_DIRECTION_WEST = 3
    LED_DIRECTION_UP = 4
    LED_DIRECTION_DOWN = 5


# Source: io/ledstrip.h
class ledModeIndex_e(enum.Enum):
    LED_MODE_ORIENTATION = 0
    LED_MODE_HEADFREE = 1
    LED_MODE_HORIZON = 2
    LED_MODE_ANGLE = 3
    LED_MODE_MAG = 4
    LED_MODE_BARO = 5
    LED_SPECIAL = 6


# Source: io/ledstrip.h
class ledOverlayId_e(enum.Enum):
    LED_OVERLAY_THROTTLE = 0
    LED_OVERLAY_LARSON_SCANNER = 1
    LED_OVERLAY_BLINK = 2
    LED_OVERLAY_LANDING_FLASH = 3
    LED_OVERLAY_INDICATOR = 4
    LED_OVERLAY_WARNING = 5
    LED_OVERLAY_STROBE = 6


# Source: io/ledstrip.h
class ledSpecialColorIds_e(enum.Enum):
    LED_SCOLOR_DISARMED = 0
    LED_SCOLOR_ARMED = 1
    LED_SCOLOR_ANIMATION = 2
    LED_SCOLOR_BACKGROUND = 3
    LED_SCOLOR_BLINKBACKGROUND = 4
    LED_SCOLOR_GPSNOSATS = 5
    LED_SCOLOR_GPSNOLOCK = 6
    LED_SCOLOR_GPSLOCKED = 7
    LED_SCOLOR_STROBE = 8


# Source: common/log.h
class logTopic_e(enum.Enum):
    LOG_TOPIC_SYSTEM = 0
    LOG_TOPIC_GYRO = 1
    LOG_TOPIC_BARO = 2
    LOG_TOPIC_PITOT = 3
    LOG_TOPIC_PWM = 4
    LOG_TOPIC_TIMER = 5
    LOG_TOPIC_IMU = 6
    LOG_TOPIC_TEMPERATURE = 7
    LOG_TOPIC_POS_ESTIMATOR = 8
    LOG_TOPIC_VTX = 9
    LOG_TOPIC_OSD = 10
    LOG_TOPIC_COUNT = 11


# Source: programming/logic_condition.h
class logicConditionFlags_e(enum.Enum):
    LOGIC_CONDITION_FLAG_LATCH = 1 << 0
    LOGIC_CONDITION_FLAG_TIMEOUT_SATISFIED = 1 << 1


# Source: programming/logic_condition.h
class logicConditionsGlobalFlags_t(enum.Enum):
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_ARMING_SAFETY = (1 << 0)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE = (1 << 1)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW = (1 << 2)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL = (1 << 3)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH = (1 << 4)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW = (1 << 5)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE = (1 << 6)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT = (1 << 7)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL = (1 << 8)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_LOITER_RADIUS = (1 << 9)
    LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_FLIGHT_AXIS = (1 << 10)
    LOGIC_CONDITION_GLOBAL_FLAG_DISABLE_GPS_FIX = (1 << 11)


# Source: programming/logic_condition.h
class logicFlightModeOperands_e(enum.Enum):
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_FAILSAFE = 0
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_MANUAL = 1
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_RTH = 2
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_POSHOLD = 3
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_CRUISE = 4
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ALTHOLD = 5
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLE = 6
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_HORIZON = 7
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_AIR = 8
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER1 = 9
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER2 = 10
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_COURSE_HOLD = 11
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER3 = 12
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER4 = 13
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ACRO = 14
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_WAYPOINT_MISSION = 15
    LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLEHOLD = 16


# Source: programming/logic_condition.h
class logicFlightOperands_e(enum.Enum):
    LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER = 0
    LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE = 1
    LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE = 2
    LOGIC_CONDITION_OPERAND_FLIGHT_RSSI = 3
    LOGIC_CONDITION_OPERAND_FLIGHT_VBAT = 4
    LOGIC_CONDITION_OPERAND_FLIGHT_CELL_VOLTAGE = 5
    LOGIC_CONDITION_OPERAND_FLIGHT_CURRENT = 6
    LOGIC_CONDITION_OPERAND_FLIGHT_MAH_DRAWN = 7
    LOGIC_CONDITION_OPERAND_FLIGHT_GPS_SATS = 8
    LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED = 9
    LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED = 10
    LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED = 11
    LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE = 12
    LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED = 13
    LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS = 14
    LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL = 15
    LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH = 16
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_ARMED = 17
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_AUTOLAUNCH = 18
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_ALTITUDE_CONTROL = 19
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_POSITION_CONTROL = 20
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_EMERGENCY_LANDING = 21
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_RTH = 22
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_LANDING = 23
    LOGIC_CONDITION_OPERAND_FLIGHT_IS_FAILSAFE = 24
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_ROLL = 25
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_PITCH = 26
    LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_YAW = 27
    LOGIC_CONDITION_OPERAND_FLIGHT_3D_HOME_DISTANCE = 28
    LOGIC_CONDITION_OPERAND_FLIGHT_LQ_UPLINK = 29
    LOGIC_CONDITION_OPERAND_FLIGHT_SNR = 30
    LOGIC_CONDITION_OPERAND_FLIGHT_GPS_VALID = 31
    LOGIC_CONDITION_OPERAND_FLIGHT_LOITER_RADIUS = 32
    LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_PROFILE = 33
    LOGIC_CONDITION_OPERAND_FLIGHT_BATT_CELLS = 34
    LOGIC_CONDITION_OPERAND_FLIGHT_AGL_STATUS = 35
    LOGIC_CONDITION_OPERAND_FLIGHT_AGL = 36
    LOGIC_CONDITION_OPERAND_FLIGHT_RANGEFINDER_RAW = 37
    LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_MIXER_PROFILE = 38
    LOGIC_CONDITION_OPERAND_FLIGHT_MIXER_TRANSITION_ACTIVE = 39
    LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_YAW = 40
    LOGIC_CONDITION_OPERAND_FLIGHT_FW_LAND_STATE = 41
    LOGIC_CONDITION_OPERAND_FLIGHT_BATT_PROFILE = 42
    LOGIC_CONDITION_OPERAND_FLIGHT_FLOWN_LOITER_RADIUS = 43
    LOGIC_CONDITION_OPERAND_FLIGHT_LQ_DOWNLINK = 44
    LOGIC_CONDITION_OPERAND_FLIGHT_UPLINK_RSSI_DBM = 45


# Source: programming/logic_condition.h
class logicOperation_e(enum.Enum):
    LOGIC_CONDITION_TRUE = 0
    LOGIC_CONDITION_EQUAL = 1
    LOGIC_CONDITION_GREATER_THAN = 2
    LOGIC_CONDITION_LOWER_THAN = 3
    LOGIC_CONDITION_LOW = 4
    LOGIC_CONDITION_MID = 5
    LOGIC_CONDITION_HIGH = 6
    LOGIC_CONDITION_AND = 7
    LOGIC_CONDITION_OR = 8
    LOGIC_CONDITION_XOR = 9
    LOGIC_CONDITION_NAND = 10
    LOGIC_CONDITION_NOR = 11
    LOGIC_CONDITION_NOT = 12
    LOGIC_CONDITION_STICKY = 13
    LOGIC_CONDITION_ADD = 14
    LOGIC_CONDITION_SUB = 15
    LOGIC_CONDITION_MUL = 16
    LOGIC_CONDITION_DIV = 17
    LOGIC_CONDITION_GVAR_SET = 18
    LOGIC_CONDITION_GVAR_INC = 19
    LOGIC_CONDITION_GVAR_DEC = 20
    LOGIC_CONDITION_PORT_SET = 21
    LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY = 22
    LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE = 23
    LOGIC_CONDITION_SWAP_ROLL_YAW = 24
    LOGIC_CONDITION_SET_VTX_POWER_LEVEL = 25
    LOGIC_CONDITION_INVERT_ROLL = 26
    LOGIC_CONDITION_INVERT_PITCH = 27
    LOGIC_CONDITION_INVERT_YAW = 28
    LOGIC_CONDITION_OVERRIDE_THROTTLE = 29
    LOGIC_CONDITION_SET_VTX_BAND = 30
    LOGIC_CONDITION_SET_VTX_CHANNEL = 31
    LOGIC_CONDITION_SET_OSD_LAYOUT = 32
    LOGIC_CONDITION_SIN = 33
    LOGIC_CONDITION_COS = 34
    LOGIC_CONDITION_TAN = 35
    LOGIC_CONDITION_MAP_INPUT = 36
    LOGIC_CONDITION_MAP_OUTPUT = 37
    LOGIC_CONDITION_RC_CHANNEL_OVERRIDE = 38
    LOGIC_CONDITION_SET_HEADING_TARGET = 39
    LOGIC_CONDITION_MODULUS = 40
    LOGIC_CONDITION_LOITER_OVERRIDE = 41
    LOGIC_CONDITION_SET_PROFILE = 42
    LOGIC_CONDITION_MIN = 43
    LOGIC_CONDITION_MAX = 44
    LOGIC_CONDITION_FLIGHT_AXIS_ANGLE_OVERRIDE = 45
    LOGIC_CONDITION_FLIGHT_AXIS_RATE_OVERRIDE = 46
    LOGIC_CONDITION_EDGE = 47
    LOGIC_CONDITION_DELAY = 48
    LOGIC_CONDITION_TIMER = 49
    LOGIC_CONDITION_DELTA = 50
    LOGIC_CONDITION_APPROX_EQUAL = 51
    LOGIC_CONDITION_LED_PIN_PWM = 52
    LOGIC_CONDITION_DISABLE_GPS_FIX = 53
    LOGIC_CONDITION_RESET_MAG_CALIBRATION = 54
    LOGIC_CONDITION_SET_GIMBAL_SENSITIVITY = 55
    LOGIC_CONDITION_LAST = 56


# Source: programming/logic_condition.h
class logicWaypointOperands_e(enum.Enum):
    LOGIC_CONDITION_OPERAND_WAYPOINTS_IS_WP = 0
    LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_INDEX = 1
    LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_ACTION = 2
    LOGIC_CONDITION_OPERAND_WAYPOINTS_NEXT_WAYPOINT_ACTION = 3
    LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_DISTANCE = 4
    LOGIC_CONDTIION_OPERAND_WAYPOINTS_DISTANCE_FROM_WAYPOINT = 5
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION = 6
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION = 7
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION = 8
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION = 9
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION_NEXT_WP = 10
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION_NEXT_WP = 11
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION_NEXT_WP = 12
    LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION_NEXT_WP = 13


# Source: telemetry/telemetry.h
class ltmUpdateRate_e(enum.Enum):
    LTM_RATE_NORMAL = 0
    LTM_RATE_MEDIUM = 1
    LTM_RATE_SLOW = 2


# Source: telemetry/ltm.h
class ltm_frame_e(enum.Enum):
    LTM_FRAME_START = 0
    LTM_AFRAME = LTM_FRAME_START


# Source: telemetry/ltm.h
class ltm_modes_e(enum.Enum):
    LTM_MODE_MANUAL = 0
    LTM_MODE_RATE = 1
    LTM_MODE_ANGLE = 2
    LTM_MODE_HORIZON = 3
    LTM_MODE_ACRO = 4
    LTM_MODE_STABALIZED1 = 5
    LTM_MODE_STABALIZED2 = 6
    LTM_MODE_STABILIZED3 = 7
    LTM_MODE_ALTHOLD = 8
    LTM_MODE_GPSHOLD = 9
    LTM_MODE_WAYPOINTS = 10
    LTM_MODE_HEADHOLD = 11
    LTM_MODE_CIRCLE = 12
    LTM_MODE_RTH = 13
    LTM_MODE_FOLLOWWME = 14
    LTM_MODE_LAND = 15
    LTM_MODE_FLYBYWIRE1 = 16
    LTM_MODE_FLYBYWIRE2 = 17
    LTM_MODE_CRUISE = 18
    LTM_MODE_UNKNOWN = 19
    LTM_MODE_LAUNCH = 20
    LTM_MODE_AUTOTUNE = 21


# Source: sensors/compass.h
class magSensor_e(enum.Enum):
    MAG_NONE = 0
    MAG_AUTODETECT = 1
    MAG_HMC5883 = 2
    MAG_AK8975 = 3
    MAG_MAG3110 = 4
    MAG_AK8963 = 5
    MAG_IST8310 = 6
    MAG_QMC5883 = 7
    MAG_MPU9250 = 8
    MAG_IST8308 = 9
    MAG_LIS3MDL = 10
    MAG_MSP = 11
    MAG_RM3100 = 12
    MAG_VCM5883 = 13
    MAG_MLX90393 = 14
    MAG_FAKE = 15
    MAG_MAX = MAG_FAKE


# Source: telemetry/telemetry.h
class mavlinkRadio_e(enum.Enum):
    MAVLINK_RADIO_GENERIC = 0
    MAVLINK_RADIO_ELRS = 1
    MAVLINK_RADIO_SIK = 2


# Source: flight/mixer_profile.h
class mixerProfileATRequest_e(enum.Enum):
    MIXERAT_REQUEST_NONE = 0
    MIXERAT_REQUEST_RTH = 1
    MIXERAT_REQUEST_LAND = 2
    MIXERAT_REQUEST_ABORT = 3


# Source: flight/mixer_profile.h
class mixerProfileATState_e(enum.Enum):
    MIXERAT_PHASE_IDLE = 0
    MIXERAT_PHASE_TRANSITION_INITIALIZE = 1
    MIXERAT_PHASE_TRANSITIONING = 2
    MIXERAT_PHASE_DONE = 3


# Source: fc/rc_modes.h
class modeActivationOperator_e(enum.Enum):
    MODE_OPERATOR_OR = 0
    MODE_OPERATOR_AND = 1


# Source: flight/mixer.h
class motorStatus_e(enum.Enum):
    MOTOR_STOPPED_USER = 0
    MOTOR_STOPPED_AUTO = 1
    MOTOR_RUNNING = 2


# Source: msp/msp_serial.h
class mspEvaluateNonMspData_e(enum.Enum):
    MSP_EVALUATE_NON_MSP_DATA = 0
    MSP_SKIP_NON_MSP_DATA = 1


# Source: msp/msp.h
class mspFlags_e(enum.Enum):
    MSP_FLAG_DONT_REPLY = (1 << 0)
    MSP_FLAG_ILMI = (1 << 1)


# Source: msp/msp_serial.h
class mspPendingSystemRequest_e(enum.Enum):
    MSP_PENDING_NONE = 0
    MSP_PENDING_BOOTLOADER = 1
    MSP_PENDING_CLI = 2


# Source: msp/msp.h
class mspResult_e(enum.Enum):
    MSP_RESULT_ACK = 1
    # Skipped Member: MSP_RESULT_ERROR = -1 # (Unparseable/Complex/Filtered)
    MSP_RESULT_NO_REPLY = 0


# Source: msp/msp_serial.h
class mspState_e(enum.Enum):
    MSP_IDLE = 0
    MSP_HEADER_START = 1
    MSP_HEADER_M = 2
    MSP_HEADER_X = 3
    MSP_HEADER_V1 = 4
    MSP_PAYLOAD_V1 = 5
    MSP_CHECKSUM_V1 = 6
    MSP_HEADER_V2_OVER_V1 = 7
    MSP_PAYLOAD_V2_OVER_V1 = 8
    MSP_CHECKSUM_V2_OVER_V1 = 9
    MSP_HEADER_V2_NATIVE = 10
    MSP_PAYLOAD_V2_NATIVE = 11
    MSP_CHECKSUM_V2_NATIVE = 12
    MSP_COMMAND_RECEIVED = 13


# Source: msp/msp.h
class mspVersion_e(enum.Enum):
    MSP_V1 = 0
    MSP_V2_OVER_V1 = 1
    MSP_V2_NATIVE = 2
    MSP_VERSION_COUNT = 3


# Source: fc/multifunction.h
class multiFunctionFlags_e(enum.Enum):
    MF_SUSPEND_SAFEHOMES = (1 << 0)
    MF_SUSPEND_TRACKBACK = (1 << 1)
    MF_TURTLE_MODE = (1 << 2)


# Source: fc/multifunction.h
class multi_function_e(enum.Enum):
    MULTI_FUNC_NONE = 0
    MULTI_FUNC_1 = 1
    MULTI_FUNC_2 = 2
    MULTI_FUNC_3 = 3
    MULTI_FUNC_4 = 4
    MULTI_FUNC_5 = 5
    MULTI_FUNC_6 = 6
    MULTI_FUNC_END = 7


# Source: navigation/navigation_pos_estimator_private.h
class navAGLEstimateQuality_e(enum.Enum):
    SURFACE_QUAL_LOW = 0
    SURFACE_QUAL_MID = 1
    SURFACE_QUAL_HIGH = 2


# Source: navigation/navigation.h
class navArmingBlocker_e(enum.Enum):
    NAV_ARMING_BLOCKER_NONE = 0
    NAV_ARMING_BLOCKER_MISSING_GPS_FIX = 1
    NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE = 2
    NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR = 3
    NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR = 4


# Source: navigation/navigation_pos_estimator_private.h
class navDefaultAltitudeSensor_e(enum.Enum):
    ALTITUDE_SOURCE_GPS = 0
    ALTITUDE_SOURCE_BARO = 1
    ALTITUDE_SOURCE_GPS_ONLY = 2
    ALTITUDE_SOURCE_BARO_ONLY = 3


# Source: navigation/navigation.h
class navExtraArmingSafety_e(enum.Enum):
    NAV_EXTRA_ARMING_SAFETY_ON = 0
    NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS = 1


# Source: navigation/navigation.h
class navFwLaunchStatus_e(enum.Enum):
    FW_LAUNCH_DETECTED = 5
    FW_LAUNCH_ABORTED = 10
    FW_LAUNCH_FLYING = 11


# Source: navigation/navigation.h
class navMcAltHoldThrottle_e(enum.Enum):
    MC_ALT_HOLD_STICK = 0
    MC_ALT_HOLD_MID = 1
    MC_ALT_HOLD_HOVER = 2


# Source: navigation/navigation.h
class navMissionRestart_e(enum.Enum):
    WP_MISSION_START = 0
    WP_MISSION_RESUME = 1
    WP_MISSION_SWITCH = 2


# Source: navigation/navigation.h
class navOverridesMotorStop_e(enum.Enum):
    NOMS_OFF_ALWAYS = 0
    NOMS_OFF = 1
    NOMS_AUTO_ONLY = 2
    NOMS_ALL_NAV = 3


# Source: navigation/navigation_pos_estimator_private.h
class navPositionEstimationFlags_e(enum.Enum):
    EST_GPS_XY_VALID = (1 << 0)
    EST_GPS_Z_VALID = (1 << 1)
    EST_BARO_VALID = (1 << 2)
    EST_SURFACE_VALID = (1 << 3)
    EST_FLOW_VALID = (1 << 4)
    EST_XY_VALID = (1 << 5)
    EST_Z_VALID = (1 << 6)


# Source: navigation/navigation.h
class navRTHAllowLanding_e(enum.Enum):
    NAV_RTH_ALLOW_LANDING_NEVER = 0
    NAV_RTH_ALLOW_LANDING_ALWAYS = 1
    NAV_RTH_ALLOW_LANDING_FS_ONLY = 2


# Source: navigation/navigation.h
class navRTHClimbFirst_e(enum.Enum):
    RTH_CLIMB_OFF = 0
    RTH_CLIMB_ON = 1
    RTH_CLIMB_ON_FW_SPIRAL = 2


# Source: navigation/navigation_private.h
class navSetWaypointFlags_t(enum.Enum):
    NAV_POS_UPDATE_NONE = 0
    NAV_POS_UPDATE_Z = 1 << 1
    NAV_POS_UPDATE_XY = 1 << 0
    NAV_POS_UPDATE_HEADING = 1 << 2
    NAV_POS_UPDATE_BEARING = 1 << 3
    NAV_POS_UPDATE_BEARING_TAIL_FIRST = 1 << 4


# Source: navigation/navigation.h
class navSystemStatus_Error_e(enum.Enum):
    MW_NAV_ERROR_NONE = 0
    MW_NAV_ERROR_TOOFAR = 1
    MW_NAV_ERROR_SPOILED_GPS = 2
    MW_NAV_ERROR_WP_CRC = 3
    MW_NAV_ERROR_FINISH = 4
    MW_NAV_ERROR_TIMEWAIT = 5
    MW_NAV_ERROR_INVALID_JUMP = 6
    MW_NAV_ERROR_INVALID_DATA = 7
    MW_NAV_ERROR_WAIT_FOR_RTH_ALT = 8
    MW_NAV_ERROR_GPS_FIX_LOST = 9
    MW_NAV_ERROR_DISARMED = 10
    MW_NAV_ERROR_LANDING = 11


# Source: navigation/navigation.h
class navSystemStatus_Flags_e(enum.Enum):
    MW_NAV_FLAG_ADJUSTING_POSITION = 1 << 0
    MW_NAV_FLAG_ADJUSTING_ALTITUDE = 1 << 1


# Source: navigation/navigation.h
class navSystemStatus_Mode_e(enum.Enum):
    MW_GPS_MODE_NONE = 0
    MW_GPS_MODE_HOLD = 1
    MW_GPS_MODE_RTH = 2
    MW_GPS_MODE_NAV = 3
    MW_GPS_MODE_EMERG = 15


# Source: navigation/navigation.h
class navSystemStatus_State_e(enum.Enum):
    MW_NAV_STATE_NONE = 0
    MW_NAV_STATE_RTH_START = 1
    MW_NAV_STATE_RTH_ENROUTE = 2
    MW_NAV_STATE_HOLD_INFINIT = 3
    MW_NAV_STATE_HOLD_TIMED = 4
    MW_NAV_STATE_WP_ENROUTE = 5
    MW_NAV_STATE_PROCESS_NEXT = 6
    MW_NAV_STATE_DO_JUMP = 7
    MW_NAV_STATE_LAND_START = 8
    MW_NAV_STATE_LAND_IN_PROGRESS = 9
    MW_NAV_STATE_LANDED = 10
    MW_NAV_STATE_LAND_SETTLE = 11
    MW_NAV_STATE_LAND_START_DESCENT = 12
    MW_NAV_STATE_HOVER_ABOVE_HOME = 13
    MW_NAV_STATE_EMERGENCY_LANDING = 14
    MW_NAV_STATE_RTH_CLIMB = 15


# Source: navigation/navigation.h
class navWaypointActions_e(enum.Enum):
    NAV_WP_ACTION_WAYPOINT = 0x01
    NAV_WP_ACTION_HOLD_TIME = 0x03
    NAV_WP_ACTION_RTH = 0x04
    NAV_WP_ACTION_SET_POI = 0x05
    NAV_WP_ACTION_JUMP = 0x06
    NAV_WP_ACTION_SET_HEAD = 0x07
    NAV_WP_ACTION_LAND = 0x08


# Source: navigation/navigation.h
class navWaypointFlags_e(enum.Enum):
    NAV_WP_FLAG_HOME = 0x48
    NAV_WP_FLAG_LAST = 0xA5


# Source: navigation/navigation.h
class navWaypointHeadings_e(enum.Enum):
    NAV_WP_HEAD_MODE_NONE = 0
    NAV_WP_HEAD_MODE_POI = 1
    NAV_WP_HEAD_MODE_FIXED = 2


# Source: navigation/navigation.h
class navWaypointP3Flags_e(enum.Enum):
    NAV_WP_ALTMODE = (1<<0)
    NAV_WP_USER1 = (1<<1)
    NAV_WP_USER2 = (1<<2)
    NAV_WP_USER3 = (1<<3)
    NAV_WP_USER4 = (1<<4)


# Source: navigation/navigation.h
class nav_reset_type_e(enum.Enum):
    NAV_RESET_NEVER = 0
    NAV_RESET_ON_FIRST_ARM = 1
    NAV_RESET_ON_EACH_ARM = 2


# Source: navigation/navigation_private.h
class navigationEstimateStatus_e(enum.Enum):
    EST_NONE = 0
    EST_USABLE = 1
    EST_TRUSTED = 2


# Source: navigation/navigation_private.h
class navigationFSMEvent_t(enum.Enum):
    NAV_FSM_EVENT_NONE = 0
    NAV_FSM_EVENT_TIMEOUT = 1
    NAV_FSM_EVENT_SUCCESS = 2
    NAV_FSM_EVENT_ERROR = 3
    NAV_FSM_EVENT_SWITCH_TO_IDLE = 4
    NAV_FSM_EVENT_SWITCH_TO_ALTHOLD = 5
    NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D = 6
    NAV_FSM_EVENT_SWITCH_TO_RTH = 7
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT = 8
    NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING = 9
    NAV_FSM_EVENT_SWITCH_TO_LAUNCH = 10
    NAV_FSM_EVENT_SWITCH_TO_COURSE_HOLD = 11
    NAV_FSM_EVENT_SWITCH_TO_CRUISE = 12
    NAV_FSM_EVENT_SWITCH_TO_COURSE_ADJ = 13
    NAV_FSM_EVENT_SWITCH_TO_MIXERAT = 14
    NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING = 15
    NAV_FSM_EVENT_SWITCH_TO_SEND_TO = 16
    NAV_FSM_EVENT_STATE_SPECIFIC_1 = 17
    NAV_FSM_EVENT_STATE_SPECIFIC_2 = 18
    NAV_FSM_EVENT_STATE_SPECIFIC_3 = 19
    NAV_FSM_EVENT_STATE_SPECIFIC_4 = 20
    NAV_FSM_EVENT_STATE_SPECIFIC_5 = 21
    NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_ABORT = NAV_FSM_EVENT_STATE_SPECIFIC_1
    NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_FINISHED = NAV_FSM_EVENT_STATE_SPECIFIC_2
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_HOLD_TIME = NAV_FSM_EVENT_STATE_SPECIFIC_1
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND = NAV_FSM_EVENT_STATE_SPECIFIC_2
    NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED = NAV_FSM_EVENT_STATE_SPECIFIC_3
    NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_INITIALIZE = NAV_FSM_EVENT_STATE_SPECIFIC_1
    NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_TRACKBACK = NAV_FSM_EVENT_STATE_SPECIFIC_2
    NAV_FSM_EVENT_SWITCH_TO_RTH_HEAD_HOME = NAV_FSM_EVENT_STATE_SPECIFIC_3
    NAV_FSM_EVENT_SWITCH_TO_RTH_LOITER_ABOVE_HOME = NAV_FSM_EVENT_STATE_SPECIFIC_4
    NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING = NAV_FSM_EVENT_STATE_SPECIFIC_5


# Source: navigation/navigation_private.h
class navigationFSMStateFlags_t(enum.Enum):
    NAV_CTL_ALT = (1 << 0)
    NAV_CTL_POS = (1 << 1)
    NAV_CTL_YAW = (1 << 2)
    NAV_CTL_EMERG = (1 << 3)
    NAV_CTL_LAUNCH = (1 << 4)
    NAV_REQUIRE_ANGLE = (1 << 5)
    NAV_REQUIRE_ANGLE_FW = (1 << 6)
    NAV_REQUIRE_MAGHOLD = (1 << 7)
    NAV_REQUIRE_THRTILT = (1 << 8)
    NAV_AUTO_RTH = (1 << 9)
    NAV_AUTO_WP = (1 << 10)
    NAV_RC_ALT = (1 << 11)
    NAV_RC_POS = (1 << 12)
    NAV_RC_YAW = (1 << 13)
    NAV_CTL_LAND = (1 << 14)
    NAV_AUTO_WP_DONE = (1 << 15)
    NAV_MIXERAT = (1 << 16)
    NAV_CTL_HOLD = (1 << 17)


# Source: navigation/navigation_private.h
class navigationFSMState_t(enum.Enum):
    NAV_STATE_UNDEFINED = 0
    NAV_STATE_IDLE = 1
    NAV_STATE_ALTHOLD_INITIALIZE = 2
    NAV_STATE_ALTHOLD_IN_PROGRESS = 3
    NAV_STATE_POSHOLD_3D_INITIALIZE = 4
    NAV_STATE_POSHOLD_3D_IN_PROGRESS = 5
    NAV_STATE_RTH_INITIALIZE = 6
    NAV_STATE_RTH_CLIMB_TO_SAFE_ALT = 7
    NAV_STATE_RTH_TRACKBACK = 8
    NAV_STATE_RTH_HEAD_HOME = 9
    NAV_STATE_RTH_LOITER_PRIOR_TO_LANDING = 10
    NAV_STATE_RTH_LOITER_ABOVE_HOME = 11
    NAV_STATE_RTH_LANDING = 12
    NAV_STATE_RTH_FINISHING = 13
    NAV_STATE_RTH_FINISHED = 14
    NAV_STATE_WAYPOINT_INITIALIZE = 15
    NAV_STATE_WAYPOINT_PRE_ACTION = 16
    NAV_STATE_WAYPOINT_IN_PROGRESS = 17
    NAV_STATE_WAYPOINT_REACHED = 18
    NAV_STATE_WAYPOINT_HOLD_TIME = 19
    NAV_STATE_WAYPOINT_NEXT = 20
    NAV_STATE_WAYPOINT_FINISHED = 21
    NAV_STATE_WAYPOINT_RTH_LAND = 22
    NAV_STATE_EMERGENCY_LANDING_INITIALIZE = 23
    NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS = 24
    NAV_STATE_EMERGENCY_LANDING_FINISHED = 25
    NAV_STATE_LAUNCH_INITIALIZE = 26
    NAV_STATE_LAUNCH_WAIT = 27
    NAV_STATE_LAUNCH_IN_PROGRESS = 28
    NAV_STATE_COURSE_HOLD_INITIALIZE = 29
    NAV_STATE_COURSE_HOLD_IN_PROGRESS = 30
    NAV_STATE_COURSE_HOLD_ADJUSTING = 31
    NAV_STATE_CRUISE_INITIALIZE = 32
    NAV_STATE_CRUISE_IN_PROGRESS = 33
    NAV_STATE_CRUISE_ADJUSTING = 34
    NAV_STATE_FW_LANDING_CLIMB_TO_LOITER = 35
    NAV_STATE_FW_LANDING_LOITER = 36
    NAV_STATE_FW_LANDING_APPROACH = 37
    NAV_STATE_FW_LANDING_GLIDE = 38
    NAV_STATE_FW_LANDING_FLARE = 39
    NAV_STATE_FW_LANDING_FINISHED = 40
    NAV_STATE_FW_LANDING_ABORT = 41
    NAV_STATE_MIXERAT_INITIALIZE = 42
    NAV_STATE_MIXERAT_IN_PROGRESS = 43
    NAV_STATE_MIXERAT_ABORT = 44
    NAV_STATE_SEND_TO_INITALIZE = 45
    NAV_STATE_SEND_TO_IN_PROGESS = 46
    NAV_STATE_SEND_TO_FINISHED = 47
    NAV_STATE_COUNT = 48


# Source: navigation/navigation_private.h
class navigationHomeFlags_t(enum.Enum):
    NAV_HOME_INVALID = 0
    NAV_HOME_VALID_XY = 1 << 0
    NAV_HOME_VALID_Z = 1 << 1
    NAV_HOME_VALID_HEADING = 1 << 2
    NAV_HOME_VALID_ALL = NAV_HOME_VALID_XY | NAV_HOME_VALID_Z | NAV_HOME_VALID_HEADING


# Source: navigation/navigation_private.h
class navigationPersistentId_e(enum.Enum):
    NAV_PERSISTENT_ID_UNDEFINED = 0
    NAV_PERSISTENT_ID_IDLE = 1
    NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE = 2
    NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS = 3
    NAV_PERSISTENT_ID_UNUSED_1 = 4
    NAV_PERSISTENT_ID_UNUSED_2 = 5
    NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE = 6
    NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS = 7
    NAV_PERSISTENT_ID_RTH_INITIALIZE = 8
    NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT = 9
    NAV_PERSISTENT_ID_RTH_HEAD_HOME = 10
    NAV_PERSISTENT_ID_RTH_LOITER_PRIOR_TO_LANDING = 11
    NAV_PERSISTENT_ID_RTH_LANDING = 12
    NAV_PERSISTENT_ID_RTH_FINISHING = 13
    NAV_PERSISTENT_ID_RTH_FINISHED = 14
    NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE = 15
    NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION = 16
    NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS = 17
    NAV_PERSISTENT_ID_WAYPOINT_REACHED = 18
    NAV_PERSISTENT_ID_WAYPOINT_NEXT = 19
    NAV_PERSISTENT_ID_WAYPOINT_FINISHED = 20
    NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND = 21
    NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE = 22
    NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS = 23
    NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED = 24
    NAV_PERSISTENT_ID_LAUNCH_INITIALIZE = 25
    NAV_PERSISTENT_ID_LAUNCH_WAIT = 26
    NAV_PERSISTENT_ID_UNUSED_3 = 27
    NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS = 28
    NAV_PERSISTENT_ID_COURSE_HOLD_INITIALIZE = 29
    NAV_PERSISTENT_ID_COURSE_HOLD_IN_PROGRESS = 30
    NAV_PERSISTENT_ID_COURSE_HOLD_ADJUSTING = 31
    NAV_PERSISTENT_ID_CRUISE_INITIALIZE = 32
    NAV_PERSISTENT_ID_CRUISE_IN_PROGRESS = 33
    NAV_PERSISTENT_ID_CRUISE_ADJUSTING = 34
    NAV_PERSISTENT_ID_WAYPOINT_HOLD_TIME = 35
    NAV_PERSISTENT_ID_RTH_LOITER_ABOVE_HOME = 36
    NAV_PERSISTENT_ID_UNUSED_4 = 37
    NAV_PERSISTENT_ID_RTH_TRACKBACK = 38
    NAV_PERSISTENT_ID_MIXERAT_INITIALIZE = 39
    NAV_PERSISTENT_ID_MIXERAT_IN_PROGRESS = 40
    NAV_PERSISTENT_ID_MIXERAT_ABORT = 41
    NAV_PERSISTENT_ID_FW_LANDING_CLIMB_TO_LOITER = 42
    NAV_PERSISTENT_ID_FW_LANDING_LOITER = 43
    NAV_PERSISTENT_ID_FW_LANDING_APPROACH = 44
    NAV_PERSISTENT_ID_FW_LANDING_GLIDE = 45
    NAV_PERSISTENT_ID_FW_LANDING_FLARE = 46
    NAV_PERSISTENT_ID_FW_LANDING_ABORT = 47
    NAV_PERSISTENT_ID_FW_LANDING_FINISHED = 48
    NAV_PERSISTENT_ID_SEND_TO_INITALIZE = 49
    NAV_PERSISTENT_ID_SEND_TO_IN_PROGRES = 50
    NAV_PERSISTENT_ID_SEND_TO_FINISHED = 51


# Source: sensors/opflow.h
class opflowQuality_e(enum.Enum):
    OPFLOW_QUALITY_INVALID = 0
    OPFLOW_QUALITY_VALID = 1


# Source: sensors/opflow.h
class opticalFlowSensor_e(enum.Enum):
    OPFLOW_NONE = 0
    OPFLOW_CXOF = 1
    OPFLOW_MSP = 2
    OPFLOW_FAKE = 3


# Source: io/osd_common.h
class osdDrawPointType_e(enum.Enum):
    OSD_DRAW_POINT_TYPE_GRID = 0
    OSD_DRAW_POINT_TYPE_PIXEL = 1


# Source: io/osd_common.h
class osdSpeedSource_e(enum.Enum):
    OSD_SPEED_SOURCE_GROUND = 0
    OSD_SPEED_SOURCE_3D = 1
    OSD_SPEED_SOURCE_AIR = 2


# Source: io/osd.h
class osd_ahi_style_e(enum.Enum):
    OSD_AHI_STYLE_DEFAULT = 0
    OSD_AHI_STYLE_LINE = 1


# Source: io/osd.h
class osd_alignment_e(enum.Enum):
    OSD_ALIGN_LEFT = 0
    OSD_ALIGN_RIGHT = 1


# Source: io/osd.h
class osd_crosshairs_style_e(enum.Enum):
    OSD_CROSSHAIRS_STYLE_DEFAULT = 0
    OSD_CROSSHAIRS_STYLE_AIRCRAFT = 1
    OSD_CROSSHAIRS_STYLE_TYPE3 = 2
    OSD_CROSSHAIRS_STYLE_TYPE4 = 3
    OSD_CROSSHAIRS_STYLE_TYPE5 = 4
    OSD_CROSSHAIRS_STYLE_TYPE6 = 5
    OSD_CROSSHAIRS_STYLE_TYPE7 = 6


# Source: io/osd.h
class osd_crsf_lq_format_e(enum.Enum):
    OSD_CRSF_LQ_TYPE1 = 0
    OSD_CRSF_LQ_TYPE2 = 1
    OSD_CRSF_LQ_TYPE3 = 2


# Source: io/osd.h
class osd_items_e(enum.Enum):
    OSD_RSSI_VALUE = 0
    OSD_MAIN_BATT_VOLTAGE = 1
    OSD_CROSSHAIRS = 2
    OSD_ARTIFICIAL_HORIZON = 3
    OSD_HORIZON_SIDEBARS = 4
    OSD_ONTIME = 5
    OSD_FLYTIME = 6
    OSD_FLYMODE = 7
    OSD_CRAFT_NAME = 8
    OSD_THROTTLE_POS = 9
    OSD_VTX_CHANNEL = 10
    OSD_CURRENT_DRAW = 11
    OSD_MAH_DRAWN = 12
    OSD_GPS_SPEED = 13
    OSD_GPS_SATS = 14
    OSD_ALTITUDE = 15
    OSD_ROLL_PIDS = 16
    OSD_PITCH_PIDS = 17
    OSD_YAW_PIDS = 18
    OSD_POWER = 19
    OSD_GPS_LON = 20
    OSD_GPS_LAT = 21
    OSD_HOME_DIR = 22
    OSD_HOME_DIST = 23
    OSD_HEADING = 24
    OSD_VARIO = 25
    OSD_VARIO_NUM = 26
    OSD_AIR_SPEED = 27
    OSD_ONTIME_FLYTIME = 28
    OSD_RTC_TIME = 29
    OSD_MESSAGES = 30
    OSD_GPS_HDOP = 31
    OSD_MAIN_BATT_CELL_VOLTAGE = 32
    OSD_SCALED_THROTTLE_POS = 33
    OSD_HEADING_GRAPH = 34
    OSD_EFFICIENCY_MAH_PER_KM = 35
    OSD_WH_DRAWN = 36
    OSD_BATTERY_REMAINING_CAPACITY = 37
    OSD_BATTERY_REMAINING_PERCENT = 38
    OSD_EFFICIENCY_WH_PER_KM = 39
    OSD_TRIP_DIST = 40
    OSD_ATTITUDE_PITCH = 41
    OSD_ATTITUDE_ROLL = 42
    OSD_MAP_NORTH = 43
    OSD_MAP_TAKEOFF = 44
    OSD_RADAR = 45
    OSD_WIND_SPEED_HORIZONTAL = 46
    OSD_WIND_SPEED_VERTICAL = 47
    OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH = 48
    OSD_REMAINING_DISTANCE_BEFORE_RTH = 49
    OSD_HOME_HEADING_ERROR = 50
    OSD_COURSE_HOLD_ERROR = 51
    OSD_COURSE_HOLD_ADJUSTMENT = 52
    OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE = 53
    OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE = 54
    OSD_POWER_SUPPLY_IMPEDANCE = 55
    OSD_LEVEL_PIDS = 56
    OSD_POS_XY_PIDS = 57
    OSD_POS_Z_PIDS = 58
    OSD_VEL_XY_PIDS = 59
    OSD_VEL_Z_PIDS = 60
    OSD_HEADING_P = 61
    OSD_BOARD_ALIGN_ROLL = 62
    OSD_BOARD_ALIGN_PITCH = 63
    OSD_RC_EXPO = 64
    OSD_RC_YAW_EXPO = 65
    OSD_THROTTLE_EXPO = 66
    OSD_PITCH_RATE = 67
    OSD_ROLL_RATE = 68
    OSD_YAW_RATE = 69
    OSD_MANUAL_RC_EXPO = 70
    OSD_MANUAL_RC_YAW_EXPO = 71
    OSD_MANUAL_PITCH_RATE = 72
    OSD_MANUAL_ROLL_RATE = 73
    OSD_MANUAL_YAW_RATE = 74
    OSD_NAV_FW_CRUISE_THR = 75
    OSD_NAV_FW_PITCH2THR = 76
    OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE = 77
    OSD_DEBUG = 78
    OSD_FW_ALT_PID_OUTPUTS = 79
    OSD_FW_POS_PID_OUTPUTS = 80
    OSD_MC_VEL_X_PID_OUTPUTS = 81
    OSD_MC_VEL_Y_PID_OUTPUTS = 82
    OSD_MC_VEL_Z_PID_OUTPUTS = 83
    OSD_MC_POS_XYZ_P_OUTPUTS = 84
    OSD_3D_SPEED = 85
    OSD_IMU_TEMPERATURE = 86
    OSD_BARO_TEMPERATURE = 87
    OSD_TEMP_SENSOR_0_TEMPERATURE = 88
    OSD_TEMP_SENSOR_1_TEMPERATURE = 89
    OSD_TEMP_SENSOR_2_TEMPERATURE = 90
    OSD_TEMP_SENSOR_3_TEMPERATURE = 91
    OSD_TEMP_SENSOR_4_TEMPERATURE = 92
    OSD_TEMP_SENSOR_5_TEMPERATURE = 93
    OSD_TEMP_SENSOR_6_TEMPERATURE = 94
    OSD_TEMP_SENSOR_7_TEMPERATURE = 95
    OSD_ALTITUDE_MSL = 96
    OSD_PLUS_CODE = 97
    OSD_MAP_SCALE = 98
    OSD_MAP_REFERENCE = 99
    OSD_GFORCE = 100
    OSD_GFORCE_X = 101
    OSD_GFORCE_Y = 102
    OSD_GFORCE_Z = 103
    OSD_RC_SOURCE = 104
    OSD_VTX_POWER = 105
    OSD_ESC_RPM = 106
    OSD_ESC_TEMPERATURE = 107
    OSD_AZIMUTH = 108
    OSD_RSSI_DBM = 109
    OSD_LQ_UPLINK = 110
    OSD_SNR_DB = 111
    OSD_TX_POWER_UPLINK = 112
    OSD_GVAR_0 = 113
    OSD_GVAR_1 = 114
    OSD_GVAR_2 = 115
    OSD_GVAR_3 = 116
    OSD_TPA = 117
    OSD_NAV_FW_CONTROL_SMOOTHNESS = 118
    OSD_VERSION = 119
    OSD_RANGEFINDER = 120
    OSD_PLIMIT_REMAINING_BURST_TIME = 121
    OSD_PLIMIT_ACTIVE_CURRENT_LIMIT = 122
    OSD_PLIMIT_ACTIVE_POWER_LIMIT = 123
    OSD_GLIDESLOPE = 124
    OSD_GPS_MAX_SPEED = 125
    OSD_3D_MAX_SPEED = 126
    OSD_AIR_MAX_SPEED = 127
    OSD_ACTIVE_PROFILE = 128
    OSD_MISSION = 129
    OSD_SWITCH_INDICATOR_0 = 130
    OSD_SWITCH_INDICATOR_1 = 131
    OSD_SWITCH_INDICATOR_2 = 132
    OSD_SWITCH_INDICATOR_3 = 133
    OSD_TPA_TIME_CONSTANT = 134
    OSD_FW_LEVEL_TRIM = 135
    OSD_GLIDE_TIME_REMAINING = 136
    OSD_GLIDE_RANGE = 137
    OSD_CLIMB_EFFICIENCY = 138
    OSD_NAV_WP_MULTI_MISSION_INDEX = 139
    OSD_GROUND_COURSE = 140
    OSD_CROSS_TRACK_ERROR = 141
    OSD_PILOT_NAME = 142
    OSD_PAN_SERVO_CENTRED = 143
    OSD_MULTI_FUNCTION = 144
    OSD_ODOMETER = 145
    OSD_PILOT_LOGO = 146
    OSD_CUSTOM_ELEMENT_1 = 147
    OSD_CUSTOM_ELEMENT_2 = 148
    OSD_CUSTOM_ELEMENT_3 = 149
    OSD_ADSB_WARNING = 150
    OSD_ADSB_INFO = 151
    OSD_BLACKBOX = 152
    OSD_FORMATION_FLIGHT = 153
    OSD_CUSTOM_ELEMENT_4 = 154
    OSD_CUSTOM_ELEMENT_5 = 155
    OSD_CUSTOM_ELEMENT_6 = 156
    OSD_CUSTOM_ELEMENT_7 = 157
    OSD_CUSTOM_ELEMENT_8 = 158
    OSD_LQ_DOWNLINK = 159
    OSD_RX_POWER_DOWNLINK = 160
    OSD_RX_BAND = 161
    OSD_RX_MODE = 162
    OSD_COURSE_TO_FENCE = 163
    OSD_H_DIST_TO_FENCE = 164
    OSD_V_DIST_TO_FENCE = 165
    OSD_NAV_FW_ALT_CONTROL_RESPONSE = 166
    OSD_ITEM_COUNT = 167


# Source: io/osd.h
class osd_sidebar_scroll_e(enum.Enum):
    OSD_SIDEBAR_SCROLL_NONE = 0
    OSD_SIDEBAR_SCROLL_ALTITUDE = 1
    OSD_SIDEBAR_SCROLL_SPEED = 2
    OSD_SIDEBAR_SCROLL_HOME_DISTANCE = 3
    OSD_SIDEBAR_SCROLL_MAX = OSD_SIDEBAR_SCROLL_HOME_DISTANCE


# Source: io/osd.h
class osd_stats_energy_unit_e(enum.Enum):
    OSD_STATS_ENERGY_UNIT_MAH = 0
    OSD_STATS_ENERGY_UNIT_WH = 1


# Source: io/osd.h
class osd_unit_e(enum.Enum):
    OSD_UNIT_IMPERIAL = 0
    OSD_UNIT_METRIC = 1
    OSD_UNIT_METRIC_MPH = 2
    OSD_UNIT_UK = 3
    OSD_UNIT_GA = 4
    OSD_UNIT_MAX = OSD_UNIT_GA


# Source: flight/mixer.h
class outputMode_e(enum.Enum):
    OUTPUT_MODE_AUTO = 0
    OUTPUT_MODE_MOTORS = 1
    OUTPUT_MODE_SERVOS = 2
    OUTPUT_MODE_LED = 3


# Source: io/dashboard.h
class pageId_e(enum.Enum):
    PAGE_WELCOME = 0
    PAGE_ARMED = 1
    PAGE_STATUS = 2


# Source: config/parameter_group.h
class pgRegistryFlags_e(enum.Enum):
    PGRF_NONE = 0
    PGRF_CLASSIFICATON_BIT = (1 << 0)


# Source: config/parameter_group.h
class pgRegistryInternal_e(enum.Enum):
    PGR_PGN_MASK = 0x0fff
    PGR_PGN_VERSION_MASK = 0xf000
    PGR_SIZE_MASK = 0x0fff
    PGR_SIZE_SYSTEM_FLAG = 0x0000
    PGR_SIZE_PROFILE_FLAG = 0x8000


# Source: common/fp_pid.h
class pidControllerFlags_e(enum.Enum):
    PID_DTERM_FROM_ERROR = 1 << 0
    PID_ZERO_INTEGRATOR = 1 << 1
    PID_SHRINK_INTEGRATOR = 1 << 2
    PID_LIMIT_INTEGRATOR = 1 << 3
    PID_FREEZE_INTEGRATOR = 1 << 4


# Source: flight/pid.h
class pidIndex_e(enum.Enum):
    PID_ROLL = 0
    PID_PITCH = 1
    PID_YAW = 2
    PID_POS_Z = 3
    PID_POS_XY = 4
    PID_VEL_XY = 5
    PID_SURFACE = 6
    PID_LEVEL = 7
    PID_HEADING = 8
    PID_VEL_Z = 9
    PID_POS_HEADING = 10
    PID_ITEM_COUNT = 11


# Source: flight/pid.h
class pidType_e(enum.Enum):
    PID_TYPE_NONE = 0
    PID_TYPE_PID = 1
    PID_TYPE_PIFF = 2
    PID_TYPE_AUTO = 3


# Source: sensors/pitotmeter.h
class pitotSensor_e(enum.Enum):
    PITOT_NONE = 0
    PITOT_AUTODETECT = 1
    PITOT_MS4525 = 2
    PITOT_ADC = 3
    PITOT_VIRTUAL = 4
    PITOT_FAKE = 5
    PITOT_MSP = 6
    PITOT_DLVR = 7


# Source: io/serial.h
class portSharing_e(enum.Enum):
    PORTSHARING_UNUSED = 0
    PORTSHARING_NOT_SHARED = 1
    PORTSHARING_SHARED = 2


# Source: sensors/rangefinder.h
class rangefinderType_e(enum.Enum):
    RANGEFINDER_NONE = 0
    RANGEFINDER_SRF10 = 1
    RANGEFINDER_VL53L0X = 2
    RANGEFINDER_MSP = 3
    RANGEFINDER_BENEWAKE = 4
    RANGEFINDER_VL53L1X = 5
    RANGEFINDER_US42 = 6
    RANGEFINDER_TOF10102I2C = 7
    RANGEFINDER_FAKE = 8
    RANGEFINDER_TERARANGER_EVO = 9
    RANGEFINDER_USD1_V0 = 10
    RANGEFINDER_NANORADAR = 11


# Source: io/rcdevice.h
class rcdeviceCamSimulationKeyEvent_e(enum.Enum):
    RCDEVICE_CAM_KEY_NONE = 0
    RCDEVICE_CAM_KEY_ENTER = 1
    RCDEVICE_CAM_KEY_LEFT = 2
    RCDEVICE_CAM_KEY_UP = 3
    RCDEVICE_CAM_KEY_RIGHT = 4
    RCDEVICE_CAM_KEY_DOWN = 5
    RCDEVICE_CAM_KEY_CONNECTION_CLOSE = 6
    RCDEVICE_CAM_KEY_CONNECTION_OPEN = 7
    RCDEVICE_CAM_KEY_RELEASE = 8


# Source: io/rcdevice.h
class rcdeviceResponseStatus_e(enum.Enum):
    RCDEVICE_RESP_SUCCESS = 0
    RCDEVICE_RESP_INCORRECT_CRC = 1
    RCDEVICE_RESP_TIMEOUT = 2


# Source: io/rcdevice.h
class rcdevice_5key_simulation_operation_e(enum.Enum):
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE = 0x00
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET = 0x01
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT = 0x02
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT = 0x03
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP = 0x04
    RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN = 0x05


# Source: io/rcdevice.h
class rcdevice_camera_control_opeation_e(enum.Enum):
    RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN = 0x00
    RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN = 0x01
    RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE = 0x02
    RCDEVICE_PROTOCOL_CAM_CTRL_START_RECORDING = 0x03
    RCDEVICE_PROTOCOL_CAM_CTRL_STOP_RECORDING = 0x04
    RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION = 0xFF


# Source: io/rcdevice.h
class rcdevice_features_e(enum.Enum):
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON = (1 << 0)
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON = (1 << 1)
    RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE = (1 << 2)
    RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3)
    RCDEVICE_PROTOCOL_FEATURE_START_RECORDING = (1 << 6)
    RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING = (1 << 7)
    RCDEVICE_PROTOCOL_FEATURE_CMS_MENU = (1 << 8)


# Source: io/rcdevice.h
class rcdevice_protocol_version_e(enum.Enum):
    RCDEVICE_PROTOCOL_RCSPLIT_VERSION = 0x00
    RCDEVICE_PROTOCOL_VERSION_1_0 = 0x01
    RCDEVICE_PROTOCOL_UNKNOWN = 2


# Source: flight/mixer.h
class reversibleMotorsThrottleState_e(enum.Enum):
    MOTOR_DIRECTION_FORWARD = 0
    MOTOR_DIRECTION_BACKWARD = 1
    MOTOR_DIRECTION_DEADBAND = 2


# Source: fc/rc_controls.h
class rollPitchStatus_e(enum.Enum):
    NOT_CENTERED = 0
    CENTERED = 1


# Source: rx/rx.h
class rssiSource_e(enum.Enum):
    RSSI_SOURCE_NONE = 0
    RSSI_SOURCE_AUTO = 1
    RSSI_SOURCE_ADC = 2
    RSSI_SOURCE_RX_CHANNEL = 3
    RSSI_SOURCE_RX_PROTOCOL = 4
    RSSI_SOURCE_MSP = 5


# Source: flight/failsafe.h
class rthState_e(enum.Enum):
    RTH_IDLE = 0
    RTH_IN_PROGRESS = 1
    RTH_HAS_LANDED = 2


# Source: navigation/navigation_private.h
class rthTargetMode_e(enum.Enum):
    RTH_HOME_ENROUTE_INITIAL = 0
    RTH_HOME_ENROUTE_PROPORTIONAL = 1
    RTH_HOME_ENROUTE_FINAL = 2
    RTH_HOME_FINAL_LOITER = 3
    RTH_HOME_FINAL_LAND = 4


# Source: navigation/navigation.h
class rthTrackbackMode_e(enum.Enum):
    RTH_TRACKBACK_OFF = 0
    RTH_TRACKBACK_ON = 1
    RTH_TRACKBACK_FS = 2


# Source: rx/rx.h
class rxFrameState_e(enum.Enum):
    RX_FRAME_PENDING = 0
    RX_FRAME_COMPLETE = (1 << 0)
    RX_FRAME_FAILSAFE = (1 << 1)
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2)
    RX_FRAME_DROPPED = (1 << 3)


# Source: rx/rx.h
class rxReceiverType_e(enum.Enum):
    RX_TYPE_NONE = 0
    RX_TYPE_SERIAL = 1
    RX_TYPE_MSP = 2
    RX_TYPE_SIM = 3


# Source: rx/rx.h
class rxSerialReceiverType_e(enum.Enum):
    SERIALRX_SPEKTRUM1024 = 0
    SERIALRX_SPEKTRUM2048 = 1
    SERIALRX_SBUS = 2
    SERIALRX_SUMD = 3
    SERIALRX_IBUS = 4
    SERIALRX_JETIEXBUS = 5
    SERIALRX_CRSF = 6
    SERIALRX_FPORT = 7
    SERIALRX_SBUS_FAST = 8
    SERIALRX_FPORT2 = 9
    SERIALRX_SRXL2 = 10
    SERIALRX_GHST = 11
    SERIALRX_MAVLINK = 12
    SERIALRX_FBUS = 13
    SERIALRX_SBUS2 = 14


# Source: navigation/navigation.h
class safehomeUsageMode_e(enum.Enum):
    SAFEHOME_USAGE_OFF = 0
    SAFEHOME_USAGE_RTH = 1
    SAFEHOME_USAGE_RTH_FS = 2


# Source: io/gps.h
class sbasMode_e(enum.Enum):
    SBAS_AUTO = 0
    SBAS_EGNOS = 1
    SBAS_WAAS = 2
    SBAS_MSAS = 3
    SBAS_GAGAN = 4
    SBAS_SPAN = 5
    SBAS_NONE = 6


# Source: sensors/sensors.h
class sensorIndex_e(enum.Enum):
    SENSOR_INDEX_GYRO = 0
    SENSOR_INDEX_ACC = 1
    SENSOR_INDEX_BARO = 2
    SENSOR_INDEX_MAG = 3
    SENSOR_INDEX_RANGEFINDER = 4
    SENSOR_INDEX_PITOT = 5
    SENSOR_INDEX_OPFLOW = 6
    SENSOR_INDEX_COUNT = 7


# Source: sensors/sensors.h
class sensors_e(enum.Enum):
    SENSOR_GYRO = 1 << 0
    SENSOR_ACC = 1 << 1
    SENSOR_BARO = 1 << 2
    SENSOR_MAG = 1 << 3
    SENSOR_RANGEFINDER = 1 << 4
    SENSOR_PITOT = 1 << 5
    SENSOR_OPFLOW = 1 << 6
    SENSOR_GPS = 1 << 7
    SENSOR_GPSMAG = 1 << 8
    SENSOR_TEMP = 1 << 9


# Source: io/serial.h
class serialPortFunction_e(enum.Enum):
    FUNCTION_NONE = 0
    FUNCTION_MSP = (1 << 0)
    FUNCTION_GPS = (1 << 1)
    FUNCTION_UNUSED_3 = (1 << 2)
    FUNCTION_TELEMETRY_HOTT = (1 << 3)
    FUNCTION_TELEMETRY_LTM = (1 << 4)
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5)
    FUNCTION_RX_SERIAL = (1 << 6)
    FUNCTION_BLACKBOX = (1 << 7)
    FUNCTION_TELEMETRY_MAVLINK = (1 << 8)
    FUNCTION_TELEMETRY_IBUS = (1 << 9)
    FUNCTION_RCDEVICE = (1 << 10)
    FUNCTION_VTX_SMARTAUDIO = (1 << 11)
    FUNCTION_VTX_TRAMP = (1 << 12)
    FUNCTION_UNUSED_1 = (1 << 13)
    FUNCTION_OPTICAL_FLOW = (1 << 14)
    FUNCTION_LOG = (1 << 15)
    FUNCTION_RANGEFINDER = (1 << 16)
    FUNCTION_VTX_FFPV = (1 << 17)
    FUNCTION_ESCSERIAL = (1 << 18)
    FUNCTION_TELEMETRY_SIM = (1 << 19)
    FUNCTION_FRSKY_OSD = (1 << 20)
    FUNCTION_DJI_HD_OSD = (1 << 21)
    FUNCTION_SERVO_SERIAL = (1 << 22)
    FUNCTION_TELEMETRY_SMARTPORT_MASTER = (1 << 23)
    FUNCTION_UNUSED_2 = (1 << 24)
    FUNCTION_MSP_OSD = (1 << 25)
    FUNCTION_GIMBAL = (1 << 26)
    FUNCTION_GIMBAL_HEADTRACKER = (1 << 27)


# Source: io/serial.h
class serialPortIdentifier_e(enum.Enum):
    # Skipped Member: SERIAL_PORT_NONE = -1 # (Unparseable/Complex/Filtered)
    SERIAL_PORT_USART1 = 0
    SERIAL_PORT_USART2 = 1
    SERIAL_PORT_USART3 = 2
    SERIAL_PORT_USART4 = 3
    SERIAL_PORT_USART5 = 4
    SERIAL_PORT_USART6 = 5
    SERIAL_PORT_USART7 = 6
    SERIAL_PORT_USART8 = 7
    SERIAL_PORT_USB_VCP = 20
    SERIAL_PORT_SOFTSERIAL1 = 30
    SERIAL_PORT_SOFTSERIAL2 = 31
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2


# Source: flight/servos.h
class servoIndex_e(enum.Enum):
    SERVO_GIMBAL_PITCH = 0
    SERVO_GIMBAL_ROLL = 1
    SERVO_ELEVATOR = 2
    SERVO_FLAPPERON_1 = 3
    SERVO_FLAPPERON_2 = 4
    SERVO_RUDDER = 5
    SERVO_BICOPTER_LEFT = 4
    SERVO_BICOPTER_RIGHT = 5
    SERVO_DUALCOPTER_LEFT = 4
    SERVO_DUALCOPTER_RIGHT = 5
    SERVO_SINGLECOPTER_1 = 3
    SERVO_SINGLECOPTER_2 = 4
    SERVO_SINGLECOPTER_3 = 5
    SERVO_SINGLECOPTER_4 = 6


# Source: fc/settings.h
class setting_mode_e(enum.Enum):
    MODE_DIRECT = (0 << SETTING_MODE_OFFSET)
    MODE_LOOKUP = (1 << SETTING_MODE_OFFSET)


# Source: fc/settings.h
class setting_section_e(enum.Enum):
    MASTER_VALUE = (0 << SETTING_SECTION_OFFSET)
    PROFILE_VALUE = (1 << SETTING_SECTION_OFFSET)
    CONTROL_RATE_VALUE = (2 << SETTING_SECTION_OFFSET)
    BATTERY_CONFIG_VALUE = (3 << SETTING_SECTION_OFFSET)
    MIXER_CONFIG_VALUE = (4 << SETTING_SECTION_OFFSET)
    EZ_TUNE_VALUE = (5 << SETTING_SECTION_OFFSET)


# Source: fc/settings.h
class setting_type_e(enum.Enum):
    VAR_UINT8 = (0 << SETTING_TYPE_OFFSET)
    VAR_INT8 = (1 << SETTING_TYPE_OFFSET)
    VAR_UINT16 = (2 << SETTING_TYPE_OFFSET)
    VAR_INT16 = (3 << SETTING_TYPE_OFFSET)
    VAR_UINT32 = (4 << SETTING_TYPE_OFFSET)
    VAR_FLOAT = (5 << SETTING_TYPE_OFFSET)
    VAR_STRING = (6 << SETTING_TYPE_OFFSET)


# Source: telemetry/sim.h
class simTxFlags_e(enum.Enum):
    SIM_TX_FLAG = (1 << 0)
    SIM_TX_FLAG_FAILSAFE = (1 << 1)
    SIM_TX_FLAG_GPS = (1 << 2)
    SIM_TX_FLAG_ACC = (1 << 3)
    SIM_TX_FLAG_LOW_ALT = (1 << 4)
    SIM_TX_FLAG_RESPONSE = (1 << 5)


# Source: fc/runtime_config.h
class simulatorFlags_t(enum.Enum):
    HITL_RESET_FLAGS = (0 << 0)
    HITL_ENABLE = (1 << 0)
    HITL_SIMULATE_BATTERY = (1 << 1)
    HITL_MUTE_BEEPER = (1 << 2)
    HITL_USE_IMU = (1 << 3)
    HITL_HAS_NEW_GPS_DATA = (1 << 4)
    HITL_EXT_BATTERY_VOLTAGE = (1 << 5)
    HITL_AIRSPEED = (1 << 6)
    HITL_EXTENDED_FLAGS = (1 << 7)
    HITL_GPS_TIMEOUT = (1 << 8)
    HITL_PITOT_FAILURE = (1 << 9)


# Source: io/vtx_smartaudio.h
class smartAudioVersion_e(enum.Enum):
    SA_UNKNOWN = 0
    SA_1_0 = 1
    SA_2_0 = 2
    SA_2_1 = 3


# Source: telemetry/telemetry.h
class smartportFuelUnit_e(enum.Enum):
    SMARTPORT_FUEL_UNIT_PERCENT = 0
    SMARTPORT_FUEL_UNIT_MAH = 1
    SMARTPORT_FUEL_UNIT_MWH = 2


# Source: fc/runtime_config.h
class stateFlags_t(enum.Enum):
    GPS_FIX_HOME = (1 << 0)
    GPS_FIX = (1 << 1)
    CALIBRATE_MAG = (1 << 2)
    SMALL_ANGLE = (1 << 3)
    FIXED_WING_LEGACY = (1 << 4)
    ANTI_WINDUP = (1 << 5)
    FLAPERON_AVAILABLE = (1 << 6)
    NAV_MOTOR_STOP_OR_IDLE = (1 << 7)
    COMPASS_CALIBRATED = (1 << 8)
    ACCELEROMETER_CALIBRATED = (1 << 9)
    GPS_ESTIMATED_FIX = (1 << 10)
    NAV_CRUISE_BRAKING = (1 << 11)
    NAV_CRUISE_BRAKING_BOOST = (1 << 12)
    NAV_CRUISE_BRAKING_LOCKED = (1 << 13)
    NAV_EXTRA_ARMING_SAFETY_BYPASSED = (1 << 14)
    AIRMODE_ACTIVE = (1 << 15)
    ESC_SENSOR_ENABLED = (1 << 16)
    AIRPLANE = (1 << 17)
    MULTIROTOR = (1 << 18)
    ROVER = (1 << 19)
    BOAT = (1 << 20)
    ALTITUDE_CONTROL = (1 << 21)
    MOVE_FORWARD_ONLY = (1 << 22)
    SET_REVERSIBLE_MOTORS_FORWARD = (1 << 23)
    FW_HEADING_USE_YAW = (1 << 24)
    ANTI_WINDUP_DEACTIVATED = (1 << 25)
    LANDING_DETECTED = (1 << 26)
    IN_FLIGHT_EMERG_REARM = (1 << 27)
    TAILSITTER = (1 << 28)


# Source: fc/rc_controls.h
class stickPositions_e(enum.Enum):
    pass # No valid members generated or all skipped


# Source: fc/fc_init.h
class systemState_e(enum.Enum):
    SYSTEM_STATE_INITIALISING = 0
    SYSTEM_STATE_CONFIG_LOADED = (1 << 0)
    SYSTEM_STATE_SENSORS_READY = (1 << 1)
    SYSTEM_STATE_MOTORS_READY = (1 << 2)
    SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3)
    SYSTEM_STATE_READY = (1 << 7)


# Source: sensors/temperature.h
class tempSensorType_e(enum.Enum):
    TEMP_SENSOR_NONE = 0
    TEMP_SENSOR_LM75 = 1
    TEMP_SENSOR_DS18B20 = 2


# Source: fc/rc_controls.h
class throttleStatusType_e(enum.Enum):
    THROTTLE_STATUS_TYPE_RC = 0
    THROTTLE_STATUS_TYPE_COMMAND = 1


# Source: fc/rc_controls.h
class throttleStatus_e(enum.Enum):
    THROTTLE_LOW = 0
    THROTTLE_HIGH = 1


# Source: common/tristate.h
class tristate_e(enum.Enum):
    TRISTATE_AUTO = 0
    TRISTATE_ON = 1
    TRISTATE_OFF = 2


# Source: common/time.h
class tz_automatic_dst_e(enum.Enum):
    TZ_AUTO_DST_OFF = 0
    TZ_AUTO_DST_EU = 1
    TZ_AUTO_DST_USA = 2


# Source: io/gps_ublox.h
class ublox_nav_sig_health_e(enum.Enum):
    UBLOX_SIG_HEALTH_UNKNOWN = 0
    UBLOX_SIG_HEALTH_HEALTHY = 1
    UBLOX_SIG_HEALTH_UNHEALTHY = 2


# Source: io/gps_ublox.h
class ublox_nav_sig_quality(enum.Enum):
    UBLOX_SIG_QUALITY_NOSIGNAL = 0
    UBLOX_SIG_QUALITY_SEARCHING = 1
    UBLOX_SIG_QUALITY_ACQUIRED = 2
    UBLOX_SIG_QUALITY_UNUSABLE = 3
    UBLOX_SIG_QUALITY_CODE_LOCK_TIME_SYNC = 4
    UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC = 5
    UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC2 = 6
    UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC3 = 7


# Source: io/gps_ublox.h
class ubs_nav_fix_type_t(enum.Enum):
    FIX_NONE = 0
    FIX_DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    FIX_GPS_DEAD_RECKONING = 4
    FIX_TIME = 5


# Source: io/gps_ublox.h
class ubx_ack_state_t(enum.Enum):
    UBX_ACK_WAITING = 0
    UBX_ACK_GOT_ACK = 1
    UBX_ACK_GOT_NAK = 2


# Source: io/gps_ublox.h
class ubx_nav_status_bits_t(enum.Enum):
    NAV_STATUS_FIX_VALID = 1


# Source: io/gps_ublox.h
class ubx_protocol_bytes_t(enum.Enum):
    PREAMBLE1 = 0xB5
    PREAMBLE2 = 0x62
    CLASS_NAV = 0x01
    CLASS_ACK = 0x05
    CLASS_CFG = 0x06
    CLASS_MON = 0x0A
    MSG_CLASS_UBX = 0x01
    MSG_CLASS_NMEA = 0xF0
    MSG_VER = 0x04
    MSG_ACK_NACK = 0x00
    MSG_ACK_ACK = 0x01
    MSG_NMEA_GGA = 0x0
    MSG_NMEA_GLL = 0x1
    MSG_NMEA_GSA = 0x2
    MSG_NMEA_GSV = 0x3
    MSG_NMEA_RMC = 0x4
    MSG_NMEA_VGS = 0x5
    MSG_POSLLH = 0x2
    MSG_STATUS = 0x3
    MSG_SOL = 0x6
    MSG_PVT = 0x7
    MSG_VELNED = 0x12
    MSG_TIMEUTC = 0x21
    MSG_SVINFO = 0x30
    MSG_NAV_SAT = 0x35
    MSG_CFG_PRT = 0x00
    MSG_CFG_RATE = 0x08
    MSG_CFG_SET_RATE = 0x01
    MSG_CFG_NAV_SETTINGS = 0x24
    MSG_CFG_SBAS = 0x16
    MSG_CFG_GNSS = 0x3e
    MSG_MON_GNSS = 0x28
    MSG_NAV_SIG = 0x43


# Source: sensors/battery_config_structs.h
class voltageSensor_e(enum.Enum):
    VOLTAGE_SENSOR_NONE = 0
    VOLTAGE_SENSOR_ADC = 1
    VOLTAGE_SENSOR_ESC = 2
    VOLTAGE_SENSOR_FAKE = 3


# Source: io/smartport_master.h
class vs600Band_e(enum.Enum):
    VS600_BAND_A = 0
    VS600_BAND_B = 1
    VS600_BAND_C = 2
    VS600_BAND_D = 3
    VS600_BAND_E = 4
    VS600_BAND_F = 5


# Source: io/smartport_master.h
class vs600Power_e(enum.Enum):
    VS600_POWER_PIT = 0
    VS600_POWER_25MW = 1
    VS600_POWER_200MW = 2
    VS600_POWER_600MW = 3


# Source: io/vtx.h
class vtxLowerPowerDisarm_e(enum.Enum):
    VTX_LOW_POWER_DISARM_OFF = 0
    VTX_LOW_POWER_DISARM_ALWAYS = 1
    VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM = 2


# Source: navigation/navigation.h
class wpFwTurnSmoothing_e(enum.Enum):
    WP_TURN_SMOOTHING_OFF = 0
    WP_TURN_SMOOTHING_ON = 1
    WP_TURN_SMOOTHING_CUT = 2


# Source: navigation/navigation.h
class wpMissionPlannerStatus_e(enum.Enum):
    WP_PLAN_WAIT = 0
    WP_PLAN_SAVE = 1
    WP_PLAN_OK = 2
    WP_PLAN_FULL = 3


# Source: common/calibration.h
class zeroCalibrationState_e(enum.Enum):
    ZERO_CALIBRATION_NONE = 0
    ZERO_CALIBRATION_IN_PROGRESS = 1
    ZERO_CALIBRATION_DONE = 2
    ZERO_CALIBRATION_FAIL = 3
