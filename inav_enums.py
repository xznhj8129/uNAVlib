# from enums/base_enums.py
armingDisableFlagNames_BF = {
    "NOGYRO": 0,
    "FAILSAFE": 1,
    "RXLOSS": 2,
    "BADRX": 3,
    "BOXFAILSAFE": 4,
    "RUNAWAY": 5,
    "CRASH": 6,
    "THROTTLE": 7,
    "ANGLE": 8,
    "BOOTGRACE": 9,
    "NOPREARM": 10,
    "LOAD": 11,
    "CALIB": 12,
    "CLI": 13,
    "CMS": 14,
    "BST": 15,
    "MSP": 16,
    "PARALYZE": 17,
    "GPS": 18,
    "RESCUE SW": 19,
    "RPMFILTER": 20,
    "ARMSWITCH": 21
}
armingDisableFlagNames_INAV = {
    "OK_TO_ARM": 0,
    "PREVENT_ARMING": 1,
    "ARMED": 2,
    "WAS_EVER_ARMED": 3,
    "SIMULATOR_MODE": 4,
    "BLOCKED_FAILSAFE_SYSTEM": 7,
    "BLOCKED_UAV_NOT_LEVEL": 8,
    "BLOCKED_SENSORS_CALIBRATING": 9,
    "BLOCKED_SYSTEM_OVERLOADED": 10,
    "BLOCKED_NAVIGATION_SAFETY": 11,
    "BLOCKED_COMPASS_NOT_CALIBRATED": 12,
    "BLOCKED_ACCELEROMETER_NOT_CALIBRATED": 13,
    "BLOCKED_ARMING_DISABLED_ARM_SWITCH": 14,
    "BLOCKED_HARDWARE_FAILURE": 15,
    "BLOCKED_ARMING_DISABLED_BOXFAILSAFE": 16,
    "BLOCKED_ARMING_DISABLED_BOXKILLSWITCH": 17,
    "BLOCKED_ARMING_DISABLED_RC_LINK": 18,
    "BLOCKED_ARMING_DISABLED_THROTTLE": 19,
    "BLOCKED_ARMING_DISABLED_CLI": 20,
    "BLOCKED_ARMING_DISABLED_CMS_MENU": 21,
    "BLOCKED_ARMING_DISABLED_OSD_MENU": 22,
    "BLOCKED_ARMING_DISABLED_ROLLPITCH_NOT_CENTERED": 23,
    "BLOCKED_ARMING_DISABLED_SERVO_AUTOTRIM": 24,
    "BLOCKED_ARMING_DISABLED_OOM": 25,
    "BLOCKED_INVALID_SETTING": 26,
    "BLOCKED_ARMING_DISABLED_PWM_OUTPUT_ERROR": 27,
    "BLOCKED_ARMING_DISABLED_NO_PREARM": 28,
    "BLOCKED_ARMING_DISABLED_DSHOT_BEEPER": 29,
    "BLOCKED_ARMING_DISABLED_LANDING_DETECTED": 30
}
modesID_INAV = {
    "ARM": 0,
    "ANGLE": 1,
    "HORIZON": 2,
    "NAV ALTHOLD": 3,
    "HEADING HOLD": 5,
    "HEADFREE": 6,
    "HEADADJ": 7,
    "CAMSTAB": 8,
    "NAV RTH": 10,
    "NAV POSHOLD": 11,
    "MANUAL": 12,
    "BEEPER": 13,
    "LEDS OFF": 15,
    "LIGHTS": 16,
    "OSD OFF": 19,
    "TELEMETRY": 20,
    "AUTO TUNE": 21,
    "BLACKBOX": 26,
    "FAILSAFE": 27,
    "NAV WP": 28,
    "AIR MODE": 29,
    "HOME RESET": 30,
    "GCS NAV": 31,
    "FPV ANGLE MIX": 32,
    "SURFACE": 33,
    "FLAPERON": 34,
    "TURN ASSIST": 35,
    "NAV LAUNCH": 36,
    "SERVO AUTOTRIM": 37,
    "KILLSWITCH": 38,
    "CAMERA CONTROL 1": 39,
    "CAMERA CONTROL 2": 40,
    "CAMERA CONTROL 3": 41,
    "OSD ALT 1": 42,
    "OSD ALT 2": 43,
    "OSD ALT 3": 44,
    "NAV COURSE HOLD": 45,
    "MC BRAKING": 46,
    "USER1": 47,
    "USER2": 48,
    "LOITER CHANGE": 49,
    "MSP RC OVERRIDE": 50,
    "PREARM": 51,
    "TURTLE": 52,
    "NAV CRUISE": 53,
    "AUTO LEVEL": 54,
    "WP PLANNER": 55,
    "MISSION CHANGE": 59,
    "BEEPER MUTE": 60,
    "MULTI FUNCTION": 61,
    "MIXER PROFILE 2": 62,
    "MIXER TRANSITION": 63,
    "ANG HOLD": 64
}


# extracted from source
#../../inav/src/main/navigation/navigation_private.h
navSetWaypointFlags = {
    "NAV_POS_UPDATE_NONE": 0,
    "NAV_POS_UPDATE_Z": "1 << 1",
    "NAV_POS_UPDATE_XY": "1 << 0",
    "NAV_POS_UPDATE_HEADING": "1 << 2",
    "NAV_POS_UPDATE_BEARING": "1 << 3",
    "NAV_POS_UPDATE_BEARING_TAIL_FIRST": "1 << 4"
}
climbRateToAltitudeControllerMode = {
    "ROC_TO_ALT_RESET": 0,
    "ROC_TO_ALT_CONSTANT": 1,
    "ROC_TO_ALT_TARGET": 2
}
navigationEstimateStatus = {
    "EST_NONE": 0,
    "EST_USABLE": 1,
    "EST_TRUSTED": 2
}
navigationHomeFlags = {
    "NAV_HOME_INVALID": 0,
    "NAV_HOME_VALID_XY": "1 << 0",
    "NAV_HOME_VALID_Z": "1 << 1",
    "NAV_HOME_VALID_HEADING": "1 << 2",
    "NAV_HOME_VALID_ALL": "NAV_HOME_VALID_XY | NAV_HOME_VALID_Z | NAV_HOME_VALID_HEADING"
}
navigationFSMEvent = {
    "NAV_FSM_EVENT_NONE": 0,
    "NAV_FSM_EVENT_TIMEOUT": 1,
    "NAV_FSM_EVENT_SUCCESS": 2,
    "NAV_FSM_EVENT_ERROR": 3,
    "NAV_FSM_EVENT_SWITCH_TO_IDLE": 4,
    "NAV_FSM_EVENT_SWITCH_TO_ALTHOLD": 5,
    "NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D": 6,
    "NAV_FSM_EVENT_SWITCH_TO_RTH": 7,
    "NAV_FSM_EVENT_SWITCH_TO_WAYPOINT": 8,
    "NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING": 9,
    "NAV_FSM_EVENT_SWITCH_TO_LAUNCH": 10,
    "NAV_FSM_EVENT_SWITCH_TO_COURSE_HOLD": 11,
    "NAV_FSM_EVENT_SWITCH_TO_CRUISE": 12,
    "NAV_FSM_EVENT_SWITCH_TO_COURSE_ADJ": 13,
    "NAV_FSM_EVENT_SWITCH_TO_MIXERAT": 14,
    "NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING": 15,
    "NAV_FSM_EVENT_STATE_SPECIFIC_1": 16,
    "NAV_FSM_EVENT_STATE_SPECIFIC_2": 17,
    "NAV_FSM_EVENT_STATE_SPECIFIC_3": 18,
    "NAV_FSM_EVENT_STATE_SPECIFIC_4": 19,
    "NAV_FSM_EVENT_STATE_SPECIFIC_5": 20,
    "NAV_FSM_EVENT_STATE_SPECIFIC_6": 21,
    "NAV_FSM_EVENT_SWITCH_TO_RTH_HEAD_HOME": 18,
    "NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING": 16,
    "NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND": 16,
    "NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED": 17,
    "NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_HOLD_TIME": 18,
    "NAV_FSM_EVENT_SWITCH_TO_RTH_HOVER_ABOVE_HOME": 19,
    "NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_TRACKBACK": 20,
    "NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_INITIALIZE": 21,
    "NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_ABORT": 21,
    "NAV_FSM_EVENT_COUNT": 31
}
navigationPersistentId = {
    "NAV_PERSISTENT_ID_UNDEFINED": 0,
    "NAV_PERSISTENT_ID_IDLE": 1,
    "NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE": 2,
    "NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS": 3,
    "NAV_PERSISTENT_ID_UNUSED_1": 4,
    "NAV_PERSISTENT_ID_UNUSED_2": 5,
    "NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE": 6,
    "NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS": 7,
    "NAV_PERSISTENT_ID_RTH_INITIALIZE": 8,
    "NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT": 9,
    "NAV_PERSISTENT_ID_RTH_HEAD_HOME": 10,
    "NAV_PERSISTENT_ID_RTH_HOVER_PRIOR_TO_LANDING": 11,
    "NAV_PERSISTENT_ID_RTH_LANDING": 12,
    "NAV_PERSISTENT_ID_RTH_FINISHING": 13,
    "NAV_PERSISTENT_ID_RTH_FINISHED": 14,
    "NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE": 15,
    "NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION": 16,
    "NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS": 17,
    "NAV_PERSISTENT_ID_WAYPOINT_REACHED": 18,
    "NAV_PERSISTENT_ID_WAYPOINT_NEXT": 19,
    "NAV_PERSISTENT_ID_WAYPOINT_FINISHED": 20,
    "NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND": 21,
    "NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE": 22,
    "NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS": 23,
    "NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED": 24,
    "NAV_PERSISTENT_ID_LAUNCH_INITIALIZE": 25,
    "NAV_PERSISTENT_ID_LAUNCH_WAIT": 26,
    "NAV_PERSISTENT_ID_UNUSED_3": 27,
    "NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS": 28,
    "NAV_PERSISTENT_ID_COURSE_HOLD_INITIALIZE": 29,
    "NAV_PERSISTENT_ID_COURSE_HOLD_IN_PROGRESS": 30,
    "NAV_PERSISTENT_ID_COURSE_HOLD_ADJUSTING": 31,
    "NAV_PERSISTENT_ID_CRUISE_INITIALIZE": 32,
    "NAV_PERSISTENT_ID_CRUISE_IN_PROGRESS": 33,
    "NAV_PERSISTENT_ID_CRUISE_ADJUSTING": 34,
    "NAV_PERSISTENT_ID_WAYPOINT_HOLD_TIME": 35,
    "NAV_PERSISTENT_ID_RTH_HOVER_ABOVE_HOME": 36,
    "NAV_PERSISTENT_ID_UNUSED_4": 37,
    "NAV_PERSISTENT_ID_RTH_TRACKBACK": 38,
    "NAV_PERSISTENT_ID_MIXERAT_INITIALIZE": 39,
    "NAV_PERSISTENT_ID_MIXERAT_IN_PROGRESS": 40,
    "NAV_PERSISTENT_ID_MIXERAT_ABORT": 41,
    "NAV_PERSISTENT_ID_FW_LANDING_CLIMB_TO_LOITER": 42,
    "NAV_PERSISTENT_ID_FW_LANDING_LOITER": 43,
    "NAV_PERSISTENT_ID_FW_LANDING_APPROACH": 44,
    "NAV_PERSISTENT_ID_FW_LANDING_GLIDE": 45,
    "NAV_PERSISTENT_ID_FW_LANDING_FLARE": 46,
    "NAV_PERSISTENT_ID_FW_LANDING_ABORT": 47
}
navigationFSMState = {
    "NAV_STATE_UNDEFINED": 0,
    "NAV_STATE_IDLE": 1,
    "NAV_STATE_ALTHOLD_INITIALIZE": 2,
    "NAV_STATE_ALTHOLD_IN_PROGRESS": 3,
    "NAV_STATE_POSHOLD_3D_INITIALIZE": 4,
    "NAV_STATE_POSHOLD_3D_IN_PROGRESS": 5,
    "NAV_STATE_RTH_INITIALIZE": 6,
    "NAV_STATE_RTH_CLIMB_TO_SAFE_ALT": 7,
    "NAV_STATE_RTH_TRACKBACK": 8,
    "NAV_STATE_RTH_HEAD_HOME": 9,
    "NAV_STATE_RTH_HOVER_PRIOR_TO_LANDING": 10,
    "NAV_STATE_RTH_HOVER_ABOVE_HOME": 11,
    "NAV_STATE_RTH_LANDING": 12,
    "NAV_STATE_RTH_FINISHING": 13,
    "NAV_STATE_RTH_FINISHED": 14,
    "NAV_STATE_WAYPOINT_INITIALIZE": 15,
    "NAV_STATE_WAYPOINT_PRE_ACTION": 16,
    "NAV_STATE_WAYPOINT_IN_PROGRESS": 17,
    "NAV_STATE_WAYPOINT_REACHED": 18,
    "NAV_STATE_WAYPOINT_HOLD_TIME": 19,
    "NAV_STATE_WAYPOINT_NEXT": 20,
    "NAV_STATE_WAYPOINT_FINISHED": 21,
    "NAV_STATE_WAYPOINT_RTH_LAND": 22,
    "NAV_STATE_EMERGENCY_LANDING_INITIALIZE": 23,
    "NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS": 24,
    "NAV_STATE_EMERGENCY_LANDING_FINISHED": 25,
    "NAV_STATE_LAUNCH_INITIALIZE": 26,
    "NAV_STATE_LAUNCH_WAIT": 27,
    "NAV_STATE_LAUNCH_IN_PROGRESS": 28,
    "NAV_STATE_COURSE_HOLD_INITIALIZE": 29,
    "NAV_STATE_COURSE_HOLD_IN_PROGRESS": 30,
    "NAV_STATE_COURSE_HOLD_ADJUSTING": 31,
    "NAV_STATE_CRUISE_INITIALIZE": 32,
    "NAV_STATE_CRUISE_IN_PROGRESS": 33,
    "NAV_STATE_CRUISE_ADJUSTING": 34,
    "NAV_STATE_FW_LANDING_CLIMB_TO_LOITER": 35,
    "NAV_STATE_FW_LANDING_LOITER": 36,
    "NAV_STATE_FW_LANDING_APPROACH": 37,
    "NAV_STATE_FW_LANDING_GLIDE": 38,
    "NAV_STATE_FW_LANDING_FLARE": 39,
    "NAV_STATE_FW_LANDING_ABORT": 40,
    "NAV_STATE_MIXERAT_INITIALIZE": 41,
    "NAV_STATE_MIXERAT_IN_PROGRESS": 42,
    "NAV_STATE_MIXERAT_ABORT": 43,
    "NAV_STATE_COUNT": 44
}
navigationFSMStateFlags = {
    "NAV_CTL_ALT": "(1 << 0)",
    "NAV_CTL_POS": "(1 << 1)",
    "NAV_CTL_YAW": "(1 << 2)",
    "NAV_CTL_EMERG": "(1 << 3)",
    "NAV_CTL_LAUNCH": "(1 << 4)",
    "NAV_REQUIRE_ANGLE": "(1 << 5)",
    "NAV_REQUIRE_ANGLE_FW": "(1 << 6)",
    "NAV_REQUIRE_MAGHOLD": "(1 << 7)",
    "NAV_REQUIRE_THRTILT": "(1 << 8)",
    "NAV_AUTO_RTH": "(1 << 9)",
    "NAV_AUTO_WP": "(1 << 10)",
    "NAV_RC_ALT": "(1 << 11)",
    "NAV_RC_POS": "(1 << 12)",
    "NAV_RC_YAW": "(1 << 13)",
    "NAV_CTL_LAND": "(1 << 14)",
    "NAV_AUTO_WP_DONE": "(1 << 15)",
    "NAV_MIXERAT": "(1 << 16)"
}
fwAutolandWaypoint = {
    "FW_AUTOLAND_WP_TURN": 0,
    "FW_AUTOLAND_WP_FINAL_APPROACH": 1,
    "FW_AUTOLAND_WP_LAND": 2,
    "FW_AUTOLAND_WP_COUNT": 3
}
rthTargetMode = {
    "RTH_HOME_ENROUTE_INITIAL": 0,
    "RTH_HOME_ENROUTE_PROPORTIONAL": 1,
    "RTH_HOME_ENROUTE_FINAL": 2,
    "RTH_HOME_FINAL_HOVER": 3,
    "RTH_HOME_FINAL_LAND": 4
}

#../../inav/src/main/navigation/navigation_pos_estimator_private.h
navAGLEstimateQuality = {
    "SURFACE_QUAL_LOW": 0,
    "SURFACE_QUAL_MID": 1,
    "SURFACE_QUAL_HIGH": 2
}
navPositionEstimationFlags = {
    "EST_GPS_XY_VALID": "(1 << 0)",
    "EST_GPS_Z_VALID": "(1 << 1)",
    "EST_BARO_VALID": "(1 << 2)",
    "EST_SURFACE_VALID": "(1 << 3)",
    "EST_FLOW_VALID": "(1 << 4)",
    "EST_XY_VALID": "(1 << 5)",
    "EST_Z_VALID": "(1 << 6)"
}

#../../inav/src/main/navigation/navigation.h
safehomeUsageMode = {
    "SAFEHOME_USAGE_OFF": 0,
    "SAFEHOME_USAGE_RTH": 1,
    "SAFEHOME_USAGE_RTH_FS": 2
}
fwAutolandState = {
    "FW_AUTOLAND_STATE_IDLE": 0,
    "FW_AUTOLAND_STATE_LOITER": 1,
    "FW_AUTOLAND_STATE_DOWNWIND": 2,
    "FW_AUTOLAND_STATE_BASE_LEG": 3,
    "FW_AUTOLAND_STATE_FINAL_APPROACH": 4,
    "FW_AUTOLAND_STATE_GLIDE": 5,
    "FW_AUTOLAND_STATE_FLARE": 6
}
nav_resetype = {
    "NAV_RESET_NEVER": 0,
    "NAV_RESET_ON_FIRST_ARM": 1,
    "NAV_RESET_ON_EACH_ARM": 2
}
navRTHAllowLanding = {
    "NAV_RTH_ALLOW_LANDING_NEVER": 0,
    "NAV_RTH_ALLOW_LANDING_ALWAYS": 1,
    "NAV_RTH_ALLOW_LANDING_FS_ONLY": 2
}
navExtraArmingSafety = {
    "NAV_EXTRA_ARMING_SAFETY_ON": 0,
    "NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS": 1
}
navArmingBlocker = {
    "NAV_ARMING_BLOCKER_NONE": 0,
    "NAV_ARMING_BLOCKER_MISSING_GPS_FIX": 1,
    "NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE": 2,
    "NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR": 3,
    "NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR": 4
}
navOverridesMotorStop = {
    "NOMS_OFF_ALWAYS": 0,
    "NOMS_OFF": 1,
    "NOMS_AUTO_ONLY": 2,
    "NOMS_ALL_NAV": 3
}
navRTHClimbFirst = {
    "RTH_CLIMB_OFF": 0,
    "RTH_CLIMB_ON": 1,
    "RTH_CLIMB_ON_FW_SPIRAL": 2
}
navFwLaunchStatus = {
    "FW_LAUNCH_DETECTED": 4,
    "FW_LAUNCH_ABORTED": 9,
    "FW_LAUNCH_FLYING": 10
}
wpMissionPlannerStatus = {
    "WP_PLAN_WAIT": 0,
    "WP_PLAN_SAVE": 1,
    "WP_PLAN_OK": 2,
    "WP_PLAN_FULL": 3
}
navMissionRestart = {
    "WP_MISSION_START": 0,
    "WP_MISSION_RESUME": 1,
    "WP_MISSION_SWITCH": 2
}
rthTrackbackMode = {
    "RTH_TRACKBACK_OFF": 0,
    "RTH_TRACKBACK_ON": 1,
    "RTH_TRACKBACK_FS": 2
}
wpFwTurnSmoothing = {
    "WP_TURN_SMOOTHING_OFF": 0,
    "WP_TURN_SMOOTHING_ON": 1,
    "WP_TURN_SMOOTHING_CUT": 2
}
navMcAltHoldThrottle = {
    "MC_ALT_HOLD_STICK": 0,
    "MC_ALT_HOLD_MID": 1,
    "MC_ALT_HOLD_HOVER": 2
}
navWaypointActions = {
    "NAV_WP_ACTION_WAYPOINT": 1,
    "NAV_WP_ACTION_HOLD_TIME": 3,
    "NAV_WP_ACTION_RTH": 4,
    "NAV_WP_ACTION_SET_POI": 5,
    "NAV_WP_ACTION_JUMP": 6,
    "NAV_WP_ACTION_SET_HEAD": 7,
    "NAV_WP_ACTION_LAND": 8
}
navWaypointHeadings = {
    "NAV_WP_HEAD_MODE_NONE": 0,
    "NAV_WP_HEAD_MODE_POI": 1,
    "NAV_WP_HEAD_MODE_FIXED": 2
}
navWaypointFlags = {
    "NAV_WP_FLAG_HOME": 72,
    "NAV_WP_FLAG_LAST": 165
}
navWaypointP3Flags = {
    "NAV_WP_ALTMODE": "(1<<0)",
    "NAV_WP_USER1": "(1<<1)",
    "NAV_WP_USER2": "(1<<2)",
    "NAV_WP_USER3": "(1<<3)",
    "NAV_WP_USER4": "(1<<4)"
}
navSystemStatus_Mode = {
    "MW_GPS_MODE_NONE": 0,
    "MW_GPS_MODE_HOLD": 1,
    "MW_GPS_MODE_RTH": 2,
    "MW_GPS_MODE_NAV": 3,
    "MW_GPS_MODE_EMERG": 15
}
navSystemStatus_State = {
    "MW_NAV_STATE_NONE": 0,
    "MW_NAV_STATE_RTH_START": 1,
    "MW_NAV_STATE_RTH_ENROUTE": 2,
    "MW_NAV_STATE_HOLD_INFINIT": 3,
    "MW_NAV_STATE_HOLD_TIMED": 4,
    "MW_NAV_STATE_WP_ENROUTE": 5,
    "MW_NAV_STATE_PROCESS_NEXT": 6,
    "MW_NAV_STATE_DO_JUMP": 7,
    "MW_NAV_STATE_LAND_START": 8,
    "MW_NAV_STATE_LAND_IN_PROGRESS": 9,
    "MW_NAV_STATE_LANDED": 10,
    "MW_NAV_STATE_LAND_SETTLE": 11,
    "MW_NAV_STATE_LAND_START_DESCENT": 12,
    "MW_NAV_STATE_HOVER_ABOVE_HOME": 13,
    "MW_NAV_STATE_EMERGENCY_LANDING": 14,
    "MW_NAV_STATE_RTH_CLIMB": 15
}
navSystemStatus_Error = {
    "MW_NAV_ERROR_NONE": 0,
    "MW_NAV_ERROR_TOOFAR": 1,
    "MW_NAV_ERROR_SPOILED_GPS": 2,
    "MW_NAV_ERROR_WP_CRC": 3,
    "MW_NAV_ERROR_FINISH": 4,
    "MW_NAV_ERROR_TIMEWAIT": 5,
    "MW_NAV_ERROR_INVALID_JUMP": 6,
    "MW_NAV_ERROR_INVALID_DATA": 7,
    "MW_NAV_ERROR_WAIT_FOR_RTH_ALT": 8,
    "MW_NAV_ERROR_GPS_FIX_LOST": 9,
    "MW_NAV_ERROR_DISARMED": 10,
    "MW_NAV_ERROR_LANDING": 11
}
navSystemStatus_Flags = {
    "MW_NAV_FLAG_ADJUSTING_POSITION": "1 << 0",
    "MW_NAV_FLAG_ADJUSTING_ALTITUDE": "1 << 1"
}
geoAltitudeConversionMode = {
    "GEO_ALT_ABSOLUTE": 0,
    "GEO_ALT_RELATIVE": 1
}
geoOriginResetMode = {
    "GEO_ORIGIN_SET": 0,
    "GEO_ORIGIN_RESET_ALTITUDE": 1
}
geoAltitudeDatumFlag = {
    "NAV_WP_TAKEOFF_DATUM": 0,
    "NAV_WP_MSL_DATUM": 1
}

#../../inav/src/main/sensors/battery_config_structs.h
currentSensor = {
    "CURRENT_SENSOR_NONE": 0,
    "CURRENT_SENSOR_ADC": 1,
    "CURRENT_SENSOR_VIRTUAL": 2,
    "CURRENT_SENSOR_FAKE": 3,
    "CURRENT_SENSOR_ESC": 4,
    "CURRENT_SENSOR_MAX": 3
}
voltageSensor = {
    "VOLTAGE_SENSOR_NONE": 0,
    "VOLTAGE_SENSOR_ADC": 1,
    "VOLTAGE_SENSOR_ESC": 2,
    "VOLTAGE_SENSOR_FAKE": 3
}
batCapacityUnit = {
    "BAT_CAPACITY_UNIT_MAH": 0,
    "BAT_CAPACITY_UNIT_MWH": 1
}
batVoltageSource = {
    "BAT_VOLTAGE_RAW": 0,
    "BAT_VOLTAGE_SAG_COMP": 1
}

#../../inav/src/main/sensors/rangefinder.h
rangefinderType = {
    "RANGEFINDER_NONE": 0,
    "RANGEFINDER_SRF10": 1,
    "RANGEFINDER_VL53L0X": 2,
    "RANGEFINDER_MSP": 3,
    "RANGEFINDER_BENEWAKE": 4,
    "RANGEFINDER_VL53L1X": 5,
    "RANGEFINDER_US42": 6,
    "RANGEFINDER_TOF10102I2C": 7,
    "RANGEFINDER_FAKE": 8
}

#../../inav/src/main/sensors/gyro.h
gyroSensor = {
    "GYRO_NONE": 0,
    "GYRO_AUTODETECT": 1,
    "GYRO_MPU6000": 2,
    "GYRO_MPU6500": 3,
    "GYRO_MPU9250": 4,
    "GYRO_BMI160": 5,
    "GYRO_ICM20689": 6,
    "GYRO_BMI088": 7,
    "GYRO_ICM42605": 8,
    "GYRO_BMI270": 9,
    "GYRO_LSM6DXX": 10,
    "GYRO_FAKE": 11
}
dynamicGyroNotchMode = {
    "DYNAMIC_NOTCH_MODE_2D": 0,
    "DYNAMIC_NOTCH_MODE_R": 1,
    "DYNAMIC_NOTCH_MODE_P": 2,
    "DYNAMIC_NOTCH_MODE_Y": 3,
    "DYNAMIC_NOTCH_MODE_RP": 4,
    "DYNAMIC_NOTCH_MODE_RY": 5,
    "DYNAMIC_NOTCH_MODE_PY": 6,
    "DYNAMIC_NOTCH_MODE_3D": 7
}

#../../inav/src/main/sensors/opflow.h
opticalFlowSensor = {
    "OPFLOW_NONE": 0,
    "OPFLOW_CXOF": 1,
    "OPFLOW_MSP": 2,
    "OPFLOW_FAKE": 3
}
opflowQuality = {
    "OPFLOW_QUALITY_INVALID": 0,
    "OPFLOW_QUALITY_VALID": 1
}

#../../inav/src/main/sensors/battery.h
batteryState = {
    "BATTERY_OK": 0,
    "BATTERY_WARNING": 1,
    "BATTERY_CRITICAL": 2,
    "BATTERY_NOT_PRESENT": 3
}

#../../inav/src/main/sensors/temperature.h
tempSensorType = {
    "TEMP_SENSOR_NONE": 0,
    "TEMP_SENSOR_LM75": 1,
    "TEMP_SENSOR_DS18B20": 2
}

#../../inav/src/main/sensors/pitotmeter.h
pitotSensor = {
    "PITOT_NONE": 0,
    "PITOT_AUTODETECT": 1,
    "PITOT_MS4525": 2,
    "PITOT_ADC": 3,
    "PITOT_VIRTUAL": 4,
    "PITOT_FAKE": 5,
    "PITOT_MSP": 6,
    "PITOT_DLVR": 7
}

#../../inav/src/main/sensors/sensors.h
sensorIndex = {
    "SENSOR_INDEX_GYRO": 0,
    "SENSOR_INDEX_ACC": 1,
    "SENSOR_INDEX_BARO": 2,
    "SENSOR_INDEX_MAG": 3,
    "SENSOR_INDEX_RANGEFINDER": 4,
    "SENSOR_INDEX_PITOT": 5,
    "SENSOR_INDEX_OPFLOW": 6,
    "SENSOR_INDEX_COUNT": 7
}
sensors = {
    "SENSOR_GYRO": "1 << 0",
    "SENSOR_ACC": "1 << 1",
    "SENSOR_BARO": "1 << 2",
    "SENSOR_MAG": "1 << 3",
    "SENSOR_RANGEFINDER": "1 << 4",
    "SENSOR_PITOT": "1 << 5",
    "SENSOR_OPFLOW": "1 << 6",
    "SENSOR_GPS": "1 << 7",
    "SENSOR_GPSMAG": "1 << 8",
    "SENSOR_TEMP": "1 << 9"
}

#../../inav/src/main/sensors/acceleration.h
accelerationSensor = {
    "ACC_NONE": 0,
    "ACC_AUTODETECT": 1,
    "ACC_MPU6000": 2,
    "ACC_MPU6500": 3,
    "ACC_MPU9250": 4,
    "ACC_BMI160": 5,
    "ACC_ICM20689": 6,
    "ACC_BMI088": 7,
    "ACC_ICM42605": 8,
    "ACC_BMI270": 9,
    "ACC_LSM6DXX": 10,
    "ACC_FAKE": 11,
    "ACC_MAX": 11
}

#../../inav/src/main/sensors/barometer.h
baroSensor = {
    "BARO_NONE": 0,
    "BARO_AUTODETECT": 1,
    "BARO_BMP085": 2,
    "BARO_MS5611": 3,
    "BARO_BMP280": 4,
    "BARO_MS5607": 5,
    "BARO_LPS25H": 6,
    "BARO_SPL06": 7,
    "BARO_BMP388": 8,
    "BARO_DPS310": 9,
    "BARO_B2SMPB": 10,
    "BARO_MSP": 11,
    "BARO_FAKE": 12,
    "BARO_MAX": 12
}

#../../inav/src/main/sensors/diagnostics.h
hardwareSensorStatus = {
    "HW_SENSOR_NONE": 0,
    "HW_SENSOR_OK": 1,
    "HW_SENSOR_UNAVAILABLE": 2,
    "HW_SENSOR_UNHEALTHY": 3
}

#../../inav/src/main/sensors/compass.h
magSensor = {
    "MAG_NONE": 0,
    "MAG_AUTODETECT": 1,
    "MAG_HMC5883": 2,
    "MAG_AK8975": 3,
    "MAG_MAG3110": 4,
    "MAG_AK8963": 5,
    "MAG_IST8310": 6,
    "MAG_QMC5883": 7,
    "MAG_MPU9250": 8,
    "MAG_IST8308": 9,
    "MAG_LIS3MDL": 10,
    "MAG_MSP": 11,
    "MAG_RM3100": 12,
    "MAG_VCM5883": 13,
    "MAG_MLX90393": 14,
    "MAG_FAKE": 15,
    "MAG_MAX": 15
}
