CONFIG = {
    'apiVersion':                       "0.0.0",
    'flightControllerIdentifier':       '',
    'flightControllerVersion':          '',
    'version':                          0,
    'buildInfo':                        '',
    'multiType':                        0,
    'msp_version':                      0, # not specified using semantic versioning
    'capability':                       0,
    'cycleTime':                        0,
    'i2cError':                         0,
    'activeSensors':                    0,
    'mode':                             0,
    'profile':                          0,
    'uid':                              [0, 0, 0],
    'accelerometerTrims':               [0, 0],
    'name':                             '',
    'displayName':                      'JOE PILOT',
    'numProfiles':                      3,
    'rateProfile':                      0,
    'boardType':                        0,
    'armingDisableCount':               0,
    'armingDisableFlags':               0,
    'armingDisabled':                   False,
    'runawayTakeoffPreventionDisabled': False,
    'boardIdentifier':                  "",
    'boardVersion':                     0,
    'commCapabilities':                 0,
    'targetName':                       "",
    'boardName':                        "",
    'manufacturerId':                   "",
    'signature':                        [],
    'cpuload':                          0,
    'mcuTypeId':                        255,
}

SENSOR_DATA = {
    'gyroscope':                  [0, 0, 0],
    'accelerometer':              [0, 0, 0],
    'magnetometer':               [0, 0, 0],
    'altitude':                   0.0,
    'altitude_vel':               0.0,
    'sonar':                      0.0,
    'kinematics':                 [0.0, 0.0, 0.0],
    'debug':                      [0, 0, 0, 0, 0, 0, 0, 0], # 8 values for special situations like MSP2_INAV_DEBUG
}

MOTOR_DATA = [0]*8

MOTOR_TELEMETRY_DATA = {
    'rpm': [0]*8,
    'invalidPercent': [0]*8,
    'temperature': [0]*8,
    'voltage': [0]*8,
    'current': [0]*8,
    'consumption': [0]*8,
}

# defaults
# roll, pitch, yaw, throttle, aux 1, ... aux n
RC = {
    'active_channels':            0,
    'channels':                   [0]*32,
}

GPS_DATA = {
    'fix':                        0,
    'numSat':                     0,
    'lat':                        0,
    'lon':                        0,
    'alt':                        0,
    'speed':                      0,
    'ground_course':              0,
    'distanceToHome':             0,
    'directionToHome':            0, # not "ditectionToHome"
    'update':                     0,
    'chn':                        [],
    'svid':                       [],
    'quality':                    [],
    'cno':                        []
}

ANALOG = {}
ANALOG['voltage'] = 0.0

VOLTAGE_METERS = []

CURRENT_METERS = []

BATTERY_STATE = {
    'cellCount':                  0,
}

SENSOR_ALIGNMENT = {
    'align_gyro':                 0,
    'align_acc':                  0,
    'align_mag':                  0,
    'gyro_detection_flags':       0,
    'gyro_to_use':                0,
    'gyro_1_align':               0,
    'gyro_2_align':               0,
}


BOARD_ALIGNMENT_CONFIG = {
    'roll':                       0,
    'pitch':                      0,
    'yaw':                        0,
}

ARMING_CONFIG = {
    'auto_disarm_delay':          0,
    'disarm_kill_switch':         0,
    'small_angle':                0,
}

FEATURE_CONFIG = {
    'featuremask':                   0,

    'features':                     {
    0: { 'group': 'rxMode', 'name': 'RX_PPM', 'enabled': False},
    2: { 'group': 'other', 'name': 'INFLIGHT_ACC_CAL', 'enabled': False},
    3: { 'group': 'rxMode', 'name': 'RX_SERIAL', 'enabled': False},
    4: { 'group': 'esc', 'name': 'MOTOR_STOP', 'enabled': False},
    5: { 'group': 'other', 'name': 'SERVO_TILT', 'enabled': False},
    6: { 'group': 'other', 'name': 'SOFTSERIAL', 'enabled': False},
    7: { 'group': 'gps', 'name': 'GPS', 'enabled': False},
    9: { 'group': 'other', 'name': 'SONAR', 'enabled': False},
    10: { 'group': 'other', 'name': 'TELEMETRY', 'enabled': False},
    12: { 'group': '3D', 'name': '3D', 'enabled': False},
    13: { 'group': 'rxMode', 'name': 'RX_PARALLEL_PWM', 'enabled': False},
    14: { 'group': 'rxMode', 'name': 'RX_MSP', 'enabled': False},
    15: { 'group': 'rssi', 'name': 'RSSI_ADC', 'enabled': False},
    16: { 'group': 'other', 'name': 'LED_STRIP', 'enabled': False},
    17: { 'group': 'other', 'name': 'DISPLAY', 'enabled': False},
    19: { 'group': 'other', 'name': 'BLACKBOX', 'enabled': False},
    20: { 'group': 'other', 'name': 'CHANNEL_FORWARDING', 'enabled': False},
    21: { 'group': 'other', 'name': 'TRANSPONDER', 'enabled': False},
    22: { 'group': 'other', 'name': 'AIRMODE', 'enabled': False},
    18: { 'group': 'other', 'name': 'OSD', 'enabled': False},
    25: { 'group': 'rxMode', 'name': 'RX_SPI', 'enabled': False},
    27: { 'group': 'other', 'name': 'ESC_SENSOR', 'enabled': False},
    28: { 'group': 'other', 'name': 'ANTI_GRAVITY', 'enabled': False},
    29: { 'group': 'other', 'name': 'DYNAMIC_FILTER', 'enabled': False}
    }
}

RX_CONFIG = {
    'serialrx_provider':            0,
    'stick_max':                    0,
    'stick_center':                 0,
    'stick_min':                    0,
    'spektrum_sat_bind':            0,
    'rx_min_usec':                  0,
    'rx_max_usec':                  0,
    'rcInterpolation':              0,
    'rcInterpolationInterval':      0,
    'rcInterpolationChannels':      0,
    'airModeActivateThreshold':     0,
    'rxSpiProtocol':                0,
    'rxSpiId':                      0,
    'rxSpiRfChannelCount':          0,
    'fpvCamAngleDegrees':           0,
    'rcSmoothingType':              0,
    'rcSmoothingInputCutoff':       0,
    'rcSmoothingDerivativeCutoff':  0, 
    'rcSmoothingInputType':         0,
    'rcSmoothingDerivativeType':    0,
}

RC_MAP = []

AUX_CONFIG_IDS = []

MODE_RANGES = []

MODE_RANGES_EXTRA = []

ADJUSTMENT_RANGES = []

RXFAIL_CONFIG = []

FAILSAFE_CONFIG = {
    'failsafe_delay':                 0,
    'failsafe_off_delay':             0,
    'failsafe_throttle':              0,
    'failsafe_switch_mode':           0,
    'failsafe_throttle_low_delay':    0,
    'failsafe_procedure':             0,
}

SERVO_DATA = []*8

VOLTAGE_METER_CONFIGS = []

CURRENT_METER_CONFIGS = []

BATTERY_CONFIG = {
    'vbatmincellvoltage':         0,
    'vbatmaxcellvoltage':         0,
    'vbatwarningcellvoltage':     0,
    'capacity':                   0,
    'voltageMeterSource':         0,
    'currentMeterSource':         0,
}

RC_TUNING = {
    'RC_RATE':                    0,
    'RC_EXPO':                    0,
    'roll_pitch_rate':            0, # pre 1.7 api only
    'roll_rate':                  0,
    'pitch_rate':                 0,
    'yaw_rate':                   0,
    'dynamic_THR_PID':            0,
    'throttle_MID':               0,
    'throttle_EXPO':              0,
    'dynamic_THR_breakpoint':     0,
    'RC_YAW_EXPO':                0,
    'rcYawRate':                  0,
    'rcPitchRate':                0,
    'RC_PITCH_EXPO':              0,
    'roll_rate_limit':            1998,
    'pitch_rate_limit':           1998,
    'yaw_rate_limit':             1998,
}

PIDs = []

PID = {
    'controller':                 0
}

FC_CONFIG = {
    'loopTime':                   0
}

MOTOR_CONFIG = {
    'minthrottle':                0,
    'maxthrottle':                0,
    'mincommand':                 0,
}

MISC = {
    # DEPRECATED = only used to store values that are written back to the fc as-is, do NOT use for any other purpose
    'failsafe_throttle':          0,
    'gps_baudrate':               0,
    'multiwiicurrentoutput':      0,
    'placeholder2':               0,
    'vbatscale':                  0,
    'vbatmincellvoltage':         0,
    'vbatmaxcellvoltage':         0,
    'vbatwarningcellvoltage':     0,
    'batterymetertype':           1, # 1=ADC, 2=ESC
}

GPS_CONFIG = {
    'provider':                   0,
    'ublox_sbas':                 0,
    'auto_config':                0,
    'auto_baud':                  0,
}

RSSI_CONFIG = {
    'channel':                    0,
}

COMPASS_CONFIG = {
    'mag_declination':            0,
}

GPS_RESCUE = {
    'angle':                          0,
    'initialAltitudeM':               0,
    'descentDistanceM':               0,
    'rescueGroundspeed':              0,
    'throttleMin':                    0,
    'throttleMax':                    0,
    'throttleHover':                  0,
    'sanityChecks':                   0,
    'minSats':                        0,
}

MOTOR_3D_CONFIG = {
    'deadband3d_low':             0,
    'deadband3d_high':            0,
    'neutral':                    0,
}


AUX_CONFIG = []

PIDNAMES = []

SERVO_CONFIG = []

RC_DEADBAND_CONFIG = {
    'deadband':                   0,
    'yaw_deadband':               0,
    'alt_hold_deadband':          0,
    'deadband3d_throttle':        0,
}

BEEPER_CONFIG = {
    'beepers':                    0,
    'dshotBeaconTone':            0,
    'dshotBeaconConditions':      0,
}

MIXER_CONFIG = {
    'mixer':                      0,
    'reverseMotorDir':            0,
}

REBOOT_TYPES = {
    'FIRMWARE': 0,
    'BOOTLOADER': 1,
    'MSC': 2,
    'MSC_UTC': 3
}

# 0 based index, must be identical to 'baudRates' in 'src/main/io/serial.c' in betaflight
BAUD_RATES = ['AUTO', '9600', '19200', '38400', '57600', '115200',
                    '230400', '250000', '400000', '460800', '500000', '921600', '1000000',
                    '1500000', '2000000', '2470000']

# needs to be identical to 'serialPortFunction_e' in 'src/main/io/serial.h' in betaflight
SERIAL_PORT_FUNCTIONS = {
    'MSP': 0,
    'GPS': 1,
    'TELEMETRY_FRSKY': 2,
    'TELEMETRY_HOTT': 3,
    'TELEMETRY_MSP': 4,
    'TELEMETRY_LTM': 4, # LTM replaced MSP
    'TELEMETRY_SMARTPORT': 5,
    'RX_SERIAL': 6,
    'BLACKBOX': 7,
    'TELEMETRY_MAVLINK': 9,
    'ESC_SENSOR': 10,
    'TBS_SMARTAUDIO': 11,
    'TELEMETRY_IBUS': 12,
    'IRC_TRAMP': 13,
    'RUNCAM_DEVICE_CONTROL': 14, # support communicate with RunCam Device
    'LIDAR_TF': 15
}

SERIAL_CONFIG = {
    'ports':                      [],

    # pre 1.6 settings
    'mspBaudRate':                0,
    'gpsBaudRate':                0,
    'gpsPassthroughBaudRate':     0,
    'cliBaudRate':                0,
}

PID_ADVANCED_CONFIG = {
    'gyro_sync_denom':            0,
    'pid_process_denom':          0,
    'use_unsyncedPwm':            0,
    'fast_pwm_protocol':          0,
    'motor_pwm_rate':             0,
    'digitalIdlePercent':         0,
    'gyroUse32kHz':               0,
}

FILTER_CONFIG = {
    'gyro_hardware_lpf':          0,
    'gyro_32khz_hardware_lpf':    0,
    'gyro_lowpass_hz':            0,
    'gyro_lowpass_dyn_min_hz':    0,
    'gyro_lowpass_dyn_max_hz':    0,
    'gyro_lowpass_type':          0,
    'gyro_lowpass2_hz':           0,
    'gyro_lowpass2_type':         0,
    'gyro_notch_hz':              0,
    'gyro_notch_cutoff':          0,
    'gyro_notch2_hz':             0,
    'gyro_notch2_cutoff':         0,
    'dterm_lowpass_hz':           0,
    'dterm_lowpass_dyn_min_hz':   0,
    'dterm_lowpass_dyn_max_hz':   0,
    'dterm_lowpass_type':         0,
    'dterm_lowpass2_hz':          0,
    'dterm_lowpass2_type':        0,
    'dterm_notch_hz':             0,
    'dterm_notch_cutoff':         0,
    'yaw_lowpass_hz':             0,
}

ADVANCED_TUNING = {
    'rollPitchItermIgnoreRate':   0,
    'yawItermIgnoreRate':         0,
    'yaw_p_limit':                0,
    'deltaMethod':                0,
    'vbatPidCompensation':        0,
    'dtermSetpointTransition':    0,
    'dtermSetpointWeight':        0,
    'toleranceBand':              0,
    'toleranceBandReduction':     0,
    'itermThrottleGain':          0,
    'pidMaxVelocity':             0,
    'pidMaxVelocityYaw':          0,
    'levelAngleLimit':            0,
    'levelSensitivity':           0,
    'itermThrottleThreshold':     0,
    'itermAcceleratorGain':       0,
    'itermRotation':              0,
    'smartFeedforward':           0,
    'itermRelax':                 0,
    'itermRelaxType':             0,
    'absoluteControlGain':        0,
    'throttleBoost':              0,
    'acroTrainerAngleLimit':      0,
    'feedforwardRoll':            0,
    'feedforwardPitch':           0,
    'feedforwardYaw':             0,
    'feedforwardTransition':      0,
    'antiGravityMode':            0,
    'dMinRoll':                   0,
    'dMinPitch':                  0,
    'dMinYaw':                    0,
    'dMinGain':                   0,
    'dMinAdvance':                0,
    'useIntegratedYaw':           0,
    'integratedYawRelax':         0,
}

SENSOR_CONFIG = {
    'acc_hardware':               0,
    'baro_hardware':              0,
    'mag_hardware':               0,
}

DATAFLASH = {
    'ready':                      False,
    'supported':                  False,
    'sectors':                    0,
    'totalSize':                  0,
    'usedSize':                   0
}

SDCARD = {
    'supported':                  False,
    'state':                      0,
    'filesystemLastError':        0,
    'freeSizeKB':                 0,
    'totalSizeKB':                0,
}

BLACKBOX = {
    'supported':                  False,
    'blackboxDevice':             0,
    'blackboxRateNum':            1,
    'blackboxRateDenom':          1,
    'blackboxPDenom':             0,
}

TRANSPONDER = {
    'supported':                  False,
    'data':                       [],
    'provider':                   0,
    'providers':                  [],
}

NAV_STATUS = {
    'mode': 0,
    'state': 0,
    'active_wp_action': 0,
    'active_wp_number': 0,
    'error': 0,
    'heading_hold_target': 0
}

INAV_MIXER = {
    'motorDirectionInverted':   False,
    'motorstopOnLow':           False,
    'platformType':             0,
    'hasFlaps':                 False,
    'appliedMixerPreset':       0,
    'MAX_SUPPORTED_MOTORS':     0,
    'MAX_SUPPORTED_SERVOS':     0
}

NAV_CONFIG = {

}

WP = {
    'wp_no': 0,
    'wp_action': 0,
    'lat': 0,
    'lon': 0,
    'alt': 0,
    'p1': 0,
    'p2': 0,
    'p3': 0,
    'flag': 0
}

WP_INFO = {
    "reserved":0,
    "NAV_MAX_WAYPOINTS": 0,
    "isWaypointListValid": 0,
    "WaypointCount": 0
}