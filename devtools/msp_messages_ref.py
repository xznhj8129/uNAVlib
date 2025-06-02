{
    "MSP_API_VERSION": {
        "code": 1,
        "direction": 1,
        "struct": "<BBB",
        "byte_count": 3,
        "payload": [
            [
                "mspProtocolVersion",
                "uint8_t"
            ],
            [
                "apiVersionMajor",
                "uint8_t"
            ],
            [
                "apiVersionMinor",
                "uint8_t"
            ]
        ]
    },
    "MSP_FC_VARIANT": {
        "code": 2,
        "direction": 1,
        "struct": "<4s",
        "byte_count": 4,
        "payload": [
            [
                "fcVariantIdentifier",
                "char[4]"
            ]
        ]
    },
    "MSP_FC_VERSION": {
        "code": 3,
        "direction": 1,
        "struct": "<BBB",
        "byte_count": 3,
        "payload": [
            [
                "fcVersionMajor",
                "uint8_t"
            ],
            [
                "fcVersionMinor",
                "uint8_t"
            ],
            [
                "fcVersionPatch",
                "uint8_t"
            ]
        ]
    },
    "MSP_BOARD_INFO": {
        "code": 4,
        "direction": 1,
        "struct": "<4sHBBBs",
        "byte_count": 10,
        "payload": [
            [
                "boardIdentifier",
                "char[4]"
            ],
            [
                "hardwareRevision",
                "uint16_t"
            ],
            [
                "osdSupport",
                "uint8_t"
            ],
            [
                "commCapabilities",
                "uint8_t"
            ],
            [
                "targetNameLength",
                "uint8_t"
            ],
            [
                "targetName",
                "char[]"
            ]
        ]
    },
    "MSP_BUILD_INFO": {
        "code": 5,
        "direction": 1,
        "struct": "<11s8s7s",
        "byte_count": 26,
        "payload": [
            [
                "buildDate",
                "char[11]"
            ],
            [
                "buildTime",
                "char[8]"
            ],
            [
                "gitRevision",
                "char[7]"
            ]
        ]
    },
    "MSP_INAV_PID": {
        "code": 6,
        "direction": 1,
        "struct": "<BHHBBHBBBBBB",
        "byte_count": 15,
        "payload": [
            [
                "legacyAsyncProcessing",
                "uint8_t"
            ],
            [
                "legacyAsyncValue1",
                "uint16_t"
            ],
            [
                "legacyAsyncValue2",
                "uint16_t"
            ],
            [
                "headingHoldRateLimit",
                "uint8_t"
            ],
            [
                "headingHoldLpfFreq",
                "uint8_t"
            ],
            [
                "legacyYawJumpLimit",
                "uint16_t"
            ],
            [
                "legacyGyroLpf",
                "uint8_t"
            ],
            [
                "accLpfHz",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint8_t"
            ],
            [
                "reserved3",
                "uint8_t"
            ],
            [
                "reserved4",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_INAV_PID": {
        "code": 7,
        "direction": 0,
        "struct": "<BHHBBHBBBBBB",
        "byte_count": 15,
        "payload": [
            [
                "legacyAsyncProcessing",
                "uint8_t"
            ],
            [
                "legacyAsyncValue1",
                "uint16_t"
            ],
            [
                "legacyAsyncValue2",
                "uint16_t"
            ],
            [
                "headingHoldRateLimit",
                "uint8_t"
            ],
            [
                "headingHoldLpfFreq",
                "uint8_t"
            ],
            [
                "legacyYawJumpLimit",
                "uint16_t"
            ],
            [
                "legacyGyroLpf",
                "uint8_t"
            ],
            [
                "accLpfHz",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint8_t"
            ],
            [
                "reserved3",
                "uint8_t"
            ],
            [
                "reserved4",
                "uint8_t"
            ]
        ]
    },
    "MSP_NAME": {
        "code": 10,
        "direction": 1,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "craftName",
                "char[]"
            ]
        ]
    },
    "MSP_SET_NAME": {
        "code": 11,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_NAV_POSHOLD": {
        "code": 12,
        "direction": 1,
        "struct": "<BHHHHBBH",
        "byte_count": 13,
        "payload": [
            [
                "userControlMode",
                "uint8_t"
            ],
            [
                "maxAutoSpeed",
                "uint16_t"
            ],
            [
                "maxAutoClimbRate",
                "uint16_t"
            ],
            [
                "maxManualSpeed",
                "uint16_t"
            ],
            [
                "maxManualClimbRate",
                "uint16_t"
            ],
            [
                "mcMaxBankAngle",
                "uint8_t"
            ],
            [
                "mcAltHoldThrottleType",
                "uint8_t"
            ],
            [
                "mcHoverThrottle",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_NAV_POSHOLD": {
        "code": 13,
        "direction": 0,
        "struct": "<BHHHHBBH",
        "byte_count": 13,
        "payload": [
            [
                "userControlMode",
                "uint8_t"
            ],
            [
                "maxAutoSpeed",
                "uint16_t"
            ],
            [
                "maxAutoClimbRate",
                "uint16_t"
            ],
            [
                "maxManualSpeed",
                "uint16_t"
            ],
            [
                "maxManualClimbRate",
                "uint16_t"
            ],
            [
                "mcMaxBankAngle",
                "uint8_t"
            ],
            [
                "mcAltHoldThrottleType",
                "uint8_t"
            ],
            [
                "mcHoverThrottle",
                "uint16_t"
            ]
        ]
    },
    "MSP_CALIBRATION_DATA": {
        "code": 14,
        "direction": 1,
        "struct": "<BHHHHHHHHHHHHH",
        "byte_count": 27,
        "payload": [
            [
                "accCalibAxisFlags",
                "uint8_t"
            ],
            [
                "accZeroX",
                "uint16_t"
            ],
            [
                "accZeroY",
                "uint16_t"
            ],
            [
                "accZeroZ",
                "uint16_t"
            ],
            [
                "accGainX",
                "uint16_t"
            ],
            [
                "accGainY",
                "uint16_t"
            ],
            [
                "accGainZ",
                "uint16_t"
            ],
            [
                "magZeroX",
                "uint16_t"
            ],
            [
                "magZeroY",
                "uint16_t"
            ],
            [
                "magZeroZ",
                "uint16_t"
            ],
            [
                "opflowScale",
                "uint16_t"
            ],
            [
                "magGainX",
                "uint16_t"
            ],
            [
                "magGainY",
                "uint16_t"
            ],
            [
                "magGainZ",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_CALIBRATION_DATA": {
        "code": 15,
        "direction": 0,
        "struct": "<HHHHHHHHHHHHH",
        "byte_count": 26,
        "payload": [
            [
                "accZeroX",
                "uint16_t"
            ],
            [
                "accZeroY",
                "uint16_t"
            ],
            [
                "accZeroZ",
                "uint16_t"
            ],
            [
                "accGainX",
                "uint16_t"
            ],
            [
                "accGainY",
                "uint16_t"
            ],
            [
                "accGainZ",
                "uint16_t"
            ],
            [
                "magZeroX",
                "uint16_t"
            ],
            [
                "magZeroY",
                "uint16_t"
            ],
            [
                "magZeroZ",
                "uint16_t"
            ],
            [
                "opflowScale",
                "uint16_t"
            ],
            [
                "magGainX",
                "uint16_t"
            ],
            [
                "magGainY",
                "uint16_t"
            ],
            [
                "magGainZ",
                "uint16_t"
            ]
        ]
    },
    "MSP_POSITION_ESTIMATION_CONFIG": {
        "code": 16,
        "direction": 1,
        "struct": "<HHHHHBB",
        "byte_count": 12,
        "payload": [
            [
                "weightZBaroP",
                "uint16_t"
            ],
            [
                "weightZGPSP",
                "uint16_t"
            ],
            [
                "weightZGPSV",
                "uint16_t"
            ],
            [
                "weightXYGPSP",
                "uint16_t"
            ],
            [
                "weightXYGPSV",
                "uint16_t"
            ],
            [
                "minSats",
                "uint8_t"
            ],
            [
                "useGPSVelNED",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_POSITION_ESTIMATION_CONFIG": {
        "code": 17,
        "direction": 0,
        "struct": "<HHHHHBB",
        "byte_count": 12,
        "payload": [
            [
                "weightZBaroP",
                "uint16_t"
            ],
            [
                "weightZGPSP",
                "uint16_t"
            ],
            [
                "weightZGPSV",
                "uint16_t"
            ],
            [
                "weightXYGPSP",
                "uint16_t"
            ],
            [
                "weightXYGPSV",
                "uint16_t"
            ],
            [
                "minSats",
                "uint8_t"
            ],
            [
                "useGPSVelNED",
                "uint8_t"
            ]
        ]
    },
    "MSP_WP_MISSION_LOAD": {
        "code": 18,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "missionID",
                "uint8_t"
            ]
        ]
    },
    "MSP_WP_MISSION_SAVE": {
        "code": 19,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "missionID",
                "uint8_t"
            ]
        ]
    },
    "MSP_WP_GETINFO": {
        "code": 20,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "wpCapabilities",
                "uint8_t"
            ],
            [
                "maxWaypoints",
                "uint8_t"
            ],
            [
                "missionValid",
                "uint8_t"
            ],
            [
                "waypointCount",
                "uint8_t"
            ]
        ]
    },
    "MSP_RTH_AND_LAND_CONFIG": {
        "code": 21,
        "direction": 1,
        "struct": "<HBBBBBHHHHHHH",
        "byte_count": 21,
        "payload": [
            [
                "minRthDistance",
                "uint16_t"
            ],
            [
                "rthClimbFirst",
                "uint8_t"
            ],
            [
                "rthClimbIgnoreEmerg",
                "uint8_t"
            ],
            [
                "rthTailFirst",
                "uint8_t"
            ],
            [
                "rthAllowLanding",
                "uint8_t"
            ],
            [
                "rthAltControlMode",
                "uint8_t"
            ],
            [
                "rthAbortThreshold",
                "uint16_t"
            ],
            [
                "rthAltitude",
                "uint16_t"
            ],
            [
                "landMinAltVspd",
                "uint16_t"
            ],
            [
                "landMaxAltVspd",
                "uint16_t"
            ],
            [
                "landSlowdownMinAlt",
                "uint16_t"
            ],
            [
                "landSlowdownMaxAlt",
                "uint16_t"
            ],
            [
                "emergDescentRate",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_RTH_AND_LAND_CONFIG": {
        "code": 22,
        "direction": 0,
        "struct": "<HBBBBBHHHHHHH",
        "byte_count": 21,
        "payload": [
            [
                "minRthDistance",
                "uint16_t"
            ],
            [
                "rthClimbFirst",
                "uint8_t"
            ],
            [
                "rthClimbIgnoreEmerg",
                "uint8_t"
            ],
            [
                "rthTailFirst",
                "uint8_t"
            ],
            [
                "rthAllowLanding",
                "uint8_t"
            ],
            [
                "rthAltControlMode",
                "uint8_t"
            ],
            [
                "rthAbortThreshold",
                "uint16_t"
            ],
            [
                "rthAltitude",
                "uint16_t"
            ],
            [
                "landMinAltVspd",
                "uint16_t"
            ],
            [
                "landMaxAltVspd",
                "uint16_t"
            ],
            [
                "landSlowdownMinAlt",
                "uint16_t"
            ],
            [
                "landSlowdownMaxAlt",
                "uint16_t"
            ],
            [
                "emergDescentRate",
                "uint16_t"
            ]
        ]
    },
    "MSP_FW_CONFIG": {
        "code": 23,
        "direction": 1,
        "struct": "<HHHBBBBH",
        "byte_count": 12,
        "payload": [
            [
                "cruiseThrottle",
                "uint16_t"
            ],
            [
                "minThrottle",
                "uint16_t"
            ],
            [
                "maxThrottle",
                "uint16_t"
            ],
            [
                "maxBankAngle",
                "uint8_t"
            ],
            [
                "maxClimbAngle",
                "uint8_t"
            ],
            [
                "maxDiveAngle",
                "uint8_t"
            ],
            [
                "pitchToThrottle",
                "uint8_t"
            ],
            [
                "loiterRadius",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_FW_CONFIG": {
        "code": 24,
        "direction": 0,
        "struct": "<HHHBBBBH",
        "byte_count": 12,
        "payload": [
            [
                "cruiseThrottle",
                "uint16_t"
            ],
            [
                "minThrottle",
                "uint16_t"
            ],
            [
                "maxThrottle",
                "uint16_t"
            ],
            [
                "maxBankAngle",
                "uint8_t"
            ],
            [
                "maxClimbAngle",
                "uint8_t"
            ],
            [
                "maxDiveAngle",
                "uint8_t"
            ],
            [
                "pitchToThrottle",
                "uint8_t"
            ],
            [
                "loiterRadius",
                "uint16_t"
            ]
        ]
    },
    "MSP_MODE_RANGES": {
        "code": 34,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "modePermanentId",
                "uint8_t"
            ],
            [
                "auxChannelIndex",
                "uint8_t"
            ],
            [
                "rangeStartStep",
                "uint8_t"
            ],
            [
                "rangeEndStep",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_MODE_RANGE": {
        "code": 35,
        "direction": 0,
        "struct": "<BBBBB",
        "byte_count": 5,
        "payload": [
            [
                "rangeIndex",
                "uint8_t"
            ],
            [
                "modePermanentId",
                "uint8_t"
            ],
            [
                "auxChannelIndex",
                "uint8_t"
            ],
            [
                "rangeStartStep",
                "uint8_t"
            ],
            [
                "rangeEndStep",
                "uint8_t"
            ]
        ]
    },
    "MSP_FEATURE": {
        "code": 36,
        "direction": 1,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "featureMask",
                "uint32_t"
            ]
        ]
    },
    "MSP_SET_FEATURE": {
        "code": 37,
        "direction": 0,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "featureMask",
                "uint32_t"
            ]
        ]
    },
    "MSP_BOARD_ALIGNMENT": {
        "code": 38,
        "direction": 1,
        "struct": "<HHH",
        "byte_count": 6,
        "payload": [
            [
                "rollAlign",
                "uint16_t"
            ],
            [
                "pitchAlign",
                "uint16_t"
            ],
            [
                "yawAlign",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_BOARD_ALIGNMENT": {
        "code": 39,
        "direction": 0,
        "struct": "<HHH",
        "byte_count": 6,
        "payload": [
            [
                "rollAlign",
                "uint16_t"
            ],
            [
                "pitchAlign",
                "uint16_t"
            ],
            [
                "yawAlign",
                "uint16_t"
            ]
        ]
    },
    "MSP_CURRENT_METER_CONFIG": {
        "code": 40,
        "direction": 1,
        "struct": "<HHBH",
        "byte_count": 7,
        "payload": [
            [
                "scale",
                "uint16_t"
            ],
            [
                "offset",
                "uint16_t"
            ],
            [
                "type",
                "uint8_t"
            ],
            [
                "capacity",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_CURRENT_METER_CONFIG": {
        "code": 41,
        "direction": 0,
        "struct": "<HHBH",
        "byte_count": 7,
        "payload": [
            [
                "scale",
                "uint16_t"
            ],
            [
                "offset",
                "uint16_t"
            ],
            [
                "type",
                "uint8_t"
            ],
            [
                "capacity",
                "uint16_t"
            ]
        ]
    },
    "MSP_MIXER": {
        "code": 42,
        "direction": 1,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "mixerMode",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_MIXER": {
        "code": 43,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "mixerMode",
                "uint8_t"
            ]
        ]
    },
    "MSP_RX_CONFIG": {
        "code": 44,
        "direction": 1,
        "struct": "<BHHHBHHBBHBIBBB",
        "byte_count": 24,
        "payload": [
            [
                "serialRxProvider",
                "uint8_t"
            ],
            [
                "maxCheck",
                "uint16_t"
            ],
            [
                "midRc",
                "uint16_t"
            ],
            [
                "minCheck",
                "uint16_t"
            ],
            [
                "spektrumSatBind",
                "uint8_t"
            ],
            [
                "rxMinUsec",
                "uint16_t"
            ],
            [
                "rxMaxUsec",
                "uint16_t"
            ],
            [
                "bfCompatRcInterpolation",
                "uint8_t"
            ],
            [
                "bfCompatRcInterpolationInt",
                "uint8_t"
            ],
            [
                "bfCompatAirModeThreshold",
                "uint16_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint32_t"
            ],
            [
                "reserved3",
                "uint8_t"
            ],
            [
                "bfCompatFpvCamAngle",
                "uint8_t"
            ],
            [
                "receiverType",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_RX_CONFIG": {
        "code": 45,
        "direction": 0,
        "struct": "<BHHHBHHBBHBIBBB",
        "byte_count": 24,
        "payload": [
            [
                "serialRxProvider",
                "uint8_t"
            ],
            [
                "maxCheck",
                "uint16_t"
            ],
            [
                "midRc",
                "uint16_t"
            ],
            [
                "minCheck",
                "uint16_t"
            ],
            [
                "spektrumSatBind",
                "uint8_t"
            ],
            [
                "rxMinUsec",
                "uint16_t"
            ],
            [
                "rxMaxUsec",
                "uint16_t"
            ],
            [
                "bfCompatRcInterpolation",
                "uint8_t"
            ],
            [
                "bfCompatRcInterpolationInt",
                "uint8_t"
            ],
            [
                "bfCompatAirModeThreshold",
                "uint16_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint32_t"
            ],
            [
                "reserved3",
                "uint8_t"
            ],
            [
                "bfCompatFpvCamAngle",
                "uint8_t"
            ],
            [
                "receiverType",
                "uint8_t"
            ]
        ]
    },
    "MSP_LED_COLORS": {
        "code": 46,
        "direction": 1,
        "struct": "<HBB",
        "byte_count": 4,
        "payload": [
            [
                "hue",
                "uint16_t"
            ],
            [
                "saturation",
                "uint8_t"
            ],
            [
                "value",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_LED_COLORS": {
        "code": 47,
        "direction": 0,
        "struct": "<HBB",
        "byte_count": 4,
        "payload": [
            [
                "hue",
                "uint16_t"
            ],
            [
                "saturation",
                "uint8_t"
            ],
            [
                "value",
                "uint8_t"
            ]
        ]
    },
    "MSP_LED_STRIP_CONFIG": {
        "code": 48,
        "direction": 1,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "legacyLedConfig",
                "uint32_t"
            ]
        ]
    },
    "MSP_SET_LED_STRIP_CONFIG": {
        "code": 49,
        "direction": 0,
        "struct": "<BI",
        "byte_count": 5,
        "payload": [
            [
                "ledIndex",
                "uint8_t"
            ],
            [
                "legacyLedConfig",
                "uint32_t"
            ]
        ]
    },
    "MSP_RSSI_CONFIG": {
        "code": 50,
        "direction": 1,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "rssiChannel",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_RSSI_CONFIG": {
        "code": 51,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "rssiChannel",
                "uint8_t"
            ]
        ]
    },
    "MSP_ADJUSTMENT_RANGES": {
        "code": 52,
        "direction": 1,
        "struct": "<BBBBBB",
        "byte_count": 6,
        "payload": [
            [
                "adjustmentIndex",
                "uint8_t"
            ],
            [
                "auxChannelIndex",
                "uint8_t"
            ],
            [
                "rangeStartStep",
                "uint8_t"
            ],
            [
                "rangeEndStep",
                "uint8_t"
            ],
            [
                "adjustmentFunction",
                "uint8_t"
            ],
            [
                "auxSwitchChannelIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_ADJUSTMENT_RANGE": {
        "code": 53,
        "direction": 0,
        "struct": "<BBBBBBB",
        "byte_count": 7,
        "payload": [
            [
                "rangeIndex",
                "uint8_t"
            ],
            [
                "adjustmentIndex",
                "uint8_t"
            ],
            [
                "auxChannelIndex",
                "uint8_t"
            ],
            [
                "rangeStartStep",
                "uint8_t"
            ],
            [
                "rangeEndStep",
                "uint8_t"
            ],
            [
                "adjustmentFunction",
                "uint8_t"
            ],
            [
                "auxSwitchChannelIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP_CF_SERIAL_CONFIG": {
        "code": 54,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_CF_SERIAL_CONFIG": {
        "code": 55,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_VOLTAGE_METER_CONFIG": {
        "code": 56,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "vbatScale",
                "uint8_t"
            ],
            [
                "vbatMinCell",
                "uint8_t"
            ],
            [
                "vbatMaxCell",
                "uint8_t"
            ],
            [
                "vbatWarningCell",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_VOLTAGE_METER_CONFIG": {
        "code": 57,
        "direction": 0,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "vbatScale",
                "uint8_t"
            ],
            [
                "vbatMinCell",
                "uint8_t"
            ],
            [
                "vbatMaxCell",
                "uint8_t"
            ],
            [
                "vbatWarningCell",
                "uint8_t"
            ]
        ]
    },
    "MSP_SONAR_ALTITUDE": {
        "code": 58,
        "direction": 1,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "rangefinderAltitude",
                "uint32_t"
            ]
        ]
    },
    "MSP_RX_MAP": {
        "code": 64,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_RX_MAP": {
        "code": 65,
        "direction": 0,
        "struct": "<<?>",
        "byte_count": 1,
        "payload": [
            [
                "uint8_t[MAX_MAPPABLE_RX_INPUTS]",
                "MAX_MAPPABLE_RX_INPUTS"
            ]
        ]
    },
    "MSP_REBOOT": {
        "code": 68,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_DATAFLASH_SUMMARY": {
        "code": 70,
        "direction": 1,
        "struct": "<BIII",
        "byte_count": 13,
        "payload": [
            [
                "flashReady",
                "uint8_t"
            ],
            [
                "sectorCount",
                "uint32_t"
            ],
            [
                "totalSize",
                "uint32_t"
            ],
            [
                "usedSize",
                "uint32_t"
            ]
        ]
    },
    "MSP_DATAFLASH_READ": {
        "code": 71,
        "direction": 2,
        "struct": "<Is",
        "byte_count": 5,
        "payload": [
            [
                "address",
                "uint32_t"
            ],
            [
                "data",
                "uint8_t[]"
            ]
        ]
    },
    "MSP_DATAFLASH_ERASE": {
        "code": 72,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_LOOP_TIME": {
        "code": 73,
        "direction": 1,
        "struct": "<H",
        "byte_count": 2,
        "payload": [
            [
                "looptime",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_LOOP_TIME": {
        "code": 74,
        "direction": 0,
        "struct": "<H",
        "byte_count": 2,
        "payload": [
            [
                "looptime",
                "uint16_t"
            ]
        ]
    },
    "MSP_FAILSAFE_CONFIG": {
        "code": 75,
        "direction": 1,
        "struct": "<BBHBHBBHHHHHB",
        "byte_count": 20,
        "payload": [
            [
                "failsafeDelay",
                "uint8_t"
            ],
            [
                "failsafeOffDelay",
                "uint8_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "legacyKillSwitch",
                "uint8_t"
            ],
            [
                "failsafeThrottleLowDelay",
                "uint16_t"
            ],
            [
                "failsafeProcedure",
                "uint8_t"
            ],
            [
                "failsafeRecoveryDelay",
                "uint8_t"
            ],
            [
                "failsafeFWRollAngle",
                "uint16_t"
            ],
            [
                "failsafeFWPitchAngle",
                "uint16_t"
            ],
            [
                "failsafeFWYawRate",
                "uint16_t"
            ],
            [
                "failsafeStickThreshold",
                "uint16_t"
            ],
            [
                "failsafeMinDistance",
                "uint16_t"
            ],
            [
                "failsafeMinDistanceProc",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_FAILSAFE_CONFIG": {
        "code": 76,
        "direction": 0,
        "struct": "<BBHBHBBHHHHHB",
        "byte_count": 20,
        "payload": [
            [
                "failsafeDelay",
                "uint8_t"
            ],
            [
                "failsafeOffDelay",
                "uint8_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "legacyKillSwitch",
                "uint8_t"
            ],
            [
                "failsafeThrottleLowDelay",
                "uint16_t"
            ],
            [
                "failsafeProcedure",
                "uint8_t"
            ],
            [
                "failsafeRecoveryDelay",
                "uint8_t"
            ],
            [
                "failsafeFWRollAngle",
                "uint16_t"
            ],
            [
                "failsafeFWPitchAngle",
                "uint16_t"
            ],
            [
                "failsafeFWYawRate",
                "uint16_t"
            ],
            [
                "failsafeStickThreshold",
                "uint16_t"
            ],
            [
                "failsafeMinDistance",
                "uint16_t"
            ],
            [
                "failsafeMinDistanceProc",
                "uint8_t"
            ]
        ]
    },
    "MSP_SDCARD_SUMMARY": {
        "code": 79,
        "direction": 1,
        "struct": "<BBBII",
        "byte_count": 11,
        "payload": [
            [
                "sdCardSupported",
                "uint8_t"
            ],
            [
                "sdCardState",
                "uint8_t"
            ],
            [
                "fsError",
                "uint8_t"
            ],
            [
                "freeSpaceKB",
                "uint32_t"
            ],
            [
                "totalSpaceKB",
                "uint32_t"
            ]
        ]
    },
    "MSP_BLACKBOX_CONFIG": {
        "code": 80,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "blackboxDevice",
                "uint8_t"
            ],
            [
                "blackboxRateNum",
                "uint8_t"
            ],
            [
                "blackboxRateDenom",
                "uint8_t"
            ],
            [
                "blackboxPDenom",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_BLACKBOX_CONFIG": {
        "code": 81,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_TRANSPONDER_CONFIG": {
        "code": 82,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_TRANSPONDER_CONFIG": {
        "code": 83,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_OSD_CONFIG": {
        "code": 84,
        "direction": 1,
        "struct": "<BBBBHHHHH<?>",
        "byte_count": 14,
        "payload": [
            [
                "osdDriverType",
                "uint8_t"
            ],
            [
                "videoSystem",
                "uint8_t"
            ],
            [
                "units",
                "uint8_t"
            ],
            [
                "rssiAlarm",
                "uint8_t"
            ],
            [
                "capAlarm",
                "uint16_t"
            ],
            [
                "timerAlarm",
                "uint16_t"
            ],
            [
                "altAlarm",
                "uint16_t"
            ],
            [
                "distAlarm",
                "uint16_t"
            ],
            [
                "negAltAlarm",
                "uint16_t"
            ],
            [
                "uint16_t[OSD_ITEM_COUNT]",
                "OSD_ITEM_COUNT * 2"
            ]
        ]
    },
    "MSP_SET_OSD_CONFIG": {
        "code": 85,
        "direction": 0,
        "struct": "<BBBBHHHHHBH",
        "byte_count": 17,
        "payload": [
            [
                "addr",
                "uint8_t"
            ],
            [
                "videoSystem",
                "uint8_t"
            ],
            [
                "units",
                "uint8_t"
            ],
            [
                "rssiAlarm",
                "uint8_t"
            ],
            [
                "capAlarm",
                "uint16_t"
            ],
            [
                "timerAlarm",
                "uint16_t"
            ],
            [
                "altAlarm",
                "uint16_t"
            ],
            [
                "distAlarm",
                "uint16_t"
            ],
            [
                "negAltAlarm",
                "uint16_t"
            ],
            [
                "itemIndex",
                "uint8_t"
            ],
            [
                "itemPosition",
                "uint16_t"
            ]
        ]
    },
    "MSP_OSD_CHAR_READ": {
        "code": 86,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_OSD_CHAR_WRITE": {
        "code": 87,
        "direction": 0,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "charData",
                "uint8_t[]"
            ]
        ]
    },
    "MSP_VTX_CONFIG": {
        "code": 88,
        "direction": 1,
        "struct": "<BBBBBBBBBBB",
        "byte_count": 11,
        "payload": [
            [
                "vtxDeviceType",
                "uint8_t"
            ],
            [
                "band",
                "uint8_t"
            ],
            [
                "channel",
                "uint8_t"
            ],
            [
                "power",
                "uint8_t"
            ],
            [
                "pitMode",
                "uint8_t"
            ],
            [
                "vtxReady",
                "uint8_t"
            ],
            [
                "lowPowerDisarm",
                "uint8_t"
            ],
            [
                "vtxTableAvailable",
                "uint8_t"
            ],
            [
                "bandCount",
                "uint8_t"
            ],
            [
                "channelCount",
                "uint8_t"
            ],
            [
                "powerCount",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_VTX_CONFIG": {
        "code": 89,
        "direction": 0,
        "struct": "<HBBBHBBHBBB",
        "byte_count": 14,
        "payload": [
            [
                "bandChannelEncoded",
                "uint16_t"
            ],
            [
                "power",
                "uint8_t"
            ],
            [
                "pitMode",
                "uint8_t"
            ],
            [
                "lowPowerDisarm",
                "uint8_t"
            ],
            [
                "pitModeFreq",
                "uint16_t"
            ],
            [
                "band",
                "uint8_t"
            ],
            [
                "channel",
                "uint8_t"
            ],
            [
                "frequency",
                "uint16_t"
            ],
            [
                "bandCount",
                "uint8_t"
            ],
            [
                "channelCount",
                "uint8_t"
            ],
            [
                "powerCount",
                "uint8_t"
            ]
        ]
    },
    "MSP_ADVANCED_CONFIG": {
        "code": 90,
        "direction": 1,
        "struct": "<BBBBHHB",
        "byte_count": 9,
        "payload": [
            [
                "gyroSyncDenom",
                "uint8_t"
            ],
            [
                "pidProcessDenom",
                "uint8_t"
            ],
            [
                "useUnsyncedPwm",
                "uint8_t"
            ],
            [
                "motorPwmProtocol",
                "uint8_t"
            ],
            [
                "motorPwmRate",
                "uint16_t"
            ],
            [
                "servoPwmRate",
                "uint16_t"
            ],
            [
                "legacyGyroSync",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_ADVANCED_CONFIG": {
        "code": 91,
        "direction": 0,
        "struct": "<BBBBHHB",
        "byte_count": 9,
        "payload": [
            [
                "gyroSyncDenom",
                "uint8_t"
            ],
            [
                "pidProcessDenom",
                "uint8_t"
            ],
            [
                "useUnsyncedPwm",
                "uint8_t"
            ],
            [
                "motorPwmProtocol",
                "uint8_t"
            ],
            [
                "motorPwmRate",
                "uint16_t"
            ],
            [
                "servoPwmRate",
                "uint16_t"
            ],
            [
                "legacyGyroSync",
                "uint8_t"
            ]
        ]
    },
    "MSP_FILTER_CONFIG": {
        "code": 92,
        "direction": 1,
        "struct": "<BHHHHHHHHHHH",
        "byte_count": 23,
        "payload": [
            [
                "gyroMainLpfHz",
                "uint8_t"
            ],
            [
                "dtermLpfHz",
                "uint16_t"
            ],
            [
                "yawLpfHz",
                "uint16_t"
            ],
            [
                "legacyGyroNotchHz",
                "uint16_t"
            ],
            [
                "legacyGyroNotchCutoff",
                "uint16_t"
            ],
            [
                "bfCompatDtermNotchHz",
                "uint16_t"
            ],
            [
                "bfCompatDtermNotchCutoff",
                "uint16_t"
            ],
            [
                "bfCompatGyroNotch2Hz",
                "uint16_t"
            ],
            [
                "bfCompatGyroNotch2Cutoff",
                "uint16_t"
            ],
            [
                "accNotchHz",
                "uint16_t"
            ],
            [
                "accNotchCutoff",
                "uint16_t"
            ],
            [
                "legacyGyroStage2LpfHz",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_FILTER_CONFIG": {
        "code": 93,
        "direction": 0,
        "struct": "<BHHHHHHHHHHH",
        "byte_count": 23,
        "payload": [
            [
                "gyroMainLpfHz",
                "uint8_t"
            ],
            [
                "dtermLpfHz",
                "uint16_t"
            ],
            [
                "yawLpfHz",
                "uint16_t"
            ],
            [
                "legacyGyroNotchHz",
                "uint16_t"
            ],
            [
                "legacyGyroNotchCutoff",
                "uint16_t"
            ],
            [
                "bfCompatDtermNotchHz",
                "uint16_t"
            ],
            [
                "bfCompatDtermNotchCutoff",
                "uint16_t"
            ],
            [
                "bfCompatGyroNotch2Hz",
                "uint16_t"
            ],
            [
                "bfCompatGyroNotch2Cutoff",
                "uint16_t"
            ],
            [
                "accNotchHz",
                "uint16_t"
            ],
            [
                "accNotchCutoff",
                "uint16_t"
            ],
            [
                "legacyGyroStage2LpfHz",
                "uint16_t"
            ]
        ]
    },
    "MSP_PID_ADVANCED": {
        "code": 94,
        "direction": 1,
        "struct": "<HHHBBBBHBHH",
        "byte_count": 17,
        "payload": [
            [
                "legacyRollPitchItermIgnore",
                "uint16_t"
            ],
            [
                "legacyYawItermIgnore",
                "uint16_t"
            ],
            [
                "legacyYawPLimit",
                "uint16_t"
            ],
            [
                "bfCompatDeltaMethod",
                "uint8_t"
            ],
            [
                "bfCompatVbatPidComp",
                "uint8_t"
            ],
            [
                "bfCompatSetpointRelaxRatio",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "legacyPidSumLimit",
                "uint16_t"
            ],
            [
                "bfCompatItermThrottleGain",
                "uint8_t"
            ],
            [
                "accelLimitRollPitch",
                "uint16_t"
            ],
            [
                "accelLimitYaw",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_PID_ADVANCED": {
        "code": 95,
        "direction": 0,
        "struct": "<HHHBBBBHBHH",
        "byte_count": 17,
        "payload": [
            [
                "legacyRollPitchItermIgnore",
                "uint16_t"
            ],
            [
                "legacyYawItermIgnore",
                "uint16_t"
            ],
            [
                "legacyYawPLimit",
                "uint16_t"
            ],
            [
                "bfCompatDeltaMethod",
                "uint8_t"
            ],
            [
                "bfCompatVbatPidComp",
                "uint8_t"
            ],
            [
                "bfCompatSetpointRelaxRatio",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "legacyPidSumLimit",
                "uint16_t"
            ],
            [
                "bfCompatItermThrottleGain",
                "uint8_t"
            ],
            [
                "accelLimitRollPitch",
                "uint16_t"
            ],
            [
                "accelLimitYaw",
                "uint16_t"
            ]
        ]
    },
    "MSP_SENSOR_CONFIG": {
        "code": 96,
        "direction": 1,
        "struct": "<BBBBBB",
        "byte_count": 6,
        "payload": [
            [
                "accHardware",
                "uint8_t"
            ],
            [
                "baroHardware",
                "uint8_t"
            ],
            [
                "magHardware",
                "uint8_t"
            ],
            [
                "pitotHardware",
                "uint8_t"
            ],
            [
                "rangefinderHardware",
                "uint8_t"
            ],
            [
                "opflowHardware",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_SENSOR_CONFIG": {
        "code": 97,
        "direction": 0,
        "struct": "<BBBBBB",
        "byte_count": 6,
        "payload": [
            [
                "accHardware",
                "uint8_t"
            ],
            [
                "baroHardware",
                "uint8_t"
            ],
            [
                "magHardware",
                "uint8_t"
            ],
            [
                "pitotHardware",
                "uint8_t"
            ],
            [
                "rangefinderHardware",
                "uint8_t"
            ],
            [
                "opflowHardware",
                "uint8_t"
            ]
        ]
    },
    "MSP_SPECIAL_PARAMETERS": {
        "code": 98,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_SPECIAL_PARAMETERS": {
        "code": 99,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_STATUS": {
        "code": 101,
        "direction": 1,
        "struct": "<HHHIB",
        "byte_count": 11,
        "payload": [
            [
                "cycleTime",
                "uint16_t"
            ],
            [
                "i2cErrors",
                "uint16_t"
            ],
            [
                "sensorStatus",
                "uint16_t"
            ],
            [
                "activeModesLow",
                "uint32_t"
            ],
            [
                "profile",
                "uint8_t"
            ]
        ]
    },
    "MSP_RAW_IMU": {
        "code": 102,
        "direction": 1,
        "struct": "<hhhhhhhhh",
        "byte_count": 18,
        "payload": [
            [
                "accX",
                "int16_t"
            ],
            [
                "accY",
                "int16_t"
            ],
            [
                "accZ",
                "int16_t"
            ],
            [
                "gyroX",
                "int16_t"
            ],
            [
                "gyroY",
                "int16_t"
            ],
            [
                "gyroZ",
                "int16_t"
            ],
            [
                "magX",
                "int16_t"
            ],
            [
                "magY",
                "int16_t"
            ],
            [
                "magZ",
                "int16_t"
            ]
        ]
    },
    "MSP_SERVO": {
        "code": 103,
        "direction": 1,
        "struct": "<<?>",
        "byte_count": 1,
        "payload": [
            [
                "int16_t[MAX_SUPPORTED_SERVOS]",
                "MAX_SUPPORTED_SERVOS * 2"
            ]
        ]
    },
    "MSP_MOTOR": {
        "code": 104,
        "direction": 1,
        "struct": "<HHHHHHHH",
        "byte_count": 16,
        "payload": [
            [
                "motorOutputs",
                "uint16_t[8]"
            ]
        ]
    },
    "MSP_RC": {
        "code": 105,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_RAW_GPS": {
        "code": 106,
        "direction": 1,
        "struct": "<BBIIHHHH",
        "byte_count": 18,
        "payload": [
            [
                "fixType",
                "uint8_t"
            ],
            [
                "numSat",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint16_t"
            ],
            [
                "speed",
                "uint16_t"
            ],
            [
                "groundCourse",
                "uint16_t"
            ],
            [
                "hdop",
                "uint16_t"
            ]
        ]
    },
    "MSP_COMP_GPS": {
        "code": 107,
        "direction": 1,
        "struct": "<HHB",
        "byte_count": 5,
        "payload": [
            [
                "distanceToHome",
                "uint16_t"
            ],
            [
                "directionToHome",
                "uint16_t"
            ],
            [
                "gpsHeartbeat",
                "uint8_t"
            ]
        ]
    },
    "MSP_ATTITUDE": {
        "code": 108,
        "direction": 1,
        "struct": "<hhh",
        "byte_count": 6,
        "payload": [
            [
                "roll",
                "int16_t"
            ],
            [
                "pitch",
                "int16_t"
            ],
            [
                "yaw",
                "int16_t"
            ]
        ]
    },
    "MSP_ALTITUDE": {
        "code": 109,
        "direction": 1,
        "struct": "<IhI",
        "byte_count": 10,
        "payload": [
            [
                "estimatedAltitude",
                "uint32_t"
            ],
            [
                "variometer",
                "int16_t"
            ],
            [
                "baroAltitude",
                "uint32_t"
            ]
        ]
    },
    "MSP_ANALOG": {
        "code": 110,
        "direction": 1,
        "struct": "<BHHh",
        "byte_count": 7,
        "payload": [
            [
                "vbat",
                "uint8_t"
            ],
            [
                "mAhDrawn",
                "uint16_t"
            ],
            [
                "rssi",
                "uint16_t"
            ],
            [
                "amperage",
                "int16_t"
            ]
        ]
    },
    "MSP_RC_TUNING": {
        "code": 111,
        "direction": 1,
        "struct": "<BBBBBBBBHB",
        "byte_count": 11,
        "payload": [
            [
                "legacyRcRate",
                "uint8_t"
            ],
            [
                "rcExpo",
                "uint8_t"
            ],
            [
                "rollRate",
                "uint8_t"
            ],
            [
                "pitchRate",
                "uint8_t"
            ],
            [
                "yawRate",
                "uint8_t"
            ],
            [
                "dynamicThrottlePID",
                "uint8_t"
            ],
            [
                "throttleMid",
                "uint8_t"
            ],
            [
                "throttleExpo",
                "uint8_t"
            ],
            [
                "tpaBreakpoint",
                "uint16_t"
            ],
            [
                "rcYawExpo",
                "uint8_t"
            ]
        ]
    },
    "MSP_ACTIVEBOXES": {
        "code": 113,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_MISC": {
        "code": 114,
        "direction": 1,
        "struct": "<HHHHHBBBBBBHBBBB",
        "byte_count": 22,
        "payload": [
            [
                "midRc",
                "uint16_t"
            ],
            [
                "legacyMinThrottle",
                "uint16_t"
            ],
            [
                "maxThrottle",
                "uint16_t"
            ],
            [
                "minCommand",
                "uint16_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "gpsType",
                "uint8_t"
            ],
            [
                "legacyGpsBaud",
                "uint8_t"
            ],
            [
                "gpsSbasMode",
                "uint8_t"
            ],
            [
                "legacyMwCurrentOut",
                "uint8_t"
            ],
            [
                "rssiChannel",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "magDeclination",
                "uint16_t"
            ],
            [
                "vbatScale",
                "uint8_t"
            ],
            [
                "vbatMinCell",
                "uint8_t"
            ],
            [
                "vbatMaxCell",
                "uint8_t"
            ],
            [
                "vbatWarningCell",
                "uint8_t"
            ]
        ]
    },
    "MSP_BOXNAMES": {
        "code": 116,
        "direction": 1,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "boxNamesString",
                "char[]"
            ]
        ]
    },
    "MSP_PIDNAMES": {
        "code": 117,
        "direction": 1,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "pidNamesString",
                "char[]"
            ]
        ]
    },
    "MSP_WP": {
        "code": 118,
        "direction": 2,
        "struct": "<BBIIIHHHB",
        "byte_count": 21,
        "payload": [
            [
                "waypointIndex",
                "uint8_t"
            ],
            [
                "action",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint32_t"
            ],
            [
                "param1",
                "uint16_t"
            ],
            [
                "param2",
                "uint16_t"
            ],
            [
                "param3",
                "uint16_t"
            ],
            [
                "flag",
                "uint8_t"
            ]
        ]
    },
    "MSP_BOXIDS": {
        "code": 119,
        "direction": 1,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "boxIds",
                "uint8_t[]"
            ]
        ]
    },
    "MSP_SERVO_CONFIGURATIONS": {
        "code": 120,
        "direction": 1,
        "struct": "<HHHBBBBI",
        "byte_count": 14,
        "payload": [
            [
                "min",
                "uint16_t"
            ],
            [
                "max",
                "uint16_t"
            ],
            [
                "middle",
                "uint16_t"
            ],
            [
                "rate",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint8_t"
            ],
            [
                "legacyForwardChan",
                "uint8_t"
            ],
            [
                "legacyReversedSources",
                "uint32_t"
            ]
        ]
    },
    "MSP_NAV_STATUS": {
        "code": 121,
        "direction": 1,
        "struct": "<BBBBBH",
        "byte_count": 7,
        "payload": [
            [
                "navMode",
                "uint8_t"
            ],
            [
                "navState",
                "uint8_t"
            ],
            [
                "activeWpAction",
                "uint8_t"
            ],
            [
                "activeWpNumber",
                "uint8_t"
            ],
            [
                "navError",
                "uint8_t"
            ],
            [
                "targetHeading",
                "uint16_t"
            ]
        ]
    },
    "MSP_3D": {
        "code": 124,
        "direction": 1,
        "struct": "<HHH",
        "byte_count": 6,
        "payload": [
            [
                "deadbandLow",
                "uint16_t"
            ],
            [
                "deadbandHigh",
                "uint16_t"
            ],
            [
                "neutral",
                "uint16_t"
            ]
        ]
    },
    "MSP_RC_DEADBAND": {
        "code": 125,
        "direction": 1,
        "struct": "<BBBH",
        "byte_count": 5,
        "payload": [
            [
                "deadband",
                "uint8_t"
            ],
            [
                "yawDeadband",
                "uint8_t"
            ],
            [
                "altHoldDeadband",
                "uint8_t"
            ],
            [
                "throttleDeadband",
                "uint16_t"
            ]
        ]
    },
    "MSP_SENSOR_ALIGNMENT": {
        "code": 126,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "gyroAlign",
                "uint8_t"
            ],
            [
                "accAlign",
                "uint8_t"
            ],
            [
                "magAlign",
                "uint8_t"
            ],
            [
                "opflowAlign",
                "uint8_t"
            ]
        ]
    },
    "MSP_LED_STRIP_MODECOLOR": {
        "code": 127,
        "direction": 1,
        "struct": "<BBB",
        "byte_count": 3,
        "payload": [
            [
                "modeIndex",
                "uint8_t"
            ],
            [
                "directionOrSpecialIndex",
                "uint8_t"
            ],
            [
                "colorIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP_BATTERY_STATE": {
        "code": 130,
        "direction": 1,
        "struct": "<BHBHhBH",
        "byte_count": 11,
        "payload": [
            [
                "cellCount",
                "uint8_t"
            ],
            [
                "capacity",
                "uint16_t"
            ],
            [
                "vbatScaled",
                "uint8_t"
            ],
            [
                "mAhDrawn",
                "uint16_t"
            ],
            [
                "amperage",
                "int16_t"
            ],
            [
                "batteryState",
                "uint8_t"
            ],
            [
                "vbatActual",
                "uint16_t"
            ]
        ]
    },
    "MSP_VTXTABLE_BAND": {
        "code": 137,
        "direction": 2,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_VTXTABLE_POWERLEVEL": {
        "code": 138,
        "direction": 2,
        "struct": "<BHBs",
        "byte_count": 5,
        "payload": [
            [
                "powerLevelIndex",
                "uint8_t"
            ],
            [
                "powerValue",
                "uint16_t"
            ],
            [
                "labelLength",
                "uint8_t"
            ],
            [
                "label",
                "char[]"
            ]
        ]
    },
    "MSP_SET_RAW_RC": {
        "code": 200,
        "direction": 0,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "rcChannels",
                "uint16_t[]"
            ]
        ]
    },
    "MSP_SET_RAW_GPS": {
        "code": 201,
        "direction": 0,
        "struct": "<BBIIHHH",
        "byte_count": 16,
        "payload": [
            [
                "fixType",
                "uint8_t"
            ],
            [
                "numSat",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint16_t"
            ],
            [
                "speed",
                "uint16_t"
            ],
            [
                "groundCourse",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_BOX": {
        "code": 203,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_RC_TUNING": {
        "code": 204,
        "direction": 0,
        "struct": "<BBBBBBBBHB",
        "byte_count": 11,
        "payload": [
            [
                "legacyRcRate",
                "uint8_t"
            ],
            [
                "rcExpo",
                "uint8_t"
            ],
            [
                "rollRate",
                "uint8_t"
            ],
            [
                "pitchRate",
                "uint8_t"
            ],
            [
                "yawRate",
                "uint8_t"
            ],
            [
                "dynamicThrottlePID",
                "uint8_t"
            ],
            [
                "throttleMid",
                "uint8_t"
            ],
            [
                "throttleExpo",
                "uint8_t"
            ],
            [
                "tpaBreakpoint",
                "uint16_t"
            ],
            [
                "rcYawExpo",
                "uint8_t"
            ]
        ]
    },
    "MSP_ACC_CALIBRATION": {
        "code": 205,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_MAG_CALIBRATION": {
        "code": 206,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_MISC": {
        "code": 207,
        "direction": 0,
        "struct": "<HHHHHBBBBBBHBBBB",
        "byte_count": 22,
        "payload": [
            [
                "midRc",
                "uint16_t"
            ],
            [
                "legacyMinThrottle",
                "uint16_t"
            ],
            [
                "legacyMaxThrottle",
                "uint16_t"
            ],
            [
                "minCommand",
                "uint16_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "gpsType",
                "uint8_t"
            ],
            [
                "legacyGpsBaud",
                "uint8_t"
            ],
            [
                "gpsSbasMode",
                "uint8_t"
            ],
            [
                "legacyMwCurrentOut",
                "uint8_t"
            ],
            [
                "rssiChannel",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "magDeclination",
                "uint16_t"
            ],
            [
                "vbatScale",
                "uint8_t"
            ],
            [
                "vbatMinCell",
                "uint8_t"
            ],
            [
                "vbatMaxCell",
                "uint8_t"
            ],
            [
                "vbatWarningCell",
                "uint8_t"
            ]
        ]
    },
    "MSP_RESET_CONF": {
        "code": 208,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_WP": {
        "code": 209,
        "direction": 0,
        "struct": "<BBIIIHHHB",
        "byte_count": 21,
        "payload": [
            [
                "waypointIndex",
                "uint8_t"
            ],
            [
                "action",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint32_t"
            ],
            [
                "param1",
                "uint16_t"
            ],
            [
                "param2",
                "uint16_t"
            ],
            [
                "param3",
                "uint16_t"
            ],
            [
                "flag",
                "uint8_t"
            ]
        ]
    },
    "MSP_SELECT_SETTING": {
        "code": 210,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "profileIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_HEAD": {
        "code": 211,
        "direction": 0,
        "struct": "<H",
        "byte_count": 2,
        "payload": [
            [
                "heading",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_SERVO_CONFIGURATION": {
        "code": 212,
        "direction": 0,
        "struct": "<BHHHBBBBI",
        "byte_count": 15,
        "payload": [
            [
                "servoIndex",
                "uint8_t"
            ],
            [
                "min",
                "uint16_t"
            ],
            [
                "max",
                "uint16_t"
            ],
            [
                "middle",
                "uint16_t"
            ],
            [
                "rate",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "reserved2",
                "uint8_t"
            ],
            [
                "legacyForwardChan",
                "uint8_t"
            ],
            [
                "legacyReversedSources",
                "uint32_t"
            ]
        ]
    },
    "MSP_SET_MOTOR": {
        "code": 214,
        "direction": 0,
        "struct": "<HHHHHHHH",
        "byte_count": 16,
        "payload": [
            [
                "motorValues",
                "uint16_t[8]"
            ]
        ]
    },
    "MSP_SET_3D": {
        "code": 217,
        "direction": 0,
        "struct": "<HHH",
        "byte_count": 6,
        "payload": [
            [
                "deadbandLow",
                "uint16_t"
            ],
            [
                "deadbandHigh",
                "uint16_t"
            ],
            [
                "neutral",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_RC_DEADBAND": {
        "code": 218,
        "direction": 0,
        "struct": "<BBBH",
        "byte_count": 5,
        "payload": [
            [
                "deadband",
                "uint8_t"
            ],
            [
                "yawDeadband",
                "uint8_t"
            ],
            [
                "altHoldDeadband",
                "uint8_t"
            ],
            [
                "throttleDeadband",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_RESET_CURR_PID": {
        "code": 219,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SET_SENSOR_ALIGNMENT": {
        "code": 220,
        "direction": 0,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "gyroAlign",
                "uint8_t"
            ],
            [
                "accAlign",
                "uint8_t"
            ],
            [
                "magAlign",
                "uint8_t"
            ],
            [
                "opflowAlign",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_LED_STRIP_MODECOLOR": {
        "code": 221,
        "direction": 0,
        "struct": "<BBB",
        "byte_count": 3,
        "payload": [
            [
                "modeIndex",
                "uint8_t"
            ],
            [
                "directionOrSpecialIndex",
                "uint8_t"
            ],
            [
                "colorIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_ACC_TRIM": {
        "code": 239,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_ACC_TRIM": {
        "code": 240,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_SERVO_MIX_RULES": {
        "code": 241,
        "direction": 1,
        "struct": "<BBHBBBB",
        "byte_count": 8,
        "payload": [
            [
                "targetChannel",
                "uint8_t"
            ],
            [
                "inputSource",
                "uint8_t"
            ],
            [
                "rate",
                "uint16_t"
            ],
            [
                "speed",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "legacyMax",
                "uint8_t"
            ],
            [
                "legacyBox",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_SERVO_MIX_RULE": {
        "code": 242,
        "direction": 0,
        "struct": "<BBBHBHB",
        "byte_count": 9,
        "payload": [
            [
                "ruleIndex",
                "uint8_t"
            ],
            [
                "targetChannel",
                "uint8_t"
            ],
            [
                "inputSource",
                "uint8_t"
            ],
            [
                "rate",
                "uint16_t"
            ],
            [
                "speed",
                "uint8_t"
            ],
            [
                "legacyMinMax",
                "uint16_t"
            ],
            [
                "legacyBox",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_PASSTHROUGH": {
        "code": 245,
        "direction": 2,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "status",
                "uint8_t"
            ]
        ]
    },
    "MSP_RTC": {
        "code": 246,
        "direction": 1,
        "struct": "<iH",
        "byte_count": 6,
        "payload": [
            [
                "seconds",
                "int32_t"
            ],
            [
                "millis",
                "uint16_t"
            ]
        ]
    },
    "MSP_SET_RTC": {
        "code": 247,
        "direction": 0,
        "struct": "<iH",
        "byte_count": 6,
        "payload": [
            [
                "seconds",
                "int32_t"
            ],
            [
                "millis",
                "uint16_t"
            ]
        ]
    },
    "MSP_EEPROM_WRITE": {
        "code": 250,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_DEBUGMSG": {
        "code": 253,
        "direction": 1,
        "struct": "<<?>",
        "byte_count": 1,
        "payload": [
            [
                "---",
                ""
            ]
        ]
    },
    "MSP_DEBUG": {
        "code": 254,
        "direction": 1,
        "struct": "<HHHH",
        "byte_count": 8,
        "payload": [
            [
                "debugValues",
                "uint16_t[4]"
            ]
        ]
    },
    "MSP_V2_FRAME": {
        "code": 255,
        "direction": -1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP_STATUS_EX": {
        "code": 150,
        "direction": 1,
        "struct": "<HHHIBHHB",
        "byte_count": 16,
        "payload": [
            [
                "cycleTime",
                "uint16_t"
            ],
            [
                "i2cErrors",
                "uint16_t"
            ],
            [
                "sensorStatus",
                "uint16_t"
            ],
            [
                "activeModesLow",
                "uint32_t"
            ],
            [
                "profile",
                "uint8_t"
            ],
            [
                "cpuLoad",
                "uint16_t"
            ],
            [
                "armingFlags",
                "uint16_t"
            ],
            [
                "accCalibAxisFlags",
                "uint8_t"
            ]
        ]
    },
    "MSP_SENSOR_STATUS": {
        "code": 151,
        "direction": 1,
        "struct": "<BBBBBBBBB",
        "byte_count": 9,
        "payload": [
            [
                "overallHealth",
                "uint8_t"
            ],
            [
                "gyroStatus",
                "uint8_t"
            ],
            [
                "accStatus",
                "uint8_t"
            ],
            [
                "magStatus",
                "uint8_t"
            ],
            [
                "baroStatus",
                "uint8_t"
            ],
            [
                "gpsStatus",
                "uint8_t"
            ],
            [
                "rangefinderStatus",
                "uint8_t"
            ],
            [
                "pitotStatus",
                "uint8_t"
            ],
            [
                "opflowStatus",
                "uint8_t"
            ]
        ]
    },
    "MSP_UID": {
        "code": 160,
        "direction": 1,
        "struct": "<III",
        "byte_count": 12,
        "payload": [
            [
                "uid0",
                "uint32_t"
            ],
            [
                "uid1",
                "uint32_t"
            ],
            [
                "uid2",
                "uint32_t"
            ]
        ]
    },
    "MSP_GPSSVINFO": {
        "code": 164,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "protocolVersion",
                "uint8_t"
            ],
            [
                "numChannels",
                "uint8_t"
            ],
            [
                "hdopHundreds",
                "uint8_t"
            ],
            [
                "hdopUnits",
                "uint8_t"
            ]
        ]
    },
    "MSP_GPSSTATISTICS": {
        "code": 166,
        "direction": 1,
        "struct": "<HIIIHHH",
        "byte_count": 20,
        "payload": [
            [
                "lastMessageDt",
                "uint16_t"
            ],
            [
                "errors",
                "uint32_t"
            ],
            [
                "timeouts",
                "uint32_t"
            ],
            [
                "packetCount",
                "uint32_t"
            ],
            [
                "hdop",
                "uint16_t"
            ],
            [
                "eph",
                "uint16_t"
            ],
            [
                "epv",
                "uint16_t"
            ]
        ]
    },
    "MSP_TX_INFO": {
        "code": 187,
        "direction": 1,
        "struct": "<BB",
        "byte_count": 2,
        "payload": [
            [
                "rssiSource",
                "uint8_t"
            ],
            [
                "rtcDateTimeIsSet",
                "uint8_t"
            ]
        ]
    },
    "MSP_SET_TX_INFO": {
        "code": 186,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "rssi",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_TZ": {
        "code": 4097,
        "direction": 1,
        "struct": "<hB",
        "byte_count": 3,
        "payload": [
            [
                "tzOffsetMinutes",
                "int16_t"
            ],
            [
                "tzAutoDst",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_TZ": {
        "code": 4098,
        "direction": 0,
        "struct": "<hhB",
        "byte_count": 5,
        "payload": [
            [
                "tzOffsetMinutes",
                "int16_t"
            ],
            [
                "tzOffsetMinutes",
                "int16_t"
            ],
            [
                "tzAutoDst",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_SETTING": {
        "code": 4099,
        "direction": 2,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "settingValue",
                "uint8_t[]"
            ]
        ]
    },
    "MSP2_COMMON_SET_SETTING": {
        "code": 4100,
        "direction": 0,
        "struct": "<<?>s",
        "byte_count": 2,
        "payload": [
            [
                "",
                "settingIdentifier"
            ],
            [
                "settingValue",
                "uint8_t[]"
            ]
        ]
    },
    "MSP2_COMMON_MOTOR_MIXER": {
        "code": 4101,
        "direction": 1,
        "struct": "<HHHHHHHH",
        "byte_count": 16,
        "payload": [
            [
                "throttleWeight",
                "uint16_t"
            ],
            [
                "rollWeight",
                "uint16_t"
            ],
            [
                "pitchWeight",
                "uint16_t"
            ],
            [
                "yawWeight",
                "uint16_t"
            ],
            [
                "throttleWeight",
                "uint16_t"
            ],
            [
                "rollWeight",
                "uint16_t"
            ],
            [
                "pitchWeight",
                "uint16_t"
            ],
            [
                "yawWeight",
                "uint16_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_MOTOR_MIXER": {
        "code": 4102,
        "direction": 0,
        "struct": "<BHHHH",
        "byte_count": 9,
        "payload": [
            [
                "motorIndex",
                "uint8_t"
            ],
            [
                "throttleWeight",
                "uint16_t"
            ],
            [
                "rollWeight",
                "uint16_t"
            ],
            [
                "pitchWeight",
                "uint16_t"
            ],
            [
                "yawWeight",
                "uint16_t"
            ]
        ]
    },
    "MSP2_COMMON_SETTING_INFO": {
        "code": 4103,
        "direction": 2,
        "struct": "<sHBBBiIHBBss",
        "byte_count": 20,
        "payload": [
            [
                "settingName",
                "char[]"
            ],
            [
                "pgn",
                "uint16_t"
            ],
            [
                "type",
                "uint8_t"
            ],
            [
                "section",
                "uint8_t"
            ],
            [
                "mode",
                "uint8_t"
            ],
            [
                "minValue",
                "int32_t"
            ],
            [
                "maxValue",
                "uint32_t"
            ],
            [
                "settingIndex",
                "uint16_t"
            ],
            [
                "profileIndex",
                "uint8_t"
            ],
            [
                "profileCount",
                "uint8_t"
            ],
            [
                "lookupNames",
                "char[]"
            ],
            [
                "settingValue",
                "uint8_t[]"
            ]
        ]
    },
    "MSP2_COMMON_PG_LIST": {
        "code": 4104,
        "direction": 2,
        "struct": "<HHH",
        "byte_count": 6,
        "payload": [
            [
                "pgn",
                "uint16_t"
            ],
            [
                "startIndex",
                "uint16_t"
            ],
            [
                "endIndex",
                "uint16_t"
            ]
        ]
    },
    "MSP2_COMMON_SERIAL_CONFIG": {
        "code": 4105,
        "direction": 1,
        "struct": "<BIBBBB",
        "byte_count": 9,
        "payload": [
            [
                "identifier",
                "uint8_t"
            ],
            [
                "functionMask",
                "uint32_t"
            ],
            [
                "mspBaudIndex",
                "uint8_t"
            ],
            [
                "gpsBaudIndex",
                "uint8_t"
            ],
            [
                "telemetryBaudIndex",
                "uint8_t"
            ],
            [
                "peripheralBaudIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_SERIAL_CONFIG": {
        "code": 4106,
        "direction": 0,
        "struct": "<BIBBBB",
        "byte_count": 9,
        "payload": [
            [
                "identifier",
                "uint8_t"
            ],
            [
                "functionMask",
                "uint32_t"
            ],
            [
                "mspBaudIndex",
                "uint8_t"
            ],
            [
                "gpsBaudIndex",
                "uint8_t"
            ],
            [
                "telemetryBaudIndex",
                "uint8_t"
            ],
            [
                "peripheralBaudIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_RADAR_POS": {
        "code": 4107,
        "direction": 0,
        "struct": "<BBIIIHHB",
        "byte_count": 19,
        "payload": [
            [
                "poiIndex",
                "uint8_t"
            ],
            [
                "state",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint32_t"
            ],
            [
                "heading",
                "uint16_t"
            ],
            [
                "speed",
                "uint16_t"
            ],
            [
                "linkQuality",
                "uint8_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_RADAR_ITD": {
        "code": 4108,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_COMMON_SET_MSP_RC_LINK_STATS": {
        "code": 4109,
        "direction": 0,
        "struct": "<BBBBBBb",
        "byte_count": 7,
        "payload": [
            [
                "sublinkID",
                "uint8_t"
            ],
            [
                "validLink",
                "uint8_t"
            ],
            [
                "rssiPercent",
                "uint8_t"
            ],
            [
                "uplinkRSSI_dBm",
                "uint8_t"
            ],
            [
                "downlinkLQ",
                "uint8_t"
            ],
            [
                "uplinkLQ",
                "uint8_t"
            ],
            [
                "uplinkSNR",
                "int8_t"
            ]
        ]
    },
    "MSP2_COMMON_SET_MSP_RC_INFO": {
        "code": 4110,
        "direction": 0,
        "struct": "<BHH4s6s",
        "byte_count": 15,
        "payload": [
            [
                "sublinkID",
                "uint8_t"
            ],
            [
                "uplinkTxPower",
                "uint16_t"
            ],
            [
                "downlinkTxPower",
                "uint16_t"
            ],
            [
                "band",
                "char[4]"
            ],
            [
                "mode",
                "char[6]"
            ]
        ]
    },
    "MSP2_INAV_STATUS": {
        "code": 8192,
        "direction": 1,
        "struct": "<HHHHBI<?>B",
        "byte_count": 14,
        "payload": [
            [
                "cycleTime",
                "uint16_t"
            ],
            [
                "i2cErrors",
                "uint16_t"
            ],
            [
                "sensorStatus",
                "uint16_t"
            ],
            [
                "cpuLoad",
                "uint16_t"
            ],
            [
                "profileAndBattProfile",
                "uint8_t"
            ],
            [
                "armingFlags",
                "uint32_t"
            ],
            [
                "boxBitmask_t",
                "sizeof(boxBitmask_t)"
            ],
            [
                "mixerProfile",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OPTICAL_FLOW": {
        "code": 8193,
        "direction": 1,
        "struct": "<Bhhhh",
        "byte_count": 9,
        "payload": [
            [
                "quality",
                "uint8_t"
            ],
            [
                "flowRateX",
                "int16_t"
            ],
            [
                "flowRateY",
                "int16_t"
            ],
            [
                "bodyRateX",
                "int16_t"
            ],
            [
                "bodyRateY",
                "int16_t"
            ]
        ]
    },
    "MSP2_INAV_ANALOG": {
        "code": 8194,
        "direction": 1,
        "struct": "<BHHIIIIBH",
        "byte_count": 24,
        "payload": [
            [
                "batteryFlags",
                "uint8_t"
            ],
            [
                "vbat",
                "uint16_t"
            ],
            [
                "amperage",
                "uint16_t"
            ],
            [
                "powerDraw",
                "uint32_t"
            ],
            [
                "mAhDrawn",
                "uint32_t"
            ],
            [
                "mWhDrawn",
                "uint32_t"
            ],
            [
                "remainingCapacity",
                "uint32_t"
            ],
            [
                "percentageRemaining",
                "uint8_t"
            ],
            [
                "rssi",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_MISC": {
        "code": 8195,
        "direction": 1,
        "struct": "<HHHHHBBBBHHBBHHHHIIIB",
        "byte_count": 41,
        "payload": [
            [
                "midRc",
                "uint16_t"
            ],
            [
                "legacyMinThrottle",
                "uint16_t"
            ],
            [
                "maxThrottle",
                "uint16_t"
            ],
            [
                "minCommand",
                "uint16_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "gpsType",
                "uint8_t"
            ],
            [
                "legacyGpsBaud",
                "uint8_t"
            ],
            [
                "gpsSbasMode",
                "uint8_t"
            ],
            [
                "rssiChannel",
                "uint8_t"
            ],
            [
                "magDeclination",
                "uint16_t"
            ],
            [
                "vbatScale",
                "uint16_t"
            ],
            [
                "vbatSource",
                "uint8_t"
            ],
            [
                "cellCount",
                "uint8_t"
            ],
            [
                "vbatCellDetect",
                "uint16_t"
            ],
            [
                "vbatMinCell",
                "uint16_t"
            ],
            [
                "vbatMaxCell",
                "uint16_t"
            ],
            [
                "vbatWarningCell",
                "uint16_t"
            ],
            [
                "capacityValue",
                "uint32_t"
            ],
            [
                "capacityWarning",
                "uint32_t"
            ],
            [
                "capacityCritical",
                "uint32_t"
            ],
            [
                "capacityUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_MISC": {
        "code": 8196,
        "direction": 0,
        "struct": "<HHHHHBBBBHHBBHHHHIIIB",
        "byte_count": 41,
        "payload": [
            [
                "midRc",
                "uint16_t"
            ],
            [
                "legacyMinThrottle",
                "uint16_t"
            ],
            [
                "legacyMaxThrottle",
                "uint16_t"
            ],
            [
                "minCommand",
                "uint16_t"
            ],
            [
                "failsafeThrottle",
                "uint16_t"
            ],
            [
                "gpsType",
                "uint8_t"
            ],
            [
                "legacyGpsBaud",
                "uint8_t"
            ],
            [
                "gpsSbasMode",
                "uint8_t"
            ],
            [
                "rssiChannel",
                "uint8_t"
            ],
            [
                "magDeclination",
                "uint16_t"
            ],
            [
                "vbatScale",
                "uint16_t"
            ],
            [
                "vbatSource",
                "uint8_t"
            ],
            [
                "cellCount",
                "uint8_t"
            ],
            [
                "vbatCellDetect",
                "uint16_t"
            ],
            [
                "vbatMinCell",
                "uint16_t"
            ],
            [
                "vbatMaxCell",
                "uint16_t"
            ],
            [
                "vbatWarningCell",
                "uint16_t"
            ],
            [
                "capacityValue",
                "uint32_t"
            ],
            [
                "capacityWarning",
                "uint32_t"
            ],
            [
                "capacityCritical",
                "uint32_t"
            ],
            [
                "capacityUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_BATTERY_CONFIG": {
        "code": 8197,
        "direction": 1,
        "struct": "<HBBHHHHHHIIIB",
        "byte_count": 29,
        "payload": [
            [
                "vbatScale",
                "uint16_t"
            ],
            [
                "vbatSource",
                "uint8_t"
            ],
            [
                "cellCount",
                "uint8_t"
            ],
            [
                "vbatCellDetect",
                "uint16_t"
            ],
            [
                "vbatMinCell",
                "uint16_t"
            ],
            [
                "vbatMaxCell",
                "uint16_t"
            ],
            [
                "vbatWarningCell",
                "uint16_t"
            ],
            [
                "currentOffset",
                "uint16_t"
            ],
            [
                "currentScale",
                "uint16_t"
            ],
            [
                "capacityValue",
                "uint32_t"
            ],
            [
                "capacityWarning",
                "uint32_t"
            ],
            [
                "capacityCritical",
                "uint32_t"
            ],
            [
                "capacityUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_BATTERY_CONFIG": {
        "code": 8198,
        "direction": 0,
        "struct": "<HBBHHHHHHIIIB",
        "byte_count": 29,
        "payload": [
            [
                "vbatScale",
                "uint16_t"
            ],
            [
                "vbatSource",
                "uint8_t"
            ],
            [
                "cellCount",
                "uint8_t"
            ],
            [
                "vbatCellDetect",
                "uint16_t"
            ],
            [
                "vbatMinCell",
                "uint16_t"
            ],
            [
                "vbatMaxCell",
                "uint16_t"
            ],
            [
                "vbatWarningCell",
                "uint16_t"
            ],
            [
                "currentOffset",
                "uint16_t"
            ],
            [
                "currentScale",
                "uint16_t"
            ],
            [
                "capacityValue",
                "uint32_t"
            ],
            [
                "capacityWarning",
                "uint32_t"
            ],
            [
                "capacityCritical",
                "uint32_t"
            ],
            [
                "capacityUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_RATE_PROFILE": {
        "code": 8199,
        "direction": 1,
        "struct": "<BBBHBBBBBBBBBB",
        "byte_count": 15,
        "payload": [
            [
                "throttleMid",
                "uint8_t"
            ],
            [
                "throttleExpo",
                "uint8_t"
            ],
            [
                "dynamicThrottlePID",
                "uint8_t"
            ],
            [
                "tpaBreakpoint",
                "uint16_t"
            ],
            [
                "stabRcExpo",
                "uint8_t"
            ],
            [
                "stabRcYawExpo",
                "uint8_t"
            ],
            [
                "stabRollRate",
                "uint8_t"
            ],
            [
                "stabPitchRate",
                "uint8_t"
            ],
            [
                "stabYawRate",
                "uint8_t"
            ],
            [
                "manualRcExpo",
                "uint8_t"
            ],
            [
                "manualRcYawExpo",
                "uint8_t"
            ],
            [
                "manualRollRate",
                "uint8_t"
            ],
            [
                "manualPitchRate",
                "uint8_t"
            ],
            [
                "manualYawRate",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_RATE_PROFILE": {
        "code": 8200,
        "direction": 0,
        "struct": "<BBBHBBBBBBBBBB",
        "byte_count": 15,
        "payload": [
            [
                "throttleMid",
                "uint8_t"
            ],
            [
                "throttleExpo",
                "uint8_t"
            ],
            [
                "dynamicThrottlePID",
                "uint8_t"
            ],
            [
                "tpaBreakpoint",
                "uint16_t"
            ],
            [
                "stabRcExpo",
                "uint8_t"
            ],
            [
                "stabRcYawExpo",
                "uint8_t"
            ],
            [
                "stabRollRate",
                "uint8_t"
            ],
            [
                "stabPitchRate",
                "uint8_t"
            ],
            [
                "stabYawRate",
                "uint8_t"
            ],
            [
                "manualRcExpo",
                "uint8_t"
            ],
            [
                "manualRcYawExpo",
                "uint8_t"
            ],
            [
                "manualRollRate",
                "uint8_t"
            ],
            [
                "manualPitchRate",
                "uint8_t"
            ],
            [
                "manualYawRate",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_AIR_SPEED": {
        "code": 8201,
        "direction": 1,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "airspeed",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_OUTPUT_MAPPING": {
        "code": 8202,
        "direction": 1,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "usageFlags",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_MC_BRAKING": {
        "code": 8203,
        "direction": 1,
        "struct": "<HHHBHHHB",
        "byte_count": 14,
        "payload": [
            [
                "brakingSpeedThreshold",
                "uint16_t"
            ],
            [
                "brakingDisengageSpeed",
                "uint16_t"
            ],
            [
                "brakingTimeout",
                "uint16_t"
            ],
            [
                "brakingBoostFactor",
                "uint8_t"
            ],
            [
                "brakingBoostTimeout",
                "uint16_t"
            ],
            [
                "brakingBoostSpeedThreshold",
                "uint16_t"
            ],
            [
                "brakingBoostDisengageSpeed",
                "uint16_t"
            ],
            [
                "brakingBankAngle",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_MC_BRAKING": {
        "code": 8204,
        "direction": 0,
        "struct": "<HHHBHHHB",
        "byte_count": 14,
        "payload": [
            [
                "brakingSpeedThreshold",
                "uint16_t"
            ],
            [
                "brakingDisengageSpeed",
                "uint16_t"
            ],
            [
                "brakingTimeout",
                "uint16_t"
            ],
            [
                "brakingBoostFactor",
                "uint8_t"
            ],
            [
                "brakingBoostTimeout",
                "uint16_t"
            ],
            [
                "brakingBoostSpeedThreshold",
                "uint16_t"
            ],
            [
                "brakingBoostDisengageSpeed",
                "uint16_t"
            ],
            [
                "brakingBankAngle",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OUTPUT_MAPPING_EXT": {
        "code": 8205,
        "direction": 1,
        "struct": "<BB",
        "byte_count": 2,
        "payload": [
            [
                "timerId",
                "uint8_t"
            ],
            [
                "usageFlags",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_TIMER_OUTPUT_MODE": {
        "code": 8206,
        "direction": 2,
        "struct": "<BBBBB",
        "byte_count": 5,
        "payload": [
            [
                "timerIndex",
                "uint8_t"
            ],
            [
                "timerIndex",
                "uint8_t"
            ],
            [
                "outputMode",
                "uint8_t"
            ],
            [
                "timerIndex",
                "uint8_t"
            ],
            [
                "outputMode",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_TIMER_OUTPUT_MODE": {
        "code": 8207,
        "direction": 0,
        "struct": "<BB",
        "byte_count": 2,
        "payload": [
            [
                "timerIndex",
                "uint8_t"
            ],
            [
                "outputMode",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OUTPUT_MAPPING_EXT2": {
        "code": 8461,
        "direction": 1,
        "struct": "<BIB",
        "byte_count": 6,
        "payload": [
            [
                "timerId",
                "uint8_t"
            ],
            [
                "usageFlags",
                "uint32_t"
            ],
            [
                "pinLabel",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_MIXER": {
        "code": 8208,
        "direction": 1,
        "struct": "<BBBBBHBB",
        "byte_count": 9,
        "payload": [
            [
                "motorDirectionInverted",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "motorStopOnLow",
                "uint8_t"
            ],
            [
                "platformType",
                "uint8_t"
            ],
            [
                "hasFlaps",
                "uint8_t"
            ],
            [
                "appliedMixerPreset",
                "uint16_t"
            ],
            [
                "maxMotors",
                "uint8_t"
            ],
            [
                "maxServos",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_MIXER": {
        "code": 8209,
        "direction": 0,
        "struct": "<BBBBBHBB",
        "byte_count": 9,
        "payload": [
            [
                "motorDirectionInverted",
                "uint8_t"
            ],
            [
                "reserved1",
                "uint8_t"
            ],
            [
                "motorStopOnLow",
                "uint8_t"
            ],
            [
                "platformType",
                "uint8_t"
            ],
            [
                "hasFlaps",
                "uint8_t"
            ],
            [
                "appliedMixerPreset",
                "uint16_t"
            ],
            [
                "maxMotors",
                "uint8_t"
            ],
            [
                "maxServos",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_LAYOUTS": {
        "code": 8210,
        "direction": 2,
        "struct": "<BBHBB<?>H",
        "byte_count": 8,
        "payload": [
            [
                "layoutIndex",
                "uint8_t"
            ],
            [
                "layoutIndex",
                "uint8_t"
            ],
            [
                "itemIndex",
                "uint16_t"
            ],
            [
                "layoutCount",
                "uint8_t"
            ],
            [
                "itemCount",
                "uint8_t"
            ],
            [
                "Packed X/Y positions for all items in the requested layout.",
                "*   **Reply Payload (Get Item):**"
            ],
            [
                "itemPosition",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_SET_LAYOUT_ITEM": {
        "code": 8211,
        "direction": 0,
        "struct": "<BBH",
        "byte_count": 4,
        "payload": [
            [
                "layoutIndex",
                "uint8_t"
            ],
            [
                "itemIndex",
                "uint8_t"
            ],
            [
                "itemPosition",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_ALARMS": {
        "code": 8212,
        "direction": 1,
        "struct": "<BHHHHHhhBHHHHHH",
        "byte_count": 28,
        "payload": [
            [
                "rssiAlarm",
                "uint8_t"
            ],
            [
                "timerAlarm",
                "uint16_t"
            ],
            [
                "altAlarm",
                "uint16_t"
            ],
            [
                "distAlarm",
                "uint16_t"
            ],
            [
                "negAltAlarm",
                "uint16_t"
            ],
            [
                "gForceAlarm",
                "uint16_t"
            ],
            [
                "gForceAxisMinAlarm",
                "int16_t"
            ],
            [
                "gForceAxisMaxAlarm",
                "int16_t"
            ],
            [
                "currentAlarm",
                "uint8_t"
            ],
            [
                "imuTempMinAlarm",
                "uint16_t"
            ],
            [
                "imuTempMaxAlarm",
                "uint16_t"
            ],
            [
                "baroTempMinAlarm",
                "uint16_t"
            ],
            [
                "baroTempMaxAlarm",
                "uint16_t"
            ],
            [
                "adsbWarnDistance",
                "uint16_t"
            ],
            [
                "adsbAlertDistance",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_SET_ALARMS": {
        "code": 8213,
        "direction": 0,
        "struct": "<BHHHHHhhBHHHH",
        "byte_count": 24,
        "payload": [
            [
                "rssiAlarm",
                "uint8_t"
            ],
            [
                "timerAlarm",
                "uint16_t"
            ],
            [
                "altAlarm",
                "uint16_t"
            ],
            [
                "distAlarm",
                "uint16_t"
            ],
            [
                "negAltAlarm",
                "uint16_t"
            ],
            [
                "gForceAlarm",
                "uint16_t"
            ],
            [
                "gForceAxisMinAlarm",
                "int16_t"
            ],
            [
                "gForceAxisMaxAlarm",
                "int16_t"
            ],
            [
                "currentAlarm",
                "uint8_t"
            ],
            [
                "imuTempMinAlarm",
                "uint16_t"
            ],
            [
                "imuTempMaxAlarm",
                "uint16_t"
            ],
            [
                "baroTempMinAlarm",
                "uint16_t"
            ],
            [
                "baroTempMaxAlarm",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_PREFERENCES": {
        "code": 8214,
        "direction": 1,
        "struct": "<BBBBBBBBB",
        "byte_count": 9,
        "payload": [
            [
                "videoSystem",
                "uint8_t"
            ],
            [
                "mainVoltageDecimals",
                "uint8_t"
            ],
            [
                "ahiReverseRoll",
                "uint8_t"
            ],
            [
                "crosshairsStyle",
                "uint8_t"
            ],
            [
                "leftSidebarScroll",
                "uint8_t"
            ],
            [
                "rightSidebarScroll",
                "uint8_t"
            ],
            [
                "sidebarScrollArrows",
                "uint8_t"
            ],
            [
                "units",
                "uint8_t"
            ],
            [
                "statsEnergyUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OSD_SET_PREFERENCES": {
        "code": 8215,
        "direction": 0,
        "struct": "<BBBBBBBBB",
        "byte_count": 9,
        "payload": [
            [
                "videoSystem",
                "uint8_t"
            ],
            [
                "mainVoltageDecimals",
                "uint8_t"
            ],
            [
                "ahiReverseRoll",
                "uint8_t"
            ],
            [
                "crosshairsStyle",
                "uint8_t"
            ],
            [
                "leftSidebarScroll",
                "uint8_t"
            ],
            [
                "rightSidebarScroll",
                "uint8_t"
            ],
            [
                "sidebarScrollArrows",
                "uint8_t"
            ],
            [
                "units",
                "uint8_t"
            ],
            [
                "statsEnergyUnit",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SELECT_BATTERY_PROFILE": {
        "code": 8216,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "batteryProfileIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_DEBUG": {
        "code": 8217,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_BLACKBOX_CONFIG": {
        "code": 8218,
        "direction": 1,
        "struct": "<BBHHI",
        "byte_count": 10,
        "payload": [
            [
                "blackboxSupported",
                "uint8_t"
            ],
            [
                "blackboxDevice",
                "uint8_t"
            ],
            [
                "blackboxRateNum",
                "uint16_t"
            ],
            [
                "blackboxRateDenom",
                "uint16_t"
            ],
            [
                "blackboxIncludeFlags",
                "uint32_t"
            ]
        ]
    },
    "MSP2_SET_BLACKBOX_CONFIG": {
        "code": 8219,
        "direction": 0,
        "struct": "<BHHI",
        "byte_count": 9,
        "payload": [
            [
                "blackboxDevice",
                "uint8_t"
            ],
            [
                "blackboxRateNum",
                "uint16_t"
            ],
            [
                "blackboxRateDenom",
                "uint16_t"
            ],
            [
                "blackboxIncludeFlags",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_TEMP_SENSOR_CONFIG": {
        "code": 8220,
        "direction": 1,
        "struct": "<BQHHB",
        "byte_count": 14,
        "payload": [
            [
                "type",
                "uint8_t"
            ],
            [
                "address",
                "uint64_t"
            ],
            [
                "alarmMin",
                "uint16_t"
            ],
            [
                "alarmMax",
                "uint16_t"
            ],
            [
                "osdSymbol",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_TEMP_SENSOR_CONFIG": {
        "code": 8221,
        "direction": 0,
        "struct": "<BQHHB<?>",
        "byte_count": 14,
        "payload": [
            [
                "type",
                "uint8_t"
            ],
            [
                "address",
                "uint64_t"
            ],
            [
                "alarmMin",
                "uint16_t"
            ],
            [
                "alarmMax",
                "uint16_t"
            ],
            [
                "osdSymbol",
                "uint8_t"
            ],
            [
                "char[TEMPERATURE_LABEL_LEN]",
                "TEMPERATURE_LABEL_LEN"
            ]
        ]
    },
    "MSP2_INAV_TEMPERATURES": {
        "code": 8222,
        "direction": 1,
        "struct": "<h",
        "byte_count": 2,
        "payload": [
            [
                "temperature",
                "int16_t"
            ]
        ]
    },
    "MSP_SIMULATOR": {
        "code": 8223,
        "direction": 2,
        "struct": "<HHHHBIhhhBBBBBs",
        "byte_count": 25,
        "payload": [
            [
                "stabilizedRoll",
                "uint16_t"
            ],
            [
                "stabilizedPitch",
                "uint16_t"
            ],
            [
                "stabilizedYaw",
                "uint16_t"
            ],
            [
                "stabilizedThrottle",
                "uint16_t"
            ],
            [
                "debugFlags",
                "uint8_t"
            ],
            [
                "debugValue",
                "uint32_t"
            ],
            [
                "attitudeRoll",
                "int16_t"
            ],
            [
                "attitudePitch",
                "int16_t"
            ],
            [
                "attitudeYaw",
                "int16_t"
            ],
            [
                "osdHeader",
                "uint8_t"
            ],
            [
                "osdRows",
                "uint8_t"
            ],
            [
                "osdCols",
                "uint8_t"
            ],
            [
                "osdStartY",
                "uint8_t"
            ],
            [
                "osdStartX",
                "uint8_t"
            ],
            [
                "osdRleData",
                "uint8_t[]"
            ]
        ]
    },
    "MSP2_INAV_SERVO_MIXER": {
        "code": 8224,
        "direction": 1,
        "struct": "<BBHBBBBHBB",
        "byte_count": 12,
        "payload": [
            [
                "targetChannel",
                "uint8_t"
            ],
            [
                "inputSource",
                "uint8_t"
            ],
            [
                "rate",
                "uint16_t"
            ],
            [
                "speed",
                "uint8_t"
            ],
            [
                "conditionId",
                "uint8_t"
            ],
            [
                "targetChannel",
                "uint8_t"
            ],
            [
                "inputSource",
                "uint8_t"
            ],
            [
                "rate",
                "uint16_t"
            ],
            [
                "speed",
                "uint8_t"
            ],
            [
                "conditionId",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_SERVO_MIXER": {
        "code": 8225,
        "direction": 0,
        "struct": "<BBBHBB",
        "byte_count": 7,
        "payload": [
            [
                "ruleIndex",
                "uint8_t"
            ],
            [
                "targetChannel",
                "uint8_t"
            ],
            [
                "inputSource",
                "uint8_t"
            ],
            [
                "rate",
                "uint16_t"
            ],
            [
                "speed",
                "uint8_t"
            ],
            [
                "conditionId",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_LOGIC_CONDITIONS": {
        "code": 8226,
        "direction": 1,
        "struct": "<BBBBIBIB",
        "byte_count": 14,
        "payload": [
            [
                "enabled",
                "uint8_t"
            ],
            [
                "activatorId",
                "uint8_t"
            ],
            [
                "operation",
                "uint8_t"
            ],
            [
                "operandAType",
                "uint8_t"
            ],
            [
                "operandAValue",
                "uint32_t"
            ],
            [
                "operandBType",
                "uint8_t"
            ],
            [
                "operandBValue",
                "uint32_t"
            ],
            [
                "flags",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_LOGIC_CONDITIONS": {
        "code": 8227,
        "direction": 0,
        "struct": "<BBBBBIBIB",
        "byte_count": 15,
        "payload": [
            [
                "conditionIndex",
                "uint8_t"
            ],
            [
                "enabled",
                "uint8_t"
            ],
            [
                "activatorId",
                "uint8_t"
            ],
            [
                "operation",
                "uint8_t"
            ],
            [
                "operandAType",
                "uint8_t"
            ],
            [
                "operandAValue",
                "uint32_t"
            ],
            [
                "operandBType",
                "uint8_t"
            ],
            [
                "operandBValue",
                "uint32_t"
            ],
            [
                "flags",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_LOGIC_CONDITIONS_STATUS": {
        "code": 8230,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_GVAR_STATUS": {
        "code": 8231,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_PROGRAMMING_PID": {
        "code": 8232,
        "direction": 1,
        "struct": "<BBIBIHHHH",
        "byte_count": 19,
        "payload": [
            [
                "enabled",
                "uint8_t"
            ],
            [
                "setpointType",
                "uint8_t"
            ],
            [
                "setpointValue",
                "uint32_t"
            ],
            [
                "measurementType",
                "uint8_t"
            ],
            [
                "measurementValue",
                "uint32_t"
            ],
            [
                "gainP",
                "uint16_t"
            ],
            [
                "gainI",
                "uint16_t"
            ],
            [
                "gainD",
                "uint16_t"
            ],
            [
                "gainFF",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_SET_PROGRAMMING_PID": {
        "code": 8233,
        "direction": 0,
        "struct": "<BBBIBIHHHH",
        "byte_count": 20,
        "payload": [
            [
                "pidIndex",
                "uint8_t"
            ],
            [
                "enabled",
                "uint8_t"
            ],
            [
                "setpointType",
                "uint8_t"
            ],
            [
                "setpointValue",
                "uint32_t"
            ],
            [
                "measurementType",
                "uint8_t"
            ],
            [
                "measurementValue",
                "uint32_t"
            ],
            [
                "gainP",
                "uint16_t"
            ],
            [
                "gainI",
                "uint16_t"
            ],
            [
                "gainD",
                "uint16_t"
            ],
            [
                "gainFF",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_PROGRAMMING_PID_STATUS": {
        "code": 8234,
        "direction": 1,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_PID": {
        "code": 8240,
        "direction": 1,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "P",
                "uint8_t"
            ],
            [
                "I",
                "uint8_t"
            ],
            [
                "D",
                "uint8_t"
            ],
            [
                "FF",
                "uint8_t"
            ]
        ]
    },
    "MSP2_SET_PID": {
        "code": 8241,
        "direction": 0,
        "struct": "<BBBB",
        "byte_count": 4,
        "payload": [
            [
                "P",
                "uint8_t"
            ],
            [
                "I",
                "uint8_t"
            ],
            [
                "D",
                "uint8_t"
            ],
            [
                "FF",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_OPFLOW_CALIBRATION": {
        "code": 8242,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_FWUPDT_PREPARE": {
        "code": 8243,
        "direction": 0,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "firmwareSize",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_FWUPDT_STORE": {
        "code": 8244,
        "direction": 0,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "firmwareChunk",
                "uint8_t[]"
            ]
        ]
    },
    "MSP2_INAV_FWUPDT_EXEC": {
        "code": 8245,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "updateType",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_FWUPDT_ROLLBACK_PREPARE": {
        "code": 8246,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_FWUPDT_ROLLBACK_EXEC": {
        "code": 8247,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_SAFEHOME": {
        "code": 8248,
        "direction": 2,
        "struct": "<BBBII",
        "byte_count": 11,
        "payload": [
            [
                "safehomeIndex",
                "uint8_t"
            ],
            [
                "safehomeIndex",
                "uint8_t"
            ],
            [
                "enabled",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_SET_SAFEHOME": {
        "code": 8249,
        "direction": 0,
        "struct": "<BBII",
        "byte_count": 10,
        "payload": [
            [
                "safehomeIndex",
                "uint8_t"
            ],
            [
                "enabled",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_MISC2": {
        "code": 8250,
        "direction": 1,
        "struct": "<IIBB",
        "byte_count": 10,
        "payload": [
            [
                "uptimeSeconds",
                "uint32_t"
            ],
            [
                "flightTimeSeconds",
                "uint32_t"
            ],
            [
                "throttlePercent",
                "uint8_t"
            ],
            [
                "autoThrottleFlag",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_LOGIC_CONDITIONS_SINGLE": {
        "code": 8251,
        "direction": 2,
        "struct": "<BBBBIBIB",
        "byte_count": 14,
        "payload": [
            [
                "enabled",
                "uint8_t"
            ],
            [
                "activatorId",
                "uint8_t"
            ],
            [
                "operation",
                "uint8_t"
            ],
            [
                "operandAType",
                "uint8_t"
            ],
            [
                "operandAValue",
                "uint32_t"
            ],
            [
                "operandBType",
                "uint8_t"
            ],
            [
                "operandBValue",
                "uint32_t"
            ],
            [
                "flags",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_ESC_RPM": {
        "code": 8256,
        "direction": 1,
        "struct": "<I",
        "byte_count": 4,
        "payload": [
            [
                "escRpm",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_ESC_TELEM": {
        "code": 8257,
        "direction": 1,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "motorCount",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_LED_STRIP_CONFIG_EX": {
        "code": 8264,
        "direction": 1,
        "struct": "<<?>",
        "byte_count": 1,
        "payload": [
            [
                "ledConfig_t",
                "sizeof(ledConfig_t)"
            ]
        ]
    },
    "MSP2_INAV_SET_LED_STRIP_CONFIG_EX": {
        "code": 8265,
        "direction": 0,
        "struct": "<B<?>",
        "byte_count": 1,
        "payload": [
            [
                "ledIndex",
                "uint8_t"
            ],
            [
                "ledConfig_t",
                "sizeof(ledConfig_t)"
            ]
        ]
    },
    "MSP2_INAV_FW_APPROACH": {
        "code": 8266,
        "direction": 2,
        "struct": "<BBIIBhhB",
        "byte_count": 16,
        "payload": [
            [
                "approachIndex",
                "uint8_t"
            ],
            [
                "approachIndex",
                "uint8_t"
            ],
            [
                "approachAlt",
                "uint32_t"
            ],
            [
                "landAlt",
                "uint32_t"
            ],
            [
                "approachDirection",
                "uint8_t"
            ],
            [
                "landHeading1",
                "int16_t"
            ],
            [
                "landHeading2",
                "int16_t"
            ],
            [
                "isSeaLevelRef",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_FW_APPROACH": {
        "code": 8267,
        "direction": 0,
        "struct": "<BIIBhhB",
        "byte_count": 15,
        "payload": [
            [
                "approachIndex",
                "uint8_t"
            ],
            [
                "approachAlt",
                "uint32_t"
            ],
            [
                "landAlt",
                "uint32_t"
            ],
            [
                "approachDirection",
                "uint8_t"
            ],
            [
                "landHeading1",
                "int16_t"
            ],
            [
                "landHeading2",
                "int16_t"
            ],
            [
                "isSeaLevelRef",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_GPS_UBLOX_COMMAND": {
        "code": 8272,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_INAV_RATE_DYNAMICS": {
        "code": 8288,
        "direction": 1,
        "struct": "<BBBBBB",
        "byte_count": 6,
        "payload": [
            [
                "sensitivityCenter",
                "uint8_t"
            ],
            [
                "sensitivityEnd",
                "uint8_t"
            ],
            [
                "correctionCenter",
                "uint8_t"
            ],
            [
                "correctionEnd",
                "uint8_t"
            ],
            [
                "weightCenter",
                "uint8_t"
            ],
            [
                "weightEnd",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_RATE_DYNAMICS": {
        "code": 8289,
        "direction": 0,
        "struct": "<BBBBBB",
        "byte_count": 6,
        "payload": [
            [
                "sensitivityCenter",
                "uint8_t"
            ],
            [
                "sensitivityEnd",
                "uint8_t"
            ],
            [
                "correctionCenter",
                "uint8_t"
            ],
            [
                "correctionEnd",
                "uint8_t"
            ],
            [
                "weightCenter",
                "uint8_t"
            ],
            [
                "weightEnd",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_EZ_TUNE": {
        "code": 8304,
        "direction": 1,
        "struct": "<BHBBBBBBBB",
        "byte_count": 11,
        "payload": [
            [
                "enabled",
                "uint8_t"
            ],
            [
                "filterHz",
                "uint16_t"
            ],
            [
                "axisRatio",
                "uint8_t"
            ],
            [
                "response",
                "uint8_t"
            ],
            [
                "damping",
                "uint8_t"
            ],
            [
                "stability",
                "uint8_t"
            ],
            [
                "aggressiveness",
                "uint8_t"
            ],
            [
                "rate",
                "uint8_t"
            ],
            [
                "expo",
                "uint8_t"
            ],
            [
                "snappiness",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_EZ_TUNE_SET": {
        "code": 8305,
        "direction": 0,
        "struct": "<BHBBBBBBBB",
        "byte_count": 11,
        "payload": [
            [
                "enabled",
                "uint8_t"
            ],
            [
                "filterHz",
                "uint16_t"
            ],
            [
                "axisRatio",
                "uint8_t"
            ],
            [
                "response",
                "uint8_t"
            ],
            [
                "damping",
                "uint8_t"
            ],
            [
                "stability",
                "uint8_t"
            ],
            [
                "aggressiveness",
                "uint8_t"
            ],
            [
                "rate",
                "uint8_t"
            ],
            [
                "expo",
                "uint8_t"
            ],
            [
                "snappiness",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SELECT_MIXER_PROFILE": {
        "code": 8320,
        "direction": 0,
        "struct": "<B",
        "byte_count": 1,
        "payload": [
            [
                "mixerProfileIndex",
                "uint8_t"
            ]
        ]
    },
    "MSP2_ADSB_VEHICLE_LIST": {
        "code": 8336,
        "direction": 1,
        "struct": "<BBII<?><?>IIIIHBBB",
        "byte_count": 31,
        "payload": [
            [
                "maxVehicles",
                "uint8_t"
            ],
            [
                "callsignLength",
                "uint8_t"
            ],
            [
                "totalVehicleMsgs",
                "uint32_t"
            ],
            [
                "totalHeartbeatMsgs",
                "uint32_t"
            ],
            [
                "",
                ""
            ],
            [
                "char[ADSB_CALL_SIGN_MAX_LENGTH]",
                "ADSB_CALL_SIGN_MAX_LENGTH"
            ],
            [
                "icao",
                "uint32_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "altitude",
                "uint32_t"
            ],
            [
                "heading",
                "uint16_t"
            ],
            [
                "tslc",
                "uint8_t"
            ],
            [
                "emitterType",
                "uint8_t"
            ],
            [
                "ttl",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_CUSTOM_OSD_ELEMENTS": {
        "code": 8448,
        "direction": 1,
        "struct": "<BBB",
        "byte_count": 3,
        "payload": [
            [
                "maxElements",
                "uint8_t"
            ],
            [
                "maxTextLength",
                "uint8_t"
            ],
            [
                "maxParts",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_CUSTOM_OSD_ELEMENT": {
        "code": 8449,
        "direction": 2,
        "struct": "<<?>BHBH",
        "byte_count": 7,
        "payload": [
            [
                "",
                ""
            ],
            [
                "partType",
                "uint8_t"
            ],
            [
                "partValue",
                "uint16_t"
            ],
            [
                "visibilityType",
                "uint8_t"
            ],
            [
                "visibilityValue",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_SET_CUSTOM_OSD_ELEMENTS": {
        "code": 8450,
        "direction": 0,
        "struct": "<B<?>BHBH",
        "byte_count": 7,
        "payload": [
            [
                "elementIndex",
                "uint8_t"
            ],
            [
                "",
                ""
            ],
            [
                "partType",
                "uint8_t"
            ],
            [
                "partValue",
                "uint16_t"
            ],
            [
                "visibilityType",
                "uint8_t"
            ],
            [
                "visibilityValue",
                "uint16_t"
            ]
        ]
    },
    "MSP2_INAV_SERVO_CONFIG": {
        "code": 8704,
        "direction": 1,
        "struct": "<HHHB",
        "byte_count": 7,
        "payload": [
            [
                "min",
                "uint16_t"
            ],
            [
                "max",
                "uint16_t"
            ],
            [
                "middle",
                "uint16_t"
            ],
            [
                "rate",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_SERVO_CONFIG": {
        "code": 8705,
        "direction": 0,
        "struct": "<BHHHB",
        "byte_count": 8,
        "payload": [
            [
                "servoIndex",
                "uint8_t"
            ],
            [
                "min",
                "uint16_t"
            ],
            [
                "max",
                "uint16_t"
            ],
            [
                "middle",
                "uint16_t"
            ],
            [
                "rate",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_GEOZONE": {
        "code": 8720,
        "direction": 2,
        "struct": "<BBBBIIBBB",
        "byte_count": 15,
        "payload": [
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "type",
                "uint8_t"
            ],
            [
                "shape",
                "uint8_t"
            ],
            [
                "minAltitude",
                "uint32_t"
            ],
            [
                "maxAltitude",
                "uint32_t"
            ],
            [
                "isSeaLevelRef",
                "uint8_t"
            ],
            [
                "fenceAction",
                "uint8_t"
            ],
            [
                "vertexCount",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_SET_GEOZONE": {
        "code": 8721,
        "direction": 0,
        "struct": "<BBBIIBBB",
        "byte_count": 14,
        "payload": [
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "type",
                "uint8_t"
            ],
            [
                "shape",
                "uint8_t"
            ],
            [
                "minAltitude",
                "uint32_t"
            ],
            [
                "maxAltitude",
                "uint32_t"
            ],
            [
                "isSeaLevelRef",
                "uint8_t"
            ],
            [
                "fenceAction",
                "uint8_t"
            ],
            [
                "vertexCount",
                "uint8_t"
            ]
        ]
    },
    "MSP2_INAV_GEOZONE_VERTEX": {
        "code": 8722,
        "direction": 2,
        "struct": "<BBBBIIBBIII",
        "byte_count": 26,
        "payload": [
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "vertexId",
                "uint8_t"
            ],
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "vertexId",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "vertexId",
                "uint8_t"
            ],
            [
                "centerLatitude",
                "uint32_t"
            ],
            [
                "centerLongitude",
                "uint32_t"
            ],
            [
                "radius",
                "uint32_t"
            ]
        ]
    },
    "MSP2_INAV_SET_GEOZONE_VERTEX": {
        "code": 8723,
        "direction": 0,
        "struct": "<BBIIBBIII",
        "byte_count": 24,
        "payload": [
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "vertexId",
                "uint8_t"
            ],
            [
                "latitude",
                "uint32_t"
            ],
            [
                "longitude",
                "uint32_t"
            ],
            [
                "geozoneIndex",
                "uint8_t"
            ],
            [
                "vertexId",
                "uint8_t"
            ],
            [
                "centerLatitude",
                "uint32_t"
            ],
            [
                "centerLongitude",
                "uint32_t"
            ],
            [
                "radius",
                "uint32_t"
            ]
        ]
    },
    "MSP2_BETAFLIGHT_BIND": {
        "code": 12288,
        "direction": 0,
        "struct": "<",
        "byte_count": 0,
        "payload": []
    },
    "MSP2_SENSOR_RANGEFINDER": {
        "code": 7937,
        "direction": 0,
        "struct": "<Bi",
        "byte_count": 5,
        "payload": [
            [
                "quality",
                "uint8_t"
            ],
            [
                "distanceMm",
                "int32_t"
            ]
        ]
    },
    "MSP2_SENSOR_OPTIC_FLOW": {
        "code": 7938,
        "direction": 0,
        "struct": "<Bii",
        "byte_count": 9,
        "payload": [
            [
                "quality",
                "uint8_t"
            ],
            [
                "motionX",
                "int32_t"
            ],
            [
                "motionY",
                "int32_t"
            ]
        ]
    },
    "MSP2_SENSOR_GPS": {
        "code": 7939,
        "direction": 0,
        "struct": "<BHIBBHHHHiiiiiiHHHBBBBB",
        "byte_count": 52,
        "payload": [
            [
                "instance",
                "uint8_t"
            ],
            [
                "gpsWeek",
                "uint16_t"
            ],
            [
                "msTOW",
                "uint32_t"
            ],
            [
                "fixType",
                "uint8_t"
            ],
            [
                "satellitesInView",
                "uint8_t"
            ],
            [
                "hPosAccuracy",
                "uint16_t"
            ],
            [
                "vPosAccuracy",
                "uint16_t"
            ],
            [
                "hVelAccuracy",
                "uint16_t"
            ],
            [
                "hdop",
                "uint16_t"
            ],
            [
                "longitude",
                "int32_t"
            ],
            [
                "latitude",
                "int32_t"
            ],
            [
                "mslAltitude",
                "int32_t"
            ],
            [
                "nedVelNorth",
                "int32_t"
            ],
            [
                "nedVelEast",
                "int32_t"
            ],
            [
                "nedVelDown",
                "int32_t"
            ],
            [
                "groundCourse",
                "uint16_t"
            ],
            [
                "trueYaw",
                "uint16_t"
            ],
            [
                "year",
                "uint16_t"
            ],
            [
                "month",
                "uint8_t"
            ],
            [
                "day",
                "uint8_t"
            ],
            [
                "hour",
                "uint8_t"
            ],
            [
                "min",
                "uint8_t"
            ],
            [
                "sec",
                "uint8_t"
            ]
        ]
    },
    "MSP2_SENSOR_COMPASS": {
        "code": 7940,
        "direction": 0,
        "struct": "<BIhhh",
        "byte_count": 11,
        "payload": [
            [
                "instance",
                "uint8_t"
            ],
            [
                "timeMs",
                "uint32_t"
            ],
            [
                "magX",
                "int16_t"
            ],
            [
                "magY",
                "int16_t"
            ],
            [
                "magZ",
                "int16_t"
            ]
        ]
    },
    "MSP2_SENSOR_BAROMETER": {
        "code": 7941,
        "direction": 0,
        "struct": "<BIfh",
        "byte_count": 11,
        "payload": [
            [
                "instance",
                "uint8_t"
            ],
            [
                "timeMs",
                "uint32_t"
            ],
            [
                "pressurePa",
                "float"
            ],
            [
                "temp",
                "int16_t"
            ]
        ]
    },
    "MSP2_SENSOR_AIRSPEED": {
        "code": 7942,
        "direction": 0,
        "struct": "<BIfh",
        "byte_count": 11,
        "payload": [
            [
                "instance",
                "uint8_t"
            ],
            [
                "timeMs",
                "uint32_t"
            ],
            [
                "diffPressurePa",
                "float"
            ],
            [
                "temp",
                "int16_t"
            ]
        ]
    },
    "MSP2_SENSOR_HEADTRACKER": {
        "code": 7943,
        "direction": 0,
        "struct": "<s",
        "byte_count": 1,
        "payload": [
            [
                "...",
                "Varies"
            ]
        ]
    }
}