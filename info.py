

class info:

    # Initialize parameters for drone data
    # send_pkt_num = [1,2,3]
    # send_pkt_num = [127, 128, 129, 130]
    # receive_pkt_num = [10]
    header, checksum  = 255, 256
    # msgID = {1:128, 2:129, 3:130, 10:190}
    msgID_send = [127, 128, 129, 130]
    msgID_receive = [131, 132, 133]
    pkt_space = {   0: [1,1,1,1,1,4,2],
                    127: [1,1,1,1,1,1,1,1,1,4,2],
                    128: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,2],
                    129: [1,1,1,1,1,1,4,2],
                    130: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2],
                    131: [1,1,1,1,1,4,1,4,2],
                    132: [1,1,1,1,1,1,1,4,4,4,4,4,2],
                    133: [1,1,1,1,1,1,4,2]
                }
    
    # Required message (type and field name)
    msgs =  {
        "ID":                   {"sys": 0, "comp": 1, "comm": 22, "time": 0},   
        "OTHER":                {"sysID": 0, "compID": 1, "commID": 22, "systime": 0},   
        "SYSTEM_TIME":          {"time_unix_usec": 0, "time_boot_ms": 0}, 
        "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
        "SCALED_IMU":           {"xacc": 0, "yacc": 0, "zacc": 0},
        "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
        "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
        "HEARTBEAT":            {"system_status": 99},
        "BATTERY_STATUS":       {"battery_remaining": 0},
        "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
        "STATUSTEXT":           {"severity": 99}
    } #AHRS2, AHRS3
    


    # Initialize packet
    pkt_item = {
        # standard packet
        0: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "OTHER.systime", "checksum"],
        # UAV to GCS
        127: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID", 
            "Mode", "Arm", "HEARTBEAT.system_status", "Failsafe", "OTHER.systime", "checksum"],
        128: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",  
            "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg",
            "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "SCALED_IMU.xacc", "SCALED_IMU.yacc", "SCALED_IMU.zacc", 
            "Dyn_waypt_lat", "Dyn_waypt_lon", "SYSTEM_TIME.time_unix_usec", "OTHER.systime", "checksum"],
        129: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Commandmessage", "OTHER.systime", "checksum"],
        130: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Other_UAV_lat", "Other_UAV_lon", "Other_UAV_alt", "Other_UAV_vx", "Other_UAV_vy", "Other_UAV_vz", "Other_UAV_hdg", "Other_UAV_gpstime", "OTHER.systime", "checksum"],
        # GCS to UAV
        131: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Desired_dist", "Waypt_count", "OTHER.systime", "checksum"],
        132: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Waypt_seqID", "Formation", "Pass_radius", "lat", "lon", "alt","OTHER.systime", "checksum"],
        133: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Mode_Arm", "OTHER.systime", "checksum"]
    }
    pkt_item_old = {
        1: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "ID.time", "SYSTEM_TIME.time_unix_usec", 
            "HEARTBEAT.system_status", "checksum"],
        2: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "ID.time", "SYSTEM_TIME.time_unix_usec", 
            "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg","checksum"],
        3: ["header", "ID.comm", "ID.sys", "ID.comp", "ID.mes", "ID.time", "ATTITUDE.time_boot_ms", "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "checksum"]
    }

    # Used to convert unit (e.g. 1 rad to 57.2958 deg)
    convert = {"ATTITUDE.roll": 57.2958, "ATTITUDE.pitch": 57.2958, "ATTITUDE.yaw": 57.2958} # need to include failsafe later
    # byte format letter for un/pack
    byte_num = {1:'B', 2:'H', 4:'i'}






    mode_mapping_acm = {0 : 'STABILIZE', 1 : 'ACRO', 2 : 'ALT_HOLD', 3 : 'AUTO', 4 : 'GUIDED',
     5 : 'LOITER', 6 : 'RTL', 7 : 'CIRCLE', 8 : 'POSITION', 9 : 'LAND'}

    def system_status(num):
        """ 
        COPIED FROM DRONEKIT __init__.py FILE
        System status (:py:class:`SystemStatus`).

        The status has a ``state`` property with one of the following values:

        * ``UNINIT``: Uninitialized system, state is unknown.
        * ``BOOT``: System is booting up.
        * ``CALIBRATING``: System is calibrating and not flight-ready.
        * ``STANDBY``: System is grounded and on standby. It can be launched any time.
        * ``ACTIVE``: System is active and might be already airborne. Motors are engaged.
        * ``CRITICAL``: System is in a non-normal flight mode. It can however still navigate.
        * ``EMERGENCY``: System is in a non-normal flight mode. It lost control over parts
        or over the whole airframe. It is in mayday and going down.
        * ``POWEROFF``: System just initialized its power-down sequence, will shut down now.
        """
        return {
            0: 'UNINIT', 
            1: 'BOOT', 
            2: 'CALIBRATING', 
            3: 'STANDBY', 
            4: 'ACTIVE', 
            5: 'CRITICAL', 
            6: 'EMERGENCY', 
            7: 'POWEROFF', 
            8: 'TERMINATION',
            255: 'None'
        }.get(num)

    def failsafe_map(num):
        '''
        0~13 check HIGH_LATENCY2.HL_FAILURE_FLAG and https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
        14~17 check STATUSTEXT.severity and https://mavlink.io/en/messages/common.html#MAV_SEVERITY
        255 for None/not getting any messages regarding failsafe
        '''
        return {
            0:    "HL_FAILURE_FLAG_GPS",
            1:    "HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE",
            2:    "HL_FAILURE_FLAG_ABSOLUTE_PRESSURE",
            3:    "HL_FAILURE_FLAG_3D_ACCEL",
            4:    "HL_FAILURE_FLAG_3D_GYRO",
            5:    "HL_FAILURE_FLAG_3D_MAG",
            6:    "HL_FAILURE_FLAG_TERRAIN",
            7:    "HL_FAILURE_FLAG_BATTERY",
            8:    "HL_FAILURE_FLAG_RC_RECEIVER",
            9:    "HL_FAILURE_FLAG_OFFBOARD_LINK",
            10:   "HL_FAILURE_FLAG_ENGINE",
            11:   "HL_FAILURE_FLAG_GEOFENCE",
            12:   "HL_FAILURE_FLAG_ESTIMATOR",
            13:   "HL_FAILURE_FLAG_MISSION",
            14:   "MAV_SEVERITY_EMERGENCY",
            15:   "MAV_SEVERITY_ALERT",
            16:   "MAV_SEVERITY_CRITICAL",
            17:   "MAV_SEVERITY_ERROR",
            255:   "None"
        }.get(num)