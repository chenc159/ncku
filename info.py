

class info:

    # Initialize parameters for drone data
    send_pkt_num = [1,2,3]
    receive_pkt_num = [10]
    header, checksum  = 255, 256
    msgID = {1:128, 2:129, 3:130, 10:190}
    pkt_space = {1: [1,1,1,1,1,4,4,1,2], 2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2], 3: [1,1,1,1,1,4,4,4,4,4,2], 10:[1,1,1,1,1,1]}
    
    # Required message (type and field name)
    msgs =  {
        "ID":                   {"sys": 0, "comp": 1, "comm": 22, "time": 0},   
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






    mode_mapping_acm = {0 : 'STABILIZE', 1 : 'ACRO', 2 : 'ALT_HOLD', 3 : 'AUTO', 4 : 'GUIDED', 5 : 'LOITER', 6 : 'RTL', 7 : 'CIRCLE', 8 : 'POSITION', 9 : 'LAND'}
