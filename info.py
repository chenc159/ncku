
from pymavlink import mavutil
from datetime import datetime
from ctypes import *
from struct import *
import time


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
                    128: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,2],
                    129: [1,1,1,1,1,1,4,2],
                    130: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2],
                    131: [1,1,1,1,1,4,1,4,2],
                    132: [1,1,1,1,1,1,1,1,4,4,4,4,4,2],
                    133: [1,1,1,1,1,1,4,2]
                }


    # Initialize packet
    pkt_item = {
        # standard packet
        0: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "OTHER.systime", "checksum"],
        # UAV to GCS
        127: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID", 
            "Mode", "Arm", "HEARTBEAT.system_status", "Failsafe", "OTHER.systime", "checksum"],
        128: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",  
            "GLOBAL_POSITION_INT.lat", "GLOBAL_POSITION_INT.lon", "GLOBAL_POSITION_INT.alt", 
            "GLOBAL_POSITION_INT.vx", "GLOBAL_POSITION_INT.vy", "GLOBAL_POSITION_INT.vz", "GLOBAL_POSITION_INT.hdg",
            "ATTITUDE.roll", "ATTITUDE.pitch",  "ATTITUDE.yaw", "SCALED_IMU2.xacc", "SCALED_IMU2.yacc", "SCALED_IMU2.zacc", 
            "Dyn_waypt_lat", "Dyn_waypt_lon", "SYSTEM_TIME.time_unix_usec", "OTHER.systime", "checksum"],
        129: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "mission_ack", "OTHER.systime", "checksum"],
        130: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Other_UAV_lat", "Other_UAV_lon", "Other_UAV_alt", "Other_UAV_vx", "Other_UAV_vy", "Other_UAV_vz", "Other_UAV_hdg", "Other_UAV_gpstime", "OTHER.systime", "checksum"],
        # GCS to UAV
        131: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Desired_dist", "Waypt_count", "OTHER.systime", "checksum"],
        132: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Waypt_seqID", "Mission_mode", "Formation", "Pass_radius", "lat", "lon", "alt","OTHER.systime", "checksum"],
        133: ["header", "msgID", "OTHER.sysID", "OTHER.compID", "OTHER.commID",
             "Mode_Arm", "OTHER.systime", "checksum"]
    }

    # gcs to uav: get mission status
    # uav to gcs: send uav mission status, current seq, download mission...

    
    # Required message (type and field name)
    msgs =  {
        "ID":                   {"sys": 0, "comp": 1, "comm": 22, "time": 0},   
        "OTHER":                {"sysID": 0, "compID": 1, "commID": 22, "systime": 0},   
        "SYSTEM_TIME":          {"time_unix_usec": 0, "time_boot_ms": 0}, 
        "ATTITUDE":             {"time_boot_ms": 0, "roll": 0, "pitch": 0, "yaw": 0},
        "SCALED_IMU2":          {"xacc": 0, "yacc": 0, "zacc": 0},
        "GPS_RAW_INT":          {"time_usec": 0, "fix_type": 0, "satellites_visible": 0},
        "GLOBAL_POSITION_INT":  {"time_boot_ms": 0, "lat": 0, "lon": 0, "alt": 0, "vx": 0, "vy": 0, "vz": 0, "hdg": 0},
        "HEARTBEAT":            {"system_status": 99},
        "BATTERY_STATUS":       {"battery_remaining": 0},
        "HIGH_LATENCY2":        {"HL_FAILURE_FLAG": 99},
        "STATUSTEXT":           {"severity": 99}
    } #AHRS2, AHRS3
    

    
    msgID_send_old = [1,2,3]
    pkt_space_old = {
        1: [1,1,1,1,1,4,4,1,2],
        2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,2],
        3: [1,1,1,1,1,4,4,4,4,4,2]
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





    # Mappings

    mode_map_n2s = {0 : 'STABILIZE', 1 : 'ACRO', 2 : 'ALT_HOLD', 3 : 'AUTO', 4 : 'GUIDED',
     5 : 'LOITER', 6 : 'RTL', 7 : 'CIRCLE', 8 : 'POSHOLD', 9 : 'LAND'}

    mode_map_s2n = {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
     'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'POSHOLD': 8, 'LAND': 9}

    # mission_mode_mapping = {0: mavutil.mavlink.MAV_CMD_DO_SET_HOME, 1: mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    #  2:  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3: mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, 4: mavutil.mavlink.MAV_CMD_NAV_LAND, 5: mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH}

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

class uav_info(object):
    def __init__(self, sysID, compID, commID, lat, lon, alt, vx, vy, vz, xgyro, ygyro, zgyro, hdg, mode, gps_time, sys_time):
        # int
        self.sysID = sysID
        self.compID = compID
        self.commID = commID

        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        # self.xacc = xacc
        # self.yacc = yacc
        self.xgyro = xgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.hdg = hdg
        self.mode = mode
        self.gps_time = gps_time
        self.sys_time = sys_time

    def update(self, lat, lon, alt, vx, vy, vz, xgyro, ygyro, zgyro, hdg, mode, gps_time, sys_time):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        # self.xacc = xacc
        # self.yacc = yacc
        self.xgyro = xgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.hdg = hdg
        self.mode = mode
        self.gps_time = gps_time
        self.sys_time = sys_time


class packet00(object):
    def __init__(self, sysID, compID, commID):
        # int
        self.msgID = 0
        self.sysID = sysID
        self.compID = compID
        self.commID = commID

    def packpkt(self):
        return pack('<BBBB', self.msgID, self.sysID, self.compID, self.commID)


'''UAV 2 GCS'''
class packet127(object):
    def __init__(self, sysID, compID, commID, mode, arm, system_status, failsafe):
        # int
        self.msgID = 127
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
        # c_int
        self.mode = mode
        self.arm = arm
        self.system_status = system_status
        self.failsafe = failsafe

    def packpkt(self):
        return pack('<BBBBBBBB', self.msgID, self.sysID, self.compID, self.commID, 
                    self.mode.value, self.arm.value, self.system_status.value, self.failsafe.value)

class packet128(object):
    def __init__(self, sysID, compID, commID, lat, lon, alt, fix, sat_num, vx, vy, vz, hdg, roll, pitch, yaw, xacc, yacc, zacc, gps_time):
        # int
        self.msgID = 128
        self.sysID = sysID
        self.compID = compID
        self.commID = commID 
        # c_int
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.fix = fix 
        self.sat_num = sat_num
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.hdg = hdg
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc
        self.gps_time = gps_time
    def packpkt(self):
        return pack('<BBBBiiiiiiiiiiiiiiii', self.msgID, self.sysID, self.compID, self.commID, 
                    self.lat.value, self.lon.value, self.alt.value, self.fix.value, self.sat_num.value,
                    self.vx.value, self.vy.value, self.vz.value,
                    self.hdg.value, self.roll.value, self.pitch.value, self.yaw.value,
                    self.xacc.value, self.yacc.value, self.zacc.value, self.gps_time.value)

class packet129(object):
    def __init__(self, sysID, compID, commID, command, result):
        # int
        self.msgID = 129
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
        # c_int
        self.command = command
        self.result = result

    def packpkt(self):
        return pack('<BBBBii', self.msgID, self.sysID, self.compID, self.commID,
                     self.command.value, self.result.value)

class packet130(object):
    def __init__(self, sysID, others_sysID, others_commID, others_lat, others_lon, others_alt, others_vx, others_vy, others_vz, others_hdg, others_gps_time):
        # int
        self.msgID = 130
        self.sysID = sysID
        # c_int
        # self.others_sysID = others_sysID
        # self.others_commID = others_commID
        self.others_lat = others_lat
        self.others_lon = others_lon
        self.others_alt = others_alt
        self.others_vx = others_vx
        self.others_vy = others_vy
        self.others_vz = others_vz
        self.others_hdg = others_hdg
    
    def calculated(self, others_sysID, others_commID, relative_dist, relative_ang, others_gps_time):
        self.others_sysID = others_sysID
        self.others_commID = others_commID
        self.relative_dist = int(relative_dist)
        self.relative_ang = int(relative_ang)
        self.others_gps_time = others_gps_time

    def packpkt(self):
        # return pack('<BBBBiiiiiiii', self.msgID, self.sysID, self.others_sysID.value, self.others_commID.value,
        #              self.others_lat.value, self.others_lon.value, self.others_alt.value, 
        #              self.others_vx.value, self.others_vy.value, self.others_vz.value, 
        #              self.others_hdg.value, self.others_gps_time.value)
        # print(self.others_sysID, self.others_commID,
        #              self.relative_dist, self.relative_ang, self.others_gps_time)
        return pack('<BBBBiii', self.msgID, self.sysID, self.others_sysID, self.others_commID,
                     self.relative_dist, self.relative_ang, self.others_gps_time)

class packet136(object):
    def __init__(self, sysID, compID, commID, Dyn_waypt_lat, Dyn_waypt_lon, Dyn_vx, Dyn_vy, Dyn_yaw, Dyn_yawr, waypt_seq):
        # int
        self.msgID = 136
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
        # c_int
        self.Dyn_waypt_lat = Dyn_waypt_lat
        self.Dyn_waypt_lon = Dyn_waypt_lon
        self.Dyn_vx = Dyn_vx
        self.Dyn_vy = Dyn_vy
        self.Dyn_yaw = Dyn_yaw
        self.Dyn_yawr = Dyn_yawr
        self.waypt_seq = waypt_seq

    def packpkt(self):
        return pack('<BBBBiiiiiii', self.msgID, self.sysID, self.compID, self.commID, 
                    self.Dyn_waypt_lat.value, self.Dyn_waypt_lon.value, self.Dyn_vx.value, self.Dyn_vy.value,
                    self.Dyn_yaw.value, self.Dyn_yawr.value, self.waypt_seq.value)

class packet137(object):
    def __init__(self, sysID, compID, commID, servo1, servo2, servo3, servo4):
        # int
        self.msgID = 137
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
        # c_int
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        self.servo4 = servo4

    def packpkt(self):
        return pack('<BBBBiiii', self.msgID, self.sysID, self.compID, self.commID, 
                    self.servo1.value, self.servo2.value, self.servo3.value, self.servo4.value)

class packet138(object):
    def __init__(self, sysID, compID, commID):
        # int
        self.msgID = 138
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
    def save_data(self, seq, cmd, lat, lon, alt):
        self.seq, self.cmd, self.lat, self.lon, self.alt = seq, cmd, lat, lon, alt

    def packpkt(self):
        print(self.seq, self.cmd, self.lat, self.lon, self.alt)
        return pack('<BBBBiiiii', self.msgID, self.sysID, self.compID, self.commID, 
                    self.seq, self.cmd, self.lat, self.lon, self.alt)

'''GCS 2 UAV'''
class packet131(object):
    def __init__(self):
        self.Formation = 0 # 0:mission, 1: triangle, 2: stright line
        self.LF = 0 # 0: leader, 1,2: follower 
        self.Desired_dist = 0 # desire distance between leader and follower
        self.Radius = 0 # leader's turning radius
        self.Angle = 0 # angle for triangle formation
        self.Waypt_num = 0 # total number of waypt for trajectory calculation
        self.Waypt_count = 0 # waypt count will be sent to nano
    def unpackpkt(self, data):
        self.Formation = data[5]
        self.LF = data[6]
        self.Desired_dist = data[7]
        self.Radius = data[8]
        self.Angle = data[9]
        self.Waypt_num = data[10]
        self.Waypt_count = data[11]
        print('Formation: ', self.Formation, 
            '\nLeader/Follower: ', self.LF,
            '\nDesired_dist: ', self.Desired_dist,
            '\nRadius: ', self.Radius,
            '\nAngle:', self.Angle,
            '\nWaypy number: ', self.Waypt_num,
            '\nWaypt count: ', self.Waypt_count
        )

class packet132(object):
    def __init__(self):
        self.Waypt_seqID = 0
        self.Mission_mode = 0
        self.Accept_radius = 1.0
        self.lat = 0
        self.lon = 0
        self.alt = 0
        # for missions:
        self.Mission_modes = []
        self.Mission_accept_radius = []
        self.Mission_lat = []
        self.Mission_lon = []
        self.Mission_alt = []

    def unpackpkt(self, data):
        self.Waypt_seqID = data[5]
        self.Mission_mode = unpack('i',data[6:10])[0]
        self.Accept_radius = unpack('i',data[10:14])[0]
        self.lat = unpack('i',data[14:18])[0]
        self.lon = unpack('i',data[18:22])[0]
        self.alt = unpack('i',data[22:26])[0]
        print('Waypt_seqID: ', self.Waypt_seqID, 
            '\nMission_mode: ', self.Mission_mode,
            '\nAccept_radius: ', self.Accept_radius,
            '\nLat: ', self.lat,
            '\nLon', self.lon,
            '\nAlt: ', self.alt
        )
    
    # for missions:
    def mission_init(self, Waypt_count):
        # self.Mission_seq = [k for k in range(Waypt_count)]
        self.Mission_modes = [999 for k in range(Waypt_count)]
        self.Mission_accept_radius =  [999 for k in range(Waypt_count)]
        self.Mission_lat = [999 for k in range(Waypt_count)]
        self.Mission_lon = [999 for k in range(Waypt_count)]
        self.Mission_alt = [999 for k in range(Waypt_count)]
    def mission_save(self):
        self.Mission_modes[self.Waypt_seqID] = self.Mission_mode
        self.Mission_accept_radius[self.Waypt_seqID] = self.Accept_radius
        self.Mission_lat[self.Waypt_seqID] = self.lat
        self.Mission_lon[self.Waypt_seqID] = self.lon
        self.Mission_alt[self.Waypt_seqID] = self.alt
    def mission_save_input(self, seq, mode, lat, lon, alt):
        self.Mission_modes[seq] = mode
        self.Mission_lat[seq] = lat
        self.Mission_lon[seq] = lon
        self.Mission_alt[seq] = alt

class packet133(object):
    def __init__(self):
        self.mode_arm = 255
        self.time = 0
    def unpackpkt(self, data):
        self.mode_arm = int(data[5])
        self.time = time.time()

class packet135(object):
    def __init__(self):
        self.item = 0
        self.param = 0
    def unpackpkt(self, data):
        self.item = data[5]
        self.param = data[6]

'''UAV 2 UAV'''
class packet134(object):
    def __init__(self, sysID, compID, commID, lat, lon, alt, vx, vy, vz, xacc, yacc, xgyro, ygyro, zgyro, hdg, yaw, mode, gps_time):
        # int
        self.msgID = 134
        self.sysID = sysID
        self.compID = compID
        self.commID = commID
        # c_int
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.xacc = xacc
        self.yacc = yacc
        self.xgyro = xgyro
        self.ygyro = ygyro
        self.zgyro = zgyro
        self.hdg = hdg
        self.yaw = yaw
        self.mode = mode
        self.gps_time = gps_time

    def packpkt(self):
        return pack('<BBBBiiiiiiiiiiiiiii', self.msgID, self.sysID, self.compID, self.commID, 
                    self.lat.value, self.lon.value, self.alt.value,
                    self.vx.value, self.vy.value, self.vz.value,
                    self.xacc.value, self.yacc.value, self.xgyro.value, self.ygyro.value, self.zgyro.value,
                    self.hdg.value, self.yaw.value, self.mode.value, self.gps_time.value)
    
    def unpackpkt(self, data):
        self.others_sysID = data[2]
        self.others_compID = data[3]
        self.others_commID = data[4]
        self.others_lat = unpack('i',data[5:9])[0]
        self.others_lon = unpack('i',data[9:13])[0]
        self.others_alt = unpack('i',data[13:17])[0]
        self.others_vx = unpack('i',data[17:21])[0]
        self.others_vy = unpack('i',data[21:25])[0]
        self.others_vz = unpack('i',data[25:29])[0]
        self.others_xacc = unpack('i',data[29:33])[0]
        self.others_yacc = unpack('i',data[33:37])[0]
        self.others_xgyro = unpack('i',data[37:41])[0]
        self.others_ygyro = unpack('i',data[41:45])[0]
        self.others_zgyro = unpack('i',data[45:49])[0]
        self.others_hdg = unpack('i',data[49:53])[0]
        self.others_yaw = unpack('i',data[53:57])[0]
        self.others_mode = unpack('i',data[57:61])[0]
        self.others_gps_time = unpack('i',data[61:65])[0]
        # print('bef')
        # print(len(data))
        # print(unpack('i',data[65:69])[0])
        self.others_sys_time = unpack('i',data[65:69])[0]
        return self.others_sysID, self.others_compID, self.others_commID, self.others_lat, self.others_lon, self.others_alt, self.others_vx, self.others_vy, self.others_vz, self.others_xgyro, self.others_ygyro, self.others_zgyro, self.others_hdg, self.others_yaw, self.others_mode, self.others_gps_time, self.others_sys_time