import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy
from ctypes import *
from info import info, packet127, packet128
from datetime import datetime






# int(math.log(msg.HL_FAILURE_FLAG,2))
# if msg.severity < 4:
# msg.severity += 14

'''
from digi.xbee.devices import DigiMeshDevice
import threading

# XBee connection
xbee = DigiMeshDevice("COM4", 115200)
xbee.open(force_settings=True)
dataLen = [0]

def readxbee_timer(func, sec):
    def func_wrapper():
        readxbee_timer(func, sec)
        func()
        t = threading.Timer(sec, func_wrapper)
        t.start()


def rx():
    try:
        xbee_message = xbee.read_data()
        data = xbee_message.data
        xbt = xbee_message.timestamp
        # textfile = open("testbytes06.txt", 'ab')
        # textfile.write(data)
        print(data)
        print(xbt)
        dataLen[0] += len(data)
        print(dataLen[0])
    except:
        pass


readxbee_timer(rx, 0.01)
'''

# import threading
# from threading import Timer
# def hello():
#     print("hello, world") 
# # Timer(10.0, hello).start()
# t = Timer(2.0, hello) 
# t.start()

# from threading import Thread
# def hello():
#     print("hello, world") 
# class MyThread(Thread):
#     def run(self):
#         hello()
# t = MyThread()
# t.start()

# class packet1(object):
#     def __init__(self, msgID, lat, lon, alt):
#         self.msgID = msgID
#         self.lat = lat
#         self.lon = lon
#         self.alt = alt
#     def pack():
#         pass

# lat = c_int(0)
# lon = c_int(1)
# alt = c_int(2)

# pkt1 = packet1(99, lat,lon,alt)
# print(pkt1.lat)

# # lat_p = pointer(lat)
# # lat_p[0] = 100

# lat.value = 88
# print(pkt1.lat)
# print(pkt1.lat.value)

sysID, compID, commID = 0, 1, 22
gps_time = c_int(0)
roll, pitch, yaw, xacc, yacc, zacc = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)  
lat, lon, alt, vx, vy, vz, hdg = c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0), c_int(0)     
Dyn_waypt_lat, Dyn_waypt_lon = c_int(0), c_int(0)
fix, sat_num = c_int(0), c_int(0)
mode, arm, system_status, failsafe = c_int(255), c_int(255), c_int(255), c_int(255)

pkt= {127: packet127(sysID, compID, commID, mode, arm, system_status, failsafe),
    128: packet128(sysID, compID, commID, lat, lon, alt, vx, vy, vz, hdg, roll, pitch, yaw, xacc, yacc, zacc, Dyn_waypt_lat, Dyn_waypt_lon, gps_time)
}

print(pkt[128].packpkt())

lat.value = 255
print(pkt[128].packpkt())

