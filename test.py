import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy
from ctypes import *
from info import info 
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


msgID_send, msgID_receive = info.msgID_send, info.msgID_receive
# Initialize packets
# send_pkt_num: list of packet number; item: items in packet; space: space allocation of packet
# val: values of packet items; bytearray: bytearray of the packet value (to be sent by xbee); res: unpacked result of the packed packet
# convert: unit conversion, preferred int; byte_num: get format letter from space allocation for un/pack usage
pkt_item, pkt_space = info.pkt_item, info.pkt_space
convert, byte_num = info.convert, info.byte_num
pkt_val, pkt_bytearray, res = {}, {}, {}
for i in msgID_send:
    res[i] = [0 for k in range(len(pkt_item[i]))]
    pkt_val[i] = [0 for k in range(len(pkt_item[i]))]
    pkt_val[i][0], pkt_val[i][1], pkt_val[i][-1] = info.header, i, info.checksum
    pkt_val[i][2], pkt_val[i][3], pkt_val[i][4] = 1, 2, 33
    pkt_bytearray[i] = bytearray([pkt_val[i][0]])
    for j in range(1, len(pkt_item[i])-1):
        pkt_bytearray[i].extend(pack(byte_num[pkt_space[i][j]], pkt_val[i][j]))
    pkt_bytearray[i].extend(pack(byte_num[2], info.checksum))
pkt_val[127][pkt_item[127].index('Mode')], pkt_val[127][pkt_item[127].index('Arm')] = 255, 255
pkt_val[127][pkt_item[127].index('HEARTBEAT.system_status')], pkt_val[127][pkt_item[127].index('Failsafe')] = 255, 255

print(pkt_val[127])

pkt_val[127][pkt_item[127].index('Mode')] = 3
pkt_val[127][pkt_item[127].index('Arm')] = 10
pkt_val[127][pkt_item[127].index('HEARTBEAT.system_status')] = 66
pkt_val[127][pkt_item[127].index('Failsafe')] = 99

print(pkt_val[127])

utctime = datetime.utcnow()
pkt_val[127][-2] = int((utctime.minute*60 + utctime.second)*1e3 + round(utctime.microsecond/1e3))
print(pkt_val[127])
