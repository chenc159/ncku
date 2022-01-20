import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy
from ctypes import *
from info import info 






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



