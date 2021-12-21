import time
from digi.xbee.devices import DigiMeshDevice
from struct import *

# Connect xbee1
xbee001 = DigiMeshDevice('/dev/ttyUSB0', 115200)
xbee001.open(force_settings=True)

# Connect xbee2
xbee002 = DigiMeshDevice('/dev/ttyUSB1', 115200)
xbee002.open(force_settings=True)



while True:
    send = bytearray(2)
    send[0] = 22
    send[1] = 25
    send.extend(pack('ii',9898989,10))
    # xbee001.send_data_broadcast("hello")
    xbee001.send_data_broadcast(send)

    xbee_message = xbee002.read_data()
    try:
        data = xbee_message.data
        print(data, data[0], data[1], unpack('i',data[2:2+4])[0], unpack('i',data[2+4:2+8])[0])
        # print(data[0], data[1])
    except:
        pass
# xbee002.

