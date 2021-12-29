import time
from struct import *
from digi.xbee.devices import DigiMeshDevice

# Connect xbee2
xbee002 = DigiMeshDevice('/dev/ttyUSB2', 115200)
xbee002.open(force_settings=True)


mesID = [128, 129]
pkt_len = {1: 9, 2: 15}
pkt_space = {1: [1,1,1,1,1,4,4,1,1], 2: [1,1,1,1,1,4,4,4,4,4,4,4,4,4,1]}
res = {1: [0 for i in range(pkt_len[1])], 2: [0 for i in range(pkt_len[2])]}

while xbee002.is_open():
    try:
        received = xbee002.read_data()
        data = received.data
        do = mesID.index(data[4]) + 1
        for i, space in enumerate(pkt_space[do][:]):
            if space == 1:
                res[do][i] = data[sum(pkt_space[do][:i])]
            else:
                res[do][i] = unpack('i',data[sum(pkt_space[do][:i]):sum(pkt_space[do][:i+1])])[0]
        print('out: ', res[do])
    except:
        pass