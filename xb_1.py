from digi.xbee.devices import DigiMeshDevice
import time

xbee003 = DigiMeshDevice("COM7", 115200)
xbee003.open(force_settings=True)

while xbee003.is_open():
    xbee_message = xbee003.read_data()
    if xbee_message is None:
        continue
    else:
        data = xbee_message.data
        xbeeTime = time.localtime(xbee_message.timestamp)
        t = time.localtime()
        data[7] = t.tm_hour
        data[8] = t.tm_min
        data[9] = t.tm_sec
        data[10] = t.tm_hour
        data[11] = t.tm_min
        data[12] = t.tm_sec
        data[13] = len(data)
        print(data)
# saveDate = bytes(data)
# with open("test.bin", "wb") as f:
# f.write(saveDate)
