import can
import time


filters = [
    {"can_id": 0x009, "can_mask": 0x01f, "extended": False},
]
bus = can.interface.Bus(channel="can0", interface="socketcan", can_filters=filters)
time.sleep(0.5)

for msg in bus:
    print(msg.arbitration_id, ": ", msg.data)
