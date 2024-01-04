import can
import struct

#https://python-can.readthedocs.io/en/stable/api.html
#https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#


class can_interface():

    def __init__(self):
        num = 0
        self.canbus = can.Bus(channel="can0", interface="socketcan")
        self.msgbuffer = can.BufferedReader()

    def set_position(self, node_num, Input_Pos, Vel_FF = 0):
        a_id = 32*node_num + 10
        payload = bytearray(struct.pack("f", Input_Pos))
        msg = can.Message(arbitration_id = a_id, is_extended_id=False, data=payload)
        self.canbus.send(msg)
        print(msg)
        return
    
    def get_encoder_estimates(self, node_num):
        a_id = 32*node_num + 10
        payload = [0x00]
        msg = can.Message(arbitration_id = a_id, is_extended_id=False, data=payload, is_remote_frame=True)
        self.canbus.send(msg)
        print(msg)
        return
    
    #repeat above for all 32 functions in api



        



