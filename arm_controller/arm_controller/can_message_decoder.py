import can
import struct

def message_decoder(msg):
    id = msg.arbitration_id
    node = 0
    function_num = 0
    data = []
    
    #hex math

    return node, data