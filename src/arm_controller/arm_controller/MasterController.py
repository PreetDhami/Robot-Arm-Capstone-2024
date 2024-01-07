import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import can
import can_message_decoder
import can_interface

from std_msgs.msg import String

import SendAngles.action


class MasterController(Node):

    def __init__(self):

        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.canif = can_interface()
        self.msgbuffer = can.BufferedReader()

        super().__init__('master_controller')
        self.angles_action_server = ActionServer(
            self, SendAngles, 'sendangles',
            self.send_angles_callback)
        self.data_publisher = self.create_publisher(String, 'topic', 1)

        self.pub_timer = self.create_timer(0.1, self.pub_data)
        self.comm_timer = self.create_timer(0.001, self.can_comms)



    def pub_data(self):
        
        msg = String()
        msg.data = 'angles: %f' % self.angles
        self.data_publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % msg.data)


    def can_comms(self):
        
        canmsg = self.msgbuffer.get_message(timeout=0.0)
        node, function_num, data = can_message_decoder.message_decoder(canmsg)

        #figure out how to do this best
        
        return




    def send_angles_callback(self, goal_handle):
        feedback_msg = SendAngles.Feedback()
        result = SendAngles.result



def main():
    mc = MasterController
    rclpy.spin(mc)

if __name__ == "__main__":
    main()
