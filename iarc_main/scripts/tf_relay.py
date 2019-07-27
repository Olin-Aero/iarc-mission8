#!/usr/bin/env python2

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TfRelay(object):
    def __init__(self):
        self.internal_sub = rospy.Subscriber(rospy.get_param('~internal', 'tf'), TFMessage, self.handleInternalTf)
        self.external_pub = rospy.Publisher(rospy.get_param('~external', '/tf'), TFMessage, queue_size=0)
        
        self.tf_namespace = rospy.get_namespace().strip('/')

        self.remaps = {}

        for s in 'base_link odom launch'.split():
            self.remaps[s] = self.tf_namespace + '/' + s

    def handleInternalTf(self, msg):
        msg = msg # type: TFMessage
        newmsg = TFMessage()
        for frame in msg.transforms: # type: TransformStamped
            if frame.header.frame_id in self.remaps:
                frame.header.frame_id = self.remaps[frame.header.frame_id]
            if frame.child_frame_id in self.remaps:
                frame.child_frame_id = self.remaps[frame.child_frame_id]

            newmsg.transforms.append(frame)

        self.external_pub.publish(newmsg)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('tf_relay', anonymous=True)
    TfRelay().run()
