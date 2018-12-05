#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from util.Drone import Drone

class voiceToDrone():
    def __init__(self):
        rospy.init_node('keyboardInput', anonymous=True)
        self.voiceSub = rospy.Subscriber('/voice',String,self.voiceCB)
        self.alpha = Drone()
        self.bravo = Drone()
        self.charlie = Drone()
        self.delta = Drone()
        self.updateRate = rospy.Rate(5)
        self.message = None

    def voiceCB(self,msg):
        self.message = msg.data
        words = self.message.split(" ")
        if(words[0] == "alpha"):
            self.moveDrone(self.alpha,words)
        elif(words[0] == "bravo"):
            self.moveDrone(self.bravo,words)
        elif(words[0] == "charlie"):
            self.moveDrone(self.charlie,words)
        elif(words[0] == "delta"):
            self.moveDrone(self.delta,words)
        else:
            rospy.loginfo("Drone not recognized")

    def moveDrone(self,drone,words):
        pos = drone.get_pos().pose.position
        try:
            distance = float(words[2])
        except ValueError:
            rospy.loginfo("Distance is not a number")
        if(words[1] == "north"):
            drone.move_to(pos.x,pos.y+distance)
        elif(words[1] == "west"):
            drone.move_to(pos.x-distance,pos.y)
        elif(words[1] == "south"):
            drone.move_to(pos.x,pos.y-distance)
        elif(words[1] == "east"):
            drone.move_to(pos.x+distance,pos.y)    
        else:
            rospy.loginfo("Direction not recognized")

    def run(self):
        while not rospy.is_shutdown() and self.message == None:
            rospy.loginfo("No messages")

        while not rospy.is_shutdown():
            self.updateRate.sleep()

if __name__ == '__main__':
    vo = voiceToDrone()
    vo.run()