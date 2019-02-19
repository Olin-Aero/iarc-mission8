#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

drones = ['red','green','blue','white']
numbers = {'oh':0,'zero':0,'one':1,'two':2,'three':3,'four':4,\
        'five':5,'six':6,'seven':7,'eight':8,'nine':9,'ten':10, \
        'point':'.','eleven':11,'twelve':12,'thirteen':13,'fourteen':14,\
        'fifteen':15,'sixteen':16,'seventeen':17,'eighteen':18,'nineteen':19}

class VoiceInterface:

    def __init__(self):
        rospy.init_node('voice_interface')
        self.pub = rospy.Publisher("/voice", String, queue_size=10)
        self.drone = 'red'

    def run(self):
        while not rospy.is_shutdown():
            output = ''
            words = listen()
            if len(words) == 0 or 'cancel' in words:
                continue
            if 'shutdown' in words:
                return
            if words[0] in drones:
                output += words[0] # drone
                self.drone = words[0]
                words = words[1:]
                if len(words) == 0:
                    continue
            else:
                output += self.drone # use previous drone
            output += ' '+words[0] # command
            thisNum = False
            for word in words[1:]:
                lastNum = thisNum
                thisNum = False
                if word in numbers:
                    word = numbers[word]
                    thisNum = True
                if not (lastNum and thisNum):
                    output += ' '
                output += str(word) # parameters
            self.pub.publish(output)

def listen():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say something!")
        audio = r.listen(source)
    print("Parsing...")
    try:
        words = r.recognize_sphinx(audio, grammar='command.gram').split(" ")
        print(words)
        return words
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))
    return []

if __name__ == '__main__':
    v = VoiceInterface()
    v.run()