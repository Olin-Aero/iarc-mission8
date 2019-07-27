#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from rospkg import RosPack

drones = ['swarm', 'alexa', 'google', 'siri', 'clippy']
numbers = {'oh': 0, 'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4,
           'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
           'point': '.', 'eleven': 11, 'twelve': 12, 'thirteen': 13, 'fourteen': 14,
           'fifteen': 15, 'sixteen': 16, 'seventeen': 17, 'eighteen': 18, 'nineteen': 19,
           'twenty': '2*', 'thirty': '3*', 'forty': '4*', 'fifty': '5*',
           'sixty': '6*', 'seventy': '7*', 'eighty': '8*', 'ninety': '9*'}
ignore = ['degree', 'degrees']

class VoiceInterface:

    def __init__(self):
        rospy.init_node('voice_interface')
        self.pub = rospy.Publisher("/voice", String, queue_size=10)
        self.drone = 'alexa'

    def run(self):
        while not rospy.is_shutdown():
            output = ''
            words = listen()
            if len(words) == 0 or 'cancel' in words:
                continue
            if 'shutdown' in words:
                self.pub.publish('swarm land')
                return
            if words[0] in drones:
                output += words[0]  # drone
                self.drone = words[0]
                words = words[1:]
                if len(words) == 0:
                    continue
            else:
                output += self.drone  # use previous drone
            output += ' '+words[0]  # command
            thisNum = False
            for word in words[1:]:
                if word in ignore:
                    continue
                lastNum = thisNum
                thisNum = False
                if word in numbers:
                    word = numbers[word]
                    thisNum = True
                    if output[-1] == '*':
                        output = output[:-1]
                if not (lastNum and thisNum):
                    output += ' '
                output += str(word)  # parameters
            output = output.replace('*', '0')
            print(output)
            self.pub.publish(output)


def listen():
    r = sr.Recognizer()

    # r.energy_threshold = 400 # 300 is default
    r.energy_threshold = 1200  # desktop
    # default 0.8, seconds of non-speaking audio before a phrase is considered complete
    r.pause_threshold = 0.5
    # default 0.3, minimum seconds of speaking audio before we consider the speaking audio a phrase - values below this are ignored (for filtering out clicks and pops)
    r.phrase_threshold = 0.1
    # seconds of non-speaking audio to keep on both sides of the recording
    r.non_speaking_duration = 0.2
    try:
        with sr.Microphone(
            #device_index=0
            ) as source:
            print("Say something!")
            audio = r.listen(source, timeout=1, phrase_time_limit=None)
        print("Parsing...")
    except sr.WaitTimeoutError as e:
        return []
    try:
        rospack = RosPack()
        words = r.recognize_sphinx(audio, grammar=rospack.get_path(
            'iarc_forebrain') + '/scripts/Planner/command.gram').split(" ")
        if len(words) > 1 and words[0] == "clip" and words[1] == 'e':
            words = ['clippy'] + words[2:]
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
