#!/usr/bin/env python2
import rospy

from iarc_forebrain.behaviortree import Behavior, Result

class Sequence(Behavior):
    def __init__(self, name=None, children=[]):
        super(Sequence, self).__init__(name, children)
        self.index = None

    def reset(self):
        if self.index is not None:
            self.children[self.index].reset()
        self.index=0

    def run()

class SayHi(Behavior):
    def reset(self):
        self.message = "hi"

    def run(self):
        print(self.message)
        return Result.SUCCESS

tree = Behavior('hello', [
    Behavior('world', [
        Behavior(),
        SayHi(),
    ])])

print(tree)

tree.reset()
tree.run()