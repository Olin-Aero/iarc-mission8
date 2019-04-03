from __future__ import print_function, division, absolute_import

from enum import Enum


class Result(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class Behavior(object):
    """
    A behavior is a node in the behavior tree. For more references about
    what behavior trees are and how they work, look online.
    """

    def __init__(self, name=None, children=[]):
        self.name = name
        self.children = children

    def reset(self):
        """
        Signals that the next call to run() will have a temporal gap from the previous call.
        Most initialization logic should happen here, instead of in __init__()

        This call should be propogated to children as needed
        """
        raise NotImplementedError

    def run(self):
        """
        Executes this behavior. Must return a valid Result.
        """
        raise NotImplementedError

    def __str__(self):
        if self.name is not None:
            name = "{}<{}>".format(type(self).__name__, self.name)
        else:
            name = type(self).__name__

        childrentext = '\n'.join([c.__str__() for c in self.children])
        childrentext = ''.join(
            ['\t'+line for line in childrentext.splitlines(True)])
        if childrentext:
            return '{}(\n{}\n)'.format(name, childrentext)
        else:
            return '{}()'.format(name)

