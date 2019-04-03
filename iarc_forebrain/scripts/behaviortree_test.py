#!/usr/bin/env python2
import rospy

from iarc_forebrain.behaviortree import Behavior

tree = Behavior('hello', [
    Behavior('world', [
        Behavior(),
        Behavior(),
    ])])

print(tree)
