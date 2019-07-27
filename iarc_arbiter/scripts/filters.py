import math
from transformers import Command

def make_speed_filter(horizontal_limit, ascent_limit, descent_limit):
    def filter(cmd):
        cmd = cmd # type: Command
        
        horizontal_speed = math.sqrt(cmd.vel.linear.x **2 + cmd.vel.linear.y ** 2)
        if horizontal_speed > horizontal_limit:
            ratio = horizontal_speed/horizontal_limit
            cmd.vel.linear.x /= ratio
            cmd.vel.linear.y /= ratio
        
        if cmd.vel.linear.z > ascent_limit:
            cmd.vel.linear.z = ascent_limit
        
        if cmd.vel.linear.z < -descent_limit:
            cmd.vel.linear.z = -descent_limit

        return cmd

    return filter