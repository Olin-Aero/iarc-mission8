#!/usr/bin/env python2

# (See that line above? Keep that line. It's important... If you really must know what it is, google "Shebang".)

# Hello! As you can see, this line starts with a hashtag. That creates something called a "comment"
# in programming. Comments are ignored by the code interpreter so you can write whatever you want
# without breaking your program. The only exception to this is the Shebang above.

# I'll walk you through this example file that can fly the drone in a square.


# So first, you should be relieved to know that we do not expect you to code the drone from
# scratch. This is due to beautiful things we have in Python called "modules". A module is basically
# a bunch of code that SOMEONE ELSE wrote that you can automatically import into your own program.
# For example, people have written hundreds of lines of code to make a drone take off from the ground.
# Is it reasonable to expect you to copy/paste those hundreds of lines of code into your program
# every time you need to make a drone take off? NO!!!!
# Programming is awesome because people can package their code into modules. Modules are condensed
# pieces of prewritten code that we can use.

# Here we import something from the Drone module (we wrote this on IARC last year - you're welcome.)
# So now to make a drone takeoff it's literally one line of code -- which you'll see very shortly.
from util.Drone import Drone

# Create a drone object. You can name your drone anything you want (I've named my drone Fred)

drone = Drone()

# First thing that Fred needs to do is take off from the ground. To make him fly we command him like this:

drone.takeoff() #Told you it was one line!!

# This command makes Fred fly upwards to an altitude of 1.5 meters. He will stay there if you dont give him
# another command! Tell him to move forward next:

drone.move_to(1.0, 0.0, 'launch')

# So what does that command mean? Fred recognizes move_to() as a command but requires
# 3 "parameters" (inputs) to customize the exact instructions we want to give him.
# drone.move_to(1.0, 0.0, 'launch') tells Fred to move to position (1.0, 0.0) on a grid that
# is centered on his 'launch' position. This command will make him move 1 meter in the
# x-direction from his launch position.
# If you wanted him to move the opposite direction, you can use the command drone.move_to(-1.0, 0.0, 'launch')

# At this point in the code, Fred has moved to position (1.0, 0.0). Good job, Fred.
# Now he needs a break. Let's tell him to hover in place for 3 seconds before landing:

drone.hover(3)
drone.land()

# Wait! One more thing. Before you try to run your file, double-check that your
# permissions allow you to run it. You can do this by following these steps:
# 1) Right click your file
# 2) Click "Properties" on the menu
# 3) Go to the "Permissions" tab
# 4) Check the "Allow executing as a file" box

# You only need to edit the permissions once. Happy flying!
