from PIL import Image
import os
import rospkg
from qrtools import QR
from random import shuffle
import itertools


def decode(image):
	my_QR = QR()
	if my_QR.decode(image): # if the image is a recognizable QR Code
		return my_QR.data # return the value of that QR Code
	else:
		return None

def combineAndDecode(images):
	width,height=images[0].size # Get the width and height of each image
	permutations = list(itertools.permutations([0, 1, 2, 3])) # creates a list containing all permutations of the 4 images
	for perm in permutations:
		combined_im = Image.new('RGB', (width*2,height*2)) # creates a new image that is twice the height and width of the quadrants
		combined_im.paste(images[perm[0]],(0,0)) # puts the first image in top left
		combined_im.paste(images[perm[1]],(height,0)) # second in top right
		combined_im.paste(images[perm[2]],(0,width)) # third in bottom left
		combined_im.paste(images[perm[3]],(height,width)) # fourth in bottom right
		combined_im.save("combined_im.png") # saves the image because the QR decoder doesn't accept a PIL Image
		result = decode("combined_im.png") # decodes the image file, returns None if it isnt a qr, otherwise the value
		if not result == None:
			os.remove("combined_im.png") # gets rid of the image from the file
			return result

rospack = rospkg.RosPack() 
pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
images = []
code = "2008"
for i in range(4):
	images.append(Image.open(os.path.join(pkgRoot,"QRCodes",code + "-%d.png" % (i+1))))
shuffle(images)
print(combineAndDecode(images))