from PIL import Image
import os
import rospy
import rospkg
from qrtools import QR
from random import shuffle
import itertools

class QRCombiner():
    def __init__(self):
        pass

    def decode(self, image):
        my_QR = QR()
        if my_QR.decode(image): # if the image is a recognizable QR Code
            return my_QR.data # return the value of that QR Code
        else:
            return None

    def __call__(self, images):
        width,height=images[0].size # Get the width and height of each image
        permutations = list(itertools.permutations([0, 1, 2, 3])) # creates a list containing all permutations of the 4 images
        for perm in permutations:
            print perm
            combined_im = Image.new('RGB', (width*2-15,height*2-15)) # creates a new image that is twice the height and width of the quadrants
            # TODO : resolve overlaps in combination
            combined_im.paste(images[perm[0]],(0,0)) # puts the first image in top left
            im2 = images[perm[1]].crop((15,0,width,height))
            combined_im.paste(im2,(height,0)) # second in top right
            im3 = images[perm[2]].crop((0,15,width,height))
            combined_im.paste(im3,(0,width)) # third in bottom left
            im4 = images[perm[3]].crop((15,15,width,height))
            combined_im.paste(im4,(height,width)) # fourth in bottom right
            combined_im.save("combined_im.png") # saves the image because the QR decoder doesn't accept a PIL Image
            result = self.decode("combined_im.png") # decodes the image file, returns None if it isnt a qr, otherwise the value-
            #os.remove("combined_im.png") # gets rid of the image from the file
            if not (result is None):
                print ':)'
                #for i in range(4):
                #    os.remove("FinalImage%d.png" %(i))
                return result
        return result

def main():
    images = []

    # Stand-in images until ros detection callback exists
    rospack = rospkg.RosPack()
    pkgRoot = rospack.get_path('iarc_fuses') # Gets the package

    for i in range(4):
        # NOTE: Get the images from the IARC google drive
        images.append(Image.open(os.path.join(pkgRoot,"QRCodes","1929-%d.png" % (i+1))))
    print images

    comb = QRCombiner()
    print comb( images )

if __name__ == '__main__':
    main()
