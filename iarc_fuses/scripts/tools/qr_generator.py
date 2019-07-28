#!/usr/bin/env python2

# Creates 20 QR codes with randomly generatod 4 digit numbers. Also creates the 4 quarters of these 20 images.
import argparse
import os
import random
import re
import rospkg
from random import randint
from shutil import copyfile
import qrcode

class QRGenerator(object):
    def __init__(self, args):
        rospack = rospkg.RosPack() 
        pkgRoot = rospack.get_path('iarc_fuses') # Gets the package
        mydir = os.path.join(pkgRoot,"QRCodes")

        if args.values()[0]: # Did user request new QR codes? Check argparse arguments.
            QRGenerator.deleteQR(mydir)

            for i in range(20): # Generate 20 QR codes
                code = '{0:04}'.format(randint(0, 10**3))
                image = QRGenerator.generateQR(pkgRoot, code)
                QRGenerator.cropQR(pkgRoot, image, code)

            QRGenerator.setQRMaterial(rospack, code) # Sets the last code as the used QR code image in the sim.
        else:
            filelist = [ f for f in os.listdir(mydir) if f.endswith(".png") ]
            rand = random.randint(0,len(filelist)-1)

            substitutions = {'-1.png': '', '-2.png': '', '-3.png': '', '-4.png': '', '.png': ''}
            regex = re.compile('|'.join(map(re.escape, substitutions)))
            code = regex.sub(lambda match: substitutions[match.group(0)], filelist[rand])

            # code = filelist[rand].replace('-1.png', '').replace('-2.png', '').replace('-3.png', '').replace('-4.png', '').replace('.png', '')
            print "Your new QR code is: " + str(code)
            QRGenerator.setQRMaterial(rospack, code)

    @staticmethod
    def generateQR(pkgRoot, code):
        """ Generates a QR code based on a random 4 digit number. """
        img = qrcode.make(code)
        img.save(os.path.join(pkgRoot,"QRCodes",code +'.png'),"PNG") # saves the qr code

        return img

    @staticmethod
    def deleteQR(mydir):
        """ Deletes previous set of generated QR codes. """
        filelist = [ f for f in os.listdir(mydir) if f.endswith(".png") ]
        for f in filelist:
            os.remove(os.path.join(mydir, f))
        return

    @staticmethod
    def cropQR(pkgRoot, img, code):
        """ Crops existing set of QR images into quarters, and saves the quarters. """
        imgWidth, imgHeight = img.size

        for i in range(2):
            for j in range(2):
                box = (imgWidth / 2 * i, int(imgHeight / 2 * j),imgWidth / 2 * (1+i), imgHeight / 2 * (1+j)) # selects a box to crop
                quart = img.crop(box) # crops the image into one quarter
                section = str(1 + i + j*2) # the section number of the image, 1 = top left, 2 = top right, 3 = bottom left, 4 = bottom right
                quart.save(os.path.join(pkgRoot,"QRCodes", code + "-" + section + '.png'),"PNG") # saves the quarter image
        return

    @staticmethod
    def replaceFilename(filepath, code, num):
        f = open(filepath,'r')
        filedata = f.read()
        f.close()

        newWord = os.path.join(str(code) + "-" + str(num) + ".png")
        oldName = 'texture (.*)-' + str(num) + '.png'
        lastCode = re.search(oldName, filedata) # Get QR code
        # print str(lastCode.group(0))
        # print str(newWord)

        newdata = filedata.replace(str(lastCode.group(0)), 'texture ' + str(newWord))

        f = open(filepath,'w')
        f.write(newdata)
        f.close()
        return

    @staticmethod
    def copyText(words, start, end):
        return words[start:end]

    @staticmethod
    def setQRMaterial(rospack, code):
        simPath = rospack.get_path('iarc_sim_3d')
        fusesPath = rospack.get_path('iarc_fuses')

        # Update iarc.material with correct .pngs
        sdfPath = os.path.join(simPath, "media/materials/scripts/iarc.material")

        # Move .jpg to media/materials/textures
        for num in range(1,5):
            newJpgFusesPath = os.path.join(fusesPath, "QRCodes/" + str(code) + "-" + str(num) + ".png")
            newJpgSimPath = os.path.join(simPath, "media/materials/textures/" + str(code) + "-" + str(num) + ".png")
            if not os.path.isfile(newJpgSimPath): copyfile(newJpgFusesPath, newJpgSimPath)

            # Add QR material to media/materials/scripts/iarc.material
            QRGenerator.replaceFilename(sdfPath, code, num)
        print "New code: " + str(code)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--new", required=False, help="generate new QR codes", action = "store_true")
    args = vars(ap.parse_args())

    qr = QRGenerator(args)