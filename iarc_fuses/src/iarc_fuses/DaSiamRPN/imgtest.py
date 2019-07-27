from tkinter import * 
from tkinter.filedialog import askopenfilename
import cv2
import pickle 
from video_labeler import Annotation
import numpy as np
from PIL import Image
from PIL import ImageTk

class MainWindow():
    #----------------

    def __init__(self, main, p_file=False, width=600, height=600):
        self.im_width = width
        self.im_height = height

        if not p_file:
            self.p_file = self.select_p_file()
        else:
            self.p_file = p_file

        # canvas for image
        self.canvas = Canvas(main, width=60, height=60)
        self.canvas.grid(row=0, column=0)

        # images
        array = np.zeros((self.im_width,self.im_height, 3))
        image =  Image.fromarray(array.astype('uint8'))
        self.disp_image = ImageTk.PhotoImage(image=image)

        # set first image on canvas
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor = NW, image = self.disp_image)

        # # button to change image
        # self.button = Button(main, text="Change", command=self.onButton)
        # self.button.grid(row=1, column=0)

        self.view_btn = Button(main, text="View Labels", command=self.view_callback)
        self.view_btn.grid(row=1, column=0)   

        self.convert_btn = Button(main, text="Convert to TF Label", command=self.convert_callback)
        self.convert_btn.grid(row=1, column=1)

        file = open(self.p_file,"rb")
        self.annotation_array = pickle.load(file)

    def disp_opencv_img(self, img):
        img = cv2.resize(img, (self.im_width, self.im_height))
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image.astype('uint8'))
        image = ImageTk.PhotoImage(image=image)
        # self.disp_image = PhotoImage(file = "ball2.gif")
        # cv2.imshow("lol", img)
        # cv2.waitKey(0)
        # self.canvas.itemconfig(self.image_on_canvas, image = image)
        # self.canvas.itemconfig(self.image_on_canvas, image = image)

        self.canvas.itemconfig(self.image_on_canvas, image = self.disp_image)
        # self.label.image = image

    def select_p_file(self):
        Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
        filename = askopenfilename(title = "Select file",filetypes = (("pickle files","*.p"),("all files","*.*"))) # show an "Open" dialog box and return the path to the selected file
        
        return filename


    def view_callback(self):
        print("View Labels!")
        cap = cv2.VideoCapture(self.annotation_array[0].file_reference)
        cap.set(2, self.annotation_array[0].frame_num)
        ret, img = cap.read()

        self.disp_opencv_img(img)

        # for label in self.annotation_array:

    def convert_callback(self):
        print("Convert Callback!")
    #----------------

    def onButton(self):

        # next image
        self.my_image_number += 1

        # return to first image
        if self.my_image_number == len(self.my_images):
            self.my_image_number = 0

        # change image
        self.canvas.itemconfig(self.image_on_canvas, image = self.my_images[self.my_image_number])

#----------------------------------------------------------------------

root = Tk()
MainWindow(root)
root.mainloop()