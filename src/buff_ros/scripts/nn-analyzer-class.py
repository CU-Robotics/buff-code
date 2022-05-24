import numpy
import argparse
import os
import cv2

class Analyzer:
    def __init__(self, loadFromFilePath):
        self.imageNames = os.listdir(loadFromFilePath)
        self.imagePaths = []
        self.imagesBGR = []
        self.imagesHSV = []
        self.imagesProcessed = []
        for imageName in self.imageNames:
            imagePath = loadFromFilePath+imageName
            imageBGR = cv2.imread(imagePath)
            self.imagePaths.append(imagePath)
            self.imagesBGR.append(imageBGR)
            self.imagesHSV.append(cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV))

    def loadImages(self, loadFromFilePath):
        """
        Function: loads additional single image or a batch of images from a given filepath, DOES NOT SHOW IMAGE
        Parameters:
            @string loadToFilePath = filepath of images
        """
        newImageNames = list(set(os.listdir(loadFromFilePath)) - set(self.imageNames))
        self.imageNames += newImageNames
        for index in range(len(self.imageNames)-len(newImageNames), len(self.imageNames)):
            self.imagePaths.append(loadFromFilePath+self.imageNames[index])
            imageBGR = cv2.imread(self.imagePaths[index])
            self.imagesBGR.append(imageBGR)
            self.imagesHSV.append(cv2.cvtColor(imageBGR, cv2.COLOR_BGR2HSV))

    def saveImages(self, images, saveToFilePath="../data/"):
        """
        Function: saves a single or a batch of images to a given filepath
        Parameters:

        """
        for image in images:
            cv2.imwrite(saveToFilePath+"Copy"+image, image)

    def analyzeImageHSV(self, imageName):
        """
        Function: analyzes images using HSV, generally better than BGR for lights, adds to processed image array
        @PARAMS: 
            string imageName = name of the image, including filetype
                Ex: "exampleImage.jpg"

        """
        #if image name is in imageNames, get index
        index = 0
        for imgName in self.imageNames:
            if imgName != imageName:
                index+=1
        if (index >= len(self.imageNames)):
            print("Image not found")
            return
        #imagePaths[index] will give the imagePath
        imagePath = self.imagePaths[index]
        imageBGR = self.imagesBGR[index]
        imageHSV = self.imagesHSV[index]
        red_lower = (0,140,100) #hsv for red, lower boundary
        red_upper = (25,255,255) #hsv for red, upper boundary
        blue_lower = (80,100,100) #hsv for blue, lower boundary
        blue_upper = (120,255,255) #hsv for blue, upper boundary
        red_mask = cv2.inRange(imageHSV, red_lower, red_upper)
        blue_mask = cv2.inRange(imageHSV, blue_lower, blue_upper)
        red_output = cv2.bitwise_and(imageHSV, imageHSV, mask=red_mask) #image red only
        blue_output = cv2.bitwise_and(imageHSV, imageHSV, mask=blue_mask) #image blue only
        red_output = cv2.cvtColor(red_output, cv2.COLOR_HSV2BGR)
        blue_output = cv2.cvtColor(blue_output, cv2.COLOR_HSV2BGR)
        self.imagesProcessed.append(red_output)
        self.imagesProcessed.append(blue_output)
        #cv2.imshow("image", imageBGR)
        #cv2.imshow("HSV red detection: " + imagePath, red_output)
        #cv2.imshow("HSV blue detection: " + imagePath, blue_output)
        #cv2.waitKey(0)

    def analyzeImageBGR(self, imageName):
        """
        Function: analyzes images using BGR, adds to processed image array
        Parameters:
            @string imageName = name of the image, including filetype
                Ex: "exampleImage.jpg"

        """
        index = 0
        for imgName in self.imageNames:
            if imgName != imageName:
                index+=1
        if (index >= len(self.imageNames)):
            print("Image not found")
            return
        imagePath = self.imagePaths[index]
        imageBGR = self.imagesBGR[index]
        red_lower = (0,0,100) #bgr for red, lower boundary # Define these as attributes of the class not local variables.
        red_upper = (80,80,255) #bgr for red, upper boundary # That way we can define them on initilization.
        blue_lower = (100,0,0) #bgr for blue, lower boundary
        blue_upper = (255,80,80) #bgr for blue, upper boundary
        red_mask = cv2.inRange(imageBGR, red_lower, red_upper)
        blue_mask = cv2.inRange(imageBGR, blue_lower, blue_upper)
        red_output = cv2.bitwise_and(imageBGR, imageBGR, mask=red_mask) #image red only
        blue_output = cv2.bitwise_and(imageBGR, imageBGR, mask=blue_mask) #image blue only
        self.imagesProcessed.append(red_output)
        self.imagesProcessed.append(blue_output)
        #cv2.imshow("image", imageBGR)
        #cv2.imshow("BGR red detection: " + imagePath, red_output)
        #cv2.imshow("BGR blue detection: " + imagePath, blue_output)
        #cv2.waitKey(0)

    def detect(self, image, display=False):


    def display(self):
        """
        Function: displays all images by batch, BGR, HSV, and then all processed images
        
        """
        if (len(self.imagesBGR) != 0):
            cv2.imshow("imagesBGR", numpy.concatenate(self.imagesBGR, axis=0))
            cv2.waitKey(0)
        #for image in self.imagesBGR:
            #cv2.imshow("imageBGR", image)
            #cv2.waitKey(0)
        if (len(self.imagesHSV) != 0):
            cv2.imshow("imagesHSV", numpy.concatenate(self.imagesHSV, axis=0))
        #for image in self.imagesHSV:
            #cv2.imshow("imageHSV", image)
            #cv2.waitKey(0)
            cv2.waitKey(0)
        if (len(self.imagesProcessed) != 0):
            cv2.imshow("imagesProcessed", numpy.concatenate(self.imagesProcessed, axis=0))
        #for image in self.imagesProcessed:
            #cv2.imshow("processedImage", image)
            #cv2.waitKey(0)
            cv2.waitKey(0)

#TEST CASES----------------------------------------------------------------------------------------------------
#images = Analyzer("../data/")
#images.loadImages("../data/")
#images.analyzeImageBGR("30.jpg")
#images.display()

"""
TODO:
1. Fix the issue that some images do not stack together (concatenation only shows 2 images)
2. Fix the issue where it will not allow loading from the file path of one single image (file instead of directory)
3. Give the processed images names so that when they are saved they have names
4. Add error handlers/exceptions
"""


    
    