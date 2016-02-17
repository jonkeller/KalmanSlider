#!/usr/bin/python

# Raspberry Pi camera imports

import io
import time
import picamera
from PIL import Image


# Take a picture with the Raspberry Pi camera, and return it as a PIL image
def takeAPicture():
    print "Going to take a picture"
    # Create the in-memory stream
    stream = io.BytesIO()
    with picamera.PiCamera() as camera:
        camera.start_preview()
        time.sleep(2)
        camera.capture(stream, format='jpeg')
    # "Rewind" the stream to the beginning so we can read its content
    stream.seek(0)
    image = Image.open(stream)
    print "Took a picture"
    return image

# In my setup, I have a camera pointing to a 1-dimensional slider. On the slider is a red piece of plastic.
# The slider is aluminum and the wall is gray.  The slider can carry the red target horizontally almost
# across the entire field of view of the camera.
# So this method takes a picture and returns a number indicating how far from the left edge the red target is.
def doHardwareSense():
    # Take a picture
    image = takeAPicture()
    image.save("image.png")

    width = image.size[0]

    # My image is 720x480, the target goes across the middle, and there are some
    # red wires at the bottom of the image that I want to crop out
    image = image.crop((0, 120, width, 300))
    image.save("croppedImage.png")
    data = image.load()

    redPixelCount = 0
    redPixelTotalX = 0

    # Isolate red pixels

    for y in xrange(image.size[1]):
        for x in xrange(width):
            pixel = data[x, y]
            if (pixel[0] > 2*pixel[1] and pixel[0] > 2*pixel[2]):
                redPixelCount += 1
                redPixelTotalX += x
                data[x, y] = (255, 0, 0, 255)
            else:
                data[x, y] = (0, 0, 0, 255)

    # Find mean of red pixels
    mean = redPixelTotalX / redPixelCount

    print "Sensed value: " + str(mean)
    image.save("redPixels.png")

doHardwareSense()

