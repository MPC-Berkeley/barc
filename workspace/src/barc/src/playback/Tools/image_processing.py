from PIL import Image
import numpy as np
import glob

num_images = len(glob.glob("/home/lukas/images/*.png"))

image_path = "/home/lukas/images/image_"
for_video = "/home/lukas/images_video/iteration_"

i = 0
for index in range(268, 999):
	image_file = image_path + str(index) + ".png"
	im_frame = Image.open(image_file)
	resized_image = im_frame.resize((928, 994))
	resized_image.save(for_video + str(i).zfill(3) + ".png")
	i += 1
