import cv2 as cv
import numpy as np
import glob
from matplotlib import pyplot as plt

image_avg = cv.imread("black/black_0.png")
images = glob.glob ("black_*.png")
print("size: ", len(images))
image_data = []
for img in images:
    this_image = cv.imread(img)
    # print("hello: ")
    image_data.append(this_image)

avg_image = image_data[0]
# print("avg_array shape: ", avg_array)
avg_array = np.array(image_data[0], dtype=np.float32)
#loop through the images and add them to the avg_array
for i in range(len(image_data)):
    if i == 0:
        pass
    else:
        print("hello2: ")
        avg_array = avg_array + (np.array(image_data[i].astype(np.float32)))
        # print(avg_array[i])
        # print("avg_array shape: ", avg_array.shape)
        # print("avg_array type: ", avg_array

#print highest value in array
# print("max value: ", np.amax(avg_array))
avg_array = avg_array / len(images)

        

print("hello3: ")
cv.imwrite('avg_black.png', avg_array)