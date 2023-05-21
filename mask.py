import cv2 as cv

path_to_save_mask = "test/brick/"
# avg_black = cv.imread("black/avg_black.png", cv.IMREAD_COLOR)

MASK_THREASHOLD =2

image_no_object = cv.imread(path_to_save_mask + "mask_without.png", cv.IMREAD_GRAYSCALE)
image_object = cv.imread(path_to_save_mask + "mask_with.png", cv.IMREAD_GRAYSCALE)
# image_no_object = image_object.copy()

##set all values to 0 in image_no_object
# for pixel in image_no_object:
#     for i in range(len(pixel)):
#         pixel[i] = 0



# cv.subtract(image_no_object, avg_black, image_no_object)
# cv.subtract(image_object, avg_black, image_object)

cv.imwrite("tmp_no.png", image_no_object)
cv.imwrite("tmp_obj.png", image_object)

# blur images
image_no_object = cv.fastNlMeansDenoising(
    image_no_object, None, 3, 11, 21)
image_object = cv.fastNlMeansDenoising(
    image_object, None, 3, 11, 21)


# convert images to grayscale
# image_no_object = cv.cvtColor(image_no_object, cv.COLOR_BGR2GRAY)
# image_object = cv.cvtColor(image_object, cv.COLOR_BGR2GRAY)

# generate mask
mask = cv.absdiff(image_object, image_no_object)
mask[mask < MASK_THREASHOLD] = 0
mask[mask >= MASK_THREASHOLD] = 255

# perform opening
kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (10, 10))
mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

# #perform closing
# kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (40, 40))
# mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)


cv.imwrite(path_to_save_mask + "mask.png", mask)
