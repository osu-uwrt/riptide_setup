import cv2
import numpy as np
import sys

ED_KERNEL_SIZE = 3
BLUR_KERNEL_SIZE = 9
CONTOUR_AREA_THRESHOLD = 800

colors = [(255,0,0),
		  (0,255,0),
		  (255,255,0),
		  (255,0,255)]

upper = (20,255,255)
lower = (0,135,100)

class BoundingBoxExtremes():
	def __init__(self, mx, mix, my, miy):
		self.max_x = mx
		self.min_x = mix
		self.max_y = my
		self.min_y = miy
		self.x_mid = int((self.max_x + self.min_x) / 2)
		self.y_mid = int((self.max_y + self.min_y) / 2)
		self.x_center = self.x_mid
		self.y_center = self.y_mid

def find_gate(original_image, draw=True):
	cv_image = original_image.copy()
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	ret_fields = list()

	image_width = original_image.shape[1]
	image_height = original_image.shape[0]

	cv_image = cv2.inRange(cv_image, lower, upper)
	cv_image = clean_noise(cv_image, mode='erode')

	contours = get_contours(cv_image)
	blank = np.zeros(cv_image.shape, np.uint8)

	blank = cv2.fillPoly(blank, contours, color=(255,255,255))
	blank = dilate_contours(blank)
	box_points = cv2.findNonZero(blank)

	if box_points is not None:
		box = bounding_box_nonzeros(box_points)
		if draw:
			cv2.drawContours(original_image, [box], 0, (255,0,0), 2)
		if draw:
			iteration = 0
			for point in tuple_points(box):
				cv2.circle(original_image, point, 3, colors[iteration], -1)
				iteration += 1
		extremes = get_extremes(tuple_points(box))
		if draw:
			cv2.circle(original_image, (extremes.x_mid, extremes.y_mid), 3, (255,0,0), -1)
			cv2.circle(original_image, (extremes.x_center, extremes.y_center), 3, (0,0,255), -1)
		ret_fields = [1,1,0, extremes.min_x, extremes.min_y, extremes.max_x, extremes.max_y, extremes.x_mid, extremes.y_mid, extremes.x_center, extremes.y_center, int(image_width / 2), int(image_height / 2)]
	
	return ret_fields, original_image

def find_pole(original_image, draw=True):
	cv_image = original_image.copy()
	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	ret_fields = list()

	image_width = original_image.shape[1]
	image_height = original_image.shape[0]

	cv_image = cv2.inRange(cv_image, lower, upper)
	cv_image = clean_noise(cv_image, mode='erode')

	contours = get_contours(cv_image, clean=False)

	blank = np.zeros(cv_image.shape, np.uint8)

	blank = cv2.fillPoly(blank, contours, color=(255,255,255))
	blank = dilate_contours(blank)
	box_points = cv2.findNonZero(blank)

	if box_points is not None:
		box = bounding_box_nonzeros(box_points)
		if draw:
			cv2.drawContours(original_image, [box], 0, (255,0,0), 2)
		if draw:
			iteration = 0
			for point in tuple_points(box):
				cv2.circle(original_image, point, 3, colors[iteration], -1)
				iteration += 1
		extremes = get_extremes(tuple_points(box))
		if draw:
			cv2.circle(original_image, (extremes.x_mid, extremes.y_mid), 3, (255,0,0), -1)
			cv2.circle(original_image, (extremes.x_center, extremes.y_center), 3, (0,0,255), -1)
		ret_fields = [0,0, extremes.min_x, extremes.min_y, extremes.max_x, extremes.max_y, extremes.x_mid, extremes.y_mid, int(image_width / 2), int(image_height / 2)]
	
	return ret_fields, original_image


def get_extremes(tuple_points):
	mx, mix, my, miy = -1, sys.maxsize, -1, sys.maxsize
	for point in tuple_points:
		mx = point[0] if point[0] > mx else mx
		mix = point[0] if point[0] < mix else mix
		my = point[1] if point[1] > my else my
		miy = point[1] if point[1] < miy else miy

	return BoundingBoxExtremes(mx, mix, my, miy)


def bounding_box_nonzeros(nz_points):
	rect = cv2.minAreaRect(nz_points)
	box = cv2.boxPoints(rect)
	return np.int0(box)

def tuple_points(box):
	points = list()
	for point in range(len(box)):
		points.append(tuple([box][0][point]))
	return points

def dilate_contours(mask):
	kernel = np.ones((ED_KERNEL_SIZE, ED_KERNEL_SIZE), np.uint8)
	mask = cv2.dilate(mask, kernel, iterations=3)
	return mask

def clean_noise(cv_image_mask, mode='erode'):
	kernel = np.ones((ED_KERNEL_SIZE, ED_KERNEL_SIZE), np.uint8)
	if mode is 'blur' or mode is 'both':
		cv_image_mask = cv2.blur(cv_image_mask, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE))

	if mode is 'erode' or mode is 'both':
		cv_image_mask = cv2.erode(cv_image_mask, kernel, iterations=1)
		cv_image_mask = cv2.dilate(cv_image_mask, kernel, iterations=3)
	return cv_image_mask	


def clean_contours(contours):
	c_contours = list()
	for contour in contours:
		passing = True
		if cv2.contourArea(contour) > CONTOUR_AREA_THRESHOLD:
			passing = True
		else:
			passing = False
		if passing and cv2.contourArea(contour) / cv2.arcLength(contour, True) < 10:
			passing = True
			c_contours.append(contour)
		else:
			passing = False

	return c_contours

def get_contours(cv_image_mask, clean=True):
	im2, contours, hierarcy = cv2.findContours(cv_image_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = clean_contours(contours) if clean else contours
	return contours

def get_points(contours):
	points = list()
	if type(contours[0]) is int:
		return contours
	else:
		points.append(get_points(contours[0]))
	return points