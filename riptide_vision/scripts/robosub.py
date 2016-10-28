import numpy as np
import cv2

class RoboSub:
	name = ""
	def __init__(self, name):
		self.name = name
	@staticmethod
	def findMostCircularContour(cnts):
		minContour = cnts[0]
		minContourDifference = 100000
		for cnt in cnts:
			contourArea = cv2.contourArea(cnt)
			if(contourArea > 1000):
				(x,y),radius = cv2.minEnclosingCircle(cnt)
				center = (int(x),int(y))
				radius = int(radius)
				circleArea = 3.14159 * radius * radius
				if (circleArea - contourArea) < minContourDifference:
					minContour = cnt
					minContourDifference = circleArea - contourArea
		return minContour
	@staticmethod
	def findColorBuoy(frame, lower, upper, color, overlay):
		blur = cv2.GaussianBlur(frame,(5,5),0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, lower, upper)
		#cv2.imshow("output",mask)

		(_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		if(len(cnts) > 0):
			c = RoboSub.findMostCircularContour(cnts)
			if cv2.contourArea(c) < 500:
				return
			peri = cv2.arcLength(c, True)

			contours_x = 0
			contours_y = 0
			contours_count = 0
			for line in c:
				contours_count = contours_count + 1
				contours_x = contours_x + line[0][0]
				contours_y = contours_y + line[0][1]
			if(contours_count > 0):
				contours_x = contours_x / contours_count
				contours_y = contours_y / contours_count

				center = (contours_x,contours_y)
				cv2.circle(overlay,center,5,(255,0,200),-1)
				cv2.putText(overlay,color,center, 1, 2,(255,255,255), 1,cv2.LINE_AA)
			approx = cv2.approxPolyDP(c, 0.1 * peri, True)
			cv2.drawContours(overlay, [approx], -1, (255, 0, 150), 3)

			x,y,w,h = cv2.boundingRect(c)
			cv2.rectangle(overlay,(x,y),(x+w,y+h),(255,0,200),2)

			(x,y),radius = cv2.minEnclosingCircle(c)
			center = (int(x),int(y))
			radius = int(radius)
			cv2.circle(overlay,center,radius,(255,0,200),2)