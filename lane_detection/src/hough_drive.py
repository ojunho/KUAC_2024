#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
motor = None # 모터 토픽을 담을 변수
img_ready = False # 카메라 토픽이 도착했는지의 여부 표시 

pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기
ROI_ROW = 250   # 차선을 찾을 ROI 영역의 시작 Row값 
ROI_HEIGHT = HEIGHT - ROI_ROW   # ROI 영역의 세로 크기  
L_ROW = ROI_HEIGHT - 120  # 차선의 위치를 찾기 위한 기준선(수평선)의 Row값

Imu_msg = None

def imu_callback(data):
    global Imu_msg
    Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z,
               data.orientation.w]

def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True
    

def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

global lower_white
lower_white = np.array([240,240,240])
global upper_white
upper_white = np.array([255,255,255])
global lower_white2
lower_white2 = np.array([130,130,130])
global upper_white2
upper_white2 = np.array([255,255,255])

#=================================
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        #img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), (255,255,255), 1)
    return img

def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255,255,255 ), 2)

    return img, int(pos)

def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    #cv2.imshow('lane', frame)


    return frame,lpos,rpos
#=================================
def color_filter(img):

    mask_white = cv2.inRange(img,lower_white,upper_white)
    white_image = cv2.bitwise_and(img,img,mask = mask_white)
    return white_image
def color_filter2(img):

    mask_white = cv2.inRange(img,lower_white2,upper_white2)
    white_image = cv2.bitwise_and(img,img,mask = mask_white)
    return white_image

def bird_eye_view(img,width,height):
	src = np.float32([[0,0],
                  [width,0],
                  [0,160],
                  [width,160]])

	dst = np.float32([[-100,0],
                  [width+90,0],
                  [160,height],
                  [460,height]])    
         
	M = cv2.getPerspectiveTransform(src,dst)
	M_inv = cv2.getPerspectiveTransform(dst,src)
	img_warped = cv2.warpPerspective(img,M,(width,height)) 
	return img_warped

def bird_eye_view2(img,width,height):
	src = np.float32([[0,0],
                  [width,0],
                  [0,160],
                  [width,160]])

	dst = np.float32([[60,0],
                  [width-60,0],
                  [250,height],
                  [390,height]])    
         
	M = cv2.getPerspectiveTransform(src,dst)
	M_inv = cv2.getPerspectiveTransform(dst,src)
	img_warped = cv2.warpPerspective(img,M,(width,height))  
	return img_warped

def region_of_interest(img):
	height = 480
	width = 640
	mask = np.zeros((height,width),dtype="uint8")

	pts = np.array([[0,0],[500,0],[500,480],[0,480]])  
	

	mask= cv2.fillPoly(mask,[pts],(255,255,255),cv2.LINE_AA)


	img_masked = cv2.bitwise_and(img,mask)
	return img_masked


def steering_Angle(R):
	if R == 0:
		return 0
	if R != 0:
		angle = np.arctan(0.34/R) 
		#if angle*180/np.pi < 0.03:  #Angle값이 매우 작은 경우 직진상태로 판단하여 0을 return 하도록 하였습니다.
		#	return 0
		#else:		
		return angle*180/np.pi # arctan로 계산한 값이 radian값이기 때문에 degree로 변환하여 return하였습니다.


def Radius(rx, ry):
	a=0
	b=0
	c=0
	R=0
	h=0
	w=0  #변수들을 초기화하는 과정입니다.
	if (rx[-3] - rx[11] != 0): 
		a = (ry[-3] - ry[11]) / (rx[-3] - rx[11])
		b = -1
		c = ry[11] - rx[11] * (ry[-3] - ry[11]) / (rx[-3] - rx[11])
		h = abs(a * np.mean(rx) + b * np.mean(ry) + c) / math.sqrt(pow(a, 2) + pow(b, 2))
		w = math.sqrt(pow((ry[-3] - ry[11]), 2) + pow((rx[-3] - rx[11]), 2))
             #rx,ry 는 각 window 조사창 내에 속해있는 흰색 픽셀들의 픽셀좌표의 평균을 담아놓은 리스트입니다.
	     #rx[-1]은 제일 위에있는 window를, rx[3]은 아래에서 4번째에 있는 window를 의미합니다.
	     #rx[0]대신 rx[3]을 이용한 이유는 시뮬레이터상의 카메라 높이가 낮아 차량에 가까운 차선이 인식이 불안정하였기 때문입니다.

	if h != 0:
		R = h / 2 + pow(w, 2) / h * 8
	
	return R*0.85/450
	#220 , 390

def mouse_callback(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDOWN:
		print("x=", x,"y=",y)
def sliding_window(img_masked,left_prob,right_prob):
    
    nwindows = 17
    window_height = 20 

    
    
 
    margin = 25
    minpix= 5
    
    out_img = np.dstack((img_masked, img_masked, img_masked)) * 255 

    histogram = np.sum(img_masked[img_masked.shape[0] // 2:, :], axis=0) 

    midpoint = 320
    leftx_current = np.argmax(histogram[:250])   
    rightx_current = np.argmax(histogram[midpoint+80:]) + midpoint+80

    if (abs(leftx_current - x_temp) >30):
        leftx_current = x_temp
    if (abs(rightx_current-y_temp)>30):
        rightx_current= y_temp

    y_temp = rightx_current    
    x_temp = leftx_current 
    nz = img_masked.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    left_lane_inds2 = []
    right_lane_inds2 = []

    lx, ly, rx, ry = [], [], [], []
    lx2, ly2, rx2, ry2 = [], [], [], [] 
 

    

    for window in range(nwindows):

        win_yl = img_masked.shape[0] - (window + 1) * window_height 
        win_yh = img_masked.shape[0] - window * window_height

        win_xll = leftx_current - margin 
        win_xlh = leftx_current + margin 
        win_xrl = rightx_current - margin 
        win_xrh = rightx_current + margin


        good_left_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0] 

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds) 

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nz[1][good_right_inds])) 

        lx.append(leftx_current) 
        ly.append((win_yl + win_yh) / 2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh) / 2) 

        cv2.rectangle(out_img, (win_xll, win_yl), (win_xlh, win_yh), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 2) 

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds], nz[1][right_lane_inds]] = [0, 0, 255] 



    

    return out_img ,lx,ly,rx,ry,good_left_inds,good_right_inds 


def hough(img):
	img_blur = cv2.GaussianBlur(img,(5,5),0)
	img_edge = cv2.Canny(img_blur,50,300)
	line = cv2.HoughLinesP(img_edge,1,np.pi/180,50,None,50,5)
	if line is not None:
		for i in range(0,len(line)):
			l = line[i][0]
			#cv2.line(img_edge,(l[0],l[1]),(l[2],l[3]),(0,0,255),3,cv2.LINE_AA)
	return img_edge
	


def start():
    rospy.init_node('h_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    #rospy.Subscriber("imu", Imu, imu_callback)
    #lx,ly,rx,ry = [220,0,0,0,0,130] , [] , [390,0,0,0,0,130] , []   
    max_angle = 0
    left_lane_inds , right_lane_inds = [], [] 
    global image, img_ready, motor
    time_flag = False
    avg_time = 0.0

    

    print ("----- Xycar self driving -----")

    left_prob,right_prob = 0,0
    lpos=rpos =0
    r_tmp = 570 
    ecnt= 0
    bcnt=0 
    tmp_angle =0
    f_yaw =0
    yaw_cnt = 0

    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    #while Imu_msg == None:
    #	continue

    while not rospy.is_shutdown():

        
		while img_ready == False:
			continue
		#(roll, pitch, yaw) = euler_from_quaternion(Imu_msg) 
		#print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (roll, pitch, yaw))
		#if(yaw_cnt ==0):
		#	f_yaw = yaw
		#	yaw_cnt +=1
		
		    
		img = image.copy() 
		display_img = img  
          
	
		image = image.copy()
		try:
			img_frame,lpos,rpos = process_image(image)
			img_roi = img_frame[315:,0:]
		except TypeError:
			img_frame = image
			img_roi = img_frame[315:,0:]
		except ValueError:
			img_frame = image
			img_roi = img_frame[315:,0:]
	
	
		height,width = 480,640
		is_curve = False
		img_filtered = color_filter(img_roi)   
	
		img_warped = bird_eye_view(img_filtered,width,height)
	
	
		_, L, _ = cv2.split(cv2.cvtColor(img_warped, cv2.COLOR_BGR2HLS))
	 

		img_masked = region_of_interest(L) 
	
	
	
		out_img,lx,ly,rx,ry,left_lane_inds,right_lane_inds = sliding_window(L,left_prob,right_prob)

	
	
	
	
		img_blended = cv2.addWeighted(out_img, 1, img_warped, 0.6, 0)

	
		left_prob = np.mean(np.count_nonzero(left_lane_inds))
		right_prob = np.mean(np.count_nonzero(right_lane_inds)) 
		lx_mean = np.mean(lx)
		rx_mean = np.mean(rx)
	
	
		if(right_prob > left_prob+100):
			R = Radius(rx,ry)
		else:
			R = Radius(lx,ly)
	
	
		angle = steering_Angle(R)*4
		if(lx[-3] < lx[11] or rx[-3]<rx[11]):	
			R = Radius(rx,ry)
			angle = steering_Angle(R)*-2
			
		if(abs(angle)<3):
			angle = 0
	

	
		
		
		
			

	
	
        
        
	
	

	


		
	
		#print(lpos,rpos,(rpos+lpos)/2)
	
        
		#cv2.imshow("Color",img_warped)
		#cv2.imshow("warped",img_frame)
		#cv2.imshow("frame",img_blended)
	
	
		cv2.waitKey(1)
 
        
        
		
	
		
	
	

		if(abs(angle)<3):
			angle= 0 
	
		
	 	


		
		#if(rpos<450 ):
		#	if(-7<=angle<=0):
		#		angle = -7	

		speed_ctrl = abs(angle)
		if(speed_ctrl==0):
			speed_ctrl=1
		elif(speed_ctrl>45):
			speed_ctrl=45
		speed = 18-((6*speed_ctrl)/30)
	
		if(speed>=18 and ecnt <=0):
			if((lpos+rpos)/2 -303 >2):
				if(angle<5):
					angle = 3
			elif((lpos+rpos)/2 -303<-2):
				if(angle>-5):
					angle = -3
			speed=18
		elif(speed >= 18 and ecnt>=0):
			speed = 12
		elif (speed<=12 or ecnt>=20):
			speed = 12
			#if(f_yaw-0.5<=yaw<=f_yaw+0.5):
			#	speed = 20
		

		if(abs(angle)>40):
			if angle>0:
				angle =50
			elif angle<0:
				angle = -50
		
		
	
		#if(abs(angle)<5):
		#	if((lpos+rpos)/2 -303 >7):
		#		if(angle<5):
		#			angle = 5
		#	elif((lpos+rpos)/2 -303<-7):
		#		if(angle>-5):
		#			angle = -5
		
		
		
			
		if(left_prob<50 and right_prob<50):
			if(tmp_angle<0):
				angle = -50
			elif(tmp_angle>0):
				angle = 50
		if(tmp_angle == -50 and ecnt>20):
			#if(right_prob <50):
			#	angle = -50
			if(angle < 0):
				angle = -50
		if(tmp_angle>=36 and ecnt>20):
			if(angle>0):
				angle=36
			
				
		if(lpos>120):
			if(0<=angle<=12):
				angle = 12
		tmp_angle = angle

		
		
		

		drive(angle-5,speed)
		if(abs(angle)>30):
			ecnt+=1
		elif(abs(angle)<10):
			ecnt-=1
		if(ecnt>50):
			ecnt = 50
		elif(ecnt<=-10):
			ecnt = -10
		
		
    	
#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

