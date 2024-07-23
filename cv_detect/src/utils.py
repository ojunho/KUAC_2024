# utils.py

import cv2
import numpy as np

def roi_for_lane(image):
    """이미지의 하단 부분만 사용하여 ROI를 설정하는 함수"""
    return image[246:396, :]

def process_image(image):
    """이미지 전처리를 수행하는 함수"""
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    
    # 트랙바에서 가져온 현재 값들
    gaussian_kernel_size = 13
    adaptive_thresh_C = -2.2
    adaptive_thresh_block_size = 9
    canny_lower = 33
    canny_upper = 255
    morph_kernel_size = 1

    # 가우시안 블러
    blurred_image = cv2.GaussianBlur(gray_img, (gaussian_kernel_size, gaussian_kernel_size), 0)

    # 적응형 이진화
    adaptive_gaussian = cv2.adaptiveThreshold(blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              cv2.THRESH_BINARY, adaptive_thresh_block_size, adaptive_thresh_C)
    
    # 캐니 에지 검출
    edged = cv2.Canny(adaptive_gaussian, canny_lower, canny_upper)

    # 형태학적 닫기 연산
    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    closed_image = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

    return gray_img, blurred_image, adaptive_gaussian, edged, closed_image

def warper(image):
    """원근 변환을 수행하는 함수"""
    y, x = image.shape[0:2]

    left_margin_1 = 0
    top_margin_1 = 130
    
    left_margin_2 = 180
    top_margin_2 = 54

    src_point1 = [left_margin_1, top_margin_1]  # 왼쪽 아래 점
    src_point2 = [left_margin_2, top_margin_2]  # 왼쪽 위 점
    src_point3 = [x - left_margin_2, top_margin_2]  # 오른쪽 위 점
    src_point4 = [x - left_margin_1, top_margin_1]  # 오른쪽 아래 점

    src_points = np.float32([src_point1, src_point2, src_point3, src_point4])  # 원본 이미지에서의 점들
    
    dst_point1 = [x // 4, y]  # 변환 이미지에서의 왼쪽 아래 점
    dst_point2 = [x // 4, 0]  # 변환 이미지에서의 왼쪽 위 점
    dst_point3 = [x // 4 * 3, 0]  # 변환 이미지에서의 오른쪽 위 점
    dst_point4 = [x // 4 * 3, y]  # 변환 이미지에서의 오른쪽 아래 점

    dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])  # 변환 이미지에서의 점들
    
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)  # 원근 변환 행렬 계산
    warped_img = cv2.warpPerspective(image, matrix, (x, y))  # 원근 변환 적용
    
    return warped_img
