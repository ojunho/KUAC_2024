import cv2
import numpy as np

def detect_traffic_light(image):
    # 이미지 색상 공간 변환 (BGR -> HSV)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 신호등의 색상 범위 정의 (HSV 색상 범위)
    # 빨간색
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # 노란색
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    # 초록색
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    # 색상 범위에 따라 마스크 생성
    red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # 마스크를 원본 이미지에 적용
    red_result = cv2.bitwise_and(image, image, mask=red_mask)
    yellow_result = cv2.bitwise_and(image, image, mask=yellow_mask)
    green_result = cv2.bitwise_and(image, image, mask=green_mask)

    # 마스크 이미지에서 신호등 위치 검출
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 신호등을 원본 이미지에 표시
    for contour in red_contours:
        if cv2.contourArea(contour) > 500:  # 면적 필터링
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)  # 빨간색 박스
            print("RED")

    for contour in yellow_contours:
        if cv2.contourArea(contour) > 500:  # 면적 필터링
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)  # 노란색 박스
            print("YELLOW")

    for contour in green_contours:
        if cv2.contourArea(contour) > 500:  # 면적 필터링
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # 초록색 박스
            print("GREEN")

    return image

# 이미지 로드
image = cv2.imread('traffic_light.jpg')

# 신호등 검출
result_image = detect_traffic_light(image)

# 결과 이미지 표시
cv2.imshow('Detected Traffic Lights', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
