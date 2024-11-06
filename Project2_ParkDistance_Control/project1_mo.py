import cv2
import numpy as np
import time
import math
from pinkylib import Camera, Pinky

# 카메라 초기화
cam = Camera()
cam.start()

pinky = Pinky()
pinky.start_motor()
pinky.enable_motor()

# 카메라 프레임의 중심점 정의
frame_width = 640
frame_height = 480
center_x = frame_width // 2
center_y = frame_height

# 사다리꼴 형태의 마스킹 좌표 정의
top_left = (frame_width // 4, frame_height // 2)
top_right = (frame_width * 3 // 4, frame_height // 2)
bottom_left = (frame_width // 8, frame_height)
bottom_right = (frame_width * 7 // 8, frame_height)
trapezoid_points = np.array([[top_left, top_right, bottom_right, bottom_left]], dtype=np.int32)

# PD 제어기의 게인 설정
# Kp = 0.7  # 비례 게인
# Kd = 0.2  # 미분 게인
Kp = 1.85  # 비례 게인
Kd = 0.35  # 미분 게인

previous_angle_error = 0.0

steering_angle = 0
end_count = 0
end_dist = 1000

def calculate_motor_input(steering_angle, speed, max_speed=99, min_speed=20):
    global previous_angle_error

    if speed >= min_speed:
        # 각도 오차를 0으로 만드는 것이 목표
        angle_error = -steering_angle  # 현재 각도와 0도 사이의 오차
        angle_derivative = angle_error - previous_angle_error  # 오차의 변화율
        previous_angle_error = angle_error  # 현재 오차를 이전 오차로 저장
        
        # PD 제어 계산
        control_output = (Kp * angle_error) + (Kd * angle_derivative)
    
        left_speed = speed + (-control_output / 90) * (max_speed - speed)
        right_speed = speed + (control_output / 90) * (max_speed - speed)
    
        # 속도 제한
        left_speed = max(min_speed, min(max_speed, left_speed))
        right_speed = max(min_speed, min(max_speed, right_speed))
    else:
        left_speed = 0
        right_speed = 0

    # 속도 값을 정수로 변환
    return int(left_speed), int(right_speed)

while True:
    # global end_count
    # global steering_angle
    # global end_dist
    
    frame = cam.get_frame()

    if end_count < end_dist:
        end_count = end_count + 1

        # 사다리꼴 영역을 마스크로 설정하여 관심 영역만 남김
        roi_mask = np.zeros_like(frame)
        cv2.fillPoly(roi_mask, [trapezoid_points], (255, 255, 255))
        roi_frame = cv2.bitwise_and(frame, roi_mask)
    
        # HSV 색 공간으로 변환
        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    
        # 검은색 HSV 범위 정의 (검은색 라인 검출)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv_frame, lower_black, upper_black)
    
        # 사다리꼴 영역에만 검은색 검출을 위한 마스크 적용
        masked_roi = cv2.bitwise_and(mask, cv2.fillPoly(np.zeros_like(mask), [trapezoid_points], 255))
    
        # 검은색 부분의 무게 중심 계산 (사다리꼴 내부만 고려)
        moments = cv2.moments(masked_roi)
        if moments["m00"] != 0:  # 면적이 0이 아닌 경우에만 계산
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
    
            cv2.line(frame, top_left, top_right, (0, 255, 0), 2)       # 위쪽 선
            cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)   # 오른쪽 선
            cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2) # 아래쪽 선
            cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)     # 왼쪽 선
            
            # 무게 중심 표시
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            print(f"Center of Mass (Black Area): ({cx}, {cy})")
            
            # 조향 선을 프레임의 하단 중앙에서 무게 중심까지 그림
            cv2.line(frame, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
            
            # 각도 계산 (atan2를 통해 세로 축 기준 각도 계산)
            angle = math.degrees(math.atan2(cy - center_y, cx - center_x))
            
            # 중심 세로선 기준 각도를 -90도 ~ 90도로 변환
            steering_angle = angle + 90

        # PD 제어를 통해 L, R 모터 입력값 계산
        L = 0
        R = 0
        L, R = calculate_motor_input(steering_angle, 88)
        print(L,R)
        
    else:
        L = 0
        R = 0

    #L = 0
    #R = 0
    pinky.move(L, R)  # 모터에 제어 신호 입력
    print(f"Motor Input - Left: {L}, Right: {R}")

    cv2.putText(frame, f"Steering Angle: {steering_angle:.2f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    #cv2.putText(frame, f"Count : {end_count}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # 결과 프레임을 Jupyter에 출력
    cam.display_jupyter(frame)
    time.sleep(0.1)
