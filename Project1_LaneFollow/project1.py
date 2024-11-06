"""
[ Mission 0 ]

- mission list
1. Lane following
2. End search and stop

- Parameter
1. Kp : PI제어기 P게인
2. Kd : PI제어기 D게인
3. speed = 0 : 주행 속도 초기화
4. end_search_y = 210 : End 검출 y좌표
5. end_dist = 10 : End 검출 후 주행 count
"""
#region import
import cv2
import numpy as np
from ultralytics import YOLO
from pinkylib import Camera
from pinkylib import Pinky
import time
import math
import threading
#endregion import

#region Init
# YOLO 모델과 카메라 초기화
model = YOLO('./yolo_weights/driving_best.pt')
cam = Camera()
cam.set_calibration()
cam.start()

pinky = Pinky()
pinky.start_motor()
pinky.enable_motor()

# 카메라 프레임의 중심점 정의
frame_width = 640
frame_height = 480
center_x = frame_width // 2
center_y = frame_height

# PD 제어기의 게인 설정
#Kp = 1.2  # 비례 게인
#Kd = 0.5  # 미분 게인
Kp = 1  # 비례 게인
Kd = 0.2  # 미분 게인

# PD 제어를 위한 이전 각도 오차 변수 초기화
previous_angle_error = 0.0

# 전역 변수로 조향각 정의
steering_angle = 0

# 전역 변수로 주행속도 정의
speed = 0

# end관련 변수
end_search_y = 210
end_flag = 0
end_cnt = 0
end_dist = 1

#endregion Init

# 스레드 동기화를 위한 Lock 객체
lock = threading.Lock()



def lane(frame, result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 사각형 그리기
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 클래스 이름 출력
    label = f"Lane : {result.cls}"  # 클래스 ID 출력, 필요시 클래스 이름으로 변경
    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    lane_crop = frame[y1:y2, x1:x2]  # 해당 영역 잘라내기

    # lane_crop 이미지를 그레이스케일로 변환
    gray_lane = cv2.cvtColor(lane_crop, cv2.COLOR_BGR2GRAY)
    
    # 이진화 적용
    _, binary_lane = cv2.threshold(gray_lane, 127, 255, cv2.THRESH_BINARY)
    
    # 검정색 부분의 무게 중심 계산
    moments = cv2.moments(binary_lane)
    if moments["m00"] != 0:  # 면적이 0이 아닌 경우에만 계산
        cx = int(moments["m10"] / moments["m00"]) + x1  # 전체 프레임 좌표로 변환
        cy = int(moments["m01"] / moments["m00"]) + y1
        
        # 무게 중심 표시
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
        #print(f"Center of Mass (Black Area): ({cx}, {cy})")
        
        # 조향 선을 프레임의 하단 중앙에서 무게 중심까지 그림
        cv2.line(frame, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
        
        # 각도 계산 (atan2를 통해 세로 축 기준 각도 계산)
        angle = math.degrees(math.atan2(cy - center_y, cx - center_x))
        
        # 중심 세로선 기준 각도를 -90도 ~ 90도로 변환
        steering_angle = angle + 90

    return steering_angle
                        
def end(frame, result):
    global end_flag
    global end_search_y
    
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 사각형 그리기
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 클래스 이름 출력
    label = f"End : {result.cls}"  # 클래스 ID 출력, 필요시 클래스 이름으로 변경
    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 바운딩 박스의 중심 좌표 계산
    end_x = (x1 + x2) // 2
    end_y = (y1 + y2) // 2
    
    # 중심 좌표에 원 그리기
    cv2.circle(frame, (end_x, end_y), 5, (0, 0, 255), -1)  # 빨간색 원
    # center_y 좌표 출력
    cv2.putText(frame, f"end_y: {end_y}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    if end_y > end_search_y :
        end_flag = 1

def calculate_motor_input(steering_angle, speed, max_speed=80, min_speed=20):
    global previous_angle_error

    if(speed >= min_speed) :
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

def motor_control():
    global steering_angle
    global speed
    global end_flag
    global end_cnt
    global end_dist
    
    try:
        while True:
            with lock:  # 조향각에 대한 Lock을 사용하여 동기화
                L, R = calculate_motor_input(steering_angle, speed)

            if end_flag == 1 :
                if end_cnt > end_dist:
                    L = 0
                    R = 0
                else:
                    end_cnt = end_cnt + 1
            pinky.move(L, R)  # 모터에 제어 신호 입력
            #print(f"Motor Input - Left: {L}, Right: {R}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Motor control thread interrupted. Stopping motor...")
        pinky.stop_motor()

def video_processing():
    global steering_angle
    global speed
    global end_flag
    global end_cnt
    global end_search_y
    
    try:
        while True:
            frame = cam.get_frame()
            results = model(frame)
            speed = 50
            
            for result in results[0].boxes:
                if result.cls == 1: # End 클래스 ID로 필터링
                    end(frame, result)
                elif result.cls == 0:  # lane 클래스 ID로 필터링
                    steering_angle = lane(frame, result)
                        
            cv2.putText(frame, f"Steering Angle: {steering_angle:.2f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"End Count : {end_cnt}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            # 결과 프레임을 Jupyter에 출력
            #cam.display_jupyter(frame)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Video processing thread interrupted. Releasing camera...")
        cam.release()

# 스레드 생성
motor_thread = threading.Thread(target=motor_control)
video_thread = threading.Thread(target=video_processing)

# 스레드 시작
motor_thread.start()
video_thread.start()

try:
    motor_thread.join()
    video_thread.join()
except KeyboardInterrupt:
    print("Main thread interrupted. Stopping all processes...")
