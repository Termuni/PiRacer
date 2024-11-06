"""
[ Mission 1 ]
- Mission List
- Class
- Parameter
"""
import cv2
import numpy as np
from ultralytics import YOLO
from pinkylib import Camera
from pinkylib import Pinky
import time
import math
import threading

#region init
# YOLO 모델과 카메라 초기화
model = YOLO('./yolo_weights/week2_best_ver2.pt')
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

# 

# 사다리꼴 형태의 마스킹 좌표 정의
top_left = (frame_width // 4, frame_height // 2)
top_right = (frame_width * 3 // 4, frame_height // 2)
bottom_left = (frame_width // 8, frame_height)
bottom_right = (frame_width * 7 // 8, frame_height)
trapezoid_points = np.array([[top_left, top_right, bottom_right, bottom_left]], dtype=np.int32)

# PD 제어기의 게인 설정
Kp = 1.0  # 비례 게인
Kd = 0.3  # 미분 게인

# PD 제어를 위한 이전 각도 오차 변수 초기화
previous_angle_error = 0.0

# 전역 변수로 조향각 정의
steering_angle = 0
speed = 0

# 미션 아루코 리스트
mission_array = [0, 8, 4, 1, 9, 5, 2, 10, 6, 3, 11, 7]
mission_cnt = 0
mission_1_flag = 0
mission_1_cnt = 0
mission_2_flag = 0
mission_2_cnt = 0
mission_3_flag = 0
mission_3_cnt = 0
mission_4_flag = 0
mission_4_cnt = 0

# 미션 관련 flag
done = 0
end_flag = 0
end_cnt = 0
edge_flag = 0
edge_cnt = 0
left_flag = 0
left_cnt = 0
right_flag = 0
right_cnt = 0
lane_cross_cnt = 0 
lane_cross_flag = 0
t_cross_cnt = 0
t_cross_flag = 0
crosswalk_cnt = 0
crosswalk_flag = 0
exit_flag = 0
exit_dir = 0


# 스레드 동기화를 위한 Lock 객체
lock = threading.Lock()

#endregion init

#region API

#region GET
def Get_Mission_Cnt():
    global mission_cnt
    return mission_cnt

def Get_Mission_1_Cnt():
    global mission_1_cnt
    return mission_1_cnt

def Get_Mission_1_Flag():
    global mission_1_flag
    return mission_1_flag

def Get_Mission_2_Cnt():
    global mission_2_cnt
    return mission_2_cnt

def Get_Mission_2_Flag():
    global mission_2_flag
    return mission_2_flag

def Get_Mission_3_Cnt():
    global mission_3_cnt
    return mission_3_cnt

def Get_Mission_3_Flag():
    global mission_3_flag
    return mission_3_flag

def Get_Mission_4_Cnt():
    global mission_4_cnt
    return mission_4_cnt

def Get_Mission_4_Flag():
    global mission_4_flag
    return mission_4_flag

def Get_End_Cnt():
    global end_cnt
    return end_cnt

def Get_End_Flag():
    global end_flag
    return end_flag

def Get_Edge_Cnt():
    global edge_cnt
    return edge_cnt

def Get_Edge_Flag():
    global edge_flag
    return edge_flag

def Get_Left_Cnt():
    global left_cnt
    return left_cnt    

def Get_Left_Flag():
    global left_flag
    return left_flag

def Get_Right_Cnt():
    global right_cnt
    return right_cnt

def Get_Right_Flag():
    global right_flag
    return right_flag

def Get_Lane_Cross_Cnt():
    global lane_cross_cnt
    return lane_cross_cnt
    
def Get_Lane_Cross_Flag():
    global lane_cross_flag
    return lane_cross_flag

def Get_T_Cross_Cnt():
    global t_cross_cnt
    return t_cross_cnt

def Get_T_Cross_Flag():
    global t_cross_flag
    return t_cross_flag

def Get_Crosswalk_Cnt():
    global crosswalk_cnt
    return crosswalk_cnt

def Get_Crosswalk_Flag():
    global crosswalk_flag
    return crosswalk_flag

def Get_Exit_Dir():
    global exit_dir
    return exit_dir

def Get_Exit_Flag():
    global exit_flag
    return exit_flag
#endregion GET

#region SET

def Set_Mission_Cnt(mission_count):
    global mission_cnt
    mission_cnt = mission_count

def Set_Mission_1_Cnt(mission_count):
    global mission_1_cnt
    mission_1_cnt = mission_count
    
def Set_Mission_1_Flag(flag):
    global mission_1_flag
    mission_1_flag = flag

def Set_Mission_2_Cnt(mission_count):
    global mission_2_cnt
    mission_2_cnt = mission_count
    
def Set_Mission_2_Flag(flag):
    global mission_2_flag
    mission_2_flag = flag

def Set_Mission_2_Cnt(mission_count):
    global mission_2_cnt
    mission_2_cnt = mission_count
    
def Set_Mission_3_Flag(flag):
    global mission_3_flag
    mission_3_flag = flag

def Set_Mission_4_Cnt(mission_count):
    global mission_4_cnt
    mission_4_cnt = mission_count
    
def Set_Mission_4_Flag(flag):
    global mission_4_flag
    mission_4_flag = flag

def Set_End_Cnt(end_count):
    global end_cnt
    end_cnt = end_count

def Set_End_Flag(flag):
    global end_flag
    end_flag = flag

def Set_Edge_Cnt(edge_count):
    global edge_cnt
    edge_cnt = edge_count

def Set_Edge_Flag(flag):
    global edge_flag
    edge_flag = flag

def Set_Left_Cnt(left_count):
    global left_cnt
    left_cnt = left_count

def Set_Left_Flag(flag):
    global left_flag
    left_flag = flag

def Set_Right_Cnt(right_count):
    global right_cnt
    right_cnt = right_count

def Set_Right_Flag(flag):
    global right_flag
    right_flag = flag

def Set_Lane_Cross_Cnt(lane_count):
    global lane_cross_cnt
    lane_cross_cnt = lane_count

def Set_Lane_Cross_Flag(flag):
    global lane_cross_flag
    lane_cross_flag = flag

def Set_T_Cross_Cnt(t_cross_count):
    global t_cross_cnt
    t_cross_cnt = t_cross_count

def Set_T_Cross_Flag(flag):
    global t_cross_flag
    t_cross_flag = flag
    
def Set_Crosswalk_Cnt(crosswalk_count):
    global crosswalk_cnt
    crosswalk_cnt = crosswalk_count

def Set_Crosswalk_Flag(flag):
    global crosswalk_flag
    crosswalk_flag = flag
    
def Set_Exit_Dir(exit_direction):
    global exit_dir
    exit_dir = exit_direction

def Set_Exit_Flag(flag):
    global exit_flag
    exit_flag = flag


def Set_All_Flag_Clr():
    Set_Lane_Cross_Flag(0)
    Set_T_Cross_Flag(0)
    Set_Crosswalk_Flag(0)
    Set_Edge_Flag(0)
    Set_End_Flag(0)

#endregion SET

#endregion API

#region Mission
def left_cross(x, y, z):
    if z < 20  and Get_Edge_Flag() + Get_Left_Flag() + Get_Right_Flag() == 0:
        Set_Left_Flag(1)    #Flag On
    
def right_cross(x, y, z):
    if z < 20 and Get_Edge_Flag() + Get_Left_Flag() + Get_Right_Flag() == 0:
        Set_Right_Flag(1)    #Flag On
    
def Mission_1(x, y, z):
    if z < 20:
        Set_Mission_1_Flag(1)
        
def Mission_2(x, y, z):
    if z < 20:
        Set_Mission_2_Flag(1)

def Mission_3(x, y, z):
    if z < 20:
        Set_Mission_3_Flag(1)

def Mission_4(x, y, z):
    if z < 20:
        Set_Mission_4_Flag(1)
    
def exit_cross(x, y, z):
    if z < 12 and Get_Edge_Flag() + Get_Left_Flag() + Get_Right_Flag() + Get_Exit_Flag() == 0:
        Set_Exit_Flag(1)

def edge(x, y, z):
    if z < 12 and Get_Edge_Flag() + Get_Left_Flag() + Get_Right_Flag() + Get_Exit_Flag() == 0:
        Set_Edge_Flag(1)
#endregion Mission

#region CV2 & YOLO
def lane(frame):
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

    handling_angle = 0
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
        #print(f"Center of Mass (Black Area): ({cx}, {cy})")
        
        # 조향 선을 프레임의 하단 중앙에서 무게 중심까지 그림
        cv2.line(frame, (center_x, center_y), (cx, cy), (255, 0, 0), 2)
        
        # 각도 계산 (atan2를 통해 세로 축 기준 각도 계산)
        angle = math.degrees(math.atan2(cy - center_y, cx - center_x))
        
        # 중심 세로선 기준 각도를 -90도 ~ 90도로 변환
        handling_angle = angle + 90

    return handling_angle
                        
# def end(frame, result): 
def end(result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 바운딩 박스의 중심 좌표 계산
    end_x = (x1 + x2) // 2
    end_y = (y1 + y2) // 2
    
    if end_y > 210 :
        Set_End_Flag(1)

'''
왼쪽과 오른쪽 cross가 동시에 확인되기 때문에,
그냥 둘 중 하나라도 인식 되면 아래의 cross를 실행하도록
'''
# def lane_cross(frame, result):
def lane_cross(result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 바운딩 박스의 중심 좌표 계산
    # lane_cross_x = (x1 + x2) // 2
    lane_cross_y = (y1 + y2) // 2
    if lane_cross_y > 210 :
        Set_Lane_Cross_Flag(1)

# def t_cross(frame, result):    
def t_cross(result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 바운딩 박스의 중심 좌표 계산
    t_crossline_y = (y1 + y2) // 2
    
    if t_crossline_y > 210 :
        Set_T_Cross_Flag(1)

# def crosswalk(frame, result):
def crosswalk(result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    # 바운딩 박스의 중심 좌표 계산
    crosswalk_y = (y1 + y2) // 2
    
    if crosswalk_y > 210 :
        Set_Crosswalk_Flag(1)

# def edge(frame, result):
def edge(result):
    x1, y1, x2, y2 = map(int, result.xyxy[0])  # 바운딩 박스 좌표 가져오기
    edge_y = (y1 + y2) // 2
    
    if edge_y > 210 :
        Set_Edge_Flag(1)

#endregion CV2 & YOLO


#region MOTOR
def calculate_motor_input(handling_angle, speed, max_speed=30, min_speed=20):
    global previous_angle_error

    if(speed >= min_speed) :
        # 각도 오차를 0으로 만드는 것이 목표
        angle_error = -handling_angle  # 현재 각도와 0도 사이의 오차
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
    global done

    speed = 22
    
    while True:
        with lock:  # 조향각에 대한 Lock을 사용하여 동기화
            L, R = calculate_motor_input(steering_angle, speed)

        
        if done == 1:
            L = 0
            R = 0

        # L = 0
        # R = 0
        pinky.move(L, R)  # 모터에 제어 신호 입력
        #print(f"Motor Input - Left: {L}, Right: {R}")
        time.sleep(0.1)

#endregion MOTOR

#region VIDEO

def video_processing():
    global steering_angle
    global mission_array
    global mission_cnt
    
    while True:
        start_time = time.time()
        frame = cam.get_frame()

        # PT 기반 인식
        '''
        1 = lane
        2 = left_cross_line
        3 = right_cross_line
        4 = T_cross_line
        5 = edge
        6 = crosswalk
        7 = end
        '''
        results = model(frame)
        for result in results[0].boxes:
            # Set_All_Flag_Clr()
            if (result.cls == 2) or (result.cls == 3):
                lane_cross(result)
            elif result.cls == 4:
                t_cross(result)
            elif result.cls == 5:
                edge(result)
            elif result.cls == 6:
                crosswalk(result)
            elif result.cls == 7:
                end(result)
        
        # Line 주행
        # lane_frame = frame.copy()
        # steering_angle = lane(lane_frame)
        steering_angle = lane(frame)
        
        #디버깅
        end_time = time.time()
        # print(f"{end_time - start_time} || CROSS : {Get_Lane_Cross_Flag()}, T-CROSS : {Get_T_Cross_Flag()}, EDGE : {Get_Edge_Flag()}, CROSSWALK : {Get_Crosswalk_Flag()}")
        
        # if Get_Lane_Cross_Flag() == 1:
            
        
        '''
        # 인식된 모든 마커 id, pose
        pose_n_ids = cam.detect_aruco(frame)
        if pose_n_ids is not None:
            for pose_id in pose_n_ids:
                aruco_id = pose_id[0]
                if aruco_id == mission_array[mission_cnt]:
                    if aruco_id == 1 or aruco_id == 3:
                        left_cross(pose_id[1], pose_id[2], pose_id[3])
                    elif aruco_id == 0 or aruco_id == 2:
                        right_cross(pose_id[1], pose_id[2], pose_id[3])
                    elif aruco_id == 4 or aruco_id == 5 or aruco_id == 6 or aruco_id == 7:
                        exit_cross(pose_id[1], pose_id[2], pose_id[3])
                    elif aruco_id == 8 or aruco_id == 9 or aruco_id == 10 or aruco_id == 11:
                        edge_turn(pose_id[1], pose_id[2], pose_id[3])
        '''                 
        
        # cv2.putText(frame, f"mission_cnt : {Get_Mission_Cnt()}", (450, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # cv2.putText(frame, f"mission_cnt : {Get_Mission_Cnt()}", (450, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # 결과 프레임을 Jupyter에 출력
        # cam.display_jupyter(frame)
        # cam.display_jupyter(lane_frame)
        # time.sleep(0.1)

#endregion VIDEO


# 스레드 생성
# motor_thread = threading.Thread(target=motor_control)
video_thread = threading.Thread(target=video_processing)

# 스레드 시작
# motor_thread.start()
video_thread.start()
