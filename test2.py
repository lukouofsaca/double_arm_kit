import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import time
import queue
import numpy as np
result_queue = queue.Queue(1)
angle_queue = queue.Queue(1)
from roh_develop_addon import *
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult

def init():    
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path='../camera/hand_landmarker.task'),
        running_mode=VisionRunningMode.LIVE_STREAM,
        num_hands=2,
        result_callback=draw_landmarks_on_image
    )
    landmarker = HandLandmarker.create_from_options(options)
    return landmarker

# def print_result(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
#     # print('hand landmarker result: {}'.format(result))
#     frame = output_image.numpy_view().copy()
#     for i, landmark in enumerate(result.hand_landmarks):
#         hand_name = result.handedness[i][0].category_name
#         for j, point in enumerate(landmark):
#             x = int(point.x * frame.shape[1])
#             y = int(point.y * frame.shape[0])
#             if hand_name == 'Left':
#                 cv2.circle(frame, (x, y), 3, (255,255,0), -1)
#             else:
#                 cv2.circle(frame, (x, y), 3, (0,255,255), -1)
#     result_queue.put(frame)

from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions

def draw_landmarks_on_image(detection_result,output_image,timestamp_ms: int):
    MARGIN = 10  # pixels
    FONT_SIZE = 1
    FONT_THICKNESS = 1
    HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green
    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness
    annotated_image = output_image.numpy_view().copy()

    # Loop through the detected hands to visualize.
    angles = []
    for idx in range(len(hand_landmarks_list)):
        hand_landmarks = hand_landmarks_list[idx]
        handedness = handedness_list[idx]



        angles_dir = []
        # try:
            
        for i in range(0,4):
            vector1 = get_vector(hand_landmarks[5+i*4],hand_landmarks[0])
            vector2 = get_vector(hand_landmarks[5+i*4],hand_landmarks[6+i*4])

            angle = calculate_vector2(vector1,vector2)
            angles_dir.append(angle)
        # print(angles)
        
        nps = np.array([[landmark.x, landmark.y, landmark.z] for landmark in hand_landmarks])
        np.linalg.norm(nps,axis=1)
        
        wrist = nps[0]
        index = nps[5:9]
        middle = nps[9:13]
        ring = nps[13:17]
        little = nps[17:21]
        thumb = nps[1:5]
        normal_v = calculate_hand(index,middle,ring,little,wrist)
        # calculate_thumb(thumb,wrist,normal_v)
        # print(calculate_vector2(normal_v,wrist-index[0]))
        print(calculate_vector2(normal_v,little[1]-little[0]))
        # print(calculate_vector2(wrist-index[0],index[1]-index[0]))
        finger_list = [index,middle,ring,little]
        
        angles = [calculate_vector2(normal_v,finger[1]-finger[0]) for finger in finger_list]
        # input(f'index-wrist = {index[0]-wrist}')   
        # print(f'angle   (dir) cals angles{angles_dir}')
        # print(f'angle(normal) cals angles{angles}')
        
        
        # Draw the hand landmarks.

        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            solutions.hands.HAND_CONNECTIONS,
            solutions.drawing_styles.get_default_hand_landmarks_style(),
            solutions.drawing_styles.get_default_hand_connections_style())

        cv2.putText(annotated_image, f"{handedness[0].category_name}", 
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 
                    FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
        
    result_queue.put(annotated_image)
    try:
        angle_queue.put(angles)
    except Exception as e:
        print(e)


def get_vector(landmark1, landmark2):  #计算向量
    vector = np.array([landmark2.x - landmark1.x, landmark2.y - landmark1.y, landmark2.z - landmark1.z])
    length = np.linalg.norm(vector)#向量的模长
    return vector/length  #返回向量和模长


DEBUG = 0
def calculate_vector2(vector1:np.array, vector2:np.array):      #计算向量角度
    vector1 = vector1/np.linalg.norm(vector1)  #单位向量
    vector2 = vector2/np.linalg.norm(vector2)  #单位向量
    
    if DEBUG:
        print(vector1,vector2)
    dot_product_result = np.dot(vector1, vector2)#cos(angle)=单位向量的点乘
    angle = np.rad2deg(np.arccos(dot_product_result))#angle=arccos(cos(angle))
    return angle
    
    
def calculate_hand(index,middle,ring,little,wrist):
    """calculate the four hand angles
    
    Args:
        index (np[4][3]): index_finger_position;
        
        middle (np[4][3]): middle_finger_position
        ring (np[4][3]): ring_finger_position
        little (np[4][3]): little_finger_position
        wrist (np[3]): wrist_position
    """
    
    # use the average of the index, middle, ring and little fingers to calculate the hand position
    ring_little_v = (little[0]+ring[0])/2-wrist
    index_middle_v = (middle[0]+index[0])/2-wrist
    # calculate the normal vector of the hand
    normal_vector = np.cross(ring_little_v,index_middle_v)
    # calculate the angle between the normal vector and the z-axis   
    return normal_vector

def calculate_thumb(thumb,wrist,normal_vector):
    thumb_angle = np.arccos(np.dot(normal_vector,thumb[1]-thumb[0]))*180/np.pi+90
        
    
def main(landmarker,arm_r):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        begin_time = time.time()
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        landmarker.detect_async(mp_image, int(time.time() * 1000))

        frame = result_queue.get()
        
        try:
            angle = angle_queue.get_nowait()
            angle = [0]+angle+[0]
            angle = [int((anglea-90)/90*65535) for anglea in angle]
            angle = [min(65535,anglea) for anglea in angle]
            angle = [max(0,anglea) for anglea in angle]
            # 滤波
            
            
            register_write(robot=arm_r,data=angle)
        except queue.Empty:
            angle = None
        
        cv2.imshow('frame', frame)
        # print(f'{1 / (time.time() - begin_time):.2f}')

        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()   
    
arm_r = arms_init(arm_ips=[R_ARM_IP])[0]    
landmarker = init()
main(landmarker,arm_r)


