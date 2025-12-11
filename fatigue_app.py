import cv2
import numpy as np
import mediapipe as mp
import serial
import time
import math
import os



arduino = None
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected on COM3")
except:
    print("WARNING: Arduino not connected – running software-only mode")



mp_face_mesh = mp.solutions.face_mesh

def eye_aspect_ratio(landmarks, eye_points):
    A = np.linalg.norm(landmarks[eye_points[1]] - landmarks[eye_points[5]])
    B = np.linalg.norm(landmarks[eye_points[2]] - landmarks[eye_points[4]])
    C = np.linalg.norm(landmarks[eye_points[0]] - landmarks[eye_points[3]])
    ear = (A + B) / (2.0 * C)
    return ear

def mouth_aspect_ratio(landmarks, mouth_points):
    upper = landmarks[mouth_points[0]]
    lower = landmarks[mouth_points[1]]
    left  = landmarks[mouth_points[2]]
    right = landmarks[mouth_points[3]]

    A = np.linalg.norm(upper - lower)   
    C = np.linalg.norm(left - right)    
    if C == 0:
        return 0.0
    return A / C

def head_tilt_angle(landmarks, left_eye_idx=33, right_eye_idx=263):
    left_eye = landmarks[left_eye_idx]
    right_eye = landmarks[right_eye_idx]

    dx = right_eye[0] - left_eye[0]
    dy = right_eye[1] - left_eye[1]

    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

def play_alert_sound():
    os.system('say "Wake up"')



LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [263, 387, 385, 362, 380, 373]
MOUTH_POINTS = [13, 14, 78, 308]



EYE_THRESHOLD      = 0.21  
CLOSED_FRAMES      = 20    

MAR_THRESHOLD      = 0.5    
YAWN_FRAMES        = 15    

HEAD_TILT_THRESHOLD = 15.0  
HEAD_TILT_FRAMES    = 30    

eye_closed_frames  = 0
yawn_open_frames   = 0
head_tilt_frames   = 0

last_alert_time = 0
ALERT_COOLDOWN = 3.0 



cap = cv2.VideoCapture(0)

with mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as face_mesh:

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = face_mesh.process(rgb)
        is_fatigue = False  

        ear_value = 0.0
        mar_value = 0.0
        tilt_angle = 0.0

        if result.multi_face_landmarks:
            mesh = result.multi_face_landmarks[0]
            landmarks = []

            for i in range(468):
                x = mesh.landmark[i].x * w
                y = mesh.landmark[i].y * h
                landmarks.append([x, y])
            landmarks = np.array(landmarks)

            left_ear = eye_aspect_ratio(landmarks, LEFT_EYE)
            right_ear = eye_aspect_ratio(landmarks, RIGHT_EYE)
            ear = (left_ear + right_ear) / 2.0
            ear_value = ear

            if ear < EYE_THRESHOLD:
                eye_closed_frames += 1
            else:
                eye_closed_frames = 0

            is_eye_fatigue = eye_closed_frames >= CLOSED_FRAMES

            mar = mouth_aspect_ratio(landmarks, MOUTH_POINTS)
            mar_value = mar

            if mar > MAR_THRESHOLD:
                yawn_open_frames += 1
            else:
                yawn_open_frames = 0

            is_yawning = yawn_open_frames >= YAWN_FRAMES

            angle = head_tilt_angle(landmarks)
            tilt_angle = angle

            if abs(angle) > HEAD_TILT_THRESHOLD:
                head_tilt_frames += 1
            else:
                head_tilt_frames = 0

            is_head_tilt = head_tilt_frames >= HEAD_TILT_FRAMES

            if is_eye_fatigue or is_yawning or is_head_tilt:
                is_fatigue = True


            cv2.putText(frame, f"EAR: {ear_value:.2f}", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, f"MAR: {mar_value:.2f}", (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.putText(frame, f"Tilt: {tilt_angle:.1f} deg", (30, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            status_text = "NORMAL"
            status_color = (0, 255, 0)

            if is_eye_fatigue:
                status_text = "EYE FATIGUE"
                status_color = (0, 0, 255)

            if is_yawning:
                status_text = "YAWNING"
                status_color = (0, 165, 255)  

            if is_head_tilt:
                status_text = "HEAD TILT"
                status_color = (255, 0, 0)

            if is_fatigue:
                cv2.putText(frame, "FATIGUE ALERT!", (50, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            cv2.putText(frame, status_text, (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)


        if is_fatigue:
            now = time.time()
            if now - last_alert_time > ALERT_COOLDOWN:
                play_alert_sound()
                last_alert_time = now


        if arduino is not None:
            try:
                if is_fatigue:
                    arduino.write(b'R')  
                else:
                    arduino.write(b'G') 
            except:
                pass

        cv2.imshow("Yaqthah - Multi-signal Fatigue Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27: 
            break

cap.release()
cv2.destroyAllWindows()
