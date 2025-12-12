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
    print("🔊 ALERT: calling say('Wake up')") 
    os.system('say "Wake up"')



LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [263, 387, 385, 362, 380, 373]
MOUTH_POINTS = [13, 14, 78, 308]



BASE_EYE_THRESHOLD = 0.21   
BASE_MAR_THRESHOLD = 0.5

HEAD_TILT_THRESHOLD = 15.0  
HEAD_TILT_FRAMES    = 30    

CLOSED_FRAMES       = 20    
YAWN_FRAMES         = 15  



CALIBRATION_DURATION = 5.0  
calibrating = True
calib_start_time = time.time()
calib_ear_sum = 0.0
calib_mar_sum = 0.0
calib_tilt_sum = 0.0
calib_count = 0

baseline_ear = None
baseline_mar = None
baseline_tilt = 0.0

dynamic_eye_threshold = BASE_EYE_THRESHOLD
dynamic_mar_threshold = BASE_MAR_THRESHOLD



eye_closed_frames  = 0
yawn_open_frames   = 0
head_tilt_frames   = 0



in_blink = False
blink_start_time = None
slow_blink_level = 0.0  
SLOW_BLINK_MIN_DURATION = 0.25  
SLOW_BLINK_MAX_DURATION = 1.00  
SLOW_BLINK_DECAY = 0.98         



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
        fatigue_score = 0.0

        now = time.time()
        slow_blink_level *= SLOW_BLINK_DECAY  

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

            mar = mouth_aspect_ratio(landmarks, MOUTH_POINTS)
            mar_value = mar

            angle = head_tilt_angle(landmarks)
            tilt_angle = angle

            if calibrating:
                elapsed = now - calib_start_time
                if elapsed <= CALIBRATION_DURATION:
           
                    calib_ear_sum  += ear
                    calib_mar_sum  += mar
                    calib_tilt_sum += abs(angle)
                    calib_count    += 1
                else:
                    if calib_count > 0:
                        baseline_ear  = calib_ear_sum / calib_count
                        baseline_mar  = calib_mar_sum / calib_count
                        baseline_tilt = calib_tilt_sum / calib_count

                        dynamic_eye_threshold = baseline_ear * 0.75   
                        dynamic_mar_threshold = baseline_mar * 1.8    

                        print("Calibration done:")
                        print("  baseline EAR:", baseline_ear)
                        print("  baseline MAR:", baseline_mar)
                        print("  baseline tilt:", baseline_tilt)
                        print("  dyn_eye_th:", dynamic_eye_threshold)
                        print("  dyn_mar_th:", dynamic_mar_threshold)
                    else:
                        dynamic_eye_threshold = BASE_EYE_THRESHOLD
                        dynamic_mar_threshold = BASE_MAR_THRESHOLD
                        print("Calibration failed, using base thresholds.")

                    calibrating = False

            eye_thresh = dynamic_eye_threshold
            mar_thresh = dynamic_mar_threshold

            if ear < eye_thresh:
                eye_closed_frames += 1
            else:
                eye_closed_frames = 0

            eye_level = min(1.0, eye_closed_frames / CLOSED_FRAMES)

            eyes_closed_now = (ear < eye_thresh)

            if eyes_closed_now and not in_blink:
                in_blink = True
                blink_start_time = now

            elif not eyes_closed_now and in_blink:
                blink_duration = now - blink_start_time
                in_blink = False
                blink_start_time = None

                if SLOW_BLINK_MIN_DURATION <= blink_duration <= SLOW_BLINK_MAX_DURATION:
                    slow_blink_level = 1.0  

            if mar > mar_thresh:
                yawn_open_frames += 1
            else:
                yawn_open_frames = 0

            yawn_level = min(1.0, yawn_open_frames / YAWN_FRAMES)

            if abs(angle) > HEAD_TILT_THRESHOLD:
                head_tilt_frames += 1
            else:
                head_tilt_frames = 0

            tilt_level = min(1.0, head_tilt_frames / HEAD_TILT_FRAMES)


            is_eye_long   = eye_closed_frames >= CLOSED_FRAMES
            is_yawn_long  = yawn_open_frames >= YAWN_FRAMES
            is_head_long  = head_tilt_frames >= HEAD_TILT_FRAMES

            fatigue_score = (
                60.0 * eye_level +
                20.0 * yawn_level +
                10.0 * tilt_level +
                10.0 * slow_blink_level
            )
            fatigue_score = max(0.0, min(100.0, fatigue_score))

 
            if not calibrating and (is_eye_long or is_yawn_long or is_head_long or fatigue_score >= 50.0):
                is_fatigue = True

            cv2.putText(frame, f"EAR: {ear_value:.2f}", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, f"MAR: {mar_value:.2f}", (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.putText(frame, f"Tilt: {tilt_angle:.1f} deg", (30, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.putText(frame, f"Score: {fatigue_score:.1f}", (30, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            status_text = "NORMAL"
            status_color = (0, 255, 0)

            if eye_level >= 1.0:
                status_text = "EYE CLOSURE"
                status_color = (0, 0, 255)

            if yawn_level >= 1.0:
                status_text = "YAWNING"
                status_color = (0, 165, 255)

            if tilt_level >= 1.0:
                status_text = "HEAD TILT"
                status_color = (255, 0, 0)

            if slow_blink_level > 0.5:
                status_text = "SLOW BLINK"
                status_color = (255, 255, 0)

            if is_fatigue:
                cv2.putText(frame, "FATIGUE ALERT!", (50, 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            cv2.putText(frame, status_text, (30, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        if calibrating:
            cv2.putText(frame, "Calibrating... Please look at the camera",
                        (30, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


        if is_fatigue and not calibrating:
            if now - last_alert_time > ALERT_COOLDOWN:
                play_alert_sound()
                last_alert_time = now


        if arduino is not None and not calibrating:
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
