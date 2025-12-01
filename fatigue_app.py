import cv2
import numpy as np
import mediapipe as mp
import serial
import time

# --------------------------
# محاولة فتح اتصال مع Arduino
# --------------------------

arduino = None
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print("Arduino connected on COM3")
except:
    print("WARNING: Arduino not connected – running software-only mode")

# --------------------------
# إعداد FaceMesh
# --------------------------

mp_face_mesh = mp.solutions.face_mesh

def eye_aspect_ratio(landmarks, eye_points):
    A = np.linalg.norm(landmarks[eye_points[1]] - landmarks[eye_points[5]])
    B = np.linalg.norm(landmarks[eye_points[2]] - landmarks[eye_points[4]])
    C = np.linalg.norm(landmarks[eye_points[0]] - landmarks[eye_points[3]])
    ear = (A + B) / (2.0 * C)
    return ear

LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [263, 387, 385, 362, 380, 373]

EYE_THRESHOLD = 0.21
CLOSED_FRAMES = 20

counter = 0

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

            if ear < EYE_THRESHOLD:
                counter += 1
            else:
                counter = 0

            cv2.putText(frame, f"EAR: {ear:.2f}", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if counter >= CLOSED_FRAMES:
                is_fatigue = True
                cv2.putText(frame, "FATIGUE ALERT!", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        # --------------------------
        # إرسال الحالة إلى Arduino
        # --------------------------

        if arduino is not None:
            try:
                if is_fatigue:
                    arduino.write(b'R')   #أحمر+جرس=ارهاق
                else:
                    arduino.write(b'G')   #اخضر=طبيعي
            except:
                pass

        cv2.imshow("Fatigue Detection", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
