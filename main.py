import cv2 as cv
import mediapipe as mp
import numpy as np
import time

###################### 
''' THIS SECTION FOR TESTING PURPOSES '''

def exit_tests():
    cap.release()
    cv.destroyAllWindows() 

######################
        
def detect_rotation(midpoints : tuple, precision: float = 0.03) -> tuple:
    res = [None, None]
    x,y = round(midpoints[0], 2), round(midpoints[1], 2)
    if x  <= 0.5 + precision and x  >= 0.5 - precision:
        pass
    elif x > 0.5:
        res[0] = "left"
    else:
        res[0] = "right"
    
    if y <= 0.5 + precision and y >= 0.5 - precision:
        pass
    elif y > 0.5:
        res[1] = "down"
    else:
        res[1] = "up"

    return tuple(res)
        
mp_face_detection = mp.solutions.face_detection
cap = cv.VideoCapture(0)

start_time = time.time()

# Initialize the FPS counter
frame_counter = 0
with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detector:
    
    while True:
        frame_counter += 1
        ret, frame = cap.read()
        if ret is False:
            break
        rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        # frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        res = face_detector.process(rgb_frame)
        
        fheight, fwidth ,_= frame.shape 
        if res.detections is not None:
            midpoint_x, midpoint_y = 0 , 0
            for face in res.detections:
                face_react = np.multiply(
                    [
                        face.location_data.relative_bounding_box.xmin,
                        face.location_data.relative_bounding_box.ymin,
                        face.location_data.relative_bounding_box.width,
                        face.location_data.relative_bounding_box.height,
                    ],
                    [fwidth, fheight, fwidth, fheight]).astype(int)
                # print(face_react)
                
                cv.rectangle(frame, face_react, color=(0, 0, 255), thickness=2)
                # print(face)
                key_points = []
                for p in face.location_data.relative_keypoints:
                    key_points.append((p.x, p.y))
                midpoint_x += key_points[2][0]
                midpoint_y += key_points[2][1]
                key_points_cords = np.multiply(np.array(key_points),
                                                [fwidth,fheight]).astype(int)
                # print(key_points_cords)
                # Initializing keypoints counter to show
                key_point_counter = 1
                for p in key_points_cords:
                    cv.circle(frame, p, 2, (255, 255, 255), 2)
                    cv.circle(frame, p, 2, (0, 0, 0),-1)
                    # putting keypoints number as text
                    cv.putText(frame, f'{key_point_counter}', p, fontFace=cv.FONT_HERSHEY_PLAIN, fontScale=1, thickness=2, color = (255,255, 0))
                    key_point_counter += 1
            avg_midpoints = (round(midpoint_x / len(res.detections), 2),round(midpoint_y / len(res.detections), 2))
            rotaition = detect_rotation(avg_midpoints, 0.05)
            cv.circle(frame, np.multiply(np.array(avg_midpoints),[fwidth,fheight]).astype(int), 4, (255, 0, 0), 2)
            cv.circle(frame, np.multiply(np.array(avg_midpoints),[fwidth,fheight]).astype(int), 4, (255, 0, 255),-1)

            cv.putText(frame, f"{rotaition[0]} {rotaition[1]}",org=(fwidth // 2,30) ,fontFace=0 ,fontScale=1,thickness=2, color=(255, 255, 0),)
            # print(avg_midpoints)
        fps = frame_counter / (time.time() - start_time)
        
        # Display the FPS on top-left  of the frame
        cv.putText(frame, f"FPS: {fps:.0f}",org=(30,30) ,fontFace=cv.FONT_HERSHEY_PLAIN,fontScale=2,thickness=2, color=(0, 255, 255),)

        cv.namedWindow("frame", cv.WINDOW_NORMAL)
        # cv.setWindowProperty("frame", prop_value=cv.WINDOW_FULLSCREEN, prop_id=cv.WND_PROP_FULLSCREEN)
        cv.imshow("frame", frame)
        if cv.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv.destroyAllWindows() 