import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import RPi.GPIO as gpio
import stepper_motor as sm
import threading

###################### 
''' THIS SECTION FOR TESTING PURPOSES '''

def exit_tests():
    cap.release()
    cv.destroyAllWindows() 


######################
        
def detect_rotation(midpoints : tuple, precision: float = 0.06) -> tuple:
    res = ["C", "C"]
    x,y = midpoints
    if x  <= 0.5 + precision and x  >= 0.5 - precision:
        horizontal.movement_direction = None
        
    elif x > 0.5:
        res[0] = "L"
        horizontal.movement_direction = False

    else:
        res[0] = "R"
        horizontal.movement_direction = True
    
    if y <= 0.5 + precision and y >= 0.5 - precision:
        vertical.movement_direction = None

    elif y > 0.5:
        res[1] = "D"
        vertical.movement_direction = False

    else:
        res[1] = "U"
        vertical.movement_direction = True

    return tuple(res)
        



# RasPi GPIO-PINs Assignment
Ypins = [31, 33, 35, 37]
Xpins = [32, 36, 38, 40]


horizontal = sm.StepperMotor(Xpins)
vertical = sm.StepperMotor(Ypins)
threading.Thread(target=horizontal.movement).start()
threading.Thread(target=vertical.movement).start()

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
        proccess_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        # frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        res = face_detector.process(proccess_frame)
        
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
                
                cv.rectangle(frame, face_react, color=(0, 0, 255), thickness=2)
                
                key_points = []
                for p in face.location_data.relative_keypoints:
                    key_points.append((p.x, p.y))
                midpoint_x += key_points[2][0]
                midpoint_y += key_points[2][1]
                key_points_cords = np.multiply(np.array(key_points),
                                                [fwidth,fheight]).astype(int)
                for p in key_points_cords:
                    cv.circle(frame, p, 2, (255, 255, 255), 2)
                    cv.circle(frame, p, 2, (0, 0, 0),-1)

            avg_midpoints = midpoint_x / len(res.detections),midpoint_y / len(res.detections)
            horizontal.movement_timeout, vertical.movement_timeout = round(0.006 - abs(avg_midpoints[0] - 0.5)/100, 4),round(0.006 - abs(avg_midpoints[1] - 0.5)/100,4 )
            # print(vertical_angel, horizontal_angel)
            rotaition = detect_rotation(avg_midpoints)
            cv.circle(frame, np.multiply(np.array(avg_midpoints),[fwidth,fheight]).astype(int), 4, (255, 0, 0), 2)
            cv.circle(frame, np.multiply(np.array(avg_midpoints),[fwidth,fheight]).astype(int), 4, (255, 0, 255),-1)

            cv.putText(frame, f"{rotaition[0]} {rotaition[1]}",org=(fwidth // 2,30) ,fontFace=0 ,fontScale=1,thickness=2, color=(255, 255, 0),)
        else :
            horizontal.movement_direction, vertical.movement_direction = None, None
            horizontal.movement_timeout, vertical.movement_timeout = 0.006, 0.006
        fps = frame_counter / (time.time() - start_time)
        
        # Display the FPS on top-left  of the frame
        cv.putText(frame, f"FPS: {fps:.1f}",org=(30,30) ,fontFace=cv.FONT_HERSHEY_PLAIN,fontScale=2,thickness=2, color=(0, 255, 255),)

        cv.namedWindow("frame", cv.WINDOW_NORMAL)
        # cv.setWindowProperty("frame", prop_value=cv.WINDOW_FULLSCREEN, prop_id=cv.WND_PROP_FULLSCREEN)
        cv.imshow("frame", frame)
        if cv.waitKey(1) & 0xFF == ord("q"):
            break
    horizontal.movement_direction = -1
    vertical.movement_direction = -1
    cap.release()
    cv.destroyAllWindows()
    gpio.cleanup()
