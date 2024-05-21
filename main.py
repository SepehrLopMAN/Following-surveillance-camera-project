import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import RPi.GPIO as gpio
import stepper_motor as sm
import multiprocessing




###################### 

''' UTILITY FUNCTIONS '''
       
def detect_rotation(midpoints : tuple, precision: float = 0.068) -> tuple:
    res = ["C", "C"]
    x, y = midpoints
    if x <= 0.5 + precision and x >= 0.5 - precision:
        horizontal_direction.value = 0
    elif x > 0.5:
        res[0] = "L"
        horizontal_direction.value = -1
    else:
        res[0] = "R"
        horizontal_direction.value = 1

    if y <= 0.5 + precision and y >= 0.5 - precision:
        vertical_direction.value = 0
    elif y > 0.5:
        res[1] = "D"
        vertical_direction.value = -1
    else:
        res[1] = "U"
        vertical_direction.value = 1

    return tuple(res)
        

######################

''' RasPi GPIO-PINs Assignment '''
Ypins = [31, 33, 35, 37]
Xpins = [32, 36, 38, 40]

''' Creating IPC variables 
    Define shared variables for movement directions and timeouts
    0: None , 
    1: Positive movement towards axis ,
    -1: negative movement towards axis
'''
horizontal_direction = multiprocessing.Value('i', 0) 
vertical_direction = multiprocessing.Value('i', 0)
horizontal_timeout = multiprocessing.Value('d', 0.006)
vertical_timeout = multiprocessing.Value('d', 0.006)

''' Tools & Object Assignment '''
mp_face_detection = mp.solutions.face_detection
horizontal_motor = sm.StepperMotor(Xpins, horizontal_direction, horizontal_timeout)
vertical_motor = sm.StepperMotor(Ypins, vertical_direction, vertical_timeout)


''' Creating Processes for stepper motors movements '''
horizontal_movement_process = multiprocessing.Process(target=horizontal_motor.movement)
vertical_movement_process = multiprocessing.Process(target=vertical_motor.movement)


######################

''' Main Function / Process '''

if __name__ == '__main__':

    ''' Assigning capture device '''
    cap = cv.VideoCapture(0)

    ''' Measuring time for Live-FPS calculations '''
    start_time = time.time()

    ''' Initialize the FPS counter '''
    frame_counter = 0

    ''' Starting rotational movement processes '''
    horizontal_movement_process.start()
    vertical_movement_process.start()

    with mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5) as face_detector:
        while True:
            frame_counter += 1

            ''' Capturing live-feed frames '''
            ret, frame = cap.read()
            if ret is False:
                break
            proccess_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            res = face_detector.process(proccess_frame)
            fheight, fwidth ,_= frame.shape 
            
            if res.detections is not None:

                ''' Initializing midoint vars '''
                midpoint_x, midpoint_y = 0 , 0

                for face in res.detections:
                    face_bounded_box = np.multiply(
                        [
                            face.location_data.relative_bounding_box.xmin,
                            face.location_data.relative_bounding_box.ymin,
                            face.location_data.relative_bounding_box.width,
                            face.location_data.relative_bounding_box.height,
                        ],
                        [fwidth, fheight, fwidth, fheight]).astype(int)
                    
                    ''' Drawing a box around faces detected '''
                    cv.rectangle(frame, face_bounded_box, color=(0, 255, 0), thickness=2)

                    ''' Extracting nose(mid) landmark cords 
                        The purpose is to find the avg mid-point of the frame '''
                    midpoint_x += face.location_data.relative_keypoints[2].x
                    midpoint_y += face.location_data.relative_keypoints[2].y
                    
                    ''' Pointing facial landmarks on the frame '''
                    for p in face.location_data.relative_keypoints:
                        cv.circle(frame, np.multiply(np.array((p.x, p.y)),[fwidth, fheight]).astype(int), 1, (255, 255, 255), 2)

                avg_midpoint = midpoint_x / len(res.detections),midpoint_y / len(res.detections)
                
                ''' stepper motors full-step timeout & rotation calculations '''
                horizontal_timeout.value, vertical_timeout.value = round(0.006 - abs(avg_midpoint[0] - 0.5)/100, 4),round(0.006 - abs(avg_midpoint[1] - 0.5)/100,4 )
                rotaition = detect_rotation(avg_midpoint)

                ''' Pointing avg mid-point on the frame '''
                cv.circle(frame, np.multiply(np.array(avg_midpoint),[fwidth,fheight]).astype(int), 2, (0, 0, 0), 2)

                ''' Putting rotation direction text on the frame '''
                cv.putText(frame, f"{rotaition[0]} {rotaition[1]}",org=(fwidth // 2,30) ,fontFace=0 ,fontScale=1,thickness=2, color=(200, 200, 0),)
            else :
                ''' Reseting rotations and stepper motors full-step timeout '''
                horizontal_direction.value, vertical_direction.value = 0, 0
                horizontal_timeout.value, vertical_timeout.value = 0.006, 0.006
            
            fps = frame_counter / (time.time() - start_time)
            
            ''' Displaying live-FPS on the frame '''
            cv.putText(frame, f"FPS:{fps:.1f}",org=(30,30) ,fontFace=cv.FONT_HERSHEY_PLAIN,fontScale=2,thickness=2, color=(0, 255, 255),)

            ''' Setting output frame props '''
            cv.namedWindow("frame", cv.WINDOW_NORMAL)
            cv.setWindowProperty("frame", prop_value=cv.WINDOW_FULLSCREEN, prop_id=cv.WND_PROP_FULLSCREEN)
            
            ''' Exposing output frame window'''
            cv.imshow("frame", frame)

            ''' Breaking out of the loop, program termination '''
            if cv.waitKey(1) & 0xFF == ord("q"):
                break
        ''' Terminating movement processes '''
        horizontal_movement_process.terminate()
        vertical_movement_process.terminate()

        ''' Reseting RasPi GPIO_PINS (Unnecessary)'''
        # horizontal_motor.reset_pins()
        # vertical_motor.reset_pins()

        ''' Releasing capture device, Final Clean Up '''
        cap.release()
        cv.destroyAllWindows()
        gpio.cleanup()