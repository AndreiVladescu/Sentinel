import cv2
import serial
import supervision
from ultralytics import YOLO
import supervision as sv
import pyzed.sl as sl

model = YOLO('yolov8m.pt')  # load an official detection model

from Sentinel.ballistic_calculator import BallisticCalculator

def main():

    zed = sl.Camera()
    ser = serial.Serial('COM8', baudrate=115200)


    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.QUALITY
    init.coordinate_units = sl.UNIT.METER
    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    image_size = zed.get_camera_information().camera_resolution
    img_width = 1591
    img_height = 881
    image_size.width = img_width
    image_size.height = img_height
    image = sl.Mat(image_size.width, image_size.height)
    runtime_parameters = sl.RuntimeParameters()
    while True:
        # Grab an image
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            #frame = cv2.imread("img2.jpg")
            ballistic_computer = BallisticCalculator(resolution=(frame.shape[1], frame.shape[0]))
            width = frame.shape[1]
            height = frame.shape[0]
            #results = model.track(source=image, tracker="bytetrack.yaml", device=0)
            results = model.track(source=frame, tracker="tracker.yaml", device=0)
            result = results[0]
            cv2.circle(frame, (int(width/2), int(height/2)), 2, (0, 255, 0), 1)

            detections = sv.Detections.from_yolov8(result)
            #detections = detections[detections.class_id == 0]
            centers = []
            for box in detections.xyxy:
                print(box)
                centers.append((int(box[0] + (box[2] - box[0]) / 2), int(box[1] + (box[3] - box[1]) / 2)))
            box_annotator = sv.BoxAnnotator()
            det_nmbr = len(detections.xyxy)

            labels = [
                f"{i}"
                for i
                in range(0, det_nmbr)
            ]

            annotated_image = box_annotator.annotate(frame.copy(), detections=detections, labels=labels)
            cv2.circle(annotated_image, (int(annotated_image.shape[1]/2), int(annotated_image.shape[0]/2)), 2, (0, 255, 0), 4)

            for center in centers:
                cv2.circle(annotated_image, center, 2, (0, 0, 255), 2)
                angles = ballistic_computer.get_camera_angles(center[0], center[1])
                print(angles)

            cv2.imshow("Webcam Live", annotated_image)
            cv2.waitKey(0)
            # Quit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
