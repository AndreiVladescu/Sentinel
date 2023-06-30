import cv2
import supervision
from ultralytics import YOLO
import supervision as sv

model = YOLO('yolov8m.pt')  # load an official detection model

from Sentinel.ballistic_calculator import BallisticCalculator

def main():
    cap = cv2.VideoCapture(0)
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        #frame = cv2.imread("img2.jpg")
        ballistic_computer = BallisticCalculator(resolution=(frame.shape[1], frame.shape[0]))
        #results = model.track(source=image, tracker="bytetrack.yaml", device=0)
        results = model.track(source=frame, tracker="tracker.yaml", device=0)
        result = results[0]

        # if result.numpy().linalg.norm > 1:
        '''dets = results[0].boxes.shape[0]
        # else:
        #    continue
        print(image.shape)
        for i in range(0, dets):
            if result.id is None:
                continue
            pred_class = result.cls.numpy()[i]
    
            id = result.id.numpy()[i]
            bboxes = result.data.numpy()[i][0:4]
            conf = result.data.numpy()[i][5]
    
            top_left = (int(bboxes[1]), int(bboxes[0]))
            bottom_right = (int(bboxes[1] + bboxes[3]), int(bboxes[0] + bboxes[2]))
            #bottom_right = (int(bboxes[3]), int(bboxes[2]))
    
            image = cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
            text_pos = (top_left[0], top_left[1] - 25)
            image = cv2.putText(image, str(id), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
            print(pred_class, id, bboxes, conf)
    '''

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

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
