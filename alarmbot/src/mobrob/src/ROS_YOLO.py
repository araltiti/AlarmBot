#!/usr/bin/env python

import rospy
from mobrob_util.msg import ImgFrame

import cv2
import numpy as np
import time


def img_processing():
    #Intitialize Node and Publisher
    rospy.init_node('img_and_camera_node', anonymous=False)
    pub_img_pos = rospy.Publisher('/img_pos_raw', ImgFrame, queue_size=10)
    img_pos_msg = ImgFrame()


    
    # Load Yolo
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # Loading camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH,240)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,320)
    font = cv2.FONT_HERSHEY_PLAIN
    starting_time = time.time()
    frame_id = 0
    while True:
        ret, frame = cap.read()
        frame_id += 1
        #print(frame_id)
        height, width, channels = frame.shape


         # Detecting objects
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)
        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        img_pos_msg.detect = False
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if((str(classes[class_id]) == "person") and (confidence > 0.5)):
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)                      
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    #Saving message
                    img_pos_msg.centerX = center_x
                    img_pos_msg.centerY = center_y
                    img_pos_msg.height = h
                    img_pos_msg.width = w
                    img_pos_msg.detect = True
                    
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                #elif(str(classes[class_id]) == "person"):
                    #img_pos_msg.detect = False

        pub_img_pos.publish(img_pos_msg)
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                if label == "person":
                    confidence = confidences[i]
                    color = colors[class_ids[i]]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.rectangle(frame, (x, y), (x + w, y + 30), color, -1)
                    cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y + 30), font, 3, (255,255,255), 3)
        elapsed_time = time.time() - starting_time
        fps = frame_id / elapsed_time
        cv2.putText(frame, "FPS: " + str(round(fps, 2)), (10, 50), font, 3, (0, 0, 0), 3)
        cv2.imshow("Image", frame)
        
        key = cv2.waitKey(1)
        if key == 7:
            break



    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try: 
        img_processing()
    except rospy.ROSInterruptException: 

        pass
