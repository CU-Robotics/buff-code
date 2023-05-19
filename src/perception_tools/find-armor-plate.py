import cv2
import numpy as np
import time
import math
import pyrealsense2 as rs

import rospy
from std_msgs.msg import Float64MultiArray


config = rs.config()
config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
config.enable_stream(rs.stream.color,1280,720,rs.format.rgb8,30)
config.enable_stream(rs.stream.gyro)

yaw = 0
pitch = 0

evaluation_ms = 0

pipe = rs.pipeline()
profile = pipe.start(config)


# TODO: Clean this up significantly
try:
    while not rospy.is_shutdown():
        detected = []

        frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        gyro = frames[2].as_motion_frame().get_motion_data() 
        yaw +=  gyro.y * (evaluation_ms / 1000.0)

        pitch += gyro.x * (evaluation_ms / 1000.0)

        if not depth or not color: continue
        depth_image = np.asanyarray(depth.get_data())
        color_image = np.asanyarray(color.get_data())
        raw_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        start_time = time.time()

        img_h, img_w = raw_frame.shape[:2]

        hsv = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2HSV)

        lowerw = np.array([0, 0, 50])
        upperw = np.array([255, 200, 150])
        maskw = cv2.inRange(hsv, lowerw, upperw)

        lowerb = np.array([0, 0, 0])
        upperb = np.array([255, 255, 50])
        maskb = cv2.inRange(hsv, lowerb, upperb)

        lower = np.array([90, 0, 250])
        upper = np.array([130, 200, 255])
        mask = cv2.inRange(hsv, lower, upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) > 0):
            epsilon = 0.1*cv2.arcLength(contours[0],True)
            approx = cv2.approxPolyDP(contours[0],epsilon,True)

        mask2 = np.zeros(raw_frame.shape[:2],dtype=np.uint8)
        for cnt in contours:
            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(mask2,(x,y),(x+w,y+h),(255),-1)

        # find the contours on the mask (with solid drawn shapes) and draw outline on input image
        contours, hierarchy=cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        vertical_lights = raw_frame.copy()
        filtered_rects = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 2: continue
            x,y,w,h = cv2.boundingRect(cnt)
            rect = ((x,y),(w,h))
            if (h / w < 1): continue
            filtered_rects.append(rect)
            cv2.rectangle(vertical_lights,rect[0],(rect[0][0]+rect[1][0],rect[0][1]+rect[1][1]),(0,255,0),1)

        detected_targets = raw_frame.copy()
        targets = []
        for i in range(0, len(filtered_rects)):
            rect1 = filtered_rects[i]
            rect1_halfpoint = rect1[0][1] + (rect1[1][1])
            for j in range(i+1, len(filtered_rects)):
                rect2 = filtered_rects[j]
                rect2_halfpoint = rect2[0][1] + (rect2[1][1])

                if (rect2[0][1] == 0): continue
                verticalSizeRatio = rect1[1][1] / rect2[1][1]
                if (verticalSizeRatio < 0.5 or verticalSizeRatio > 2): continue

                if (rect1_halfpoint > rect2[0][1] and rect2_halfpoint > rect1[0][1]):
                    x = min(rect1[0][0], rect2[0][0])
                    y = min(rect1[0][1], rect2[0][1])
                    w = max(rect1[0][0]+rect1[1][0], rect2[0][0]+rect2[1][0]) - x
                    h = max(rect1[0][1]+rect1[1][1], rect2[0][1]+rect2[1][1]) - y
                    if (w / h > 6): continue
                    if (w / h < 1): continue
                    new_rect = ((x, y), (w, h))

                    inner_target_x = min(rect1[0][0]+rect1[1][0], rect2[0][0]+rect2[1][0])
                    inner_target_y = y
                    inner_target_w = max(rect1[0][0], rect2[0][0]) - inner_target_x
                    inner_target_h = h
                    inner_target = ((inner_target_x, inner_target_y), (inner_target_w, inner_target_h))

                    if (inner_target_w*1.5 < rect1[1][0] or inner_target_w*1.5 < rect2[1][0]): continue

                    target_whitemask = maskw[y:y+h,x:x+w]
                    h, w = target_whitemask.shape
                    third_width = int(w/3.0)

                    center_third_white = target_whitemask[:,third_width:2*third_width]
                    white_density = np.sum(center_third_white) / (h * (w/3.0))
                    if (white_density < 30): continue

                    number_mask_inner_target = maskw[inner_target_y-inner_target_h:inner_target_y+2*inner_target_h, inner_target_x:inner_target_x+inner_target_w]
                    number_contours, number_hierarchy = cv2.findContours(number_mask_inner_target,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    numFound = False
                    for cnt in number_contours:
                        x2,y2,w2,h2 = cv2.boundingRect(cnt)
                        if h2 < h: continue
                        if w2 < inner_target_w / 5: continue
                        center = (x2+int(w2/2.0), y2+int(h2/2.0))
                        if (center[0] > inner_target_w*0.65 or center[0] < inner_target_w*0.35): continue
                        if (center[1] > 1.65*inner_target_h or center[1] < 1.35*inner_target_h): continue
                        numFound = True
                        cv2.rectangle(number_mask_inner_target,(x2,y2),(x2+w2,y2+h2),(255),1)

                    if (len(number_contours) < 1): continue
                    if not numFound: continue

                    target_colormask = mask2[inner_target_y:inner_target_y+inner_target_h,inner_target_x:inner_target_x+inner_target_w]
                    color_density = np.sum(target_colormask)
                    if (color_density > 0): continue

                    targets.append(new_rect)
                    cv2.rectangle(vertical_lights,new_rect[0],(new_rect[0][0]+new_rect[1][0],new_rect[0][1]+new_rect[1][1]),(0,0,255),2)
                    cv2.rectangle(vertical_lights,inner_target[0],(inner_target[0][0]+inner_target[1][0],inner_target[0][1]+inner_target[1][1]),(255,0,0),2)
                    
                    target_center = (int(x+(w/2.0)), int(y+(h/2.0)))
                    # cv2.line(vertical_lights, (target_center[0]-10, target_center[1]), (target_center[0]+10, target_center[1]), (0,255,0), 2)
                    # cv2.line(vertical_lights, (target_center[0], target_center[1]-10), (target_center[0], target_center[1]+10), (0,255,0), 2)

                    #print()
                    # Calculate depth
                    detected.append((target_center, depth.get_distance(target_center[0], target_center[1])))

        if (detected):
            centermost_target = detected[0]
            for target in detected:
                euclidean_center_dist_curr = math.sqrt((img_w - target[0][0])^2 + (img_h - target[0][1])^2)
                euclidean_center_dist_centermost = math.sqrt((img_w - centermost_target[0][0])^2 + (img_h - centermost_target[0][1])^2)
                if (euclidean_center_dist_curr > euclidean_center_dist_centermost):
                    centermost_target = target
            cv2.line(vertical_lights, (centermost_target[0][0]-10, centermost_target[0][1]), (centermost_target[0][0]+10, centermost_target[0][1]), (0,255,0), 2)
            cv2.line(vertical_lights, (centermost_target[0][0], centermost_target[0][1]-10), (centermost_target[0][0], centermost_target[0][1]+10), (0,255,0), 2)
            normalized_target = [centermost_target[0][0] / (img_w * 1.0), centermost_target[0][1] / (img_h * 1.0), centermost_target[1],yaw,pitch]
           

            pub = rospy.Publisher("vision", Float64MultiArray, queue_size = 1)
            rospy.init_node('detection')
            msg = Float64MultiArray()

            rate = rospy.Rate(10)

            # normalized_target[3] = yaw
            # normalized_target[4] = pi
            
            
        
            msg.data = np.array(normalized_target)
            

            pub.publish(msg)
            rate.sleep()
            


        evaluation_time = (time.time() - start_time)
        evaluation_ms = evaluation_time * 1000.0
        evaluation_fps = 1000.0 / (evaluation_ms + 0.0001)

        
        cv2.imshow("Detected Targets", vertical_lights)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            pipe.stop()
            break

    cv2.destroyAllWindows()

finally:
    pipe.stop()