# Publisher that publish the offset result of the lane
# We can choose use the video or the camera by giving parameter in the launch file

import rospy
import rospkg
import cv2
import numpy as np
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError

class LaneDetectorNode:
    def __init__(self, use_video=False, video_path=None):
        #  Create cv_bridge object
        self.bridge = CvBridge()
        # Create object that use video or not
        self.use_video = use_video
        self.video_path = video_path
        # Use to visualize the lane
        self.visualization = rospy.get_param('~visualization', False)

        # Create a publisher to publish offset
        self.offset_pub = rospy.Publisher('lane_offset', Float32, queue_size=10)

        # Create a publisher to publish the arrows direction
        self.arrow_pub = rospy.Publisher('arrow_direction', String, queue_size=10 )
        # Parameters for the process image
        self.height_ratio = 0.65
        self.width_offset = 0
        self.canny_low_threshold = 130
        self.canny_high_threshold = 200
        self.binary_lower_bound = 140
        self.binary_upper_bound = 255

        # PArameters for arrow detection
        self.arrow_roi_height_upper = 0.0
        self.arrow_roi_height_lower = 0.5
        self.arrow_roi_width_upper = 0.77
        self.arrow_roi_width_lower = 0.22
        self.arrow_canny_low_threshold = 100
        self.arrow_canny_high_threshold = 150

        # Create parameters to record the last frame's left and right lanes
        self.last_left_x = None
        self.last_right_x = None
        self.lane_match_threshold = 120

        # Recoed the last offset to avoid the lane change too fast
        self.last_offset = None
        self.alpha = 0.7
        self.max_jump = 50

        # Control the distance for the car to detect triangle, 1 sec before the turning part
        self.triangle_area_threshold = 2300


        # Video mode
        if self.use_video:
            self.cap = cv2.VideoCapture(self.video_path)
            # If the video is not opened, exit
            if not self.cap.isOpened():
                # Use logerr to print error message
                rospy.logerr("Faile to open file: %s", self.video_path)
                sys.exit(1)
            rospy.loginfo("Use video mode")
        # ROS camera subscribe mode
        else:
            # subscribe the camera topic
            self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
            rospy.loginfo("Use camera mode")
    

    def run_video_loop(self):
        """
        In the video mode, we use this function to process the video
        """
        # Set the rate of the offset publisher
        rate = rospy.Rate(10)
        # When the node is running, check if the video is opened
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            # if the video is not opened, exit
            if not ret:
                rospy.logerr("Failed to read video frame")
                break
            # Process the frame
            self.process_frame(frame)
            rate.sleep()
        self.cap.release()
        cv2.destroyAllWindows()

    def image_callback(self, data):
        """
        Callback function for the camera image 
        1. Change the image to cv2 format
        2. Process the image
        """
        try:
            # Convert the ROS image to the Opencv format
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_frame(frame)
        except CvBridgeError as e:
            rospy.logerr("Failed to convert image to opencv format: %s", e)
            

    def process_frame(self, frame):
        """Respond to publish the offset of the lane
        1. Use process_image to process the image
        2. Call all the functions to classify the lane
        3. Publish the offset
        4. Visualize the lane
        """
        # Show the arrow ROI
        direction = self.detect_arrow(frame)

        left_lane, right_lane, lane_center = self.process_image(frame)



        # Publish the offset
        height, width = frame.shape[:2]
        center = width/2
        if lane_center is not None and not np.isnan(lane_center):
            # Calculate the offset
            offset = lane_center-center
            if self.last_offset is not None:
                # Apply a low-pass filter to smooth the offset, last_offset*alpha + offset*(1-alpha)
                offset = self.alpha * self.last_offset + (1 - self.alpha) * offset
                if abs(offset - self.last_offset) > self.max_jump:
                    if offset > self.last_offset:
                        offset = self.last_offset + self.max_jump
                    elif offset < self.last_offset:
                        offset = self.last_offset - self.max_jump
            
            self.last_offset = offset
            # Publish the offset
            self.offset_pub.publish(float(offset))
            rospy.logdebug("Offset: %f", offset)
        # Publish the direction of the arrow
        if(direction != 0):
            self.arrow_pub.publish(direction)
        if self.visualization:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

    def process_image(self, frame):
        """
        Use findContours to detect the lane lines instead of Hough Transform
        1. Convert the image to grayscale
        2. Apply Gaussian blur to reduce noise
        3. Apply binary threshold to isolate the lane lines
        4. Define the ROI region to reduce the noise
        5. Use canny edge detection to detect the edges
        6. Use cv2.findContours to find the contours of the image
        """
        # Get the height and width of the image
        height, width = frame.shape[:2]

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce the noise
        blur = cv2.GaussianBlur(gray, (9, 9), 0)

        # Apply threshold to isolate the lane lines
        _, binary = cv2.threshold(blur, self.binary_lower_bound, self.binary_upper_bound, cv2.THRESH_BINARY_INV)

        binary = cv2.medianBlur(binary, 5)

        # Define the ROI region
        roi_region = np.array(
            [[(0, height*self.height_ratio),
              (0, height),
              (width, height),
              (width, height*self.height_ratio)
              ]], dtype=np.int32
        )
        roi_mask = np.zeros_like(binary)
        cv2.fillPoly(roi_mask, roi_region, 255)
        roi_binary = cv2.bitwise_and(binary, roi_mask)


        # Canny edge detection to detect the edges
        edges = cv2.Canny(roi_binary, self.canny_low_threshold, self.canny_high_threshold)
        cv2.imshow('edges', edges)

        # Find the contours of the image
        contours, _ = cv2.findContours(roi_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return self.classify_lanes(frame, contours) 

    def detect_arrow(self, frame):
        """ Detect the arrow direction in upper-part of the image
        1. Convert to grayscale
        2. Apply Gaussian blur to reduce noise
        3. Apply adaptive binary threshold to isolate the arrow
        4. Apply the ROI mask to the image to reduce the noise
        5. Use cv2.findCountour to find the countour of the image
        6. Use cv2.arcLength to calculate the length of the contours
        7. Use cv2.approxPolyDP to approximate the polygon of the contours
        8. Judge the direction of the arrow
        9. Return the direction of the arrow
        """
        
        # Get the height and width of the image
        height, width = frame.shape[:2]
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce the noise
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        # Don't apply  adaptive binary threshold to isolate the arrow, because the difference in white paper and black arrow
        # Have high contrast, the adaptive binary threshold is not necessary, instead use binary thresho;d 
        # adaptive_binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        # binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        
        # Use canny edge detection to detect the edges
        binary = cv2.Canny(blur, self.arrow_canny_low_threshold, self.arrow_canny_high_threshold)
        # Define the ROI region
        roi_region_arrow = np.array([[(width*self.arrow_roi_width_lower,height*self.arrow_roi_height_upper), 
                                      (width*self.arrow_roi_width_upper, height*self.arrow_roi_height_upper),
                                        (width*self.arrow_roi_width_upper,height*self.arrow_roi_height_lower),
                                        (width*self.arrow_roi_width_lower, height*self.arrow_roi_height_lower)]], dtype=np.int32)
        roi_mask_arrow = np.zeros_like(binary)
        # White the ROI region and put it into the mask, only the ROI region are white
        cv2.fillPoly(roi_mask_arrow, roi_region_arrow, 255)
        masked_binary = cv2.bitwise_and(binary, roi_mask_arrow)
        cv2.imshow('masked_binary', masked_binary)

        # Find the contours of the image
        contours, _ = cv2.findContours(masked_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        

        for contour in contours:
            # Filter the triangle caused by lane
            area = cv2.contourArea(contour)
            if area < self.triangle_area_threshold:
                continue
            # Calculate and estimate the length of the contour
            epsilon = 0.02*cv2.arcLength(contour, True)
            # Approximate the ploygon of the contour
            # approx are the points of the polygon
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # When the approx is 3, we assume it is a triangle
            if(len(approx) == 3):
                cv2.drawContours(frame, [approx], 0, (255, 255, 0), 2)
                
                # Get the points of the triangle
                # Use np.squeeze to compress the points to 2D array
                vertices = np.squeeze(approx)
                # Calculate the length of the lines
                check1 = self.get_line_len(vertices[0][0], vertices[0][1], vertices[1][0], vertices[1][1])
                check2 = self.get_line_len(vertices[1][0], vertices[1][1], vertices[2][0], vertices[2][1])
                check3 = self.get_line_len(vertices[0][0], vertices[0][1], vertices[2][0], vertices[2][1])

                # Find the point at the upper part of the triangle
                vertices = sorted(vertices, key=lambda x:x[1])
                top = vertices[0]
                bottom_left, bottom_right = vertices[1], vertices[2]

                # Make sure the buttom left point is the left point and the bottom right point is the right point
                if(bottom_left[0] > bottom_right[0]):
                    bottom_left, bottom_right = bottom_right, bottom_left
                # Calculate the center of the triangle
                center = (bottom_left[0] + bottom_right[0]) / 2

                # Determine the direction of arrow
                if center < top[0]:
                    direction = "left"
                else:
                    direction = "right"
                print("Direction: ", direction)
                #Check the length of the lines
                if(check1 > 15 and check2 > 15 and check3 > 15):

                    # Publish the direction of the arrow
                    return direction
        return 0


    def get_line_len(self, x1, y1, x2, y2):
        """Helper function to calculate the length of the line for triangle detection"""
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)
            

    def classify_lanes(self,frame, contours):
        """ 
        """
        height, width = frame.shape[:2]
        video_center = width / 2
        
        left_lane = None
        right_lane = None

        
        # Filter the small contours and weird contours
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 200:
                continue

            # Calculate the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)
            
            if h < 0.2*height:
                continue
            valid_contours.append(contour)

    
        # Display the contours
        image = frame.copy()
        cv2.drawContours(image, valid_contours, -1, (0, 255, 0), 2)
        cv2.imshow('all contours', image)



        if valid_contours:
            contour_x_positions = []
            for contour in valid_contours:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    contour_x_positions.append((contour, cx))

            contour_x_positions.sort(key=lambda x: x[1])

            left_candidates = []
            right_candidates = []

            for contour, cx in contour_x_positions:
                if cx < video_center:
                    left_candidates.append((contour, cx))
                else:
                    right_candidates.append((contour, cx))

            # 選擇最接近上一幀的 x 的作為 left/right
            left_lane = None
            right_lane = None

            if left_candidates:
                if self.last_left_x is not None:
                    left_lane, left_x = min(left_candidates, key=lambda x: abs(x[1] - self.last_left_x))
                else:
                    left_lane, left_x = max(left_candidates, key=lambda x: x[1])  # 靠近中心的左邊
                self.last_left_x = left_x

            if right_candidates:
                if self.last_right_x is not None:
                    right_lane, right_x = min(right_candidates, key=lambda x: abs(x[1] - self.last_right_x))
                else:
                    right_lane, right_x = min(right_candidates, key=lambda x: x[1])  # 靠近中心的右邊
                self.last_right_x = right_x




        # Create results image
        result_image = frame

        # Have left lane and right lane
        if left_lane is not None and right_lane is not None:
            left_points = left_lane.reshape(-1, 2)
            right_points = right_lane.reshape(-1, 2)

            # 找左車道底部最右側點
            left_rightmost = left_points[left_points[:, 0].argmax()]
            # 找右車道底部最左側點
            right_leftmost = right_points[right_points[:, 0].argmin()]

            center_bottom_x = (left_rightmost[0] + right_leftmost[0]) // 2

            # 畫中心線
            cv2.line(result_image, (center_bottom_x, height), (center_bottom_x, height-100), (0, 0, 255), 2)

            lane_center = center_bottom_x
        # Only have left lane    
        elif left_lane is not None:
            left_points = left_lane.reshape(-1, 2)
            # Calculate the estimated right lane position
            estimated_right_x = left_points[:, 0].max() + (width * 0.65)
            lane_center = left_points[:, 0].max() + (width * 0.65) / 2

        # Only have right lane
        elif right_lane is not None:
            right_points = right_lane.reshape(-1, 2)
            # Calculate the estimated left lane position
            estimated_left_x = right_points[:, 0].min() - (width * 0.65)
            lane_center = right_points[:, 0].min() - (width * 0.65) / 2
        
        # If no lane is detected, return None
        else:
            lane_center = None

        # Draw the left and right lanes
        if left_lane is not None:
            cv2.drawContours(result_image, [left_lane], -1, (0, 255, 0), 2)
        
        if right_lane is not None:
            cv2.drawContours(result_image, [right_lane], -1, (255, 0, 0), 2)
        
        if (right_lane is not None and left_lane is None) or (right_lane is None and left_lane is not None):
            cv2.line(result_image, (int(lane_center), height), (int(lane_center), height-100), (0, 0, 255), 2)

        cv2.imshow('result', result_image)
        return left_lane, right_lane, lane_center    

if __name__ == '__main__':
    # Initialize the ROS node
    try:
        rospy.init_node("lane_detector_node", anonymous=True)
        use_video = rospy.get_param('~use_video')
        video_path = rospy.get_param('~video_path')
        # Use the rospack to get the path of the video
        #rospack = rospkg.RosPack()
        #video_path = rospack.get_path('lane_follower') + video_path

        node = LaneDetectorNode(use_video=use_video, video_path=video_path)

        if use_video:
            # Run the video loop
            node.run_video_loop()
        else:
            # Keep the node running until shutdown
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
