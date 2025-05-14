# Publisher that publish the offset result of the lane
# We can choose use the video or the camera by giving parameter in the launch file

import rospy
import rospkg
import cv2
import numpy as np
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
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

        # Parameters for the process image
        self.height_ratio = 0.6
        self.width_offset = 0
        self.canny_low_threshold = 90
        self.canny_high_threshold = 160
        self.hough_threshold = 30#20
        self.hough_min_line_length = 20#5
        self.hough_max_line_gap = 30#60


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
        # Process the image
        lines = self.process_image(frame)
        # Classify the lines into left and right lines
        left_lines, right_lines = self.classify_lines(frame, lines)
        # Calculate the center of the lane
        lane_center = self.classify_center(frame, left_lines, right_lines)

        height, width = frame.shape[:2]
        center = width/2
        if lane_center is not None and not np.isnan(lane_center):
            # Calculate the offset
            offset = lane_center-center
            self.offset_pub.publish(float(offset))
            rospy.logdebug("Offset: %f", offset)
        if self.visualization:
            # Visualize the lane
            result_frame = self.visualize_lines(frame, left_lines, right_lines, lane_center)
            cv2.imshow('result', result_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User requested shutdown")

    def process_image(self, frame):
        """ Preprocess the image for lane detection 
        1. Convert to grayscale 
        2. Define the ROI region first and then apply the mask
        3. Apply the Gaussian blur to reduce noise
        4. Apply the binary threshold to isolate the lane lines
        5. Apply the ROI mask to the image to reduce the noise and help canny edge detection
        6.Applu erosion and dilation to remove noise
        7. Apply the Canny edge detection to detect the edges
        8. Apply the Hough transform to detect the lines
        9. Return the lines
        """
        # Define the ROI region first and then apply the mask

        height, width = frame.shape[:2]
        # Convert to grayscale(0-255)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        # Apply Gaussian blur to reduce noise and improve edge detection
        blur = cv2.GaussianBlur(gray, (5,5), 0)    
        # Apply adaptive_binary threshold to isolate the lane lines, very helpful
        adaptive_binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2) 
        #cv2.imshow('binary', adaptive_binary)  
        # Define the ROI region
        roi_region = np.array([[(0, height), (0+self.width_offset,  height*self.height_ratio), (width-self.width_offset, height*self.height_ratio), (width, height)]], dtype=np.int32)
        roi_mask = np.zeros_like(adaptive_binary)
        # White the ROI region and put it into the mask, only ROI region is white
        cv2.fillPoly(roi_mask, roi_region, 255)

        #Apply morphological operations, to create kernals for dilation and erosion
        kernal = np.ones((3,3), np.uint8)
        # Erosion to remove noise
        eroded = cv2.erode(adaptive_binary, kernal, iterations=1)
        # Dilation to fill gaps in the lane lines
        dilated = cv2.dilate(eroded, kernal, iterations=2)
        #cv2.imshow('dilated', dilated)

        # Use and to access the origin image's ROI region and the other region is black
        dilated_roi = cv2.bitwise_and(dilated, roi_mask)
        # Show the ROI region
        cv2.imshow('dilated_roi', dilated_roi)

        #Apply Canny edge detection after applying mask to let canny focus on the lane lines
        edges = cv2.Canny(dilated_roi, self.canny_low_threshold, self.canny_high_threshold)
        cv2.imshow('edges', edges)

        # Apply ROI mask again on the edges to avoid noise
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        #cv2.imshow('masked_edges', masked_edges)

        # Use hough transform to detect lines
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        return lines
    def classify_lines(self,frame, lines):
        """ Classify line into right lanes and left lanes
        1. When only detect one lines, and all the x value is larger than 0.25*width, we classify it as right lane
        2. When only detect one lines, and all the x value is smaller than 0.75*width, we classify it as left lane
        3. When detect two lines, we classify the left and right lanes depends on if x value exceed 0.5*width
        """
        if lines is None:
            return None, None
        height, width = frame.shape[:2]
        left_lines, right_lines = [], []
            
        # Only detect one lane
        if len(lines) <= 2:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                avg_x = (x1 + x2) / 2
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
                if slope > 0.15:
                    if avg_x < 0.6*width and x1 < x2:                    
                        left_lines.append(line)
                elif slope < -0.15:       
                    if avg_x > 0.4*width and x1 > x2:
                        right_lines.append(line)
                else:
                    left_lines.append(line)
        else:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
                avg_x = (x1 + x2) / 2
                if slope < -0.15 and avg_x < 0.5*width:
                    left_lines.append(line)
                elif slope > 0.15:
                    right_lines.append(line)
                else:
                    left_lines.append(line)
    
        return left_lines, right_lines

    def classify_center(self, frame, left_lines, right_lines):
        """Calculate the center of lane
        1. If only have one lane, we assume the width of lane and get the center
        2. If have two lanes, just average the x value of the two lanes
        """
        height, width = frame.shape[:2]
        
        # 初始化一个默认值，以防所有情况都失败
        default_center = width * 0.7
        
        # 检查是否有空列表或None
        if left_lines is None or len(left_lines) == 0:
            if right_lines is not None and len(right_lines) > 0:
                # 只有右侧车道线
                lane_width = 0.7 * width
                # 使用列表推导式前先检查列表不为空
                right_x_values = [line[0][0] for line in right_lines if len(line) > 0 and len(line[0]) >= 1]
                if right_x_values:  # 确保列表不为空
                    right_x_mean = np.mean(right_x_values)
                    return right_x_mean - lane_width / 2
            return default_center  # 如果没有有效数据，返回默认值
            
        elif right_lines is None or len(right_lines) == 0:
            if left_lines is not None and len(left_lines) > 0:
                # 只有左侧车道线
                lane_width = 0.7 * width
                # 使用列表推导式前先检查列表不为空
                left_x_values = [line[0][0] for line in left_lines if len(line) > 0 and len(line[0]) >= 1]
                if left_x_values:  # 确保列表不为空
                    left_x_mean = np.mean(left_x_values)
                    return left_x_mean + lane_width / 2
            return default_center  # 如果没有有效数据，返回默认值
            
        else:
            # 两侧都有车道线
            try:
                # 添加错误处理，确保line[0][0]存在
                left_x_values = [line[0][0] for line in left_lines if len(line) > 0 and len(line[0]) >= 1]
                right_x_values = [line[0][0] for line in right_lines if len(line) > 0 and len(line[0]) >= 1]
                
                if left_x_values and right_x_values:  # 确保两个列表都不为空
                    left_x = np.mean(left_x_values)
                    right_x = np.mean(right_x_values)
                    return (left_x + right_x) / 2
                else:
                    return default_center
            except (IndexError, ValueError) as e:
                print(f"error when calculating: {e}")
                return default_center  # 发生错误时返回默认值

    def visualize_lines(self, frame, left_lines, right_lines, lane_center):
        """ Just draw the line that hough transform detected """
        height, width = frame.shape[:2]
        line_image = np.zeros_like(frame)
        # Draw the left and right lanes
        if left_lines is not None:
            for line in left_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(line_image, 'left lane', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if right_lines is not None:
            for line in right_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(line_image, 'right lane', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # Draw the center of the lane in red
        if lane_center is not None and not np.isnan(lane_center):
            cv2.line(line_image, (int(lane_center), height), (int(lane_center), height-50), (0, 0, 255), 2)
            cv2.putText(line_image, 'lane center', (int(lane_center), height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # Draw the center of the video in yellow
        center_x = int(width/2)
        cv2.line(line_image, (center_x, height), (center_x, height-50), (255, 255, 0), 2)
        cv2.putText(line_image, 'video center', (center_x, height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        # Show the offset between the lane center and the video center
        offset = lane_center - center_x
        if offset > 0:
            cv2.putText(line_image, f'offset left: {offset:.2f}', (width-20, height-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        elif offset < 0:
            cv2.putText(line_image, f'offset right: {offset:.2f}', (width-20, height-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        else:
            cv2.putText(line_image, 'offset: 0', (width-20, height-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # Show the line
        #cv2.imshow('line_image', line_image)
        return cv2.addWeighted(frame, 0.8, line_image, 1, 0)
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
