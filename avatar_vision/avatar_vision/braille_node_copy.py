# 더 수정한 버전입니다. 금요일에 돌린 코드도 같이 업로드 해 달라 하셔서 ai한테 더 물어 본 건 copy 붙여서 올립니당

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image 
from std_msgs.msg import String
from cv_bridge import CvBridge 
import cv2
import numpy as np

class BrailleNode(Node):
    def __init__(self):
        super().__init__('braille_translator')
        self.bridge = CvBridge()

        # 1. Input: RealSense (Compressed)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            '/realsense/color/image_raw/compressed', 
            self.image_callback,
            qos_profile)

        # 2. Output A: Text (The translated words)
        self.text_publisher = self.create_publisher(String, '/braille_text', 10)
        
        # 3. Output B: Debug Image (The video with green boxes)
        self.img_publisher = self.create_publisher(Image, '/braille_debug_img', 10)
        
        self.last_text = ""
        self.get_logger().info('Braille Publisher Node Started.')

        # Full Braille Map
        self.braille_map = {
            (1, 0, 0, 0, 0, 0): 'a', (1, 1, 0, 0, 0, 0): 'b', (1, 0, 0, 1, 0, 0): 'c',
            (1, 0, 0, 1, 1, 0): 'd', (1, 0, 0, 0, 1, 0): 'e', (1, 1, 0, 1, 0, 0): 'f',
            (1, 1, 0, 1, 1, 0): 'g', (1, 1, 0, 0, 1, 0): 'h', (0, 1, 0, 1, 0, 0): 'i',
            (0, 1, 0, 1, 1, 0): 'j', (1, 0, 1, 0, 0, 0): 'k', (1, 1, 1, 0, 0, 0): 'l',
            (1, 0, 1, 1, 0, 0): 'm', (1, 0, 1, 1, 1, 0): 'n', (1, 0, 1, 0, 1, 0): 'o',
            (1, 1, 1, 1, 0, 0): 'p', (1, 1, 1, 1, 1, 0): 'q', (1, 1, 1, 0, 1, 0): 'r',
            (0, 1, 1, 1, 0, 0): 's', (0, 1, 1, 1, 1, 0): 't', (1, 0, 1, 0, 0, 1): 'u',
            (1, 1, 1, 0, 0, 1): 'v', (0, 1, 0, 1, 1, 1): 'w', (1, 0, 1, 1, 0, 1): 'x',
            (1, 0, 1, 1, 1, 1): 'y', (1, 0, 1, 0, 1, 1): 'z',
            (0, 0, 0, 0, 0, 0): ' ' 
        }

    def image_callback(self, msg):
        try:
            # Decode Input
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Process
            text, debug_image = self.process_braille(cv_image)
            
            # --- PUBLISH IMAGE (Screen Info) ---
            ros_image = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            self.img_publisher.publish(ros_image)

            # --- PUBLISH TEXT ---
            if text and text != self.last_text:
                msg_out = String()
                msg_out.data = text
                self.text_publisher.publish(msg_out)
                self.get_logger().info(f'Published: "{text}"')
                self.last_text = text
            
            if not text:
                self.last_text = ""
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def process_braille(self, img):
        debug_img = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Adaptive Threshold (Handles shadows well)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2)
            
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 1. Filter Contours (Must be round-ish dots)
        dots = []
        for c in contours:
            area = cv2.contourArea(c)
            if area > 20: # Min size
                perimeter = cv2.arcLength(c, True)
                if perimeter == 0: continue
                
                # Circularity Check: 1.0 is perfect circle
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                if 0.7 < circularity < 1.2:
                    bbox = cv2.boundingRect(c)
                    dots.append(bbox)
                    
                    # Draw Green Box
                    x, y, w, h = bbox
                    cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                else:
                    pass

        if not dots:
            return "", debug_img

        # 2. Sort Dots (Y then X)
        dots.sort(key=lambda b: (b[1] // 30, b[0])) 

        # 3. Group into Lines
        lines = []
        current_line = []
        last_y = dots[0][1]
        
        for dot in dots:
            x, y, w, h = dot
            if abs(y - last_y) > 30: # New line detection
                lines.append(current_line)
                current_line = []
                last_y = y
            current_line.append(dot)
        lines.append(current_line)

        translated_text = ""

        # 4. Process Lines & Characters
        for line in lines:
            line.sort(key=lambda b: b[0]) # Left to right
            if not line: continue
            
            # Group into Clusters (Characters)
            clusters = [] 
            current_cluster = [line[0]]
            for i in range(1, len(line)):
                prev_x = line[i-1][0]
                curr_x = line[i][0]
                if curr_x - prev_x > 40: # Gap between letters
                    clusters.append(current_cluster)
                    current_cluster = []
                current_cluster.append(line[i])
            clusters.append(current_cluster)

            # Decode Clusters
            for cluster in clusters:
                min_x = min(d[0] for d in cluster)
                min_y = min(d[1] for d in cluster)
                
                pattern = [0] * 6
                for (x, y, w, h) in cluster:
                    # Determine Grid Position (Left/Right, Top/Mid/Bot)
                    rel_x = 0 if (x - min_x) < 15 else 1 
                    rel_y = 0 
                    if (y - min_y) > 12: rel_y = 1
                    if (y - min_y) > 28: rel_y = 2
                    
                    index = rel_y + (rel_x * 3) 
                    if 0 <= index < 6:
                        pattern[index] = 1
                
                char = self.braille_map.get(tuple(pattern), '?')
                translated_text += char
            
            translated_text += " " 

        return translated_text.strip(), debug_img

def main(args=None):
    rclpy.init(args=args)
    node = BrailleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
