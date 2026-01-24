import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np

class BrailleNode(Node):
    def __init__(self):
        super().__init__('braille_translator')

        # QoS for RealSense
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
            
        self.publisher_ = self.create_publisher(String, '/braille_text', 10)
        
        # Variable to remember the last thing we said
        self.last_text = ""
        
        self.get_logger().info('RealSense Braille Reader (Stable Version) Started.')

        # Braille Map
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
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Process Braille
            text, debug_image = self.process_braille(cv_image)
            
            # Show the view WITH red boxes around detections
            cv2.imshow("RealSense View", debug_image)
            cv2.waitKey(1)

            # --- SPAM FILTER ---
            # Only publish if we found text AND it is different from the last time
            if text and text != self.last_text:
                msg_out = String()
                msg_out.data = text
                self.publisher_.publish(msg_out)
                self.get_logger().info(f'Translated: "{text}"')
                self.last_text = text # Update memory
            
            # If we lose the text, reset memory so we can detect it again later
            if not text:
                self.last_text = ""
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def process_braille(self, img):
        debug_img = img.copy() # Copy image for drawing boxes
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Threshold: Adjust 140 if your room is darker/lighter
        _, binary = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # --- NOISE FILTER ---
        # Increased min area from 10 to 30 to ignore small speckles
        dots = []
        for c in contours:
            if cv2.contourArea(c) > 30: 
                bbox = cv2.boundingRect(c)
                dots.append(bbox)
                # Draw red box around detected dot
                x, y, w, h = bbox
                cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 0, 255), 2)

        if not dots:
            return "", debug_img

        # Sort dots: Y (rows), then X (columns)
        dots.sort(key=lambda b: (b[1] // 30, b[0])) 

        # Group into Lines
        lines = []
        current_line = []
        last_y = dots[0][1]
        for dot in dots:
            x, y, w, h = dot
            if abs(y - last_y) > 30: 
                lines.append(current_line)
                current_line = []
                last_y = y
            current_line.append(dot)
        lines.append(current_line)

        translated_text = ""

        for line in lines:
            line.sort(key=lambda b: b[0]) 
            if not line: continue
            
            # Group dots into Characters
            clusters = [] 
            current_cluster = [line[0]]
            for i in range(1, len(line)):
                prev_x = line[i-1][0]
                curr_x = line[i][0]
                if curr_x - prev_x > 40: 
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
